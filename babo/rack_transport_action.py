# rack_transport_action v3.201 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능변경) /robot_action Action 서버에서 MOVE/IN/OUT 3가지 작업을 단일 파일로 통합 실행
# - (기능변경) MOVE(Transport)는 요청하신 Pick&Place 시퀀스(Approach Y-250, Lift Z+30, Place Z+30, Retract Y-250)를 그대로 적용
# - (구조정리) DSR_ROBOT2 import 위치/초기화 규칙 적용: (1) ROBOT 상수 바로 뒤 DR_init.__dsr__id/__dsr__model 1회 (2) main()에서 노드 생성 후 initialize_robot() 1회
# - (안정화) rack 키 입력을 A-1 / a1 / A_1 형태로 받아 A-1로 정규화
# - (버그수정) execute_callback에서 DSR_ROBOT2를 import 하며 g_node(None)로 크래시 나는 문제 해결 (main() 초기화 1회로 고정)

"""[모듈] rack_transport_action

[역할]
- main_integrated에서 전달된 명령을 Action(RobotMove)로 받아 Doosan 로봇 동작을 실행한다.

[Action]
- name: /robot_action
- goal.command payload 예시:
  - MOVE,A-1,B-2
  - IN,NONE,A-1
  - OUT,A-1,NONE

[MOVE(Transport) 시퀀스 - 요청 반영]
1) Home 이동(안전 홈)
2) Pick Approach: 타겟 기준 BASE Y -250mm
3) Pick Target: 타겟 진입
4) Grip Close: 파지
5) Pick Lift: BASE Z +30mm 상승
6) Pick Retract: BASE Y -250mm 후퇴 (Lift 상태 유지)
7) Place Approach: 목적지 기준 BASE Z +30mm 상공 진입
8) Place Target(Down): BASE Z -30mm 하강(=목적지 타겟)
9) Grip Open: 놓기
10) Place Retract: BASE Y -250mm 후퇴
11) Home 이동(종료)

"""

from __future__ import annotations

import re
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import DR_init

try:
    from biobank_interfaces.action import RobotMove
except ImportError:
    # 타입/구문 체크용 더미
    class RobotMove:  # pragma: no cover
        class Goal:
            command = ""
        class Result:
            def __init__(self, success=False, message=""):
                self.success = success
                self.message = message
        class Feedback:
            status = ""


# ==========================================================
# ROBOT 상수 (사용자 규칙: 파람/상수 정의 바로 뒤 DR_init 세팅 1회)
# ==========================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ==========================================================
# 기본 모션 파라미터
# ==========================================================
V_J = 60
A_J = 60

V_L = 200.0
A_L = 200.0

# 느린 리프트/리트랙트용
V_L_SLOW = 50.0
A_L_SLOW = 50.0

# 홈(조인트)
HOME_J_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

# MOVE(Transport) 오프셋 (요청값)
MOVE_PICK_APP_DY = -250.0
MOVE_PICK_LIFT_DZ = 30.0
MOVE_PICK_RET_DY = -250.0

MOVE_PLACE_APP_DZ = 30.0
MOVE_PLACE_RET_DY = -250.0

# IN/OUT 기본값(기존 단발 노드 계열 - 안전한 기본)
IN_WB_APP_DY = -50.0
IN_RACK_APP_DY = -250.0
IN_TOOL_LIFT_Z = 20.0
IN_BASE_LIFT_Z = 250.0
IN_TARGET_TOP_DZ = 20.0
IN_FINAL_RETRACT_DY = -150.0

OUT_RACK_APP_DY = -100.0
OUT_WB_APP_DZ = 100.0
GRIP_WAIT_SEC = 1.0


def _norm(s: Optional[str]) -> Optional[str]:
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


def _normalize_rack_key(raw: Optional[str]) -> str:
    """A-1 / a_1 / A1 같은 입력을 A-1로 정규화"""
    if raw is None:
        return ""
    s = str(raw).strip().upper()
    s = s.replace("_", "-")
    s = re.sub(r"\s+", "", s)

    m = re.match(r"^([A-Z])\-([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.match(r"^([A-Z])([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    return s


def _apply_offset(dr, pose: Sequence[float], dx=0.0, dy=0.0, dz=0.0):
    """posx(x,y,z,rx,ry,rz) 형태에 오프셋 적용"""
    return dr.posx(
        float(pose[0]) + float(dx),
        float(pose[1]) + float(dy),
        float(pose[2]) + float(dz),
        float(pose[3]),
        float(pose[4]),
        float(pose[5]),
    )


def initialize_robot(node: Node):
    """사용자 규칙: main()에서 노드 생성 후 1회만 호출."""
    # ✅ DSR_ROBOT2 import 위치는 여기(함수 내부)로 고정
    import DSR_ROBOT2 as dr

    node.get_logger().info("#" * 50)
    node.get_logger().info("Initializing robot with the following settings:")
    node.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
    node.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
    node.get_logger().info(f"ROBOT_TCP: {ROBOT_TCP}")
    node.get_logger().info(f"ROBOT_TOOL: {ROBOT_TOOL}")
    node.get_logger().info("#" * 50)

    # (필요 시) 모드 설정
    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    # tool/tcp 1회 설정
    dr.set_tool(ROBOT_TOOL)
    dr.set_tcp(ROBOT_TCP)

    return dr


class RackTransportAction(Node):
    def __init__(self):
        super().__init__("rack_transport_action", namespace=ROBOT_ID)

        # main()에서 주입
        self.dr = None

        # dry_run=True면 로봇 이동 없이 성공만 반환
        self.declare_parameter("dry_run", False)

        # station builders / targets
        from .rack_stations import (
            build_rack_stations,
            build_workbench_station_dy,
            build_workbench_station_top,
            RACK_TARGETS,
        )
        self.RACK_TARGETS = RACK_TARGETS
        self.build_rack_stations = build_rack_stations
        self.build_wb_dy = build_workbench_station_dy
        self.build_wb_top = build_workbench_station_top

        # IO helpers
        from .gripper_io import grip_open, grip_close, grip_init_open
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.grip_init_open = grip_init_open

        # rel move helpers (IN/OUT에 사용)
        from .rel_move import rel_movel_tool, rel_movel_base
        self.rel_movel_tool = rel_movel_tool
        self.rel_movel_base = rel_movel_base

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ✅ ACTION 이름(/robot_action)은 절대 변경 금지
        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_service_qos_profile=qos,
            result_service_qos_profile=qos,
            cancel_service_qos_profile=qos,
            feedback_pub_qos_profile=qos,
            status_pub_qos_profile=qos,
        )

        self.get_logger().info("✅ [v3.200] /robot_action ready (MOVE/IN/OUT)")

    def set_dr(self, dr):
        self.dr = dr

    def _home(self):
        home_j = self.dr.posj(*HOME_J_DEG)
        self.dr.movej(home_j, vel=V_J, acc=A_J)

    def _valid_keys(self):
        keys = list(self.RACK_TARGETS.keys())
        keys.sort()
        return keys

    async def execute_callback(self, goal_handle):
        cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 EXEC: {cmd}")

        if bool(self.get_parameter("dry_run").value):
            goal_handle.succeed()
            return RobotMove.Result(success=True, message="Dry Run Success")

        if self.dr is None:
            goal_handle.abort()
            return RobotMove.Result(success=False, message="Robot not initialized (dr is None)")

        parts = [p.strip() for p in cmd.split(",") if p.strip() != ""]
        op = parts[0].upper() if parts else ""
        a1 = _norm(parts[1]) if len(parts) > 1 else None
        a2 = _norm(parts[2]) if len(parts) > 2 else None

        src = _normalize_rack_key(a1) if a1 else None
        dst = _normalize_rack_key(a2) if a2 else None

        try:
            if op == "MOVE":
                if not src or not dst:
                    raise ValueError("MOVE requires src & dest")
                ok, msg = self._do_transport(src, dst)

            elif op == "IN":
                # IN,NONE,A-1 형태 권장 (src는 무시)
                if not dst:
                    raise ValueError("IN requires dest")
                ok, msg = self._do_inbound(dst)

            elif op == "OUT":
                if not src:
                    raise ValueError("OUT requires src")
                ok, msg = self._do_outbound(src)

            else:
                ok, msg = False, f"Unknown op: {op}"

        except Exception as e:
            ok, msg = False, f"Error: {e}"

        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return RobotMove.Result(success=ok, message=msg)

    # ==========================================================
    # MOVE (Transport) - 요청 시퀀스 적용
    # ==========================================================
    def _do_transport(self, src: str, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid or dest not in valid:
            return False, f"Invalid rack key: src={src}, dest={dest}"
        if src == dest:
            return False, "src == dest"

        self.get_logger().info(f"[MOVE] {src} -> {dest}")

        rack = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        st_src = rack[src]
        st_dst = rack[dest]

        # 1) Home
        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # 2) Pick Approach: target 기준 BASE Y -250
        pick_app = _apply_offset(self.dr, st_src["target"], dy=MOVE_PICK_APP_DY)
        self.dr.movel(pick_app, vel=V_L, acc=A_L)

        # 3) Pick Target
        self.dr.movel(st_src["target"], vel=V_L, acc=A_L)

        # 4) Grip Close
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # 5) Pick Lift: BASE Z +30
        pick_lift = _apply_offset(self.dr, st_src["target"], dz=MOVE_PICK_LIFT_DZ)
        self.dr.movel(pick_lift, vel=V_L_SLOW, acc=A_L_SLOW)

        # 6) Pick Retract: BASE Y -250 (Lift 유지)
        pick_ret = _apply_offset(self.dr, pick_lift, dy=MOVE_PICK_RET_DY)
        self.dr.movel(pick_ret, vel=V_L_SLOW, acc=A_L_SLOW)

        # 7) Place Approach: 목적지 기준 BASE Z +30
        place_app = _apply_offset(self.dr, st_dst["target"], dz=MOVE_PLACE_APP_DZ)
        self.dr.movel(place_app, vel=V_L, acc=A_L)

        # 8) Place Target(Down): = target
        self.dr.movel(st_dst["target"], vel=V_L, acc=A_L)

        # 9) Grip Open
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # 10) Place Retract: BASE Y -250
        place_ret = _apply_offset(self.dr, st_dst["target"], dy=MOVE_PLACE_RET_DY)
        self.dr.movel(place_ret, vel=V_L_SLOW, acc=A_L_SLOW)

        # 11) Home
        self._home()

        return True, "Transport Done"

    # ==========================================================
    # INBOUND : WORKBENCH -> RACK
    # ==========================================================
    def _do_inbound(self, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if dest not in valid:
            return False, f"Invalid rack key: dest={dest}"

        self.get_logger().info(f"[IN] WB -> {dest}")

        wb = self.build_wb_dy(self.dr, approach_dy=IN_WB_APP_DY)
        rack = self.build_rack_stations(self.dr, approach_dy=IN_RACK_APP_DY)
        st = rack[dest]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # WB approach -> target
        self.rel_movel_base(self.dr, 0, -180.0, 0, 0, 0, 0, 50.0)
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        # Tool 상대 +Z lift
        self.rel_movel_tool(self.dr, 0, 0, IN_TOOL_LIFT_Z, 0, 0, 0, 20.0)

        # Grip close
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # Base 상대 +Z 크게 lift
        self.rel_movel_base(self.dr, 0, 0, IN_BASE_LIFT_Z, 0, 0, 0, 50.0)

        # Rack approach -> top -> target
        self.dr.movel(st["approach"], vel=V_L, acc=A_L)
        top = _apply_offset(self.dr, st["target"], dz=IN_TARGET_TOP_DZ)
        self.dr.movel(top, vel=V_L, acc=A_L)
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        # Grip open
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # retract
        ret = _apply_offset(self.dr, st["target"], dy=IN_FINAL_RETRACT_DY)
        self.dr.movel(ret, vel=V_L_SLOW, acc=A_L_SLOW)

        self._home()
        return True, "Inbound Done"

    # ==========================================================
    # OUTBOUND : RACK -> WORKBENCH
    # ==========================================================
    def _do_outbound(self, src: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid:
            return False, f"Invalid rack key: src={src}"

        self.get_logger().info(f"[OUT] {src} -> WB")

        rack = self.build_rack_stations(self.dr, approach_dy=OUT_RACK_APP_DY)
        wb = self.build_wb_top(self.dr, approach_dz=OUT_WB_APP_DZ)
        st = rack[src]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # Rack approach -> target
        self.rel_movel_base(self.dr, 0, -100.0, 0, 0, 0, 0, 50.0)
        self.dr.movel(st["approach"], vel=V_L, acc=A_L)
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        # Grip close
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # Lift up slightly (tool Z)
        self.rel_movel_tool(self.dr, 0, 0, IN_TOOL_LIFT_Z, 0, 0, 0, 20.0)

        # Retract (base Y -250mm) : OUT_RACK_APP_DY와 반대 방향으로 안전 후퇴
        self.rel_movel_base(self.dr, 0, -250.0, 0, 0, 0, 0, 50.0)

        # WB approach(top) -> target(top)
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        # Grip open
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # Post retract: base -Y50, +Z50
        self.rel_movel_base(self.dr, 0, -50.0, 0, 0, 0, 0, 50.0)
        self.rel_movel_base(self.dr, 0, 0, 50.0, 0, 0, 0, 50.0)

        self._home()
        return True, "Outbound Done"


def main(args=None):
    rclpy.init(args=args)
    node = RackTransportAction()

    # ✅ 사용자 규칙: main()에서 노드 생성 후 1회 초기화
    DR_init.__dsr__node = node
    dr = initialize_robot(node)
    node.set_dr(dr)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()