# rack_transport_action v3.400 2026-01-23
# [이번 버전에서 수정된 사항]
# - (기능추가) TubeTransport job_id가 TUBE_WASTE/WASTE/DISPOSE일 때 폐기 시퀀스(J5/J2 회전 + gripper open + JReady 복귀) 수행
# - (유지) /robot_action Deadlock 회피 구조(dsr_internal_worker 분리) 및 기존 Rack 동작 유지

"""[모듈] rack_transport_action

[역할]
- main_integrated에서 전달된 명령을 Action(RobotMove)로 받아 Doosan 로봇 동작을 실행한다.

[Action]
- name: /robot_action
- goal.command payload 예시:
  - MOVE,A-1,B-2
  - IN,NONE,A-1
  - OUT,A-1,NONE
"""

from __future__ import annotations

import re
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from biobank_interfaces.action import TubeTransport


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
VELOCITY = 60
ACC = 60



# [Dispose(WASTE) 전용 파라미터]
VELOCITY_DISPOSE = 100
ACC_DISPOSE = 50
DISPOSE_J5_ROTATE_DEG = -70.0
DISPOSE_J2_ROTATE_DEG = 15.0
DISPOSE_OPEN_WAIT_SEC = 1.0
# [Pick 관련 파라미터]
DEFAULT_PICK_PRE_TOOL_MM = 18.0   # 잡기 전 Tool Z축 상승
DEFAULT_PICK_POST_BASE_MM = 30.0  # 잡은 후 Base Z축 상승

V_J = 60
A_J = 60

V_L = 200.0
A_L = 200.0

V_L_SLOW = 50.0
A_L_SLOW = 50.0

# [Pick 관련 파라미터]
DEFAULT_PICK_PRE_TOOL_MM = 18.0   # 잡기 전 Tool Z축 상승
DEFAULT_PICK_POST_BASE_MM = 30.0  # 잡은 후 Base Z축 상승

# 홈(조인트)
HOME_J_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

# MOVE(Transport) 관련 상수
MOVE_PICK_APP_DY = -100.0   # 1. 픽 접근 시 Y -100mm
MOVE_RETRACT_DY = -300.0    # 2. 픽 후퇴 시 Y -300mm
MOVE_PLACE_APP_DZ = 60.0 

# IN/OUT 기본값
IN_WB_APP_DY = -50.0
IN_RACK_APP_DY = -100.0
IN_BASE_LIFT_Z = 250.0    

OUT_RACK_APP_DY = -100.0
OUT_WB_APP_DZ = 200.0     
OUT_WB_POST_X_MM = 200.0

GRIP_WAIT_SEC = 1.0

# =========================
# QoS (latched)
# =========================
qos_latched = QoSProfile(
history=HistoryPolicy.KEEP_LAST,
depth=1,
reliability=ReliabilityPolicy.RELIABLE,
durability=DurabilityPolicy.VOLATILE,
)



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

def _import_dsr():
    """
    DR_init.__dsr__node 주입 이후에만 호출되어야 합니다.
    - movel/posx는 dr.movel/dr.posx로만 사용
    """
    import DSR_ROBOT2 as dr
    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_ref_coord,
    )
    DR_BASE = getattr(dr, "DR_BASE", None)
    return {
        "dr": dr,
        "set_tool": set_tool,
        "set_tcp": set_tcp,
        "set_robot_mode": set_robot_mode,
        "ROBOT_MODE_AUTONOMOUS": ROBOT_MODE_AUTONOMOUS,
        "set_ref_coord": set_ref_coord,
        "DR_BASE": DR_BASE,
    }


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
            durability=DurabilityPolicy.VOLATILE,
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

        self.get_logger().info("✅ [v3.300] /robot_action ready (Deadlock Fixed)")

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

        # ==========================================================
        # [핵심 수정 1] 매 요청마다 로봇 모드를 강제로 리셋 (2번째 동작 멈춤 해결)
        # ==========================================================
        try:
            # 혹시 모를 잔여 상태 클리어 및 AUTONOMOUS 모드 확정
            if self.dr:
                self.dr.set_robot_mode(self.dr.ROBOT_MODE_AUTONOMOUS)
        except Exception as e:
            self.get_logger().warn(f"Mode set warning: {e}")
        # ==========================================================

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
            self.get_logger().info(f"📥 sub_end: {cmd}")
        else:
            goal_handle.abort()

        return RobotMove.Result(success=ok, message=msg)

    # ==========================================================
    # MOVE (Transport)
    # ==========================================================
    def _do_transport(self, src: str, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid or dest not in valid:
            return False, f"Invalid keys: {src}->{dest}"
        if src == dest:
            return False, "src == dest"

        self.get_logger().info(f"[MOVE] {src} -> {dest}")

        # Pick 시 Y -100mm 접근
        rack_pick = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        # Place 시 일단 접근 좌표 계산용
        rack_place = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        
        st_src = rack_pick[src]
        st_dst = rack_place[dest]

        # 1) Home & Init
        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # ------------------------------------------------------
        # [PICK SEQUENCE]
        # ------------------------------------------------------
        # 1-1. Y -100mm 접근
        self.dr.movel(st_src["approach"], vel=V_L, acc=A_L)
        
        # 1-2. Target 진입
        self.dr.movel(st_src["target"], vel=V_L, acc=A_L)

        # 1-3. Pre-Lift (Tool +Z)
        if DEFAULT_PICK_PRE_TOOL_MM > 0:
            self.rel_movel_tool(self.dr, 0, 0, DEFAULT_PICK_PRE_TOOL_MM, 0, 0, 0, V_L_SLOW)

        # 1-4. Grip
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        # 1-5. Post-Lift (Base +Z)
        if DEFAULT_PICK_POST_BASE_MM > 0:
            self.rel_movel_base(self.dr, 0, 0, DEFAULT_PICK_POST_BASE_MM, 0, 0, 0, V_L_SLOW)

        # 1-6. Retract (-300mm)
        self.rel_movel_base(self.dr, 0, MOVE_RETRACT_DY, 0, 0, 0, 0, V_L)
        # ------------------------------------------------------

        # ======================================================
        # [LATERAL MOVE] Home 안 들르고 X축 수평 이동
        # ======================================================
        current_pos = self.dr.get_current_posx()[0] 
        target_x = st_dst["target"][0] # Destination Rack의 기본 X 좌표
        
        # 안전을 위해 현재 높이(Z)와 깊이(Y)를 유지하고 X만 변경하여 movel
        lateral_pos = self.dr.posx(
            target_x,           
            current_pos[1],     # Current Y (Retracted)
            current_pos[2],     # Current Z (Retracted/Lifted)
            current_pos[3],     
            current_pos[4],     
            current_pos[5]      
        )
        
        self.get_logger().info(f"[MOVE] Sliding X to {dest}...")
        self.dr.movel(lateral_pos, vel=V_L, acc=A_L)
        # ======================================================

        # ------------------------------------------------------
        # [PLACE SEQUENCE] (With ID-specific Y offset)
        # ------------------------------------------------------
        
        # [요청사항] 목적지 ID별 추가 Y 깊이 설정
        y_offset = 0.0
        if dest == "A-2":
            y_offset = 23.0
        elif dest in ["A-3", "B-1"]:
            y_offset = 20.0
        elif dest == "B-2":
            y_offset = 10.0
        
        # 오프셋이 적용된 최종 타겟 좌표 계산
        final_target = _apply_offset(self.dr, st_dst["target"], dy=y_offset)
        
        self.get_logger().info(f"[MOVE] Place logic: Dest={dest}, Added Y={y_offset}mm")

        # 2-1. Approach (최종 타겟 기준 위에서 접근)
        place_app = _apply_offset(self.dr, final_target, dz=MOVE_PLACE_APP_DZ)
        self.dr.movel(place_app, vel=V_L, acc=A_L)
        
        # 2-2. Target Place (깊이 보정된 위치)
        self.dr.movel(final_target, vel=V_L, acc=A_L)

        # 2-3. Open
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # 2-4. Retract
        # 빠져나올 때는 안전하게 뒤로(Y -300)
        self.rel_movel_base(self.dr, 0, MOVE_RETRACT_DY, 0, 0, 0, 0, V_L)

        self._home()
        return True, "Transport Done"

   # ==========================================================
   # INBOUND : WORKBENCH -> RACK
   # ==========================================================
    def _do_inbound(self, dest: str) -> Tuple[bool, str]:
       valid = self._valid_keys()
       if dest not in valid:
           return False, f"Invalid rack key: {dest}"
      
       # [1] 목적지 스테이션 및 오프셋 설정
       rack_place = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
       st_dst = rack_place[dest]


       y_offset = 0.0
       if dest == "A-2":
           y_offset = 16.0
       elif dest in ["A-3"]:
           y_offset = 13.0
       elif dest in ["B-1"]:
           y_offset = 9.0
       elif dest == "B-2":
           y_offset = 10.0
      
       self.get_logger().info(f"[IN] WB -> {dest} (Offset: {y_offset}mm)")


       wb = self.build_wb_dy(self.dr, approach_dy=IN_WB_APP_DY)
       rack = self.build_rack_stations(self.dr, approach_dy=IN_RACK_APP_DY)
       st = rack[dest]


       self._home()
       self.grip_init_open(self.dr, wait_sec=0.2)


       # ---------------------------------------------------------
       # WB Pick Sequence (기존 유지)
       # ---------------------------------------------------------
       self.rel_movel_base(self.dr, 0, -180.0, 0, 0, 0, 0, 50.0)
      
       cur_pos, _ = self.dr.get_current_posx(self.dr.DR_BASE)
       x, y, z, rx, ry, rz = [float(v) for v in cur_pos]
       back_mm = 150
       target = self.dr.posx(x, y - back_mm, z, rx, ry, rz)
       self.dr.movel(target, vel=V_L, acc=A_L)
      
       self.dr.movel(wb["approach"], vel=V_L, acc=A_L)


       self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)
       self.rel_movel_tool(self.dr, 0, 0, 8.0, 0, 0, 0, 20.0)
       self.grip_init_open(self.dr, wait_sec=0.2)


       self.dr.movel(wb["target"], vel=V_L, acc=A_L)
       self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)


       # WB에서 들어올림 (High Z)
       self.rel_movel_base(self.dr, 0, 0, IN_BASE_LIFT_Z, 0, 0, 0, 50.0)


       # ---------------------------------------------------------
       # Rack Approach Sequence (X -> Z -> Y)
       # ---------------------------------------------------------
       # 1. 최종 타겟 및 상단 접근 위치 계산 (오프셋 반영)
       final_target = _apply_offset(self.dr, st_dst["target"], dy=y_offset)
       place_app = _apply_offset(self.dr, final_target, dz=MOVE_PLACE_APP_DZ)
      
       self.get_logger().info(f"[MOVE] Rack Approach: X(Align) -> Z(Lower) -> Y(Enter)")


       # 2. 현재 위치 (WB 상단 High Z)
       cur_pos_lifted = self.dr.get_current_posx()[0]


       # 3. [Step 1] X축 정렬 (Align X)
       # - 목표: 타겟 X
       # - 유지: 현재 Y, 현재 Z (높은 상태)
       waypoint_x = self.dr.posx(
           place_app[0],       # Target X
           cur_pos_lifted[1],  # Current Y
           cur_pos_lifted[2],  # Current Z (High)
           cur_pos_lifted[3], cur_pos_lifted[4], cur_pos_lifted[5]
       )
       self.get_logger().info("[INBOUND] Step 1: X Align")
       self.dr.movel(waypoint_x, vel=V_L, acc=A_L)


       # 4. [Step 2] Z축 하강 (Lower Z)
       # - 목표: 타겟 Z (place_app 높이)
       # - 유지: 타겟 X, 현재 Y (뒤쪽)
       # ※ Y로 진입하기 전에 미리 내려갑니다.
       waypoint_z = self.dr.posx(
           place_app[0],       # Target X
           cur_pos_lifted[1],  # Current Y (Still Back)
           place_app[2],       # Target Z (Low)
           cur_pos_lifted[3], cur_pos_lifted[4], cur_pos_lifted[5]
       )
       self.get_logger().info("[INBOUND] Step 2: Z Lower (Pre-Leveling)")
       self.dr.movel(waypoint_z, vel=V_L, acc=A_L)


       # 5. [Step 3] Y축 진입 (Enter Y)
       # - 목표: 랙 앞 (Approach Y)
       # - 낮은 자세로 앞으로 전진
       self.get_logger().info("[INBOUND] Step 3: Y Enter (Approach)")
       self.dr.movel(place_app, vel=V_L, acc=A_L)


       # 6. [Step 4] 최종 안착 (Inbound)
       self.get_logger().info("[INBOUND] Step 4: Final Inbound")
       self.dr.movel(final_target, vel=V_L, acc=A_L)


       # ---------------------------------------------------------
       # Finish
       # ---------------------------------------------------------
       self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)


       # Retract (뒤로 빠지기)
       ret = _apply_offset(self.dr, st["target"], dy=-150.0)
       self.dr.movel(ret, vel=V_L_SLOW, acc=A_L_SLOW)


       self._home()
       return True, "Inbound Done"

    # ==========================================================
    # OUTBOUND : RACK -> WORKBENCH
    # ==========================================================
    def _do_outbound(self, src: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid:
            return False, f"Invalid rack key: {src}"

        self.get_logger().info(f"[OUT] {src} -> WB")

        rack = self.build_rack_stations(self.dr, approach_dy=OUT_RACK_APP_DY)
        wb = self.build_wb_top(self.dr, approach_dz=OUT_WB_APP_DZ)
        st = rack[src]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        # ------------------------------------------------------
        # [PICK SEQUENCE] from Rack
        # ------------------------------------------------------
        self.rel_movel_base(self.dr, 0, -100.0, 0, 0, 0, 0, 50.0)
        
        self.dr.movel(st["approach"], vel=V_L, acc=A_L)
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        if DEFAULT_PICK_PRE_TOOL_MM > 0:
            self.rel_movel_tool(self.dr, 0, 0, DEFAULT_PICK_PRE_TOOL_MM, 0, 0, 0, 20.0)

        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        if DEFAULT_PICK_POST_BASE_MM > 0:
            self.rel_movel_base(self.dr, 0, 0, DEFAULT_PICK_POST_BASE_MM, 0, 0, 0, 20.0)
        
        self.rel_movel_base(self.dr, 0, -250.0, 0, 0, 0, 0, 50.0)
        # ------------------------------------------------------

        # WB approach -> target
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        # Grip open
        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        # Post Retract Sequence: Y -> X -> Z
        self.rel_movel_base(self.dr, 0, -50.0, 0, 0, 0, 0, 50.0)

        if OUT_WB_POST_X_MM > 0:
            self.rel_movel_base(self.dr, OUT_WB_POST_X_MM, 0, 0, 0, 0, 0, 50.0)

        self.rel_movel_base(self.dr, 0, 0, 50.0, 0, 0, 0, 50.0)

        self._home()
        return True, "Outbound Done"
    
def _set_ref_base(dsr, node: Node):
    """
    기준 좌표계 BASE로 고정(상대이동/안전성 위해).
    """
    try:
        if dsr["DR_BASE"] is not None:
            dsr["set_ref_coord"](dsr["DR_BASE"])
            node.get_logger().info("set_ref_coord: DR_BASE")
        else:
            node.get_logger().info("set_ref_coord: DR_BASE not found (skip)")
    except Exception as e:
        node.get_logger().warn(f"set_ref_coord failed: {repr(e)}")


def _posx_from_list(dr, arr6):
    return dr.posx(
        float(arr6[0]), float(arr6[1]), float(arr6[2]),
        float(arr6[3]), float(arr6[4]), float(arr6[5]),
    )


class TubeTransportNode(Node):


    def __init__(self):
        super().__init__("tube_transport_node", namespace=ROBOT_ID)

        self.pub_done = self.create_publisher(Bool, "tube_transport_done", qos_latched)

        self._as = ActionServer(
            self,
            TubeTransport,
            "/tube_transport",
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._on_execute,
        )

        self.get_logger().info("TubeTransportNode ready (ActionServer: /tube_transport)")

    # -------------------------
    # Robot init
    # -------------------------
    def initialize_robot(self):
        dsr = _import_dsr()

        self.get_logger().info("[INIT] set_tool")
        dsr["set_tool"](ROBOT_TOOL)

        self.get_logger().info("[INIT] set_tcp")
        dsr["set_tcp"](ROBOT_TCP)

        self.get_logger().info("[INIT] set_robot_mode")
        dsr["set_robot_mode"](dsr["ROBOT_MODE_AUTONOMOUS"])

        _set_ref_base(dsr, self)

        self.get_logger().info("#" * 50)
        self.get_logger().info("Robot initialized")
        self.get_logger().info(f"ROBOT_ID={ROBOT_ID}, MODEL={ROBOT_MODEL}, TCP={ROBOT_TCP}, TOOL={ROBOT_TOOL}")
        self.get_logger().info(f"VELOCITY={VELOCITY}, ACC={ACC}")
        self.get_logger().info("#" * 50)

    # -------------------------
    # Action callbacks
    # -------------------------
    def _on_goal(self, goal_request: TubeTransport.Goal):
        if not hasattr(goal_request, "job_id"):
            return GoalResponse.REJECT
        if len(goal_request.pick_posx) != 6 or len(goal_request.place_posx) != 6:
            self.get_logger().error("Rejected goal: pick_posx/place_posx must be length 6")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        return CancelResponse.ACCEPT

    def _fb(self, goal_handle, stage: str, progress: float, detail: str):
        fb = TubeTransport.Feedback()
        fb.stage = stage
        fb.progress = float(progress)
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _cancel_check(self, goal_handle, where: str) -> bool:
        if goal_handle.is_cancel_requested:
            self.get_logger().warn(f"[CANCEL] requested at {where}")
            return True
        return False

    def _ret_ok(self, ret, where: str) -> bool:
        """
        실패(-1 등)면 즉시 중단.
        """
        if ret is None:
            return True
        try:
            r = float(ret)
            if r < 0:
                self.get_logger().error(f"[MOTION] {where} rejected ret={ret}")
                return False
            return True
        except Exception:
            self.get_logger().error(f"[MOTION] {where} ret(not-numeric)={ret}")
            return False

    def _movel_abs(self, dr, target_posx6, where: str):
        pos = _posx_from_list(dr, target_posx6)
        kwargs = {"vel": float(VELOCITY), "acc": float(ACC)}

        # 가능하면 ref/mod 명시(있을 때만)
        if hasattr(dr, "DR_BASE"):
            kwargs["ref"] = dr.DR_BASE
        if hasattr(dr, "DR_MV_MOD_ABS"):
            kwargs["mod"] = dr.DR_MV_MOD_ABS

        self.get_logger().info(f"[MOTION] {where} movel -> {target_posx6} kwargs={kwargs}")
        ret = dr.movel(pos, **kwargs)
        self.get_logger().info(f"[MOTION] {where} movel done ret={ret}")
        return ret

    # -------------------------
    # Execute (단순 시퀀스)
    # -------------------------
    def _on_execute(self, goal_handle):
        goal = goal_handle.request

        result = TubeTransport.Result()
        result.success = False
        result.error_code = ""
        result.message = ""

        job_id = goal.job_id
        job_u = str(job_id).upper().strip()
        is_waste = any(k in job_u for k in ("WASTE", "DISPOSE"))
        pick_posx_6 = list(goal.pick_posx)
        place_posx_6 = list(goal.place_posx)

        dsr = _import_dsr()
        dr = dsr["dr"]

        from .gripper_io import grip_open, grip_close
        from .rel_move import rel_movel_base

        # ---- 파라미터(요청대로) ----
        PICK_DOWN_MM = 30.0
        PICK_UP_MM = 140.0

        PLACE_DOWN_MM = 80.0
        PLACE_OPEN_WAIT = 1.2
        PLACE_UP_MM = 90.0

        try:
            self.get_logger().info(f"[EXEC] start job_id={job_id}")
            self._fb(goal_handle, "INIT", 0.01, f"Job start job_id={job_id}")

            # === 1) PICK 위치로 이동 ===
            self._fb(goal_handle, "PICK_MOVE", 0.10, "Move to pick_posx")
            if self._cancel_check(goal_handle, "PICK_MOVE"):
                result.error_code = "CANCELED"
                result.message = "Canceled before pick move"
                goal_handle.abort()
                return result

            ret = self._movel_abs(dr, pick_posx_6, "PICK->pick_posx")
            if not self._ret_ok(ret, "PICK movel->pick_posx"):
                result.error_code = "PICK_MOVE_REJECTED"
                result.message = f"Pick movel rejected ret={ret}"
                self._fb(goal_handle, "FAIL", 1.0, result.message)
                self.pub_done.publish(Bool(data=False))
                goal_handle.abort()
                return result

            _set_ref_base(dsr, self)

            # === 2) PICK: OPEN -> down 20 -> CLOSE -> up 130 ===
            self._fb(goal_handle, "PICK_SEQ", 0.30, "OPEN -> down 20 -> CLOSE -> up 130")
            if self._cancel_check(goal_handle, "PICK_SEQ"):
                result.error_code = "CANCELED"
                result.message = "Canceled during pick seq"
                goal_handle.abort()
                return result

            self.get_logger().info("[GRIP] grip_open()")
            grip_open(dr)

            self.get_logger().info(f"[PICK] down {PICK_DOWN_MM}mm")
            rel_movel_base(dr, 0, 0, -PICK_DOWN_MM, 0, 0, 0, vel=VELOCITY)

            self.get_logger().info("[GRIP] grip_close()")
            grip_close(dr)

            self.get_logger().info(f"[PICK] up {PICK_UP_MM}mm")
            rel_movel_base(dr, 0, 0, +PICK_UP_MM, 0, 0, 0, vel=VELOCITY)

            # === 3) PLACE 위치로 이동 ===
            self._fb(goal_handle, "PLACE_MOVE", 0.70, "Move to place_posx")
            if self._cancel_check(goal_handle, "PLACE_MOVE"):
                result.error_code = "CANCELED"
                result.message = "Canceled before place move"
                goal_handle.abort()
                return result

            ret = self._movel_abs(dr, place_posx_6, "PLACE->place_posx")
            if not self._ret_ok(ret, "PLACE movel->place_posx"):
                result.error_code = "PLACE_MOVE_REJECTED"
                result.message = f"Place movel rejected ret={ret}"
                self._fb(goal_handle, "FAIL", 1.0, result.message)
                self.pub_done.publish(Bool(data=False))
                goal_handle.abort()
                return result

            _set_ref_base(dsr, self)
            # === 4) PLACE / DISPOSE ===
            if is_waste:
                # WASTE(폐기): 회전 + OPEN + JReady 복귀
                self._fb(goal_handle, "DISPOSE_SEQ", 0.85, "rotate(J5/J2) -> OPEN -> JReady")
                if self._cancel_check(goal_handle, "DISPOSE_SEQ"):
                    result.error_code = "CANCELED"
                    result.message = "Canceled during dispose seq"
                    goal_handle.abort()
                    return result

                # J5 rotate
                try:
                    cur_j = [float(v) for v in dr.get_current_posj()]
                    tgt = cur_j[:]
                    tgt[4] += float(DISPOSE_J5_ROTATE_DEG)
                    self.get_logger().info(f"[DISPOSE] movej J5 += {DISPOSE_J5_ROTATE_DEG}deg")
                    retj = dr.movej(tgt, vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                    if not self._ret_ok(retj, "DISPOSE movej(J5)"):
                        raise RuntimeError(f"DISPOSE movej(J5) rejected ret={retj}")
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] J5 rotate skipped/failed: {repr(e)}")

                # J2 rotate
                try:
                    cur_j = [float(v) for v in dr.get_current_posj()]
                    tgt = cur_j[:]
                    tgt[1] += float(DISPOSE_J2_ROTATE_DEG)
                    self.get_logger().info(f"[DISPOSE] movej J2 += {DISPOSE_J2_ROTATE_DEG}deg")
                    retj = dr.movej(tgt, vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                    if not self._ret_ok(retj, "DISPOSE movej(J2)"):
                        raise RuntimeError(f"DISPOSE movej(J2) rejected ret={retj}")
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] J2 rotate skipped/failed: {repr(e)}")

                # gripper open (drop)
                self.get_logger().info(f"[DISPOSE] grip_open(wait={DISPOSE_OPEN_WAIT_SEC})")
                grip_open(dr, wait_sec=float(DISPOSE_OPEN_WAIT_SEC))
                try:
                    dr.wait(0.2)
                except Exception:
                    pass

                # (선택) close로 초기화
                try:
                    self.get_logger().info("[DISPOSE] grip_close()")
                    grip_close(dr)
                    dr.wait(0.2)
                except Exception:
                    pass

                # JReady 복귀
                try:
                    self.get_logger().info("[DISPOSE] return HOME_J_DEG")
                    dr.movej(list(HOME_J_DEG), vel=float(VELOCITY_DISPOSE), acc=float(ACC_DISPOSE))
                except Exception as e:
                    self.get_logger().warn(f"[DISPOSE] movej HOME_J_DEG failed: {repr(e)}")

            else:
                # 일반 PLACE: down -> OPEN(wait) -> up
                self._fb(goal_handle, "PLACE_SEQ", 0.85, "down -> OPEN(wait) -> up")
                if self._cancel_check(goal_handle, "PLACE_SEQ"):
                    result.error_code = "CANCELED"
                    result.message = "Canceled during place seq"
                    goal_handle.abort()
                    return result

                self.get_logger().info(f"[PLACE] down {PLACE_DOWN_MM}mm")
                rel_movel_base(dr, 0, 0, -PLACE_DOWN_MM, 0, 0, 0, vel=VELOCITY)

                self.get_logger().info(f"[GRIP] grip_open(wait={PLACE_OPEN_WAIT})")
                grip_open(dr, wait_sec=PLACE_OPEN_WAIT)

                self.get_logger().info(f"[PLACE] up {PLACE_UP_MM}mm")
                rel_movel_base(dr, 0, 0, +PLACE_UP_MM, 0, 0, 0, vel=VELOCITY)

            # DONE
            result.success = True
            result.error_code = "OK"
            result.message = "Dispose sequence done" if is_waste else "Simple pick&place done"

            self.get_logger().info(f"[EXEC] done job_id={job_id}")
            self._fb(goal_handle, "DONE", 1.0, result.message)

            self.pub_done.publish(Bool(data=True))
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"[EXEC] Exception: {repr(e)}")
            self.pub_done.publish(Bool(data=False))

            result.success = False
            result.error_code = "EXCEPTION"
            result.message = repr(e)
            self._fb(goal_handle, "FAIL", 1.0, result.message)

            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    
    # 1. Action 처리를 담당할 본체 노드
    action_node = RackTransportAction()

    # Tube 처리 담당 노드
    tube_node = TubeTransportNode()

    # =========================================================
    # [핵심 수정 2] 로봇 통신(DSR)만을 위한 '전용 노드' 생성
    # =========================================================
    # 기존 node를 공유하지 않고, 로봇용 통신 채널을 물리적으로 분리하여 Deadlock 방지
    dsr_node = rclpy.create_node("dsr_internal_worker", namespace=ROBOT_ID)
    
    # DSR 라이브러리에게 "너는 이제부터 이 전용 노드를 써"라고 지정
    DR_init.__dsr__node = dsr_node 
    
    # 2. 로봇 초기화 (로그는 action_node에 찍히지만, 실제 통신은 dsr_node로 나감)
    dr = initialize_robot(action_node)
    action_node.set_dr(dr)

    # 3. 스레드 8개 유지 (Action용 + DSR 통신용 넉넉하게)
    executor = MultiThreadedExecutor(num_threads=8)
    
    # 4. 세 노드를 모두 Executor에 등록 (이제 서로 방해하지 않음)
    executor.add_node(action_node)
    executor.add_node(dsr_node)
    executor.add_node(tube_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # 종료 시 세 노드 모두 정리
            action_node.destroy_node()
            dsr_node.destroy_node()
            tube_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()