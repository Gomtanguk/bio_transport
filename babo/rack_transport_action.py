# rack_transport_action v2.400 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능추가) 동작(MOVE/IN/OUT) 시작마다 set_robot_mode(AUTONOMOUS) + set_tool + set_tcp를 재확인/재설정하여 초기화 타이밍 레이스를 더 강하게 방지
# - (기능추가) ensure_robot_ready() 추가: 서비스 준비 확인 후 초기화 수행, 실패 시 명확한 메시지 반환
# - (유지) /dsr01 namespace로 서비스 경로 정합 유지, ActionServer는 절대경로(/robot_action)로 고정
# - (유지) DSR_ROBOT2 import 순서(노드 생성→DR_init 주입→import) 유지

"""rack_transport_action

[역할]
- /robot_action (RobotMove) Action 서버
- main_orchestrator로부터 전달된 IN/OUT/MOVE 명령 수행

[핵심 포인트]
- DSR_ROBOT2는 import 시점에 DR_init.__dsr__node를 참조해 create_client를 만들 수 있으므로,
  Node 생성 후 DR_init 주입 → DSR_ROBOT2 import 순서를 반드시 지킨다.
- 서비스 네임스페이스 불일치 방지:
  - 노드를 /dsr01 namespace로 생성하면, DSR_ROBOT2가 상대 서비스명("system/...")을 써도 /dsr01/system/...로 해석된다.
- ActionServer는 절대경로 "/robot_action"으로 고정해 UI/메인 연동을 유지한다.
- v2.400: 각 동작 시작마다 robot_mode/tool/tcp를 재확인하여 레이스/일시적 미준비 상황을 더 강하게 흡수한다.
"""

import os
import sys
import importlib

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import DR_init
from biobank_interfaces.action import RobotMove

# =========================
# ROBOT 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# =========================
# DEFAULT_ 상수
# =========================
DEFAULT_APPROACH_DY = -250.0
DEFAULT_VJ = 60
DEFAULT_AJ = 60
DEFAULT_INIT_TIMER_SEC = 0.5

# 동작 시작 시 초기화 재확인에서 서비스 대기 시간(초)
DEFAULT_READY_WAIT_SEC = 0.3


def _normalize_token(s: str | None):
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


def _inject_dr_init(node: Node):
    """DSR_ROBOT2가 import 시점에 참조하는 DR_init에 node/id/model을 주입."""
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    sys.modules["DR_init"] = DR_init


def _import_dsr_robot2_fresh():
    """DSR_ROBOT2를 깨끗하게 재import."""
    if "DSR_ROBOT2" in sys.modules:
        del sys.modules["DSR_ROBOT2"]
    import DSR_ROBOT2 as dr
    return dr


class RackTransportAction(Node):
    def __init__(self):
        # ✅ /dsr01 네임스페이스로 노드 생성
        # ✅ 노드 이름 중복 경고 줄이기 위해 PID 포함
        super().__init__(f"rack_transport_action_{os.getpid()}", namespace=f"/{ROBOT_ID}")

        self.dsr = None
        self.robot_ready = False
        self._init_in_progress = False

        self._helpers_loaded = False
        self.rack_stations = {}
        self.wb_station = None
        self.home_j = None

        # ActionServer 절대경로 고정
        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "✅ [v2.400] rack_transport_action 노드 생성 완료 "
            f"({self.get_namespace()} namespace, /robot_action absolute)"
        )

        # 초기화 타이머(서비스 준비되면 1회 성공 후 중지)
        self._init_timer = self.create_timer(DEFAULT_INIT_TIMER_SEC, self._try_robot_init_once)

    def attach_dsr(self, dsr_mod):
        """DSR_ROBOT2 모듈을 연결하고 helper/스테이션을 준비."""
        self.dsr = dsr_mod

        from babo.gripper_io import grip_open, grip_close, grip_init_open
        from babo.probe_io import probe_contact_for_rack
        from babo.rel_move import rel_movel_tool, rel_movel_base
        from babo.rack_stations import build_rack_stations, build_workbench_station_dy
        from babo.rack_pick_io import rack_pick_only
        from babo.rack_place_io import rack_place_only
        from babo.workbench_place_io import workbench_place_only

        self._grip_open = grip_open
        self._grip_close = grip_close
        self._grip_init_open = grip_init_open
        self._probe_contact_for_rack = probe_contact_for_rack
        self._rel_movel_tool = rel_movel_tool
        self._rel_movel_base = rel_movel_base
        self._rack_pick_only = rack_pick_only
        self._rack_place_only = rack_place_only
        self._workbench_place_only = workbench_place_only

        self._helpers_loaded = True

        # 스테이션/홈
        self.home_j = self.dsr.posj(0, 0, 90, 0, 90, 0)
        self.rack_stations = build_rack_stations(self.dsr, approach_dy=DEFAULT_APPROACH_DY)
        self.wb_station = build_workbench_station_dy(self.dsr)

        self.get_logger().info(f"[DBG] DR_init.__dsr__id={getattr(DR_init, '__dsr__id', None)}")
        self.get_logger().info(f"[DBG] node namespace={self.get_namespace()}")

    # -------------------------
    # 초기화/준비 보장 로직
    # -------------------------
    def ensure_robot_ready(self, timeout_sec: float = DEFAULT_READY_WAIT_SEC):
        """동작 시작 전 robot_mode/tool/tcp가 준비되었는지 보장.

        반환: (ok: bool, msg: str)
        """
        if self.dsr is None:
            return False, "DSR_ROBOT2 미연결(dsr is None)"
        if not self._helpers_loaded:
            return False, "helper 미로딩(attach_dsr 미완료)"

        # DSR_ROBOT2 내부 서비스 클라이언트 확인
        client = getattr(self.dsr, "_ros2_set_robot_mode", None)
        if client is None:
            # 이 경우는 set_robot_mode를 perform_task에 넣어도 해결 안 됨(=import/DR_init 주입 문제)
            return False, "_ros2_set_robot_mode client is None (import 순서/DR_init 주입 문제)"

        # 서비스 준비 확인(짧게 기다림)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "set_robot_mode 서비스 준비 필요"

        # 서비스가 있으면 매 동작 시작에 한번 더 세팅(요구사항)
        try:
            self.dsr.set_robot_mode(self.dsr.ROBOT_MODE_AUTONOMOUS)
            self.dsr.set_tool(ROBOT_TOOL)
            self.dsr.set_tcp(ROBOT_TCP)
            self.robot_ready = True
            return True, "robot_mode/tool/tcp OK"
        except Exception as e:
            return False, f"robot_mode/tool/tcp 설정 실패: {e}"

    def _try_robot_init_once(self):
        """타이머 기반 초기화(서비스 준비되면 1회)."""
        if self.robot_ready:
            return True
        if self.dsr is None:
            return False
        if self._init_in_progress:
            return False

        self._init_in_progress = True
        try:
            ok, msg = self.ensure_robot_ready(timeout_sec=0.0)
            if ok:
                self.get_logger().info("✅ 로봇 초기화 완료 (타이머): " + msg)
                try:
                    self._init_timer.cancel()
                except Exception:
                    pass
                return True
            else:
                # 스팸 줄이기 위해 info 한 줄만
                if "서비스" in msg:
                    self.get_logger().info("Set Robot Mode Service is not available, waiting for service to become available...")
                else:
                    self.get_logger().warn("⚠️ 초기화 대기/실패: " + msg)
                return False
        finally:
            self._init_in_progress = False

    # -------------------------
    # 동작
    # -------------------------
    def _go_home(self):
        if self.home_j is None:
            return
        self.dsr.movej(self.home_j, vel=DEFAULT_VJ, acc=DEFAULT_AJ)

    async def execute_callback(self, goal_handle):
        raw_cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 UI 명령 수신: {raw_cmd}")

        # v2.400: 명령 수신 시에도 즉시 준비 체크
        ok, msg = self.ensure_robot_ready(timeout_sec=DEFAULT_READY_WAIT_SEC)
        if not ok:
            goal_handle.abort()
            return RobotMove.Result(success=False, message=f"로봇 준비 안됨: {msg}")

        parts = [p.strip() for p in raw_cmd.split(",")]
        cmd_type = parts[0].upper() if parts else ""

        src = _normalize_token(parts[1]) if len(parts) > 1 else None
        dst = _normalize_token(parts[2]) if len(parts) > 2 else None

        success, rmsg = False, ""
        try:
            if cmd_type in ("MOVE", "이동"):
                if not src or not dst:
                    raise ValueError("MOVE는 src,dst 필요")
                success, rmsg = self.do_move(src, dst, goal_handle)

            elif cmd_type in ("IN", "입고"):
                dest = dst if dst else src
                if not dest:
                    raise ValueError("IN은 목적지 필요")
                success, rmsg = self.do_inbound(dest, goal_handle)

            elif cmd_type in ("OUT", "출고"):
                if not src:
                    raise ValueError("OUT은 src 필요")
                success, rmsg = self.do_outbound(src, goal_handle)

            else:
                raise ValueError(f"Unknown command: {cmd_type}")

        except Exception as e:
            success, rmsg = False, f"Error: {e}"
            self.get_logger().error(rmsg)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return RobotMove.Result(success=success, message=rmsg)

    def do_move(self, fr, tr, goal_handle):
        # 동작 시작마다 set_robot_mode/tool/tcp 재확인(요구사항)
        ok, msg = self.ensure_robot_ready(timeout_sec=DEFAULT_READY_WAIT_SEC)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"{fr}->{tr} 이동"))
        sp_fr, sp_tr = self.rack_stations.get(fr), self.rack_stations.get(tr)
        if not sp_fr or not sp_tr:
            return False, "좌표 없음"

        self._go_home()
        self._grip_init_open(self.dsr)

        ok, info = self._rack_pick_only(
            node=self, dr=self.dsr, station=sp_fr, tag=f"PICK_{fr}",
            probe_fn=self._probe_contact_for_rack,
            grip_open_fn=self._grip_open, grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool, rel_move_base_fn=self._rel_movel_base
        )
        if not ok:
            self._go_home()
            return False, f"Pick 실패: {info}"

        ok, info = self._rack_place_only(
            node=self, dr=self.dsr, station=sp_tr, tag=f"PLACE_{tr}",
            grip_open_fn=self._grip_open
        )
        self._go_home()
        return ok, ("성공" if ok else f"Place 실패: {info}")

    def do_inbound(self, tr, goal_handle):
        ok, msg = self.ensure_robot_ready(timeout_sec=DEFAULT_READY_WAIT_SEC)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"{tr} 입고"))
        sp_tr = self.rack_stations.get(tr)
        if not sp_tr:
            return False, "대상 없음"

        self._go_home()
        self._grip_init_open(self.dsr)

        ok, info = self._rack_pick_only(
            node=self, dr=self.dsr, station=self.wb_station, tag="PICK_WB",
            probe_fn=self._probe_contact_for_rack,
            grip_open_fn=self._grip_open, grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool, rel_move_base_fn=self._rel_movel_base
        )
        if not ok:
            self._go_home()
            return False, f"WB Pick 실패: {info}"

        ok, info = self._rack_place_only(
            node=self, dr=self.dsr, station=sp_tr, tag=f"PLACE_{tr}",
            grip_open_fn=self._grip_open
        )
        self._go_home()
        return ok, ("입고 완료" if ok else f"Place 실패: {info}")

    def do_outbound(self, fr, goal_handle):
        ok, msg = self.ensure_robot_ready(timeout_sec=DEFAULT_READY_WAIT_SEC)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"{fr} 출고"))
        sp_fr = self.rack_stations.get(fr)
        if not sp_fr:
            return False, "출발지 없음"

        self._go_home()
        self._grip_init_open(self.dsr)

        ok, info = self._rack_pick_only(
            node=self, dr=self.dsr, station=sp_fr, tag=f"PICK_{fr}",
            probe_fn=self._probe_contact_for_rack,
            grip_open_fn=self._grip_open, grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool, rel_move_base_fn=self._rel_movel_base
        )
        if not ok:
            self._go_home()
            return False, f"Rack Pick 실패: {info}"

        ok, info = self._workbench_place_only(
            node=self, dr=self.dsr, wb_station=self.wb_station, tag="PLACE_WB",
            grip_open_fn=self._grip_open
        )
        self._go_home()
        return ok, ("출고 완료" if ok else f"WB Place 실패: {info}")


def main(args=None):
    rclpy.init(args=args)

    # 1) Action 노드 생성(/dsr01 namespace)
    action_node = RackTransportAction()

    # 2) DR_init 주입(DSR import 전)
    _inject_dr_init(action_node)

    # 3) DSR_ROBOT2 import (이 시점엔 g_node가 action_node)
    dsr_mod = _import_dsr_robot2_fresh()

    # 4) helper/스테이션 attach
    action_node.attach_dsr(dsr_mod)

    executor = MultiThreadedExecutor()
    executor.add_node(action_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
