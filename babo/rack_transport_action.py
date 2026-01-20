# rack_transport_action v2.410 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능추가) skip_probe 파라미터 추가: 에뮬레이터/virtual에서 probe로 멈추는 문제 회피(즉시 contact 처리)
# - (기능추가) dry_run 파라미터 추가: 모션 없이 즉시 성공 반환(액션 파이프라인 확인용)
# - (변수수정) 단계별 feedback/status를 더 자주 발행하여 UI의 "분석 중" 정체 구간 추적 가능
# - (유지) 동작마다 set_robot_mode(AUTONOMOUS)+set_tool+set_tcp 재확인(ensure_robot_ready) 유지
# - (유지) /dsr01 namespace 유지, ActionServer는 절대경로(/robot_action)로 고정

"""rack_transport_action
- /robot_action (RobotMove) Action 서버
- main_orchestrator로부터 전달된 IN/OUT/MOVE 명령 수행

v2.410 메모
- UI가 "분석 중"에서 멈추는 경우는 대부분 sub 액션이 Result를 못 반환(블로킹)하는 상황.
- virtual/emulator에서 probe(접촉) 대기가 무한정 걸릴 수 있어 skip_probe 옵션 제공.
"""

import os
import sys

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
DEFAULT_READY_WAIT_SEC = 0.3
DEFAULT_INIT_TIMER_SEC = 0.5


def _normalize_token(s):
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


def _inject_dr_init(node: Node):
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    sys.modules["DR_init"] = DR_init


def _import_dsr_robot2_fresh():
    if "DSR_ROBOT2" in sys.modules:
        del sys.modules["DSR_ROBOT2"]
    import DSR_ROBOT2 as dr
    return dr


class RackTransportAction(Node):
    def __init__(self):
        super().__init__(f"rack_transport_action_{os.getpid()}", namespace=f"/{ROBOT_ID}")

        # ---- 파라미터 (v2.410) ----
        self.declare_parameter("skip_probe", False)  # virtual에서 True 추천
        self.declare_parameter("dry_run", False)     # 파이프라인 확인용
        self.declare_parameter("ready_wait_sec", float(DEFAULT_READY_WAIT_SEC))

        self.dsr = None
        self._helpers_loaded = False
        self.robot_ready = False

        self.rack_stations = {}
        self.wb_station = None
        self.home_j = None

        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"✅ [v2.410] rack_transport_action 노드 생성 완료 (ns={self.get_namespace()}, action=/robot_action)"
        )

        self._init_timer = self.create_timer(DEFAULT_INIT_TIMER_SEC, self._try_robot_init_once)

    def attach_dsr(self, dsr_mod):
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

        # v2.410: skip_probe 옵션이면 probe를 즉시 성공 처리하는 스텁으로 교체
        if bool(self.get_parameter("skip_probe").value):
            def _probe_stub(node, dr, tag):
                node.get_logger().warn(f"[SKIP_PROBE] {tag}: virtual/emulator mode → probe skipped (forced contact=True)")
                return True, 0.0, 0.0
            self._probe_fn = _probe_stub
        else:
            self._probe_fn = self._probe_contact_for_rack

        self._helpers_loaded = True

        self.home_j = self.dsr.posj(0, 0, 90, 0, 90, 0)
        self.rack_stations = build_rack_stations(self.dsr, approach_dy=DEFAULT_APPROACH_DY)
        self.wb_station = build_workbench_station_dy(self.dsr)

        self.get_logger().info(f"[DBG] DR_init.__dsr__id={getattr(DR_init, '__dsr__id', None)}")
        self.get_logger().info(f"[DBG] skip_probe={self.get_parameter('skip_probe').value}, dry_run={self.get_parameter('dry_run').value}")

    def ensure_robot_ready(self, timeout_sec: float):
        if self.dsr is None:
            return False, "DSR_ROBOT2 미연결"
        if not self._helpers_loaded:
            return False, "helper 미로딩(attach_dsr 미완료)"

        client = getattr(self.dsr, "_ros2_set_robot_mode", None)
        if client is None:
            return False, "_ros2_set_robot_mode client is None (import/DR_init 주입 문제)"

        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "set_robot_mode 서비스 준비 필요"

        try:
            self.dsr.set_robot_mode(self.dsr.ROBOT_MODE_AUTONOMOUS)
            self.dsr.set_tool(ROBOT_TOOL)
            self.dsr.set_tcp(ROBOT_TCP)
            self.robot_ready = True
            return True, "robot_mode/tool/tcp OK"
        except Exception as e:
            return False, f"robot_mode/tool/tcp 설정 실패: {e}"

    def _try_robot_init_once(self):
        if self.robot_ready or self.dsr is None:
            return
        ok, msg = self.ensure_robot_ready(timeout_sec=0.0)
        if ok:
            self.get_logger().info("✅ 로봇 초기화 완료(타이머): " + msg)
            try:
                self._init_timer.cancel()
            except Exception:
                pass
        else:
            # 스팸 줄이기: 서비스 미준비만 info
            if "서비스" in msg:
                self.get_logger().info("Set Robot Mode Service is not available, waiting for service to become available...")
            else:
                self.get_logger().warn("⚠️ 초기화 대기/실패: " + msg)

    def _go_home(self):
        self.dsr.movej(self.home_j, vel=DEFAULT_VJ, acc=DEFAULT_AJ)

    async def execute_callback(self, goal_handle):
        raw_cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 UI 명령 수신: {raw_cmd}")

        # v2.410: dry_run이면 즉시 성공
        if bool(self.get_parameter("dry_run").value):
            goal_handle.publish_feedback(RobotMove.Feedback(status="[DRY_RUN] accepted"))
            goal_handle.succeed()
            return RobotMove.Result(success=True, message="[DRY_RUN] success")

        ready_wait = float(self.get_parameter("ready_wait_sec").value)
        goal_handle.publish_feedback(RobotMove.Feedback(status="READY_CHECK"))
        ok, msg = self.ensure_robot_ready(timeout_sec=ready_wait)
        if not ok:
            goal_handle.abort()
            return RobotMove.Result(success=False, message=f"로봇 준비 안됨: {msg}")

        parts = [p.strip() for p in raw_cmd.split(",")]
        cmd_type = parts[0].upper() if parts else ""
        src = _normalize_token(parts[1]) if len(parts) > 1 else None
        dst = _normalize_token(parts[2]) if len(parts) > 2 else None

        try:
            if cmd_type in ("IN", "입고"):
                dest = dst if dst else src
                if not dest:
                    raise ValueError("IN은 목적지 필요")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"IN_START:{dest}"))
                ok, rmsg = self.do_inbound(dest, goal_handle)

            elif cmd_type in ("OUT", "출고"):
                if not src:
                    raise ValueError("OUT은 src 필요")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"OUT_START:{src}"))
                ok, rmsg = self.do_outbound(src, goal_handle)

            elif cmd_type in ("MOVE", "이동"):
                if not src or not dst:
                    raise ValueError("MOVE는 src,dst 필요")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"MOVE_START:{src}->{dst}"))
                ok, rmsg = self.do_move(src, dst, goal_handle)

            else:
                raise ValueError(f"Unknown command: {cmd_type}")

        except Exception as e:
            ok, rmsg = False, f"Error: {e}"
            self.get_logger().error(rmsg)

        if ok:
            goal_handle.publish_feedback(RobotMove.Feedback(status="DONE_OK"))
            goal_handle.succeed()
        else:
            goal_handle.publish_feedback(RobotMove.Feedback(status="DONE_FAIL"))
            goal_handle.abort()

        return RobotMove.Result(success=ok, message=rmsg)

    def do_inbound(self, tr, goal_handle):
        # 동작 시작마다 다시 한번 보장
        ready_wait = float(self.get_parameter("ready_wait_sec").value)
        ok, msg = self.ensure_robot_ready(timeout_sec=ready_wait)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        goal_handle.publish_feedback(RobotMove.Feedback(status="HOME"))
        self._go_home()

        goal_handle.publish_feedback(RobotMove.Feedback(status="GRIP_INIT_OPEN"))
        self._grip_init_open(self.dsr)

        sp_tr = self.rack_stations.get(tr)
        if not sp_tr:
            return False, "대상 없음"

        goal_handle.publish_feedback(RobotMove.Feedback(status="PICK_WB_START"))
        ok, info = self._rack_pick_only(
            node=self,
            dr=self.dsr,
            station=self.wb_station,
            tag="PICK_WB",
            probe_fn=self._probe_fn,  # v2.410: skip_probe면 stub 사용
            grip_open_fn=self._grip_open,
            grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool,
            rel_move_base_fn=self._rel_movel_base,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status="PICK_WB_DONE"))
        if not ok:
            return False, f"WB Pick 실패: {info}"

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PLACE_{tr}_START"))
        ok, info = self._rack_place_only(
            node=self,
            dr=self.dsr,
            station=sp_tr,
            tag=f"PLACE_{tr}",
            grip_open_fn=self._grip_open,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PLACE_{tr}_DONE"))

        self._go_home()
        return ok, ("입고 완료" if ok else f"Place 실패: {info}")

    def do_outbound(self, fr, goal_handle):
        ready_wait = float(self.get_parameter("ready_wait_sec").value)
        ok, msg = self.ensure_robot_ready(timeout_sec=ready_wait)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        sp_fr = self.rack_stations.get(fr)
        if not sp_fr:
            return False, "출발지 없음"

        goal_handle.publish_feedback(RobotMove.Feedback(status="HOME"))
        self._go_home()
        goal_handle.publish_feedback(RobotMove.Feedback(status="GRIP_INIT_OPEN"))
        self._grip_init_open(self.dsr)

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PICK_{fr}_START"))
        ok, info = self._rack_pick_only(
            node=self,
            dr=self.dsr,
            station=sp_fr,
            tag=f"PICK_{fr}",
            probe_fn=self._probe_fn,
            grip_open_fn=self._grip_open,
            grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool,
            rel_move_base_fn=self._rel_movel_base,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PICK_{fr}_DONE"))
        if not ok:
            return False, f"Rack Pick 실패: {info}"

        goal_handle.publish_feedback(RobotMove.Feedback(status="PLACE_WB_START"))
        ok, info = self._workbench_place_only(
            node=self,
            dr=self.dsr,
            wb_station=self.wb_station,
            tag="PLACE_WB",
            grip_open_fn=self._grip_open,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status="PLACE_WB_DONE"))

        self._go_home()
        return ok, ("출고 완료" if ok else f"WB Place 실패: {info}")

    def do_move(self, fr, tr, goal_handle):
        ready_wait = float(self.get_parameter("ready_wait_sec").value)
        ok, msg = self.ensure_robot_ready(timeout_sec=ready_wait)
        if not ok:
            return False, f"로봇 준비 안됨: {msg}"

        sp_fr = self.rack_stations.get(fr)
        sp_tr = self.rack_stations.get(tr)
        if not sp_fr or not sp_tr:
            return False, "좌표 없음"

        goal_handle.publish_feedback(RobotMove.Feedback(status="HOME"))
        self._go_home()
        goal_handle.publish_feedback(RobotMove.Feedback(status="GRIP_INIT_OPEN"))
        self._grip_init_open(self.dsr)

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PICK_{fr}_START"))
        ok, info = self._rack_pick_only(
            node=self,
            dr=self.dsr,
            station=sp_fr,
            tag=f"PICK_{fr}",
            probe_fn=self._probe_fn,
            grip_open_fn=self._grip_open,
            grip_close_fn=self._grip_close,
            rel_move_tool_fn=self._rel_movel_tool,
            rel_move_base_fn=self._rel_movel_base,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PICK_{fr}_DONE"))
        if not ok:
            return False, f"Pick 실패: {info}"

        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PLACE_{tr}_START"))
        ok, info = self._rack_place_only(
            node=self,
            dr=self.dsr,
            station=sp_tr,
            tag=f"PLACE_{tr}",
            grip_open_fn=self._grip_open,
        )
        goal_handle.publish_feedback(RobotMove.Feedback(status=f"PLACE_{tr}_DONE"))

        self._go_home()
        return ok, ("이동 완료" if ok else f"Place 실패: {info}")


def main(args=None):
    rclpy.init(args=args)

    action_node = RackTransportAction()
    _inject_dr_init(action_node)

    dsr_mod = _import_dsr_robot2_fresh()
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
