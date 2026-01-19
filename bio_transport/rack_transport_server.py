# rack_transport_server v2.100 2026-01-19
# [이번 버전에서 수정된 사항]
# - DSR_ROBOT2 import 순서 보장: (DR_init id/model 설정) -> (node 생성) -> (DR_init.__dsr__node 설정) -> (DSR_ROBOT2 import)
# - 토픽 네임스페이스 고정: /<ROBOT_ID>/rack_transport_cmd, /<ROBOT_ID>/rack_transport_result
# - std_msgs/String(JSON) 명령 수신 -> rack_to_rack 호출 -> result JSON 발행

"""[모듈] rack_transport_server

[역할]
- UI/상위노드가 보내는 rack 이동 명령을 구독하여 단발 실행
- 기존 rack_transport 로직(rack_to_rack)을 재사용

[토픽]
- sub : /<ROBOT_ID>/rack_transport_cmd      (std_msgs/String, JSON)
- pub : /<ROBOT_ID>/rack_transport_result  (std_msgs/String, JSON)

[명령 JSON 예시]
{"cmd":"MOVE","from":"A-1","to":"WORKBENCH_A"}

[결과 JSON 예시]
{"ts":1234567890,"ok":true,"cmd":"MOVE","from":"A-1","to":"WORKBENCH_A","message":"done","error":""}
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import DR_init

from .rack_transport_node import rack_to_rack
from .rack_stations import build_rack_stations  # (필요 시 너가 만든 workbench 포함 버전으로 동작하게 구성돼 있어야 함)
from .gripper_io import grip_init_open


# =========================
# 로봇 설정 상수 (기본 템플릿)
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 60
ACC = 60

# rack_to_rack 기본 파라미터 (rack_transport_node와 동일)
DEFAULT_APPROACH_DY = -100.0
DEFAULT_MOVE_VEL = 200.0
DEFAULT_MOVE_ACC = 200.0
DEFAULT_GRIP_WAIT_SEC = 1.0

DEFAULT_PRE_LIFT_TOOL_MM = 20.0
DEFAULT_PRE_LIFT_TOOL_VEL = 20.0
DEFAULT_POST_LIFT_BASE_MM = 20.0
DEFAULT_POST_LIFT_BASE_VEL = 20.0
DEFAULT_RETRACT_REL_Y_MM = 100.0
DEFAULT_RETRACT_REL_VEL = 50.0
DEFAULT_PLACE_PRE_OPEN = True


def initialize_robot(node, dr):
    """로봇 Tool/TCP 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    node.get_logger().info("#" * 50)
    node.get_logger().info("Initializing robot with the following settings:")
    node.get_logger().info("ROBOT_ID: %s" % ROBOT_ID)
    node.get_logger().info("ROBOT_MODEL: %s" % ROBOT_MODEL)
    node.get_logger().info("ROBOT_TCP: %s" % ROBOT_TCP)
    node.get_logger().info("ROBOT_TOOL: %s" % ROBOT_TOOL)
    node.get_logger().info("VELOCITY: %s" % VELOCITY)
    node.get_logger().info("ACC: %s" % ACC)
    node.get_logger().info("#" * 50)

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass


class RackTransportServer(Node):
    def __init__(self, dr):
        super().__init__("rack_transport_server", namespace=ROBOT_ID)
        self._dr = dr

        self._lock = threading.Lock()
        self._busy = False

        # 파라미터
        self.declare_parameter("approach_dy", float(DEFAULT_APPROACH_DY))
        self.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
        self.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))
        self.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))

        self.declare_parameter("pre_lift_tool_mm", float(DEFAULT_PRE_LIFT_TOOL_MM))
        self.declare_parameter("pre_lift_tool_vel", float(DEFAULT_PRE_LIFT_TOOL_VEL))
        self.declare_parameter("post_lift_base_mm", float(DEFAULT_POST_LIFT_BASE_MM))
        self.declare_parameter("post_lift_base_vel", float(DEFAULT_POST_LIFT_BASE_VEL))
        self.declare_parameter("retract_rel_y_mm", float(DEFAULT_RETRACT_REL_Y_MM))
        self.declare_parameter("retract_rel_vel", float(DEFAULT_RETRACT_REL_VEL))

        self.declare_parameter("place_pre_open", bool(DEFAULT_PLACE_PRE_OPEN))

        # 토픽 (네임스페이스는 Node 생성 시 ROBOT_ID로 자동 적용됨)
        self._cmd_topic = f"/{ROBOT_ID}/rack_transport_cmd"
        self._result_topic = f"/{ROBOT_ID}/rack_transport_result"

        self._pub_result = self.create_publisher(String, self._result_topic, 10)
        self._sub_cmd = self.create_subscription(String, self._cmd_topic, self._on_cmd, 10)

        self.get_logger().info(f"[READY] sub={self._cmd_topic} pub={self._result_topic}")

        # station build
        approach_dy = float(self.get_parameter("approach_dy").value)
        self._stations = build_rack_stations(self._dr, approach_dy=approach_dy)

        # 시작 시 홈/그립 오픈(선택)
        try:
            home_j = self._dr.posj(0, 0, 90, 0, 90, 0)
            self._dr.movej(home_j, vel=VELOCITY, acc=ACC)
            grip_init_open(self._dr, wait_sec=0.2)
        except Exception as e:
            self.get_logger().warn(f"[WARN] init motion skipped: {e}")

    def _publish_result(self, ok, cmd, fr, tr, message="", error=""):
        payload = {
            "ts": int(time.time() * 1000),
            "ok": bool(ok),
            "cmd": cmd,
            "from": fr,
            "to": tr,
            "message": message,
            "error": error,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_result.publish(msg)

    def _on_cmd(self, msg: String):
        # JSON 파싱
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[CMD] invalid json: {e} raw={msg.data}")
            self._publish_result(False, "UNKNOWN", None, None, error="invalid_json")
            return

        cmd = str(data.get("cmd", "")).upper().strip()
        fr = data.get("from", None)
        tr = data.get("to", None)

        if cmd != "MOVE":
            self.get_logger().error(f"[CMD] unsupported cmd={cmd}")
            self._publish_result(False, cmd, fr, tr, error="unsupported_cmd")
            return

        if (fr is None) or (tr is None):
            self.get_logger().error("[CMD] missing from/to")
            self._publish_result(False, cmd, fr, tr, error="missing_from_to")
            return

        # 동시 실행 방지
        with self._lock:
            if self._busy:
                self.get_logger().warn("[CMD] busy -> reject")
                self._publish_result(False, cmd, fr, tr, error="busy")
                return
            self._busy = True

        th = threading.Thread(target=self._run_move, args=(cmd, fr, tr), daemon=True)
        th.start()

    def _run_move(self, cmd, fr, tr):
        try:
            move_vel = float(self.get_parameter("move_vel").value)
            move_acc = float(self.get_parameter("move_acc").value)
            grip_wait_sec = float(self.get_parameter("grip_wait_sec").value)

            pre_lift_tool_mm = float(self.get_parameter("pre_lift_tool_mm").value)
            pre_lift_tool_vel = float(self.get_parameter("pre_lift_tool_vel").value)
            post_lift_base_mm = float(self.get_parameter("post_lift_base_mm").value)
            post_lift_base_vel = float(self.get_parameter("post_lift_base_vel").value)
            retract_rel_y_mm = float(self.get_parameter("retract_rel_y_mm").value)
            retract_rel_vel = float(self.get_parameter("retract_rel_vel").value)
            place_pre_open = bool(self.get_parameter("place_pre_open").value)

            self.get_logger().info(f"[RUN] {cmd} {fr} -> {tr}")

            ok, info = rack_to_rack(
                node=self,
                dr=self._dr,
                rack_stations=self._stations,
                fr=str(fr),
                tr=str(tr),
                move_vel=move_vel,
                move_acc=move_acc,
                grip_wait_sec=grip_wait_sec,
                pre_lift_tool_mm=pre_lift_tool_mm,
                pre_lift_tool_vel=pre_lift_tool_vel,
                post_lift_base_mm=post_lift_base_mm,
                post_lift_base_vel=post_lift_base_vel,
                retract_rel_y_mm=retract_rel_y_mm,
                retract_rel_vel=retract_rel_vel,
                place_pre_open=place_pre_open,
            )

            if ok:
                self._publish_result(True, cmd, fr, tr, message="done")
            else:
                self._publish_result(False, cmd, fr, tr, message=str(info), error="move_failed")

        except Exception as e:
            self.get_logger().error(f"[ERR] exception: {e}")
            self._publish_result(False, cmd, fr, tr, error=str(e))
        finally:
            with self._lock:
                self._busy = False


def main(args=None):
    global ROBOT_ID, ROBOT_MODEL

    rclpy.init(args=args)

    # ✅ DSR_ROBOT2 import 전에 id/model 세팅
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # ✅ node 먼저 만들고 __dsr__node 세팅 후 DSR_ROBOT2 import
    boot = rclpy.create_node("_rack_transport_boot", namespace=ROBOT_ID)
    DR_init.__dsr__node = boot

    import DSR_ROBOT2 as dr

    try:
        initialize_robot(boot, dr)

        server = RackTransportServer(dr)

        # boot 노드는 역할 끝났으니 정리
        try:
            boot.destroy_node()
        except Exception:
            pass

        rclpy.spin(server)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
