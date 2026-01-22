# tube_transport_node v2.132 2026-01-22
# [목표: 가장 단순 Pick&Place 테스트]
# - 힘/순응제어 전부 제거
# - goal로 받은 pick_posx로 이동
# - gripper OPEN -> BASE 기준 20mm DOWN -> gripper CLOSE -> BASE 기준 130mm UP
# - goal로 받은 place_posx로 이동
# - BASE 기준 80mm DOWN -> gripper OPEN(wait 길게) -> BASE 기준 90mm UP
#
# [중요]
# - movel/posx는 dr.movel/dr.posx로만 사용(환경에 따라 from-import movel이 -1 나오는 경우 회피)
# - movel ret < 0 이면 즉시 FAIL 처리
# - rel_movel_base는 기존 모듈(.rel_move) 그대로 사용

import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from biobank_interfaces.action import TubeTransport

# =========================
# QoS (latched)
# =========================
qos_latched = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# =========================
# 로봇 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# =========================
# 모션 파라미터
# =========================
VELOCITY = 60
ACC = 60

# =========================
# DR_init 기본 설정(노드 주입은 main에서 수행)
# =========================
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


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
            "tube_transport",
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._on_execute,
        )

        self.get_logger().info("TubeTransportNode ready (ActionServer: /dsr01/tube_transport)")

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

            # === 4) PLACE: down -> OPEN(wait) -> up ===
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
            result.message = "Simple pick&place done"

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

    node = TubeTransportNode()

    # ✅ 가장 중요: 노드 주입을 최우선
    DR_init.__dsr__node = node

    # 로봇 초기화
    node.initialize_robot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
