import traceback
import importlib
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import DR_init
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


# =========================
# QoS (latched)
# =========================
qos_latched = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
VELOCITY_PICK = 60
ACC_PICK = 60

VELOCITY_DISPOSE = 100
ACC_DISPOSE = 50

JReady = [0, 0, 90, 0, 90, 0]

# =========================
# 절대 좌표 (Pick)
# =========================
A_OUT_1 = [300, -25, 420, 120.22, -180, 120.22]
A_OUT_2 = [300, 14, 420, 156.15, -180, 155.93]
A_OUT_3 = [300, 52, 420, 9.89, 180, 9.69]
A_OUT_4 = [300, 88, 420, 20.96, 180, 20.63]

B_OUT_1 = [300, -30, 420, 159.74, -180, 159.87]
B_OUT_2 = [300, 4, 420, 2.79, 180, 3.06]
B_OUT_3 = [300, 40, 420, 18.42, 180, 18.74]
B_OUT_4 = [300, 80, 420, 16.66, 180, 17.21]

OUT_POSES = {
    "A": [A_OUT_1, A_OUT_2, A_OUT_3, A_OUT_4],
    "B": [B_OUT_1, B_OUT_2, B_OUT_3, B_OUT_4],
}

# =========================
# Dispose 시퀀스 파라미터
# =========================
J5_ROTATE_DEG = -70.0
J2_ROTATE_DEG = 15.0
GRIP_OPEN_WAIT_SEC = 1.0

# 폐기 위치(절대)
DISPOSE_POSX = [640, -160, 410, 11.8, 180, 105]


# =========================
# DR_init 설정 (id/model)
# =========================
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def bind_and_load_dsr(node: Node):
    """
    DSR_ROBOT2는 import 시점에 g_node.create_client(...)를 만들기 때문에
    반드시 DR_init.__dsr__node 바인딩 후 import/reload 해야 합니다.
    """
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    import DSR_ROBOT2
    importlib.reload(DSR_ROBOT2)
    return DSR_ROBOT2


def initialize_robot(node: Node, dr):
    from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS
    node.get_logger().info("set_tool/set_tcp/set_robot_mode ...")
    dr.set_tool(ROBOT_TOOL)
    dr.set_tcp(ROBOT_TCP)
    dr.set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    node.get_logger().info("initialize_robot: done")


class TubePickAndDisposeNode(Node):
    def __init__(self):
        super().__init__("tube_pick_disposal_node", namespace=ROBOT_ID)

        # 퍼블리셔(기존 기능 유지)
        self.pub_done = self.create_publisher(Bool, "tube_pick_done", qos_latched)
        self.pub_mode = self.create_publisher(String, "tube_move_mode", qos_latched)

        # 파라미터(선택 튜브)
        self.declare_parameter("rack", "A")
        self.declare_parameter("slot", 1)

        # pick 동작 튜닝
        self.declare_parameter("pick_down_mm", 30.0)    # Z -30mm
        self.declare_parameter("pick_up_mm", 150.0)     # Z +150mm

        self._started = False
        self._done = False
        self._ok = False

        # 실행 트리거: 타이머에서 1회만 스레드 실행
        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        if not self._started:
            self._started = True
            threading.Thread(target=self._run_once, daemon=True).start()

        if self._done:
            try:
                self._timer.cancel()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _run_once(self):
        try:
            self.get_logger().info("[RUN] bind_and_load_dsr() ...")
            dr = bind_and_load_dsr(self)
            self.get_logger().info("[RUN] DSR_ROBOT2 loaded OK")

            initialize_robot(self, dr)

            rack = str(self.get_parameter("rack").value).upper().strip()
            slot = int(self.get_parameter("slot").value)

            pick_down_mm = float(self.get_parameter("pick_down_mm").value)
            pick_up_mm = float(self.get_parameter("pick_up_mm").value)

            # 1) PICK
            self.get_logger().info("[RUN] PICK start ...")
            ok_pick = self.perform_pick_once(
                dr=dr,
                rack=rack,
                slot=slot,
                down_mm=pick_down_mm,
                up_mm=pick_up_mm,
            )

            # pick 결과 토픽 publish (기존 기능 유지)
            if ok_pick:
                self.pub_mode.publish(String(data="OUT"))
                self.pub_done.publish(Bool(data=True))
            else:
                self.pub_done.publish(Bool(data=False))
                self._ok = False
                return  # pick 실패면 dispose 안 함

            self.get_logger().info("[RUN] PICK done -> DISPOSE start ...")

            # 2) DISPOSE
            self.go_dispose(dr)

            self.get_logger().info("[RUN] PICK+DISPOSE all complete")
            self._ok = True

        except Exception as e:
            self._ok = False
            self.get_logger().error(f"[ERR] Failed: {repr(e)}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self._done = True

    def perform_pick_once(
        self,
        dr,
        rack: str,
        slot: int,
        down_mm: float,
        up_mm: float,
    ) -> bool:
        """
        절대좌표 기반 pick
        - JReady
        - gripper open
        - Z down (-down_mm)
        - gripper close
        - Z up (+up_mm)
        """
        from .gripper_io import grip_open, grip_close, grip_fully_close

        rack = rack.upper().strip()
        if rack not in OUT_POSES:
            self.get_logger().error(f"Invalid rack: {rack} (use 'A' or 'B')")
            return False
        if not (1 <= slot <= 4):
            self.get_logger().error(f"Invalid slot: {slot} (use 1~4)")
            return False

        x, y, z, rx, ry, rz = [float(v) for v in OUT_POSES[rack][slot -1]]

        target_pos = dr.posx(B_OUT_4)


        self.get_logger().info(f"[PICK] rack={rack}, slot={slot}")

        # 1) 초기
        dr.movej(JReady, vel=VELOCITY_PICK, acc=ACC_PICK)
        grip_fully_close(dr)
        dr.wait(0.2)

        # 2) 이동
        dr.movel(target_pos, vel=VELOCITY_PICK, acc=ACC_PICK)

        # 3) 그리퍼 열기
        grip_open(dr)
        dr.wait(0.2)

        # 5) Z -down_mm
        cur_pos, _ = dr.get_current_posx(dr.DR_BASE)
        cx, cy, cz, crx, cry, crz = [float(v) for v in cur_pos]
        down_pos = dr.posx(cx, cy, cz - down_mm, crx, cry, crz)
        dr.movel(down_pos, vel=VELOCITY_PICK, acc=ACC_PICK)

        # 6) 닫기
        grip_close(dr)
        dr.wait(0.3)

        # 7) Z +up_mm
        cur_pos, _ = dr.get_current_posx(dr.DR_BASE)
        cx, cy, cz, crx, cry, crz = [float(v) for v in cur_pos]
        up_pos = dr.posx(cx, cy, cz + up_mm, crx, cry, crz)
        dr.movel(up_pos, vel=VELOCITY_PICK, acc=ACC_PICK)

        self.get_logger().info("[PICK] done")
        return True

    def go_dispose(self, dr):
        """
        dispose 시퀀스:
        - 폐기 위치로 movel
        - J5 회전
        - J2 회전
        - gripper open/close
        - JReady 복귀
        """
        self.get_logger().info("[DISPOSE] movel -> dispose pose")
        dx, dy, dz, drx, dry, drz = [float(v) for v in DISPOSE_POSX]
        target_loc = dr.posx(dx, dy, dz, drx, dry, drz)
        dr.movel(target_loc, vel=VELOCITY_DISPOSE, acc=ACC_DISPOSE)

        # J5
        cur_j = [float(v) for v in dr.get_current_posj()]
        target_j = cur_j[:]
        target_j[4] += J5_ROTATE_DEG
        self.get_logger().info(f"[DISPOSE] movej -> J5 += {J5_ROTATE_DEG}deg")
        dr.movej(target_j, vel=VELOCITY_DISPOSE, acc=ACC_DISPOSE)

        # J2
        cur_j = [float(v) for v in dr.get_current_posj()]
        target_j = cur_j[:]
        target_j[1] += J2_ROTATE_DEG
        self.get_logger().info(f"[DISPOSE] movej -> J2 += {J2_ROTATE_DEG}deg")
        dr.movej(target_j, vel=VELOCITY_DISPOSE, acc=ACC_DISPOSE)

        # gripper
        from .gripper_io import grip_open, grip_close
        self.get_logger().info("[DISPOSE] gripper open/close")
        grip_open(dr)
        dr.wait(GRIP_OPEN_WAIT_SEC)
        grip_close(dr)
        dr.wait(0.2)

        # home
        self.get_logger().info("[DISPOSE] return JReady")
        dr.movej(JReady, vel=VELOCITY_DISPOSE, acc=ACC_DISPOSE)
        self.get_logger().info("[DISPOSE] done")


def main(args=None):
    rclpy.init(args=args)
    node = TubePickAndDisposeNode()

    executor = MultiThreadedExecutor()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
