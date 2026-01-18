# rack_pick_node v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - Pick 단독 테스트 노드

import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_tool as rel_move
from .rack_stations import build_rack_stations
from .rack_pick_io import rack_pick_only

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot(node):
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rack_pick", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import wait
        import DSR_ROBOT2 as dr

        initialize_robot(node)
        wait(1.0)

        stations = build_rack_stations(dr, approach_dy=-100.0)
        grip_init_open(dr, wait_sec=0.2)

        ok, info = rack_pick_only(
            node=node,
            dr=dr,
            station=stations["A-1"],
            tag="PICK_A-1",
            probe_fn=probe_contact_for_rack,
            grip_open_fn=grip_open,
            grip_close_fn=grip_close,
            rel_move_fn=rel_move,
            grip_wait_sec=1.0,
            lift_mm=20.0,
            lift_vel=20.0,
            abort_to_approach=True,
            move_vel=200,
            move_acc=200,
        )
        node.get_logger().info("[RESULT] ok=%s info=%s" % (str(ok), str(info)))

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
