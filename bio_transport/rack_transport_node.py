# rack_transport_node v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - Pick/Place 모듈을 순서대로 호출하는 오케스트레이션 노드

import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_xyzabc as rel_move
from .rack_stations import build_rack_stations
from .rack_pick_io import rack_pick_only
from .rack_place_io import rack_place_only

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 60
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

V_J = VELOCITY
A_J = ACC

MOVE_VEL = 200
MOVE_ACC = 200

APPROACH_DY = -100.0
GRIP_WAIT_SEC = 1.0

PICK_LIFT_MM = 20.0
PICK_LIFT_VEL = 20.0
PLACE_PRE_OPEN = True


def initialize_robot(node):
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


def rack_to_rack(node, dr, rack_stations, from_rack, to_rack):
    valid = ["A", "B", "C", "D", "E", "F"]
    fr = str(from_rack).strip().upper()
    tr = str(to_rack).strip().upper()

    if (fr not in valid) or (tr not in valid) or (fr == tr):
        node.get_logger().error("Invalid racks (from=%s, to=%s)" % (str(from_rack), str(to_rack)))
        return False, {"reason": "invalid_rack"}

    sp_from = rack_stations.get(fr)
    sp_to = rack_stations.get(tr)

    node.get_logger().info("[RUN] rack_to_rack: %s -> %s" % (fr, tr))

    ok_pick, pick_info = rack_pick_only(
        node=node,
        dr=dr,
        station=sp_from,
        tag="PICK_%s" % fr,
        probe_fn=probe_contact_for_rack,
        grip_open_fn=grip_open,
        grip_close_fn=grip_close,
        rel_move_fn=rel_move,
        grip_wait_sec=GRIP_WAIT_SEC,
        lift_mm=PICK_LIFT_MM,
        lift_vel=PICK_LIFT_VEL,
        abort_to_approach=True,
        move_vel=MOVE_VEL,
        move_acc=MOVE_ACC,
    )
    if not ok_pick:
        return False, {"phase": "pick", "from": fr, "to": tr, "pick": pick_info}

    ok_place, place_info = rack_place_only(
        node=node,
        dr=dr,
        station=sp_to,
        tag="PLACE_%s" % tr,
        grip_open_fn=grip_open,
        grip_wait_sec=GRIP_WAIT_SEC,
        move_vel=MOVE_VEL,
        move_acc=MOVE_ACC,
        pre_open=PLACE_PRE_OPEN,
    )
    if not ok_place:
        return False, {"phase": "place", "from": fr, "to": tr, "pick": pick_info, "place": place_info}

    return True, {"from": fr, "to": tr, "pick": pick_info, "place": place_info}


def perform_task(node):
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    home_j = dr.posj(0, 0, 90, 0, 90, 0)
    rack_stations = build_rack_stations(dr, approach_dy=APPROACH_DY)

    dr.movej(home_j, vel=V_J, acc=A_J)
    grip_init_open(dr, wait_sec=0.2)

    # v1.000 기본 테스트: A -> D
    ok, info = rack_to_rack(node, dr, rack_stations, "A", "D")
    node.get_logger().info("[RESULT] ok=%s info=%s" % (str(ok), str(info)))

    dr.movej(home_j, vel=V_J, acc=A_J)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rack_transport", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import wait
        initialize_robot(node)
        wait(1.0)
        perform_task(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
