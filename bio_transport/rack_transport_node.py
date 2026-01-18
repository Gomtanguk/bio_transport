# rack_transport_node v1.100 2026-01-18
# [이번 버전에서 수정된 사항]
# - (기능추가) from_rack/to_rack를 ROS2 parameter로 받아 실행 가능
#   - 기본값: from_rack="A", to_rack="D"
#   - 실행 예: ros2 run bio_transport rack_transport --ros-args -p from_rack:=B -p to_rack:=E
# - (기능추가) 접근 오프셋/이동 속도 등 주요 파라미터를 ROS2 parameter로 오버라이드 가능
# - (유지) v1.000 Pick/Place 분리 구조 및 안전 그리퍼 OPEN 초기화 유지

import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_xyzabc as rel_move
from .rack_stations import build_rack_stations
from .rack_pick_io import rack_pick_only
from .rack_place_io import rack_place_only

# =========================
# 로봇 설정 상수 (기본 템플릿)
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 60
ACC = 60

# DR_init 설정 (DSR_ROBOT2 import 전에 세팅)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================
# v1.000 기본값 (필요 시 파라미터로 override)
# =========================
DEFAULT_MOVE_VEL = 200.0
DEFAULT_MOVE_ACC = 200.0
DEFAULT_APPROACH_DY = -100.0
DEFAULT_GRIP_WAIT_SEC = 1.0
DEFAULT_PICK_LIFT_MM = 20.0
DEFAULT_PICK_LIFT_VEL = 20.0
DEFAULT_PLACE_PRE_OPEN = True

V_J = VELOCITY
A_J = ACC


def initialize_robot(node):
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


def rack_to_rack(node, dr, rack_stations, from_rack, to_rack,
                move_vel, move_acc,
                grip_wait_sec,
                pick_lift_mm, pick_lift_vel,
                place_pre_open):
    """
    Orchestration:
      Pick(from_rack) -> Place(to_rack)

    Returns:
      (ok: bool, info: dict)
    """
    valid = ["A", "B", "C", "D", "E", "F"]
    fr = str(from_rack).strip().upper()
    tr = str(to_rack).strip().upper()

    if (fr not in valid) or (tr not in valid) or (fr == tr):
        node.get_logger().error("Invalid racks (from=%s, to=%s)" % (str(from_rack), str(to_rack)))
        return False, {"reason": "invalid_rack"}

    sp_from = rack_stations.get(fr)
    sp_to = rack_stations.get(tr)
    if (sp_from is None) or (sp_to is None):
        node.get_logger().error("Unknown station (from=%s, to=%s)" % (str(fr), str(tr)))
        return False, {"reason": "unknown_station"}

    node.get_logger().info("[RUN] rack_to_rack: %s -> %s" % (fr, tr))

    # ---- Pick ----
    ok_pick, pick_info = rack_pick_only(
        node=node,
        dr=dr,
        station=sp_from,
        tag="PICK_%s" % fr,
        probe_fn=probe_contact_for_rack,
        grip_open_fn=grip_open,
        grip_close_fn=grip_close,
        rel_move_fn=rel_move,
        grip_wait_sec=grip_wait_sec,
        lift_mm=pick_lift_mm,
        lift_vel=pick_lift_vel,
        abort_to_approach=True,
        move_vel=move_vel,
        move_acc=move_acc,
    )

    if not ok_pick:
        node.get_logger().error("[ABORT] PICK failed at %s" % fr)
        return False, {"phase": "pick", "from": fr, "to": tr, "pick": pick_info}

    # ---- Place ----
    ok_place, place_info = rack_place_only(
        node=node,
        dr=dr,
        station=sp_to,
        tag="PLACE_%s" % tr,
        grip_open_fn=grip_open,
        grip_wait_sec=grip_wait_sec,
        move_vel=move_vel,
        move_acc=move_acc,
        pre_open=place_pre_open,
    )

    if not ok_place:
        node.get_logger().error("[ABORT] PLACE failed at %s" % tr)
        return False, {"phase": "place", "from": fr, "to": tr, "pick": pick_info, "place": place_info}

    return True, {"from": fr, "to": tr, "pick": pick_info, "place": place_info}


def _declare_params(node):
    # 작업 대상
    node.declare_parameter("from_rack", "A")
    node.declare_parameter("to_rack", "D")

    # motion/station
    node.declare_parameter("approach_dy", float(DEFAULT_APPROACH_DY))
    node.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
    node.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))

    # gripper/pick/place
    node.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))
    node.declare_parameter("pick_lift_mm", float(DEFAULT_PICK_LIFT_MM))
    node.declare_parameter("pick_lift_vel", float(DEFAULT_PICK_LIFT_VEL))
    node.declare_parameter("place_pre_open", bool(DEFAULT_PLACE_PRE_OPEN))


def _get_param(node, name, cast_fn):
    # rclpy Parameter -> python 값 변환을 안전하게 처리
    try:
        v = node.get_parameter(name).value
        return cast_fn(v)
    except Exception:
        # 파라미터 타입이 꼬였거나 값이 비정상이면 기본값 유지
        node.get_logger().warn("[PARAM] invalid %s, fallback to default" % str(name))
        return None


def perform_task(node):
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    # ---- 파라미터 선언/읽기 ----
    _declare_params(node)

    from_rack = node.get_parameter("from_rack").value
    to_rack = node.get_parameter("to_rack").value

    approach_dy = node.get_parameter("approach_dy").value
    move_vel = float(node.get_parameter("move_vel").value)
    move_acc = float(node.get_parameter("move_acc").value)

    grip_wait_sec = float(node.get_parameter("grip_wait_sec").value)
    pick_lift_mm = float(node.get_parameter("pick_lift_mm").value)
    pick_lift_vel = float(node.get_parameter("pick_lift_vel").value)
    place_pre_open = bool(node.get_parameter("place_pre_open").value)

    node.get_logger().info(
        "[PARAM] from_rack=%s to_rack=%s approach_dy=%.1f move_vel=%.1f move_acc=%.1f "
        "grip_wait_sec=%.2f pick_lift_mm=%.1f pick_lift_vel=%.1f place_pre_open=%s" %
        (str(from_rack), str(to_rack), float(approach_dy), move_vel, move_acc,
         grip_wait_sec, pick_lift_mm, pick_lift_vel, str(place_pre_open))
    )

    # ---- 준비 ----
    home_j = dr.posj(0, 0, 90, 0, 90, 0)
    rack_stations = build_rack_stations(dr, approach_dy=approach_dy)

    dr.movej(home_j, vel=V_J, acc=A_J)
    grip_init_open(dr, wait_sec=0.2)

    ok, info = rack_to_rack(
        node=node,
        dr=dr,
        rack_stations=rack_stations,
        from_rack=from_rack,
        to_rack=to_rack,
        move_vel=move_vel,
        move_acc=move_acc,
        grip_wait_sec=grip_wait_sec,
        pick_lift_mm=pick_lift_mm,
        pick_lift_vel=pick_lift_vel,
        place_pre_open=place_pre_open,
    )
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
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user. Shutting down...")
    except Exception as e:
        node.get_logger().error("An unexpected error occurred: %s" % str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
