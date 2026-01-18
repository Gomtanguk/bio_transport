# rack_outbound_node v1.600 2026-01-18
# [이번 버전에서 수정된 사항]
# - (기능변경) WORKBENCH Place를 전용 함수(workbench_place_only)로 분리
#   - WORKBENCH 접근은 Z+ approach(위에서 내려오기)
#   - retract movel 제거, 후처리 rel_movel_base(-Y50, +Z50)로 대체
# - (유지) Pick은 rack_pick_only(v1.500) 호출부(TOOL/BASE 혼합 lift + BASE -Y100 retract) 유지
# - (유지) A안(1회 실행): 수행 후 종료 (spin 없음)

import re
import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_tool, rel_movel_base
from .rack_stations import build_rack_stations, build_workbench_station_top, RACK_TARGETS
from .rack_pick_io import rack_pick_only
from .workbench_place_io import workbench_place_only

# =========================
# 로봇 설정
# =========================
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

# =========================
# 파라미터 기본값
# =========================
DEFAULT_FROM_RACK = "A-1"

DEFAULT_MOVE_VEL = 200.0
DEFAULT_MOVE_ACC = 200.0
DEFAULT_APPROACH_DY_RACK = -100.0

DEFAULT_GRIP_WAIT_SEC = 1.0

# Pick 내부 혼합 규칙(v1.500)
DEFAULT_PRE_LIFT_TOOL_MM = 20.0
DEFAULT_PRE_LIFT_TOOL_VEL = 20.0
DEFAULT_POST_LIFT_BASE_MM = 20.0
DEFAULT_POST_LIFT_BASE_VEL = 20.0
DEFAULT_RETRACT_REL_Y_MM = 100.0
DEFAULT_RETRACT_REL_VEL = 50.0

# WORKBENCH approach (Z+)
DEFAULT_WB_APPROACH_DZ = 100.0

# WORKBENCH place 후 후처리 (BASE 상대)
DEFAULT_POST_PLACE_RETRACT_Y_MM = 50.0  # -Y 50
DEFAULT_POST_PLACE_LIFT_Z_MM = 50.0     # +Z 50
DEFAULT_POST_PLACE_REL_VEL = 100.0


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


def _valid_rack_keys():
    keys = list(RACK_TARGETS.keys())
    keys.sort()
    return keys


def _normalize_rack_name(raw):
    if raw is None:
        return ""
    s = str(raw).strip().upper()
    s = s.replace("_", "-")
    s = re.sub(r"\s+", "", s)

    m = re.match(r"^([A-Z])\-([0-9]+)$", s)
    if m:
        return "%s-%s" % (m.group(1), m.group(2))
    m = re.match(r"^([A-Z])([0-9]+)$", s)
    if m:
        return "%s-%s" % (m.group(1), m.group(2))
    return s


def _declare_params(node):
    node.declare_parameter("from_rack", DEFAULT_FROM_RACK)

    node.declare_parameter("approach_dy_rack", float(DEFAULT_APPROACH_DY_RACK))
    node.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
    node.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))
    node.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))

    # pick(v1.500)
    node.declare_parameter("pre_lift_tool_mm", float(DEFAULT_PRE_LIFT_TOOL_MM))
    node.declare_parameter("pre_lift_tool_vel", float(DEFAULT_PRE_LIFT_TOOL_VEL))
    node.declare_parameter("post_lift_base_mm", float(DEFAULT_POST_LIFT_BASE_MM))
    node.declare_parameter("post_lift_base_vel", float(DEFAULT_POST_LIFT_BASE_VEL))
    node.declare_parameter("retract_rel_y_mm", float(DEFAULT_RETRACT_REL_Y_MM))
    node.declare_parameter("retract_rel_vel", float(DEFAULT_RETRACT_REL_VEL))

    # workbench approach
    node.declare_parameter("wb_approach_dz", float(DEFAULT_WB_APPROACH_DZ))

    # post place
    node.declare_parameter("post_place_retract_y_mm", float(DEFAULT_POST_PLACE_RETRACT_Y_MM))
    node.declare_parameter("post_place_lift_z_mm", float(DEFAULT_POST_PLACE_LIFT_Z_MM))
    node.declare_parameter("post_place_rel_vel", float(DEFAULT_POST_PLACE_REL_VEL))


def _post_place_motion(node, dr, retract_y_mm, lift_z_mm, rel_vel):
    y = float(retract_y_mm)
    z = float(lift_z_mm)
    v = float(rel_vel)

    node.get_logger().info("[POST_PLACE] BASE -Y %.1fmm -> +Z %.1fmm" % (y, z))
    rel_movel_base(dr, 0, -y, 0, 0, 0, 0, v)
    rel_movel_base(dr, 0, 0, z, 0, 0, 0, v)
    node.get_logger().info("[POST_PLACE] done")


def perform_task(node):
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    _declare_params(node)

    valid = _valid_rack_keys()
    fr_raw = node.get_parameter("from_rack").value
    fr = _normalize_rack_name(fr_raw)
    if fr not in valid:
        node.get_logger().error("[ABORT] invalid from_rack=%s (normalized=%s) valid=%s" %
                                (str(fr_raw), str(fr), ",".join(valid)))
        return

    approach_dy_rack = float(node.get_parameter("approach_dy_rack").value)
    move_vel = float(node.get_parameter("move_vel").value)
    move_acc = float(node.get_parameter("move_acc").value)
    grip_wait_sec = float(node.get_parameter("grip_wait_sec").value)

    pre_tool_z = float(node.get_parameter("pre_lift_tool_mm").value)
    pre_tool_v = float(node.get_parameter("pre_lift_tool_vel").value)
    post_base_z = float(node.get_parameter("post_lift_base_mm").value)
    post_base_v = float(node.get_parameter("post_lift_base_vel").value)
    ret_y = float(node.get_parameter("retract_rel_y_mm").value)
    ret_v = float(node.get_parameter("retract_rel_vel").value)

    wb_dz = float(node.get_parameter("wb_approach_dz").value)

    post_y = float(node.get_parameter("post_place_retract_y_mm").value)
    post_z = float(node.get_parameter("post_place_lift_z_mm").value)
    post_v = float(node.get_parameter("post_place_rel_vel").value)

    node.get_logger().info(
        "[PARAM] outbound from=%s move(vel=%.1f,acc=%.1f) pick(pre_tool_z=%.1f, post_base_z=%.1f, ret_y=%.1f) "
        "wb(dz=%.1f) post(-Y=%.1f,+Z=%.1f)" %
        (fr, move_vel, move_acc, pre_tool_z, post_base_z, ret_y, wb_dz, post_y, post_z)
    )

    home_j = dr.posj(0, 0, 90, 0, 90, 0)

    rack_stations = build_rack_stations(dr, approach_dy=approach_dy_rack)
    wb_station = build_workbench_station_top(dr, approach_dz=wb_dz)

    dr.movej(home_j, vel=V_J, acc=A_J)
    grip_init_open(dr, wait_sec=0.2)

    # 1) Pick(from_rack) - v1.500 혼합 규칙
    ok_pick, pick_info = rack_pick_only(
        node=node,
        dr=dr,
        station=rack_stations[fr],
        tag="OUT_PICK_%s" % fr,
        probe_fn=probe_contact_for_rack,
        grip_open_fn=grip_open,
        grip_close_fn=grip_close,
        rel_move_tool_fn=rel_movel_tool,
        rel_move_base_fn=rel_movel_base,
        grip_wait_sec=grip_wait_sec,
        pre_lift_tool_mm=pre_tool_z,
        pre_lift_tool_vel=pre_tool_v,
        post_lift_base_mm=post_base_z,
        post_lift_base_vel=post_base_v,
        retract_rel_y_mm=ret_y,
        retract_rel_vel=ret_v,
        abort_to_approach=True,
        move_vel=move_vel,
        move_acc=move_acc,
        align_to_retract_pose=False,
    )

    if not ok_pick:
        node.get_logger().error("[RESULT] pick failed info=%s" % str(pick_info))
        dr.movej(home_j, vel=V_J, acc=A_J)
        return

    # 2) Place(WORKBENCH) - Z+ approach, retract movel 없음
    ok_place, place_info = workbench_place_only(
        node=node,
        dr=dr,
        wb_station=wb_station,
        tag="OUT_WB_PLACE",
        grip_open_fn=grip_open,
        grip_wait_sec=grip_wait_sec,
        move_vel=move_vel,
        move_acc=move_acc,
        pre_open=True,
    )

    # 3) Place 후 후처리: BASE -Y50, +Z50
    if ok_place:
        _post_place_motion(node, dr, post_y, post_z, post_v)

    node.get_logger().info("[RESULT] outbound ok=%s pick=%s place=%s" %
                           (str(ok_place), str(pick_info), str(place_info)))

    dr.movej(home_j, vel=V_J, acc=A_J)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rack_outbound", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import wait
        initialize_robot(node)
        wait(1.0)
        perform_task(node)
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
