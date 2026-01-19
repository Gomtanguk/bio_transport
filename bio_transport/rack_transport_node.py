# rack_transport_node v2.200 2026-01-19
# [이번 버전에서 수정된 사항]
# - PLACE에 target_top(BASE Z +place_top_dz_mm) -> target 단계 추가
# - place_pre_open 기본값 False로 변경(조기 오픈 방지)
# - 파라미터 place_top_dz_mm 추가(기본 20mm)
# - 버그 수정: node.ge30t_parameter 오타 수정, main()의 dr 미정의 grip_close 호출 제거

"""[모듈] rack_transport_node

[역할]
- from_rack -> to_rack (랙간 이송) 시퀀스를 단발 실행
"""

import re
import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_tool, rel_movel_base
from .rack_stations import build_rack_stations, RACK_TARGETS
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

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================
# 기본값 (파라미터로 override)
# =========================
DEFAULT_FROM_RACK = "A-1"
DEFAULT_TO_RACK = "B-1"

DEFAULT_MOVE_VEL = 200.0
DEFAULT_MOVE_ACC = 200.0
DEFAULT_APPROACH_DY = -250.0

DEFAULT_GRIP_WAIT_SEC = 1.0

# pick 관련 기본값
DEFAULT_PRE_LIFT_TOOL_MM = 20.0
DEFAULT_PRE_LIFT_TOOL_VEL = 20.0
DEFAULT_POST_LIFT_BASE_MM = 50.0
DEFAULT_POST_LIFT_BASE_VEL = 20.0
DEFAULT_RETRACT_REL_Y_MM = 250.0
DEFAULT_RETRACT_REL_VEL = 50.0

# place 관련 기본값
DEFAULT_PLACE_PRE_OPEN = False
DEFAULT_PLACE_TOP_DZ_MM = 50.0

V_J = VELOCITY
A_J = ACC


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


def _resolve_rack_param(node, name, default_key, valid_keys):
    raw = node.get_parameter(name).value
    norm = _normalize_rack_name(raw)

    if norm in valid_keys:
        if str(raw) != norm:
            node.get_logger().info("[PARAM] %s normalized: '%s' -> '%s'" % (name, str(raw), norm))
        return norm, {"raw": str(raw), "normalized": norm, "used_default": False}

    node.get_logger().warn(
        "[PARAM] %s invalid: '%s' (normalized='%s'), fallback to '%s'. valid=%s" %
        (name, str(raw), norm, default_key, ",".join(valid_keys))
    )
    return default_key, {"raw": str(raw), "normalized": norm, "used_default": True, "fallback": default_key}


def rack_to_rack(
    node,
    dr,
    rack_stations,
    fr,
    tr,
    move_vel,
    move_acc,
    grip_wait_sec,
    # pick
    pre_lift_tool_mm,
    pre_lift_tool_vel,
    post_lift_base_mm,
    post_lift_base_vel,
    retract_rel_y_mm,
    retract_rel_vel,
    # place
    place_pre_open,
    place_top_dz_mm,
):
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
        tag=f"PICK_{fr}",
        probe_fn=probe_contact_for_rack,
        grip_open_fn=grip_open,
        grip_close_fn=grip_close,
        rel_move_tool_fn=rel_movel_tool,
        rel_move_base_fn=rel_movel_base,
        grip_wait_sec=grip_wait_sec,
        pre_lift_tool_mm=pre_lift_tool_mm,
        pre_lift_tool_vel=pre_lift_tool_vel,
        post_lift_base_mm=post_lift_base_mm,
        post_lift_base_vel=post_lift_base_vel,
        retract_rel_y_mm=retract_rel_y_mm,
        retract_rel_vel=retract_rel_vel,
        abort_to_approach=True,
        move_vel=move_vel,
        move_acc=move_acc,
        align_to_retract_pose=False,
    )

    if not ok_pick:
        node.get_logger().error("[ABORT] PICK failed at %s" % fr)
        return False, {"phase": "pick", "from": fr, "to": tr, "pick": pick_info}

    # ---- Place ----
    ok_place, place_info = rack_place_only(
        node=node,
        dr=dr,
        station=sp_to,
        tag=f"PLACE_{tr}",
        grip_open_fn=grip_open,
        grip_wait_sec=grip_wait_sec,
        move_vel=move_vel,
        move_acc=move_acc,
        pre_open=place_pre_open,
        top_dz_mm=place_top_dz_mm,
    )

    if not ok_place:
        node.get_logger().error("[ABORT] PLACE failed at %s" % tr)
        return False, {"phase": "place", "from": fr, "to": tr, "pick": pick_info, "place": place_info}

    return True, {"from": fr, "to": tr, "pick": pick_info, "place": place_info}


def _declare_params(node):
    node.declare_parameter("from_rack", DEFAULT_FROM_RACK)
    node.declare_parameter("to_rack", DEFAULT_TO_RACK)

    node.declare_parameter("approach_dy", float(DEFAULT_APPROACH_DY))
    node.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
    node.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))
    node.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))

    # pick
    node.declare_parameter("pre_lift_tool_mm", float(DEFAULT_PRE_LIFT_TOOL_MM))
    node.declare_parameter("pre_lift_tool_vel", float(DEFAULT_PRE_LIFT_TOOL_VEL))
    node.declare_parameter("post_lift_base_mm", float(DEFAULT_POST_LIFT_BASE_MM))
    node.declare_parameter("post_lift_base_vel", float(DEFAULT_POST_LIFT_BASE_VEL))
    node.declare_parameter("retract_rel_y_mm", float(DEFAULT_RETRACT_REL_Y_MM))
    node.declare_parameter("retract_rel_vel", float(DEFAULT_RETRACT_REL_VEL))

    # place
    node.declare_parameter("place_pre_open", bool(DEFAULT_PLACE_PRE_OPEN))
    node.declare_parameter("place_top_dz_mm", float(DEFAULT_PLACE_TOP_DZ_MM))


def perform_task(node):
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    _declare_params(node)

    valid_keys = _valid_rack_keys()
    fr, fr_meta = _resolve_rack_param(node, "from_rack", DEFAULT_FROM_RACK, valid_keys)
    tr, tr_meta = _resolve_rack_param(node, "to_rack", DEFAULT_TO_RACK, valid_keys)

    if fr == tr:
        node.get_logger().error("[ABORT] from_rack == to_rack (%s). Change parameters." % fr)
        return

    approach_dy = float(node.get_parameter("approach_dy").value)
    move_vel = float(node.get_parameter("move_vel").value)
    move_acc = float(node.get_parameter("move_acc").value)
    grip_wait_sec = float(node.get_parameter("grip_wait_sec").value)

    pre_lift_tool_mm = float(node.get_parameter("pre_lift_tool_mm").value)
    pre_lift_tool_vel = float(node.get_parameter("pre_lift_tool_vel").value)
    post_lift_base_mm = float(node.get_parameter("post_lift_base_mm").value)
    post_lift_base_vel = float(node.get_parameter("post_lift_base_vel").value)
    retract_rel_y_mm = float(node.get_parameter("retract_rel_y_mm").value)
    retract_rel_vel = float(node.get_parameter("retract_rel_vel").value)

    place_pre_open = bool(node.get_parameter("place_pre_open").value)
    place_top_dz_mm = float(node.get_parameter("place_top_dz_mm").value)

    node.get_logger().info(
        "[PARAM] from=%s(raw=%s) to=%s(raw=%s) approach_dy=%.1f move_vel=%.1f move_acc=%.1f grip_wait_sec=%.2f "
        "pick(pre_tool_z=%.1f, post_base_z=%.1f, retract_y=%.1f) place(pre_open=%s, top_dz=%.1f)" %
        (
            fr, fr_meta.get("raw", ""),
            tr, tr_meta.get("raw", ""),
            approach_dy, move_vel, move_acc, grip_wait_sec,
            pre_lift_tool_mm, post_lift_base_mm, retract_rel_y_mm,
            str(place_pre_open), place_top_dz_mm
        )
    )

    home_j = dr.posj(0, 0, 90, 0, 90, 0)
    rack_stations = build_rack_stations(dr, approach_dy=approach_dy)

    dr.movej(home_j, vel=V_J, acc=A_J)
    grip_init_open(dr, wait_sec=0.2)

    ok, info = rack_to_rack(
        node=node,
        dr=dr,
        rack_stations=rack_stations,
        fr=fr,
        tr=tr,
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
        place_top_dz_mm=place_top_dz_mm,
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
        node.get_logger().info("Interrupted by user (KeyboardInterrupt).")
    except Exception as e:
        node.get_logger().error("An unexpected error occurred: %s" % str(e))
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

