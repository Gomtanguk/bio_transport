# rack_transport_node v1.111 2026-01-18
# [이번 버전에서 수정된 사항]
# - (버그수정) A안(1회 실행) 모드에서 rclpy.spin(node)로 인해 대기 상태로 멈추는 문제 수정
# - (버그수정) rclpy.shutdown() 중복 호출로 인한 예외 가능성 완화
# - (유지) v1.110 파라미터 안정화(from_rack/to_rack normalize/validate/fallback) 및 Pick/Place 구조 유지

import re
import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_xyzabc as rel_move
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

# DR_init 설정 (DSR_ROBOT2 import 전에 세팅)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================
# 기본값 (파라미터로 override)
# =========================
DEFAULT_FROM_RACK = "A-1"
DEFAULT_TO_RACK = "B-1"

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


def _valid_rack_keys():
    """rack_stations.RACK_TARGETS 기반으로 유효 rack key 생성"""
    keys = list(RACK_TARGETS.keys())
    keys.sort()
    return keys


def _normalize_rack_name(raw):
    """
    사용자 입력을 최대한 관대하게 보정해서 표준 키(A-1 형태)로 변환

    허용 입력 예:
      - "a-1", " A - 1 ", "A_1", "a 1"  -> "A-1"
      - "b3", "B 3"                     -> "B-3"
    """
    if raw is None:
        return ""

    s = str(raw).strip().upper()

    # 구분자 통일 및 공백 제거
    s = s.replace("_", "-")
    s = re.sub(r"\s+", "", s)

    # "A-1"
    m = re.match(r"^([A-Z])\-([0-9]+)$", s)
    if m:
        return "%s-%s" % (m.group(1), m.group(2))

    # "A1"
    m = re.match(r"^([A-Z])([0-9]+)$", s)
    if m:
        return "%s-%s" % (m.group(1), m.group(2))

    return s


def _resolve_rack_param(node, name, default_key, valid_keys):
    """
    파라미터(name)의 입력값을 읽고:
      - normalize
      - valid 검사
      - invalid면 default로 폴백

    Returns: (resolved_key, meta_dict)
    """
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


def rack_to_rack(node, dr, rack_stations, fr, tr,
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
    node.declare_parameter("from_rack", DEFAULT_FROM_RACK)
    node.declare_parameter("to_rack", DEFAULT_TO_RACK)

    node.declare_parameter("approach_dy", float(DEFAULT_APPROACH_DY))
    node.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
    node.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))

    node.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))
    node.declare_parameter("pick_lift_mm", float(DEFAULT_PICK_LIFT_MM))
    node.declare_parameter("pick_lift_vel", float(DEFAULT_PICK_LIFT_VEL))
    node.declare_parameter("place_pre_open", bool(DEFAULT_PLACE_PRE_OPEN))


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

    # 같은 랙이면 즉시 abort
    if fr == tr:
        node.get_logger().error(
            "[ABORT] from_rack == to_rack (%s). Change parameters. valid=%s" %
            (fr, ",".join(valid_keys))
        )
        return

    approach_dy = float(node.get_parameter("approach_dy").value)
    move_vel = float(node.get_parameter("move_vel").value)
    move_acc = float(node.get_parameter("move_acc").value)

    grip_wait_sec = float(node.get_parameter("grip_wait_sec").value)
    pick_lift_mm = float(node.get_parameter("pick_lift_mm").value)
    pick_lift_vel = float(node.get_parameter("pick_lift_vel").value)
    place_pre_open = bool(node.get_parameter("place_pre_open").value)

    node.get_logger().info(
        "[PARAM] from_rack=%s(to raw=%s) to_rack=%s(to raw=%s) approach_dy=%.1f move_vel=%.1f move_acc=%.1f "
        "grip_wait_sec=%.2f pick_lift_mm=%.1f pick_lift_vel=%.1f place_pre_open=%s" %
        (fr, fr_meta.get("raw", ""), tr, tr_meta.get("raw", ""),
         approach_dy, move_vel, move_acc,
         grip_wait_sec, pick_lift_mm, pick_lift_vel, str(place_pre_open))
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
        pick_lift_mm=pick_lift_mm,
        pick_lift_vel=pick_lift_vel,
        place_pre_open=place_pre_open,
    )
    node.get_logger().info("[RESULT] ok=%s info=%s" % (str(ok), str(info)))

    dr.movej(home_j, vel=V_J, acc=A_J)


def main(args=None):
    # ✅ A안(1회 실행): spin 없이 파라미터 읽고 수행 후 종료
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
