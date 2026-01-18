# rack_inbound_node v1.700 2026-01-18
# [이번 버전에서 수정된 사항]
# - (기능추가) 인바운드 구현: WORKBENCH -> to_rack (요구 시퀀스 그대로)
#   1) WB approach(Y-50) -> WB target
#   2) probe (TOOL -Z 3mm retract 포함)
#   3) TOOL +Z 20
#   4) grip_close
#   5) retract: BASE +Z 50
#   6) to_rack approach -> (target BASE Z +20) -> target
#   7) grip_open
# - (유지) A안 1회 실행: 수행 후 종료 (spin 없음)

import re
import rclpy
import DR_init

from .gripper_io import grip_open, grip_close, grip_init_open
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_tool, rel_movel_base
from .rack_stations import build_rack_stations, build_workbench_station_dy, RACK_TARGETS

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
DEFAULT_TO_RACK = "B-1"

DEFAULT_MOVE_VEL = 200.0
DEFAULT_MOVE_ACC = 200.0

DEFAULT_APPROACH_DY_RACK = -100.0          # 목표 랙 접근(Y)
DEFAULT_WB_APPROACH_DY = -50.0             # WORKBENCH 접근(Y)

DEFAULT_GRIP_WAIT_SEC = 1.0

# (요구사항) probe 후 TOOL +Z 20
DEFAULT_WB_TOOL_LIFT_Z_MM = 20.0
DEFAULT_WB_TOOL_LIFT_VEL = 20.0

# (요구사항) grip_close 후 retract: BASE +Z 50
DEFAULT_AFTER_GRIP_BASE_LIFT_Z_MM = 50.0
DEFAULT_AFTER_GRIP_BASE_LIFT_VEL = 50.0

# (요구사항) 목표 타겟 접근: BASE Z +20에서 내려오기
DEFAULT_TARGET_TOP_Z_MM = 20.0


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
    node.declare_parameter("to_rack", DEFAULT_TO_RACK)

    node.declare_parameter("move_vel", float(DEFAULT_MOVE_VEL))
    node.declare_parameter("move_acc", float(DEFAULT_MOVE_ACC))

    node.declare_parameter("approach_dy_rack", float(DEFAULT_APPROACH_DY_RACK))
    node.declare_parameter("wb_approach_dy", float(DEFAULT_WB_APPROACH_DY))

    node.declare_parameter("grip_wait_sec", float(DEFAULT_GRIP_WAIT_SEC))

    node.declare_parameter("wb_tool_lift_z_mm", float(DEFAULT_WB_TOOL_LIFT_Z_MM))
    node.declare_parameter("wb_tool_lift_vel", float(DEFAULT_WB_TOOL_LIFT_VEL))

    node.declare_parameter("after_grip_base_lift_z_mm", float(DEFAULT_AFTER_GRIP_BASE_LIFT_Z_MM))
    node.declare_parameter("after_grip_base_lift_vel", float(DEFAULT_AFTER_GRIP_BASE_LIFT_VEL))

    node.declare_parameter("target_top_z_mm", float(DEFAULT_TARGET_TOP_Z_MM))


def perform_task(node):
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    _declare_params(node)

    valid = _valid_rack_keys()
    tr_raw = node.get_parameter("to_rack").value
    tr = _normalize_rack_name(tr_raw)
    if tr not in valid:
        node.get_logger().error("[ABORT] invalid to_rack=%s (normalized=%s) valid=%s" %
                                (str(tr_raw), str(tr), ",".join(valid)))
        return

    move_vel = float(node.get_parameter("move_vel").value)
    move_acc = float(node.get_parameter("move_acc").value)

    approach_dy_rack = float(node.get_parameter("approach_dy_rack").value)
    wb_approach_dy = float(node.get_parameter("wb_approach_dy").value)

    grip_wait_sec = float(node.get_parameter("grip_wait_sec").value)

    wb_tool_lift_z = float(node.get_parameter("wb_tool_lift_z_mm").value)
    wb_tool_lift_v = float(node.get_parameter("wb_tool_lift_vel").value)

    after_grip_base_lift_z = float(node.get_parameter("after_grip_base_lift_z_mm").value)
    after_grip_base_lift_v = float(node.get_parameter("after_grip_base_lift_vel").value)

    target_top_z = float(node.get_parameter("target_top_z_mm").value)

    node.get_logger().info(
        "[PARAM] inbound to=%s move(vel=%.1f,acc=%.1f) wb(dy=%.1f tool+Z=%.1f) "
        "after_grip(base+Z=%.1f) target(topZ=%.1f) rack(dy=%.1f)" %
        (tr, move_vel, move_acc, wb_approach_dy, wb_tool_lift_z,
         after_grip_base_lift_z, target_top_z, approach_dy_rack)
    )

    # 홈
    home_j = dr.posj(0, 0, 90, 0, 90, 0)

    # 스테이션들
    rack_stations = build_rack_stations(dr, approach_dy=approach_dy_rack)
    wb_station = build_workbench_station_dy(dr, approach_dy=wb_approach_dy)

    # 준비
    dr.movej(home_j, vel=V_J, acc=A_J)
    grip_init_open(dr, wait_sec=0.2)

    # =========================
    # 1) WORKBENCH 접근/타겟 이동
    # =========================
    node.get_logger().info("[INBOUND] WB: approach -> target")
    dr.movel(wb_station["approach"], vel=move_vel, acc=move_acc)
    dr.movel(wb_station["target"], vel=move_vel, acc=move_acc)

    # =========================
    # 2) probe (내부 retract 3mm 포함)
    # =========================
    node.get_logger().info("[INBOUND] WB: probe")
    contacted, traveled, last_force = probe_contact_for_rack(node, dr, "WB_INBOUND")
    if not contacted:
        node.get_logger().error("[ABORT] WB probe failed (no contact).")
        return

    # =========================
    # 3) TOOL +Z 20
    # =========================
    node.get_logger().info("[INBOUND] WB: TOOL +Z %.1f" % wb_tool_lift_z)
    if wb_tool_lift_z != 0.0:
        rel_movel_tool(dr, 0, 0, wb_tool_lift_z, 0, 0, 0, wb_tool_lift_v)

    # =========================
    # 4) grip_close
    # =========================
    node.get_logger().info("[INBOUND] WB: grip_close")
    grip_close(dr, wait_sec=grip_wait_sec)

    # =========================
    # 5) retract: BASE +Z 50
    # =========================
    node.get_logger().info("[INBOUND] retract: BASE +Z %.1f" % after_grip_base_lift_z)
    if after_grip_base_lift_z != 0.0:
        rel_movel_base(dr, 0, 0, after_grip_base_lift_z, 0, 0, 0, after_grip_base_lift_v)

    # =========================
    # 6) 목표 랙 approach로 이동
    # =========================
    st = rack_stations[tr]
    node.get_logger().info("[INBOUND] to_rack: movel(approach)")
    dr.movel(st["approach"], vel=move_vel, acc=move_acc)

    # =========================
    # 7) 타겟의 BASE Z+20 위치로 이동 -> 타겟 안착
    # =========================
    tgt = st["target"]
    top_pose = dr.posx(tgt[0], tgt[1], tgt[2] + target_top_z, tgt[3], tgt[4], tgt[5])

    node.get_logger().info("[INBOUND] to_rack: movel(target_top z+%.1f) -> movel(target)" % target_top_z)
    dr.movel(top_pose, vel=move_vel, acc=move_acc)
    dr.movel(tgt, vel=move_vel, acc=move_acc)

    # =========================
    # 8) 그립 오픈(놓기)
    # =========================
    node.get_logger().info("[INBOUND] place: grip_open")
    grip_open(dr, wait_sec=grip_wait_sec)

    # =========================
    # 9) retract
    # =========================
    rel_movel_base(dr, 0, -100.0, 0, 0, 0, 0, 50.0)

    # 마무리 홈
    dr.movej(home_j, vel=V_J, acc=A_J)
    node.get_logger().info("[RESULT] inbound ok=True to=%s" % tr)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rack_inbound", namespace=ROBOT_ID)
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
