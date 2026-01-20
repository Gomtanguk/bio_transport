# rack_pick_io v2.100 2026-01-19
# [이번 버전에서 수정된 사항]
# - probe(접촉) 수행 전에 gripper를 CLOSE 상태로 만드는 옵션(pre_probe_close) 추가
# - pick info에 pre_probe_close 기록 추가

"""[모듈] rack_pick_io

[역할]
- 랙에서 랙/워크벤치로 이동하기 위한 Pick 동작 IO 시퀀스 제공

[일반 흐름]
1) approach로 안전 접근
2) target 접근
3) (옵션) probe 전에 gripper close
4) probe(내부 retract 3mm 포함)
5) 그리퍼 open -> pre lift -> close -> post lift -> retract
"""

DEFAULT_GRIP_WAIT_SEC = 1.0

# A) 집기 전 TOOL +Z
DEFAULT_PRE_LIFT_TOOL_MM = 50.0
DEFAULT_PRE_LIFT_TOOL_VEL = 20.0

# B) 집은 후 BASE +Z
DEFAULT_POST_LIFT_BASE_MM = 50.0
DEFAULT_POST_LIFT_BASE_VEL = 20.0

# 집은 후 BASE -Y retract
DEFAULT_RETRACT_REL_Y_MM = 100.0
DEFAULT_RETRACT_REL_VEL = 50.0

DEFAULT_ABORT_TO_APPROACH = True


def rack_pick_only(
    node,
    dr,
    station,
    tag,
    probe_fn,
    grip_open_fn,
    grip_close_fn,
    rel_move_tool_fn,      # ✅ TOOL 기준 상대이동 (rel_movel_tool)
    rel_move_base_fn,      # ✅ BASE 기준 상대이동 (rel_movel_base)
    grip_wait_sec=None,
    pre_lift_tool_mm=None,
    pre_lift_tool_vel=None,
    post_lift_base_mm=None,
    post_lift_base_vel=None,
    retract_rel_y_mm=None,
    retract_rel_vel=None,
    abort_to_approach=None,
    move_vel=200,
    move_acc=200,
    align_to_retract_pose=False,
    pre_probe_close=True,  # ✅ v2.100: probe 전에 gripper close
):
    """
    [Pick 시퀀스]
      approach -> target
      -> (옵션) pre_probe_close: grip_close
      -> probe(내부 retract 3mm 포함)
      -> grip_open
      -> (A) TOOL +Z pre_lift_tool_mm
      -> grip_close
      -> (B) BASE +Z post_lift_base_mm
      -> BASE -Y retract_rel_y_mm
      -> (optional) align retract pose
    """
    gsec = DEFAULT_GRIP_WAIT_SEC if grip_wait_sec is None else float(grip_wait_sec)

    pre_z = DEFAULT_PRE_LIFT_TOOL_MM if pre_lift_tool_mm is None else float(pre_lift_tool_mm)
    pre_v = DEFAULT_PRE_LIFT_TOOL_VEL if pre_lift_tool_vel is None else float(pre_lift_tool_vel)

    post_z = DEFAULT_POST_LIFT_BASE_MM if post_lift_base_mm is None else float(post_lift_base_mm)
    post_v = DEFAULT_POST_LIFT_BASE_VEL if post_lift_base_vel is None else float(post_lift_base_vel)

    ry = DEFAULT_RETRACT_REL_Y_MM if retract_rel_y_mm is None else float(retract_rel_y_mm)
    rvel = DEFAULT_RETRACT_REL_VEL if retract_rel_vel is None else float(retract_rel_vel)

    abort_back = DEFAULT_ABORT_TO_APPROACH if abort_to_approach is None else bool(abort_to_approach)

    if station is None:
        node.get_logger().error("[%s][PICK] station is None" % str(tag))
        return False, {"reason": "station_none"}

    for k in ["approach", "target", "retract"]:
        if k not in station:
            node.get_logger().error("[%s][PICK] station missing key: %s" % (str(tag), str(k)))
            return False, {"reason": "station_key_missing", "key": k}

    if rel_move_tool_fn is None:
        node.get_logger().error("[%s][PICK] rel_move_tool_fn is None" % str(tag))
        return False, {"reason": "rel_move_tool_fn_none"}

    if rel_move_base_fn is None:
        node.get_logger().error("[%s][PICK] rel_move_base_fn is None" % str(tag))
        return False, {"reason": "rel_move_base_fn_none"}

    node.get_logger().info("[%s][PICK] start (pre_probe_close=%s)" % (str(tag), str(bool(pre_probe_close))))

    # 1) approach -> target
    dr.movel(station["approach"], vel=move_vel, acc=move_acc)
    dr.movel(station["target"], vel=move_vel, acc=move_acc)

    # 2) (v2.100) probe 전에 gripper close
    if pre_probe_close:
        node.get_logger().info("[%s][PICK] pre-probe grip_close" % str(tag))
        grip_close_fn(dr, wait_sec=gsec)

    # 3) probe (내부 retract 3mm 포함)
    contacted, traveled, last_force = probe_fn(node, dr, "%s_PICK" % str(tag))
    info = {
        "contacted": bool(contacted),
        "traveled": float(traveled),
        "last_force": last_force,
        "pre_probe_close": bool(pre_probe_close),
    }

    if not contacted:
        node.get_logger().error("[%s][PICK][ABORT] no contact" % str(tag))
        if abort_back:
            dr.movel(station["approach"], vel=move_vel, acc=move_acc)
        return False, info

    if contacted:
        # dr 객체에 해당 함수가 있는지 확인하고 호출 (가장 안전한 방법)
        if hasattr(dr, 'get_current_posx'):
            actual_pos = dr.get_current_posx()
        else:
            # 시뮬레이션 등 함수가 없는 경우를 대비한 기본값 (예: 타겟 좌표 그대로 사용)
            actual_pos = station["target"]
            
        node.get_logger().info(f"📍 현재 위치 확인: {actual_pos}")


        # 4) open (집기 준비)
        grip_open_fn(dr, wait_sec=gsec)

        # 5) (A) 집기 전: TOOL +Z
        if pre_z != 0.0:
            node.get_logger().info("[%s][PICK] pre-lift (TOOL) +Z %.1fmm" % (str(tag), pre_z))
            rel_move_tool_fn(dr, 0, 0, pre_z, 0, 0, 0, pre_v)

        # 6) close (집기)
        grip_close_fn(dr, wait_sec=gsec)

        # 7) (B) 집은 후: BASE +Z
        if post_z != 0.0:
            node.get_logger().info("[%s][PICK] post-lift (BASE) +Z %.1fmm" % (str(tag), post_z))
            # rel_move_base_fn(dr, 0, 0, post_z, 0, 0, 0, post_v)
            # 보정된 퇴출(Retract) 예시
            current_p = dr.get_current_posx()
            # 현재 실제 높이에서 안전하게 50mm 더 들어올림 (절대 좌표 방식 보정)
            retract_pos = [current_p[0], current_p[1], current_p[2] + 50.0, current_p[3], current_p[4], current_p[5]]
            dr.movel(retract_pos, vel=post_v, acc=move_acc)

        # 8) BASE -Y retract
        if ry != 0.0:
            node.get_logger().info("[%s][RETRACT] BASE rel move: Y -%.1fmm" % (str(tag), ry))
            rel_move_base_fn(dr, 0, -ry, 0, 0, 0, 0, rvel)

        # 9) optional align
        if align_to_retract_pose:
            dr.movel(station["retract"], vel=move_vel, acc=move_acc)

        node.get_logger().info("[%s][PICK] success" % str(tag))
        return True, info