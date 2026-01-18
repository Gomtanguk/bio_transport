# rack_pick_io v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - Pick(랙에서 빼기) 동작만 모듈화

DEFAULT_GRIP_WAIT_SEC = 1.0
DEFAULT_LIFT_MM = 20.0
DEFAULT_LIFT_VEL = 20.0
DEFAULT_ABORT_TO_APPROACH = True


def rack_pick_only(node, dr, station, tag,
                   probe_fn,
                   grip_open_fn,
                   grip_close_fn,
                   rel_move_fn,
                   grip_wait_sec=None,
                   lift_mm=None,
                   lift_vel=None,
                   abort_to_approach=None,
                   move_vel=200,
                   move_acc=200):
    gsec = DEFAULT_GRIP_WAIT_SEC if grip_wait_sec is None else float(grip_wait_sec)
    lmm = DEFAULT_LIFT_MM if lift_mm is None else float(lift_mm)
    lvel = DEFAULT_LIFT_VEL if lift_vel is None else float(lift_vel)
    abort_back = DEFAULT_ABORT_TO_APPROACH if abort_to_approach is None else bool(abort_to_approach)

    if station is None:
        node.get_logger().error("[%s][PICK] station is None" % str(tag))
        return False, {"reason": "station_none"}

    for k in ["approach", "target", "retract"]:
        if k not in station:
            node.get_logger().error("[%s][PICK] station missing key: %s" % (str(tag), str(k)))
            return False, {"reason": "station_key_missing", "key": k}

    node.get_logger().info("[%s][PICK] start" % str(tag))

    dr.movel(station["approach"], vel=move_vel, acc=move_acc)
    dr.movel(station["target"], vel=move_vel, acc=move_acc)

    contacted, traveled, last_force = probe_fn(node, dr, "%s_PICK" % str(tag))
    info = {"contacted": bool(contacted), "traveled": float(traveled), "last_force": last_force}

    if not contacted:
        node.get_logger().error("[%s][PICK][ABORT] no contact" % str(tag))
        if abort_back:
            dr.movel(station["approach"], vel=move_vel, acc=move_acc)
        return False, info

    grip_open_fn(dr, wait_sec=gsec)

    if lmm > 0.0:
        rel_move_fn(dr, 0, 0, lmm, 0, 0, 0, lvel)

    grip_close_fn(dr, wait_sec=gsec)

    dr.movel(station["retract"], vel=move_vel, acc=move_acc)

    node.get_logger().info("[%s][PICK] success" % str(tag))
    return True, info
