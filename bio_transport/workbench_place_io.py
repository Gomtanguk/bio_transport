# workbench_place_io v1.000 2026-01-18
# [이번 버전에서 수정된 사항]
# - (기능추가) WORKBENCH 전용 Place 함수 제공
#   - 시퀀스: movel(approach) -> movel(target) -> grip_open
#   - retract(movel)은 수행하지 않음 (후처리 rel_move로 대체)

DEFAULT_GRIP_WAIT_SEC = 1.0


def workbench_place_only(
    node,
    dr,
    wb_station,
    tag,
    grip_open_fn,
    grip_wait_sec=None,
    move_vel=200,
    move_acc=200,
    pre_open=True,
):
    """
    WORKBENCH place 전용 (retract movel 없음)

    wb_station: {"approach": posx, "target": posx, "retract": posx}
    """
    gsec = DEFAULT_GRIP_WAIT_SEC if grip_wait_sec is None else float(grip_wait_sec)

    if wb_station is None:
        node.get_logger().error("[%s][WB_PLACE] station is None" % str(tag))
        return False, {"reason": "station_none"}

    for k in ["approach", "target"]:
        if k not in wb_station:
            node.get_logger().error("[%s][WB_PLACE] station missing key: %s" % (str(tag), str(k)))
            return False, {"reason": "station_key_missing", "key": k}

    node.get_logger().info("[%s][WB_PLACE] start (top-approach, no retract movel)" % str(tag))

    if pre_open:
        grip_open_fn(dr, wait_sec=gsec)

    dr.movel(wb_station["approach"], vel=move_vel, acc=move_acc)
    dr.movel(wb_station["target"], vel=move_vel, acc=move_acc)

    grip_open_fn(dr, wait_sec=gsec)

    node.get_logger().info("[%s][WB_PLACE] success" % str(tag))
    return True, {"pre_open": bool(pre_open), "retract_movel": False}
