# workbench_place_io v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - WORKBENCH place 시퀀스 설명 주석 강화(기능 변경 없음)

# [모듈 역할]
# - WORKBENCH 전용 Place 시퀀스 함수 제공
# - 시퀀스: (옵션)pre_open -> movel(approach) -> movel(target) -> grip_open
# - retract(movel)은 수행하지 않음(후처리는 rel_move 사용)
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
