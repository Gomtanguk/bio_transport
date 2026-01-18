# rack_place_io v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - Place(랙에 놓기) 동작만 모듈화

DEFAULT_GRIP_WAIT_SEC = 1.0


def rack_place_only(node, dr, station, tag,
                    grip_open_fn,
                    grip_wait_sec=None,
                    move_vel=200,
                    move_acc=200,
                    pre_open=True):
    gsec = DEFAULT_GRIP_WAIT_SEC if grip_wait_sec is None else float(grip_wait_sec)

    if station is None:
        node.get_logger().error("[%s][PLACE] station is None" % str(tag))
        return False, {"reason": "station_none"}

    for k in ["approach", "target", "retract"]:
        if k not in station:
            node.get_logger().error("[%s][PLACE] station missing key: %s" % (str(tag), str(k)))
            return False, {"reason": "station_key_missing", "key": k}

    node.get_logger().info("[%s][PLACE] start" % str(tag))

    if pre_open:
        grip_open_fn(dr, wait_sec=gsec)

    dr.movel(station["approach"], vel=move_vel, acc=move_acc)
    dr.movel(station["target"], vel=move_vel, acc=move_acc)

    grip_open_fn(dr, wait_sec=gsec)

    dr.movel(station["retract"], vel=move_vel, acc=move_acc)

    node.get_logger().info("[%s][PLACE] success" % str(tag))
    return True, {"pre_open": bool(pre_open)}
