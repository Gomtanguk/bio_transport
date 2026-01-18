# rack_place_io v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 기능별 주석(모듈 역할/시퀀스) 추가

"""[모듈] rack_place_io

[역할]
- 랙/워크벤치에 Place 동작 수행을 위한 IO 시퀀스 제공

[일반 흐름]
1) approach 이동
2) target 이동
3) release/open
4) 안전 리트랙트
"""

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
