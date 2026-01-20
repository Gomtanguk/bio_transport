# rack_place_io v2.200 2026-01-19
# [이번 버전에서 수정된 사항]
# - PLACE 접근을 station["approach"] 기준 BASE Z +top_dz_mm(기본 20mm) 오프셋으로 수행
# - PLACE 시퀀스에 target_zup(target 기준 BASE Z +top_dz_mm) 경유 추가: approach_zup -> target_zup -> target
# - 내려놓은 뒤에도 target_zup로 올라간 뒤 retract_zup로 빠지도록 안전 동작 추가
# - pre_open 기본값 False 유지(이송 중 조기 오픈 방지)

"""[모듈] rack_place_io

[역할]
- 랙/워크벤치 등 station(target)에 내려놓는 Place 시퀀스 수행

[시퀀스]
1) (옵션) pre_open: 시작 시 open (복구/정리용)
2) approach_zup = station["approach"]의 BASE Z +top_dz_mm 로 이동
3) target_zup   = station["target"]  의 BASE Z +top_dz_mm 로 이동
4) target 이동(내려놓기)
5) open
6) target_zup로 다시 상승
7) retract_zup  = station["retract"] 의 BASE Z +top_dz_mm 로 복귀
"""

DEFAULT_GRIP_WAIT_SEC = 1.0
DEFAULT_TOP_DZ_MM = 20.0


def _offset_posx_base_z(dr, pose_posx, dz_mm):
    """posx 포즈에서 BASE Z만 +dz_mm 오프셋한 posx 생성"""
    dz = float(dz_mm)
    x = float(pose_posx[0])
    y = float(pose_posx[1])
    z = float(pose_posx[2])
    rx = float(pose_posx[3])
    ry = float(pose_posx[4])
    rz = float(pose_posx[5])
    return dr.posx(x, y, z + dz, rx, ry, rz)


def rack_place_only(
    node,
    dr,
    station,
    tag,
    grip_open_fn,
    grip_wait_sec=None,
    move_vel=200,
    move_acc=200,
    pre_open=False,
    top_dz_mm=DEFAULT_TOP_DZ_MM,   # ✅ BASE Z +20mm
):
    gsec = DEFAULT_GRIP_WAIT_SEC if grip_wait_sec is None else float(grip_wait_sec)
    dz = float(DEFAULT_TOP_DZ_MM if top_dz_mm is None else top_dz_mm)

    if station is None:
        node.get_logger().error("[%s][PLACE] station is None" % str(tag))
        return False, {"reason": "station_none"}

    for k in ["approach", "target", "retract"]:
        if k not in station:
            node.get_logger().error("[%s][PLACE] station missing key: %s" % (str(tag), str(k)))
            return False, {"reason": "station_key_missing", "key": k}

    node.get_logger().info(
        "[%s][PLACE] start (pre_open=%s, base_z_offset=+%.1fmm)" %
        (str(tag), str(bool(pre_open)), dz)
    )

    # (옵션) 시작 시 open (복구/정리용)
    if pre_open:
        grip_open_fn(dr, wait_sec=gsec)

    # ✅ approach/target/retract 각각을 BASE Z +dz 로 올린 포즈 생성
    approach_zup = _offset_posx_base_z(dr, station["approach"], dz)
    target_zup = _offset_posx_base_z(dr, station["target"], dz)
    retract_zup = _offset_posx_base_z(dr, station["retract"], dz)

    # 1) approach_zup -> 2) target_zup -> 3) target
    dr.movel(approach_zup, vel=move_vel, acc=move_acc)
    dr.movel(target_zup, vel=move_vel, acc=move_acc)
    dr.movel(station["target"], vel=move_vel, acc=move_acc)

    # 4) open
    grip_open_fn(dr, wait_sec=gsec)

    # 5) target_zup로 상승 후 retract_zup로 이탈
    dr.movel(target_zup, vel=move_vel, acc=move_acc)
    dr.movel(retract_zup, vel=move_vel, acc=move_acc)

    node.get_logger().info("[%s][PLACE] success" % str(tag))
    return True, {
        "pre_open": bool(pre_open),
        "top_dz_mm": float(dz),
        "path": "approach_zup->target_zup->target->open->target_zup->retract_zup",
    }
