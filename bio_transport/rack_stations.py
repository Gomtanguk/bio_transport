# rack_stations v3.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - WORKBENCH_A / WORKBENCH_B / WORKBENCH_C-1 / WORKBENCH_C-2 타겟(티칭값) 추가
# - build_rack_stations()가 랙 스테이션 + 워크벤치 스테이션을 함께 생성하도록 확장
# - get_all_station_keys() 추가 (서버/노드에서 유효 키 검증용)
# - 기존 build_workbench_station_* API는 유지하되, key 인자를 받을 수 있게 확장

"""[모듈] rack_stations

[역할]
- 랙/워크벤치 스테이션(approach/target/retract) 좌표 정의 및 생성
- UI/노드에서 동일 키(A-1 등)로 참조하도록 중앙화

[포인트]
- build_* 함수가 실제 posx 생성 책임
- RACK_TARGETS/WORKBENCH_TARGETS는 유효 키 검증에도 사용
"""

DEFAULT_APPROACH_DY = -100.0

# WORKBENCH 접근: 위에서 접근(Z +)
WORKBENCH_APPROACH_DZ = 100.0

# WORKBENCH 접근: 옆에서 접근(Y -)
WORKBENCH_APPROACH_DY = -50.0

# =========================
# ✅ 랙 타겟(teach)
# =========================
RACK_TARGETS = {
    "A-1": (396.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "A-2": (446.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "A-3": (496.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "B-1": (619.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "B-2": (669.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "B-3": (719.810, 407.870, 216.970, 90.0, 90.0, 90.0),
}

# =========================
# ✅ WORKBENCH 타겟(teach) - 사용자 제공
# =========================
WORKBENCH_TARGETS = {
    "WORKBENCH_A":   (400.0, 0.0, 100.0, 90.0,  90.0, 90.0),
    "WORKBENCH_B":   (450.0, 0.0, 100.0, 90.0,  90.0, 90.0),
    "WORKBENCH_C-1": (500.0, 0.0,  80.0, 90.0, 180.0, 90.0),  # 튜브 입고 WB
    "WORKBENCH_C-2": (500.0, 50.0, 80.0, 90.0, 180.0, 90.0),  # 튜브 출고 WB
}


def get_all_station_keys():
    """랙 + 워크벤치 전체 키 반환"""
    keys = list(RACK_TARGETS.keys()) + list(WORKBENCH_TARGETS.keys())
    keys = sorted(list(set(keys)))
    return keys


def _to_posx(dr, vals):
    return dr.posx(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])


def _mk_station_from_target_dy(dr, target_posx, approach_dy):
    """Y 오프셋 접근 station"""
    x, y, z = target_posx[0], target_posx[1], target_posx[2]
    rx, ry, rz = target_posx[3], target_posx[4], target_posx[5]
    approach = dr.posx(x, y + float(approach_dy), z, rx, ry, rz)
    retract = approach
    return {"approach": approach, "target": target_posx, "retract": retract}


def _mk_station_from_target_dz(dr, target_posx, approach_dz):
    """Z 오프셋 접근 station"""
    x, y, z = target_posx[0], target_posx[1], target_posx[2]
    rx, ry, rz = target_posx[3], target_posx[4], target_posx[5]
    approach = dr.posx(x, y, z + float(approach_dz), rx, ry, rz)
    retract = approach
    return {"approach": approach, "target": target_posx, "retract": retract}


def build_workbench_station_top(dr, key="WORKBENCH_A", approach_dz=None):
    """WORKBENCH station: 위에서 접근 (Z+)"""
    dz = WORKBENCH_APPROACH_DZ if approach_dz is None else float(approach_dz)

    if key not in WORKBENCH_TARGETS:
        raise KeyError("Unknown workbench key: %s" % str(key))

    t = _to_posx(dr, WORKBENCH_TARGETS[key])
    return _mk_station_from_target_dz(dr, t, dz)


def build_workbench_station_dy(dr, key="WORKBENCH_A", approach_dy=None):
    """WORKBENCH station: 옆에서 접근 (Y -)"""
    dy = WORKBENCH_APPROACH_DY if approach_dy is None else float(approach_dy)

    if key not in WORKBENCH_TARGETS:
        raise KeyError("Unknown workbench key: %s" % str(key))

    t = _to_posx(dr, WORKBENCH_TARGETS[key])
    return _mk_station_from_target_dy(dr, t, dy)


def build_rack_stations(dr, approach_dy=None, include_workbench=True, workbench_mode="top"):
    """
    랙 + (선택) 워크벤치 station dict 생성

    - rack: approach = Y + approach_dy
    - workbench:
        - mode="top": approach = Z + WORKBENCH_APPROACH_DZ
        - mode="dy" : approach = Y + WORKBENCH_APPROACH_DY
    """
    dy = DEFAULT_APPROACH_DY if approach_dy is None else float(approach_dy)

    stations = {}

    # ---- Rack stations ----
    for k in RACK_TARGETS:
        t = _to_posx(dr, RACK_TARGETS[k])
        stations[k] = _mk_station_from_target_dy(dr, t, dy)

    # ---- Workbench stations ----
    if include_workbench:
        mode = str(workbench_mode).lower().strip()
        for k in WORKBENCH_TARGETS:
            if mode == "dy":
                stations[k] = build_workbench_station_dy(dr, key=k, approach_dy=None)
            else:
                stations[k] = build_workbench_station_top(dr, key=k, approach_dz=None)

    return stations
