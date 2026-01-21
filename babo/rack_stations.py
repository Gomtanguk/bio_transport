# rack_stations v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 기능별 주석(모듈 역할/시퀀스) 추가

"""[모듈] rack_stations

[역할]
- 랙/워크벤치 스테이션(approach/target) 좌표 정의 및 생성
- UI/노드에서 동일 키(A-1 등)로 참조하도록 중앙화

[포인트]
- build_* 함수가 실제 posx/posj 생성 책임
- RACK_TARGETS는 유효 키 검증에도 사용
"""

DEFAULT_APPROACH_DY = -250.0

# WORKBENCH 접근: 위에서 접근(Z +)
WORKBENCH_APPROACH_DZ = 150.0

# WORKBENCH 접근: 옆에서 접근(Y -)
WORKBENCH_APPROACH_DY = -50.0

# ✅ 랙 타겟(teach)
RACK_TARGETS = {
    "A-1": (280.0, 120.000, 265.000, 90.0, 90.0, 90.0),
    "A-2": (339.0, 120.000, 265.000, 90.0, 90.0, 90.0),
    "A-3": (452.0, 120.000, 265.000, 90.0, 90.0, 90.0),
    "B-1": (586.0, 120.000, 265.000, 90.0, 90.0, 90.0),
    "B-2": (699.0, 120.000, 265.000, 90.0, 90.0, 90.0),
    "B-3": (720.0, 120.000, 265.000, 90.0, 90.0, 90.0),
}

# ✅ WORKBENCH target(teach) - 사용자 제공
WORKBENCH_TARGET = (282.350, -372.0, 113.0, 90.0, 90.0, 90.0)


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


def build_rack_stations(dr, approach_dy=None):
    """랙 A-1~B-3 station dict 생성 (approach: Y + approach_dy)"""
    dy = DEFAULT_APPROACH_DY if approach_dy is None else float(approach_dy)
    stations = {}
    for k in RACK_TARGETS:
        t = _to_posx(dr, RACK_TARGETS[k])
        stations[k] = _mk_station_from_target_dy(dr, t, dy)
    return stations


def build_workbench_station_top(dr, approach_dz=None):
    """WORKBENCH station: 위에서 접근 (Z+)"""
    dz = WORKBENCH_APPROACH_DZ if approach_dz is None else float(approach_dz)
    t = _to_posx(dr, WORKBENCH_TARGET)
    return _mk_station_from_target_dz(dr, t, dz)


def build_workbench_station_dy(dr, approach_dy=None):
    """WORKBENCH station: 옆에서 접근 (Y -50)"""
    dy = WORKBENCH_APPROACH_DY if approach_dy is None else float(approach_dy)
    t = _to_posx(dr, WORKBENCH_TARGET)
    return _mk_station_from_target_dy(dr, t, dy)