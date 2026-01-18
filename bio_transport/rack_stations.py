# rack_stations v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - 랙 A~F 타겟 좌표 단일 관리 + station(approach/target/retract) 생성

DEFAULT_APPROACH_DY = -100.0

RACK_TARGETS = {
    "A": (396.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "B": (446.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "C": (496.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "D": (619.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "E": (669.810, 407.870, 216.970, 90.0, 90.0, 90.0),
    "F": (719.810, 407.870, 216.970, 90.0, 90.0, 90.0),
}


def build_rack_stations(dr, approach_dy=None):
    dy = DEFAULT_APPROACH_DY if approach_dy is None else float(approach_dy)

    def to_posx(vals):
        return dr.posx(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])

    def mk_station(target_posx):
        x, y, z = target_posx[0], target_posx[1], target_posx[2]
        rx, ry, rz = target_posx[3], target_posx[4], target_posx[5]
        approach = dr.posx(x, y + dy, z, rx, ry, rz)
        retract = approach
        return {"approach": approach, "target": target_posx, "retract": retract}

    stations = {}
    for k in RACK_TARGETS:
        t = to_posx(RACK_TARGETS[k])
        stations[k] = mk_station(t)

    return stations
