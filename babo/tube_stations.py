# tube_stations.py v4.000 2026-01-22
# [수정] TUBE_LOADING_ZONE 추가 및 워크벤치 상대 좌표 계산 로직 추가

"""[모듈] tube_stations
"""

# =========================================================
# [1] 튜브 픽(Pick) 고정 위치 (절대값 변수)
# =========================================================
# 로봇이 새로운 튜브를 집어오는 고정 위치 (예: 공급 장치)
TUBE_LOADING_ZONE = {
    "target":   [300.0, 300.0, 150.0, 0.0, 180.0, 0.0], # 튜브 잡는 위치
    "approach": [300.0, 300.0, 350.0, 0.0, 180.0, 0.0], # Z + 200
    "retract":  [300.0, 300.0, 350.0, 0.0, 180.0, 0.0], # Z + 200
}

# =========================================================
# [2] 랙 파라미터 및 워크벤치 설정
# =========================================================
RX, RY, RZ = 0.0, 180.0, 0.0
BASE_Z     = 150.0 

# 워크벤치에서 랙이 놓이는 원점 (랙 핸들 기준 등)
# **중요**: 랙이 워크벤치에 놓였을 때, 랙의 1번(또는 기준) 구멍 위치
WB_RACK_ORIGIN_X = 500.0
WB_RACK_ORIGIN_Y = 0.0
WB_RACK_Z = 150.0

# 튜브 구멍 간격
PITCH_X = 25.0
PITCH_Y = 25.0

def get_tube_loading_station(dr):
    """튜브 공급 위치(고정) 반환"""
    return {
        "target": dr.posx(*TUBE_LOADING_ZONE["target"]),
        "approach": dr.posx(*TUBE_LOADING_ZONE["approach"]),
        "retract": dr.posx(*TUBE_LOADING_ZONE["retract"])
    }

def get_workbench_slot_station(dr, row_idx, col_idx, approach_z_offset=200.0):
    """
    [워크벤치 위에 있는 랙]의 특정 구멍(Slot) 좌표 계산
    Rack이 Outbound 되어 WB에 있을 때 사용
    """
    # 워크벤치 원점 기준 오프셋 계산 (랙 구조에 따라 방향 수정 필요)
    # 예: X는 열(Col) 증가, Y는 행(Row) 감소 방향
    x = WB_RACK_ORIGIN_X + (col_idx * PITCH_X)
    y = WB_RACK_ORIGIN_Y - (row_idx * PITCH_Y)
    z = WB_RACK_Z
    
    target = dr.posx(x, y, z, RX, RY, RZ)
    
    # Approach/Retract: BASE Z + 200 (사용자 요청)
    app_ret = dr.posx(x, y, z + approach_z_offset, RX, RY, RZ)
    
    return {
        "target": target,
        "approach": app_ret,
        "retract": app_ret
    }