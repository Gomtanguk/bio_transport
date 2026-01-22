# tube_place_io.py v1.000 2026-01-21
"""[모듈] tube_place_io
[역할] 튜브 전용 Place 동작 (Target 상공 -> Target 삽입 -> Open -> 이탈)
"""

DEFAULT_TOP_DZ_MM = 40.0 # 꽂기 전 대기 높이

def tube_place_only(
    node, dr, station, tag,
    grip_open_fn,
    grip_wait_sec=0.5,
    top_dz_mm=None
):
    dz = DEFAULT_TOP_DZ_MM if top_dz_mm is None else float(top_dz_mm)
    
    tgt = station["target"]
    # Target 바로 위 (Z + dz) 좌표 생성
    tgt_zup = dr.posx(tgt[0], tgt[1], tgt[2] + dz, tgt[3], tgt[4], tgt[5])

    node.get_logger().info(f"[{tag}] START Tube Place")

    # 1. Approach -> Target 상공
    dr.movel(station["approach"], vel=100, acc=100)
    dr.movel(tgt_zup, vel=80, acc=80)

    # 2. 삽입 (천천히)
    dr.movel(tgt, vel=30, acc=30)

    # 3. 놓기
    grip_open_fn(dr, wait_sec=grip_wait_sec)

    # 4. 상공으로 빠지기 -> Retract
    dr.movel(tgt_zup, vel=60, acc=60)
    dr.movel(station["retract"], vel=100, acc=100)

    node.get_logger().info(f"[{tag}] SUCCESS")
    return True, {}