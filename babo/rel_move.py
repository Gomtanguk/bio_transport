# rel_move v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 상대이동 래퍼 목적/사용처 주석 추가(기능 변경 없음)

# [모듈 역할]
# - DSR_ROBOT2 movel을 REL(상대이동) 모드로 호출하기 위한 얇은 래퍼
# - 픽/플레이스 후 retract(후퇴) 등에 사용
REL_ACC = 20


def rel_movel_tool(dr, x, y, z, a, b, c, vel):
    required = ["movel", "posx", "DR_TOOL", "DR_MV_MOD_REL"]
    for fn in required:
        if not hasattr(dr, fn):
            raise AttributeError("DSR_ROBOT2 missing API: %s" % fn)

    dr.movel(
        dr.posx(float(x), float(y), float(z), float(a), float(b), float(c)),
        vel=float(vel),
        acc=REL_ACC,
        ref=dr.DR_TOOL,
        mod=dr.DR_MV_MOD_REL
    )

def rel_movel_base(dr, x, y, z, a, b, c, vel):
    required = ["movel", "posx", "DR_BASE", "DR_MV_MOD_REL"]
    for fn in required:
        if not hasattr(dr, fn):
            raise AttributeError("DSR_ROBOT2 missing API: %s" % fn)

    dr.movel(
        dr.posx(float(x), float(y), float(z), float(a), float(b), float(c)),
        vel=float(vel),
        acc=REL_ACC,
        ref=dr.DR_BASE,
        mod=dr.DR_MV_MOD_REL
    )