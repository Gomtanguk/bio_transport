# rel_move v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - Tool 기준 상대이동 래퍼(vel 인자만 외부에서 받고 acc는 모듈 상수)

REL_ACC = 20


def rel_movel_xyzabc(dr, x, y, z, a, b, c, vel):
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
