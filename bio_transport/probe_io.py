# probe_io v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 기능별 주석(모듈 역할/시퀀스) 추가

"""[모듈] probe_io

[역할]
- 순응제어/힘제어 기반 접촉(Probe) 루틴 제공

[핵심 흐름]
1) TOOL 좌표계 설정
2) compliance ON
3) desired force 설정
4) step 이동하며 힘 모니터
5) 접촉 판정 후 retract
6) compliance OFF

[함수]
- probe_contact_for_rack(): 랙/워크벤치 접촉 프로빙
"""

# probe_io v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - 순응제어 + 힘 모니터링 기반 접촉 판정 + 접촉 즉시 백오프(retract)

PROBE_FORCE_N = 10.0
CONTACT_THRESHOLD_N = 8.0
PROBE_STEP_MM = 1.0
PROBE_MAX_MM = 10.0
PROBE_VEL = 10
PROBE_ACC = 20
PROBE_WAIT_SEC = 0.05
PROBE_RETRACT_MM = 3.0


def _log_tool_force(node, f, prefix):
    node.get_logger().info(
        "%s F[N]=(%.2f, %.2f, %.2f)  M[Nm]=(%.2f, %.2f, %.2f)" %
        (prefix, f[0], f[1], f[2], f[3], f[4], f[5])
    )


def probe_contact_for_rack(node, dr, tag,
                           probe_force_n=None,
                           contact_threshold_n=None,
                           probe_step_mm=None,
                           probe_max_mm=None,
                           probe_vel=None,
                           probe_acc=None,
                           probe_wait_sec=None,
                           probe_retract_mm=None):
    pf = PROBE_FORCE_N if probe_force_n is None else float(probe_force_n)
    th = CONTACT_THRESHOLD_N if contact_threshold_n is None else float(contact_threshold_n)
    step = PROBE_STEP_MM if probe_step_mm is None else float(probe_step_mm)
    max_mm = PROBE_MAX_MM if probe_max_mm is None else float(probe_max_mm)
    vel = PROBE_VEL if probe_vel is None else float(probe_vel)
    acc = PROBE_ACC if probe_acc is None else float(probe_acc)
    wsec = PROBE_WAIT_SEC if probe_wait_sec is None else float(probe_wait_sec)
    rmm = PROBE_RETRACT_MM if probe_retract_mm is None else float(probe_retract_mm)

    required = [
        "task_compliance_ctrl", "set_desired_force", "release_compliance_ctrl",
        "get_tool_force", "set_ref_coord", "movel", "posx", "wait",
        "DR_TOOL", "DR_BASE", "DR_FC_MOD_REL", "DR_MV_MOD_REL"
    ]
    for fn in required:
        if not hasattr(dr, fn):
            node.get_logger().error("[%s] DSR_ROBOT2 missing API: %s" % (tag, fn))
            return False, 0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    contacted = False
    traveled = 0.0
    last_force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    try:
        dr.set_ref_coord(dr.DR_TOOL)
        dr.task_compliance_ctrl([2000, 2000, 200, 300, 300, 300], time=0.0)
        dr.set_desired_force([0, 0, pf, 0, 0, 0], [0, 0, 1, 0, 0, 0], time=1, mod=dr.DR_FC_MOD_REL)

        last_force = dr.get_tool_force(dr.DR_TOOL)
        _log_tool_force(node, last_force, "[%s][probe][start]" % tag)

        while traveled < max_mm:
            dr.movel(
                dr.posx(0, 0, step, 0, 0, 0),
                vel=vel,
                acc=acc,
                ref=dr.DR_TOOL,
                mod=dr.DR_MV_MOD_REL
            )
            traveled += step
            dr.wait(wsec)

            last_force = dr.get_tool_force(dr.DR_TOOL)
            _log_tool_force(node, last_force, "[%s][probe][%.1fmm]" % (tag, traveled))

            if abs(last_force[2]) <= th:
                contacted = True

                if rmm > 0.0:
                    try:
                        dr.movel(
                            dr.posx(0, 0, -rmm, 0, 0, 0),
                            vel=vel,
                            acc=acc,
                            ref=dr.DR_TOOL,
                            mod=dr.DR_MV_MOD_REL
                        )
                        node.get_logger().info("[%s][probe][retract] %.2fmm" % (tag, rmm))
                    except Exception as e:
                        node.get_logger().warn("[%s][probe][retract][skip] %s" % (tag, str(e)))
                break

    finally:
        try:
            dr.release_compliance_ctrl()
        except Exception:
            pass
        try:
            dr.set_ref_coord(dr.DR_BASE)
        except Exception:
            pass

    node.get_logger().info("[%s][probe][result] contacted=%s traveled=%.1fmm Fz=%.2fN" %
                           (tag, str(contacted), traveled, last_force[2]))
    return contacted, traveled, last_force
