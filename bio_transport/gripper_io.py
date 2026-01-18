# gripper_io v2.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - v2.000 기준 헤더 포맷 통일
# - 기능별 주석(모듈 역할/시퀀스) 추가

"""[모듈] gripper_io

[역할]
- DO 기반 그리퍼 OPEN/CLOSE/초기 OPEN 제공

[함수]
- grip_open(): 그리퍼 열기
- grip_close(): 그리퍼 닫기
- grip_init_open(): 노드 시작 시 안전 상태(OPEN)
"""

# gripper_io v1.000
# [이번 버전에서 수정된 사항]
# - v1.000 기준선(Baseline) 설정
# - 그리퍼 DO open/close 및 노드 시작 시 OPEN 강제 세팅 함수 제공

def grip_open(dr, do1=1, do2=2, wait_sec=1.0):
    """Gripper OPEN: DO1=OFF, DO2=ON"""
    dr.set_digital_output(do1, dr.OFF)
    dr.set_digital_output(do2, dr.ON)
    dr.wait(wait_sec)


def grip_close(dr, do1=1, do2=2, wait_sec=1.0):
    """Gripper CLOSE: DO1=ON, DO2=OFF"""
    dr.set_digital_output(do1, dr.ON)
    dr.set_digital_output(do2, dr.OFF)
    dr.wait(wait_sec)


def grip_init_open(dr, do1=1, do2=2, wait_sec=0.2):
    """
    노드 시작 시 안전 상태 확보용: OPEN 강제
    - OPEN: DO1=OFF, DO2=ON
    """
    dr.set_digital_output(do1, dr.OFF)
    dr.set_digital_output(do2, dr.ON)
    dr.wait(wait_sec)
