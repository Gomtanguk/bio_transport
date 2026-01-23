# main_integrated v2.200 2026-01-22
# [이번 버전에서 수정된 사항]
# - (기능추가) TubeTransport Action(tube_main_control)을 수신해 로봇(/tube_transport)으로 중계 + 피드백(stage/progress/detail) 전달
# - (기능변경) Rack(BioCommand)과 Tube(TubeTransport)를 동일 asyncio.Lock으로 직렬화(1대 로봇 보호)
# - (유지) QoS는 코드 설정값(ACTION_QOS: RELIABLE/VOLATILE/KEEP_LAST depth=5) 그대로 사용

"""[모듈] main_integrated

[역할]
- UI(Action: BioCommand, bio_main_control)로부터 Rack 명령을 받아,
  하위 로봇 Action(RobotMove, /robot_action)으로 전달하고 결과를 UI에 반환한다.
- UI(Action: TubeTransport, tube_main_control)로부터 Tube 이동 명령을 받아,
  하위 로봇 Action(TubeTransport, /tube_transport)으로 전달하고 결과/피드백을 UI에 중계한다.
"""

from __future__ import annotations

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from biobank_interfaces.action import BioCommand, RobotMove, TubeTransport
except ImportError:
    class BioCommand:  # pragma: no cover
        class Goal: command = ""
        class Result:
            def __init__(self, success=True, message=""):
                self.success = success
                self.message = message
        class Feedback:
            def __init__(self, status=""):
                self.status = status

    class RobotMove:  # pragma: no cover
        class Goal: command = ""
        class Result:
            success = True
            message = ""
        class Feedback:
            status = ""

    class TubeTransport:  # pragma: no cover
        class Goal:
            job_id = ""
            pick_posx = [0.0] * 6
            place_posx = [0.0] * 6
        class Result:
            success = True
            error_code = ""
            message = ""
        class Feedback:
            stage = ""
            progress = 0.0
            detail = ""


ACTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# =========================
# TUBE 명령 파싱 (기존 유지)
# =========================
def parse_command(cmd: str):
    # 좌표 기준(현재 코드에선 사용 안하지만 기존 유지)
    ORIGIN_POINT = [367.32, 6.58, 422.710, 103.18, 179.97, 103.14]

    A_OUT_1 = [300.11, -24.86, 421.12, 120.22, -179.78, 120.22]
    A_OUT_2 = [300.98, 13.85, 420.48, 156.15, -179.77, 155.93]
    A_OUT_3 = [302.63, 51.61, 419.08, 9.89, 179.71, 9.69]
    A_OUT_4 = [301.87, 87.68, 418.39, 20.96, 179.69, 20.63]

    B_OUT_1 = [299.76, -30.52, 416.24, 159.74, -179.66, 159.87]
    B_OUT_2 = [301.22, 3.92, 417.98, 2.79, 179.42, 3.06]
    B_OUT_3 = [299.42, 40.17, 418.31, 18.42, 179.13, 18.74]
    B_OUT_4 = [300.03, 80.63, 417.88, 16.66, 179.08, 17.21]

    OUT_1 = [627.11, -154.34, 414.82, 116.42, 180.0, 116.05]
    OUT_2 = [632.19, -116.61, 411.86, 169.15, 179.67, 168.46]
    OUT_3 = [634.42, -75.46, 411.88, 173.08, 179.62, 172.65]
    OUT_4 = [634.45, -39.53, 403.94, 165.87, -179.97, 165.84]

    A_IN_1 = [300, -24.86, 540, 120, 180, 120]
    A_IN_2 = [300, 13.85, 540, 156, 180, 156]
    A_IN_3 = [300, 51.61, 540, 10, 180, 10]
    A_IN_4 = [300, 87.68, 540, 21, 180, 21]

    B_IN_1 = [300, -30.52, 540, 160, 180, 160]
    B_IN_2 = [300, 3.92, 540, 3, 180, 3]
    B_IN_3 = [300, 40.17, 540, 18, 180, 18]
    B_IN_4 = [300, 80.63, 540, 17, 180, 17]

    IN_1 = [624.18, -154.70, 359.04, 2.33, 178.99, 2.90]
    IN_2 = [626.52, -116.78, 358.43, 5.68, 179.09, 6.08]
    IN_3 = [628.10, -81.45, 355.87, 12.00, 179.23, 12.20]
    IN_4 = [629.05, -42.82, 351.11, 18.24, 179.32, 18.48]

    RACK_OUT_POINTS = {
        "A": {1: A_OUT_1, 2: A_OUT_2, 3: A_OUT_3, 4: A_OUT_4},
        "B": {1: B_OUT_1, 2: B_OUT_2, 3: B_OUT_3, 4: B_OUT_4},
    }
    RACK_IN_POINTS = {
        "A": {1: A_IN_1, 2: A_IN_2, 3: A_IN_3, 4: A_IN_4},
        "B": {1: B_IN_1, 2: B_IN_2, 3: B_IN_3, 4: B_IN_4},
    }

    OUT_POINTS = {1: OUT_1, 2: OUT_2, 3: OUT_3, 4: OUT_4}
    IN_POINTS = {1: IN_1, 2: IN_2, 3: IN_3, 4: IN_4}

    # 1) 콤마 분해 + 공백 제거
    parts = [p.strip() for p in str(cmd).split(",")]
    if len(parts) < 4:
        raise ValueError("Invalid command format (need at least 4 comma-separated fields)")

    cmd_type = parts[0].upper()

    # 2) IN / OUT 파싱 (입고/출고 허용)
    mode_str = parts[1].upper()
    if mode_str in ("IN", "입고"):
        mode = "IN"
    elif mode_str in ("OUT", "출고"):
        mode = "OUT"
    else:
        raise ValueError(f"Unknown mode: {mode_str}")

    # 3) 위치 문자열 결정 (요구사항 반영: mode별 고정)
    if mode == "OUT":
        loc_str = parts[2]
    else:
        loc_str = parts[3]

    if loc_str.upper() == "NONE":
        raise ValueError(f"Location is NONE for mode={mode}. Expected a rack location like A-2-1")

    loc_parts = [p.strip() for p in loc_str.split("-")]
    if len(loc_parts) != 3:
        raise ValueError(f"Invalid location format: {loc_str} (expected like A-2-1)")

    # 4) 랙/슬롯 파싱
    rack_letter = loc_parts[0].upper()
    slot = int(loc_parts[2])

    if rack_letter not in ("A", "B"):
        raise ValueError("Rack must be A or B")
    if slot not in (1, 2, 3, 4):
        raise ValueError("Slot must be 1~4")

    # 5) IN/OUT 소스/목적 결정
    if mode == "IN":
        pick_pose = IN_POINTS[1]  # IN_1 고정
        place_pose = RACK_IN_POINTS[rack_letter][slot]
    else:
        pick_pose = RACK_OUT_POINTS[rack_letter][slot]
        place_pose = OUT_POINTS[1]  # OUT_1 고정

    return cmd_type, pick_pose, place_pose

class MainIntegrated(Node):
    def __init__(self):
        super().__init__("main_orchestrator")
        self.callback_group = ReentrantCallbackGroup()
        self._robot_lock = asyncio.Lock()

        # Rack server
        self._rack_server = ActionServer(
            self,
            BioCommand,
            "bio_main_control",
            execute_callback=self.handle_rack_command,
            callback_group=self.callback_group,
            goal_callback=self.rack_goal_callback,
            cancel_callback=self.rack_cancel_callback,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_pub_qos_profile=ACTION_QOS,
            status_pub_qos_profile=ACTION_QOS,
        )

        # Rack client -> /robot_action
        self.robot_client = ActionClient(
            self,
            RobotMove,
            "robot_action",
            callback_group=self.callback_group,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_sub_qos_profile=ACTION_QOS,
            status_sub_qos_profile=ACTION_QOS,
        )

        # Tube server (UI -> main)
        self._tube_server = ActionServer(
            self,
            BioCommand,
            "tube_main_control",
            execute_callback=self.handle_ui_command,
            callback_group=self.callback_group,
            goal_callback=self.tube_goal_callback,
            cancel_callback=self.tube_cancel_callback,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_pub_qos_profile=ACTION_QOS,
            status_pub_qos_profile=ACTION_QOS,
        )

        # Tube client (main -> robot)
        self.tube_client = ActionClient(
            self,
            TubeTransport,
            "/tube_transport",
            callback_group=self.callback_group,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_sub_qos_profile=ACTION_QOS,
            status_sub_qos_profile=ACTION_QOS,
        )

        self.get_logger().info("🧠 [Integrated] main_integrated ready (Rack+Tube).")

    # ---------------- Rack ----------------
    def rack_goal_callback(self, goal_request: BioCommand.Goal):
        self.get_logger().info(f"📩 [Rack] Goal: {getattr(goal_request, 'command', '')}")
        return GoalResponse.ACCEPT

    def _make_rack_pull_return_cmd(self, raw_cmd: str):
        """
        raw_cmd 예) "TUBE,IN,NONE,A-2-1"
        반환) (pull_cmd, return_cmd) -> call_robot로 보낼 문자열

        기본 정책:
        - IN    : dst 슬롯의 랙을 OUT으로 꺼내고 작업 후 IN으로 복귀
        - OUT   : src 슬롯의 랙을 OUT으로 꺼내고 작업 후 IN으로 복귀
        - MOVE  : src 기준(필요하면 추후 dst도 확장)
        - WASTE : src 기준
        """
        parts = [p.strip() for p in str(raw_cmd).split(",") if p.strip()]
        if len(parts) < 4:
            raise ValueError(f"Invalid TUBE cmd (need 4 fields): {raw_cmd}")

        mode = parts[1].upper()
        src = parts[2].upper()
        dst = parts[3].upper()

        # IN이면 dst에서, 그 외는 src에서 랙을 결정
        loc = dst if mode == "IN" else src
        if not loc or loc == "NONE":
            # 랙을 못 정하면 랙 동작 스킵(원하면 여기서 에러로 바꿔도 됨)
            return "", ""

        # loc: "A-2-1" -> rack_id: "A-2"
        loc = loc.replace("_", "-")
        toks = [t for t in loc.split("-") if t]
        if len(toks) < 2:
            raise ValueError(f"Invalid slot format for rack resolve: {loc}")

        rack_id = f"{toks[0].upper()}-{toks[1]}"

        pull_cmd = f"OUT,{rack_id},NONE"
        return_cmd = f"IN,NONE,{rack_id}"
        return pull_cmd, return_cmd


    def rack_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 [Rack] Cancel")
        return CancelResponse.ACCEPT

    async def handle_rack_command(self, goal_handle):
        raw_cmd = goal_handle.request.command

        # "RACK,IN,NONE,A-2" -> "IN,NONE,A-2"
        try:
            parts = [p.strip() for p in str(raw_cmd).split(",")]
            sub_cmd = ",".join(parts[1:]) if len(parts) >= 2 else str(raw_cmd)
        except Exception:
            sub_cmd = str(raw_cmd)

        if self._robot_lock.locked():
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"대기열: 다른 작업 실행 중 ({sub_cmd})"))
            except Exception:
                pass

        async with self._robot_lock:
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"실행 중: {sub_cmd}"))
            except Exception:
                pass
            success, msg = await self.call_robot(sub_cmd)

        goal_handle.succeed() if success else goal_handle.abort()
        return BioCommand.Result(success=success, message=msg)



    # ---------------- Tube ----------------
    def tube_goal_callback(self, goal_request: TubeTransport.Goal):
        self.get_logger().info(f"📩 [Tube] Goal: job_id={getattr(goal_request, 'job_id', '')}")
        return GoalResponse.ACCEPT

    def tube_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 [Tube] Cancel")
        return CancelResponse.ACCEPT
    
    async def call_robot(self, cmd_str: str):
        cmd_str = str(cmd_str).strip()
        if not cmd_str:
            return False, "robot_action으로 보낼 cmd가 비어 있습니다."

        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            return False, "하위 로봇 Action(/robot_action) 서버 연결 실패"

        goal = RobotMove.Goal()
        goal.command = cmd_str

        gh = await self.robot_client.send_goal_async(goal)
        if not gh.accepted:
            return False, "하위 로봇 Action Goal 거절됨"

        result = await gh.get_result_async()
        return bool(result.result.success), str(result.result.message)

    # --------------------------
    # execute
    # --------------------------
    async def handle_ui_command(self, goal_handle):
        raw_cmd = str(goal_handle.request.command).strip()
        self.get_logger().info(f"📥 명령 실행 시작: {raw_cmd}")

        if self._robot_lock.locked():
            try:
                goal_handle.publish_feedback(
                    BioCommand.Feedback(status=f"대기열: 다른 작업 실행 중 ({raw_cmd})")
                )
            except Exception:
                pass

        async with self._robot_lock:
            # cmd_type 추출 (버그 수정: parts0/parts 혼용 제거)
            parts = [p.strip() for p in raw_cmd.split(",")]
            cmd_type = parts[0].upper() if parts else ""

            success = False
            msg = ""

            if cmd_type == "TUBE":
                # 1) TUBE 파싱
                try:
                    _, pick_pose, place_pose = parse_command(raw_cmd)
                except Exception as e:
                    msg = f"명령 파싱 실패(TUBE): {e}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                try:
                    goal_handle.publish_feedback(BioCommand.Feedback(status="실행 중: TUBE (RACK->TUBE->RACK)"))
                except Exception:
                    pass

                pull_cmd, return_cmd = self._make_rack_pull_return_cmd(raw_cmd)

                # 2) 랙 빼기
                ok_pull, pull_msg = await self.call_robot(pull_cmd)
                if not ok_pull:
                    msg = f"랙 빼기 실패: {pull_msg}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                # 3) 튜브 이송
                ok_tube, err_code, tube_msg = await self.call_tube_transport(cmd_type, pick_pose, place_pose)
                if not ok_tube:
                    # 실패여도 랙 복귀는 시도하는 정책(안전)
                    self.get_logger().error(f"튜브 이송 실패: {tube_msg} (error_code={err_code})")

                    ok_ret, ret_msg = await self.call_robot(return_cmd)
                    if not ok_ret:
                        msg = f"튜브 이송 실패({err_code}): {tube_msg} / 랙 복귀도 실패: {ret_msg}"
                        self.get_logger().error(msg)
                        goal_handle.abort()
                        return BioCommand.Result(success=False, message=msg)

                    msg = f"튜브 이송 실패({err_code}): {tube_msg} (랙은 복귀 완료)"
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                # 4) 랙 원위치
                ok_ret, ret_msg = await self.call_robot(return_cmd)
                if not ok_ret:
                    msg = f"랙 원위치 실패: {ret_msg}"
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return BioCommand.Result(success=False, message=msg)

                success = True
                msg = "TUBE 작업 완료"

            elif cmd_type == "RACK":
                # 기존대로 prefix 제거해 robot_action에 전달
                try:
                    sub_cmd = ",".join(parts[1:]) if len(parts) >= 2 else raw_cmd
                except Exception:
                    sub_cmd = raw_cmd

                try:
                    goal_handle.publish_feedback(BioCommand.Feedback(status="실행 중: RACK Move"))
                except Exception:
                    pass

                success, msg = await self.call_robot(sub_cmd)

            else:
                msg = f"지원하지 않는 cmd_type: '{cmd_type}' (TUBE 또는 RACK만 처리)"
                self.get_logger().warn(msg)
                goal_handle.abort()
                return BioCommand.Result(success=False, message=msg)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return BioCommand.Result(success=success, message=str(msg))

    
    async def call_tube_transport(self, _id, pick_pose, place_pose):
        if not self.tube_client.wait_for_server(timeout_sec=2.0):
            return False, "NO_SERVER", "하위 튜브 Action(/tube_transport) 서버 연결 실패"

        goal = TubeTransport.Goal()

        # job_id는 '필드가 있으면' 채움 (기존 의도 유지)
        if hasattr(goal, "job_id"):
            goal.job_id = "tube_in_or_out"

        # 인터페이스 필드명은 TubeTransport.action 기준(pick_posx/place_posx) 가정
        goal.pick_posx = [float(x) for x in pick_pose]
        goal.place_posx = [float(x) for x in place_pose]

        gh = await self.tube_client.send_goal_async(goal)
        if not gh.accepted:
            return False, "GOAL_REJECTED", "하위 튜브 Action Goal 거절됨"

        result = await gh.get_result_async()
        return bool(result.result.success), str(getattr(result.result, "error_code", "")), str(getattr(result.result, "message", ""))


def main():
    rclpy.init()
    node = MainIntegrated()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
