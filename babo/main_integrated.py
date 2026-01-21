# main_integrated v2.100 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능변경) bio_main_control(BioCommand) 목표 수신 시 즉시 수신 로그(goal_callback) 출력 추가
# - (기능변경) 로봇 실행은 asyncio.Lock으로 직렬화(1대 로봇 보호)하되, 목표 수신/대기열 피드백은 즉시 처리
# - (구조정리) QoS를 RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(depth=5)로 단일 상수(ACTION_QOS)로 고정
# - (안정화) MultiThreadedExecutor(num_threads=4)로 명령 수신/피드백/하위 Action 대기 중에도 콜백 처리 유지

"""[모듈] main_integrated

[역할]
- UI(Action: BioCommand, /bio_main_control)로부터 Rack 명령을 받아,
  하위 로봇 Action(RobotMove, /robot_action)으로 전달하고 결과를 UI에 반환한다.

[핵심 포인트]
- 로봇은 동시에 2개 작업을 실행하면 위험하므로, asyncio.Lock으로 실행을 직렬화한다.
- 다만 "명령 수신" 로그가 실행 시작 시점에만 찍히면 사용자는 '한번만 받는다'고 느끼므로,
  goal_callback에서 즉시 수신 로그를 남기고, execute 콜백에서는 대기열 피드백을 제공한다.

[명령 예]
- UI -> main_integrated: "RACK,IN,NONE,A-2"
- main_integrated -> robot_action: "IN,NONE,A-2"
"""


import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 인터페이스 정의
try:
    from biobank_interfaces.action import BioCommand, RobotMove
except ImportError:
    # Dummy for check
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


# ✅ [QoS] Reliable + Transient Local + Keep Last
ACTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class MainIntegrated(Node):
    def __init__(self):
        super().__init__("main_orchestrator")

        self.callback_group = ReentrantCallbackGroup()

        # 로봇 실행 직렬화(1대 로봇 보호)
        self._robot_lock = asyncio.Lock()

        # 1) [Server] UI로부터 명령 수신
        self._ui_server = ActionServer(
            self,
            BioCommand,
            "bio_main_control",
            execute_callback=self.handle_ui_command,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            goal_service_qos_profile=ACTION_QOS,
            result_service_qos_profile=ACTION_QOS,
            cancel_service_qos_profile=ACTION_QOS,
            feedback_pub_qos_profile=ACTION_QOS,
            status_pub_qos_profile=ACTION_QOS,
        )

        # 2) [Client] 하위 로봇 액션으로 전달
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

        self.get_logger().info("🧠 [Integrated] 통합 메인 노드 시작됨 (QoS Applied).")

    # ==========================================================
    # ActionServer callbacks
    # ==========================================================
    def goal_callback(self, goal_request: BioCommand.Goal):
        # ⚠️ 이 로그는 '수신 즉시' 찍힘(사용자 체감 개선)
        self.get_logger().info(f"📩 Goal 요청 수신: {getattr(goal_request, 'command', '')}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 Cancel 요청 수신")
        return CancelResponse.ACCEPT

    async def handle_ui_command(self, goal_handle):
        raw_cmd = goal_handle.request.command
        self.get_logger().info(f"📥 명령 실행 시작: {raw_cmd}")

        # prefix 제거: "RACK,..." -> "IN,NONE,A-2"
        try:
            parts = [p.strip() for p in str(raw_cmd).split(",")]
            sub_cmd = ",".join(parts[1:]) if len(parts) >= 2 else str(raw_cmd)
        except Exception:
            sub_cmd = str(raw_cmd)

        # 로봇이 이미 실행 중이면, 즉시 '대기열' 피드백
        if self._robot_lock.locked():
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"대기열: 다른 작업 실행 중 ({sub_cmd})"))
            except Exception:
                pass

        # 실제 로봇 실행은 직렬화
        async with self._robot_lock:
            try:
                goal_handle.publish_feedback(BioCommand.Feedback(status=f"실행 중: {sub_cmd}"))
            except Exception:
                pass

            success, msg = await self.call_robot(sub_cmd)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return BioCommand.Result(success=success, message=msg)

    # ==========================================================
    # Robot Action client
    # ==========================================================
    async def call_robot(self, cmd_str: str):
        # wait_for_server는 blocking이지만 짧게만 사용(2s)
        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            return False, "하위 로봇 Action(/robot_action) 서버 연결 실패"

        goal = RobotMove.Goal()
        goal.command = str(cmd_str)

        goal_handle = await self.robot_client.send_goal_async(goal)
        if not goal_handle.accepted:
            return False, "하위 로봇 Action Goal 거절됨"

        result = await goal_handle.get_result_async()
        return bool(result.result.success), str(result.result.message)


def main(args=None):
    rclpy.init(args=args)
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
