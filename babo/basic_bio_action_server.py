# basic_bio_action_server.py v2.100 2026-01-22
# [이번 버전에서 수정된 사항]
# - (기능추가) BioCommand 액션 서버(/test_bio_command) 기본 동작 구현
# - (기능추가) 다른 터미널에서 command(goal) 수신 시 "2초동안 동작해요" 로그 1회 출력 후 성공 반환
# - (구조정리) 액션 이름(action_name)과 동작 시간(work_sec)을 ROS 파라미터로 변경 가능

"""[모듈] basic_bio_action_server

[역할]
- 통신 점검용 최소 Action Server.
- 다른 터미널에서 BioCommand goal을 보내면 서버가 로그 1줄 출력 후 2초 대기하고 성공 Result를 반환한다.

[사용 예]
1) 서버 실행:
   ros2 run <패키지명> basic_bio_action_server
   (또는) python3 basic_bio_action_server.py

2) 다른 터미널에서 goal 전송:
   ros2 action send_goal /test_bio_command biobank_interfaces/action/BioCommand "{command: 'PING'}"

[파라미터]
- action_name (string) : 기본 "/test_bio_command"
- work_sec    (double) : 기본 2.0
"""

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from biobank_interfaces.action import BioCommand

# =========================
# DEFAULT_ 상수
# =========================
DEFAULT_ACTION_NAME = "/test_bio_command"
DEFAULT_WORK_SEC = 2.0


class BasicBioCommandServer(Node):
    def __init__(self):
        super().__init__("basic_bio_action_server")

        # 파라미터
        self.declare_parameter("action_name", DEFAULT_ACTION_NAME)
        self.declare_parameter("work_sec", DEFAULT_WORK_SEC)

        self._action_name = str(self.get_parameter("action_name").value)
        self._work_sec = float(self.get_parameter("work_sec").value)

        # Action Server
        self._server = ActionServer(
            self,
            BioCommand,
            self._action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(f"✅ Basic ActionServer started: {self._action_name} (work_sec={self._work_sec})")

    # =========================
    # Action callbacks
    # =========================
    def goal_callback(self, goal_request: BioCommand.Goal):
        cmd = getattr(goal_request, "command", "")
        self.get_logger().info(f"📩 Goal 수신: {cmd}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 Cancel 요청 수신")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        cmd = getattr(goal_handle.request, "command", "")
        # 요청대로 “이 한 줄”이 핵심
        self.get_logger().info(f"✅ '{cmd}' 받았고 {self._work_sec:.1f}초동안 동작해요")

        remaining = self._work_sec
        step = 0.1  # cancel 체크용 짧은 sleep

        while remaining > 0.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return BioCommand.Result(success=False, message="canceled")

            await asyncio.sleep(min(step, remaining))
            remaining -= step

        goal_handle.succeed()
        return BioCommand.Result(success=True, message="done")


def main(args=None):
    rclpy.init(args=args)
    node = BasicBioCommandServer()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
