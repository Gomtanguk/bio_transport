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
            TubeTransport,
            "tube_main_control",
            execute_callback=self.handle_tube_command,
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

    async def call_robot(self, cmd_str: str):
        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            return False, "하위 로봇 Action(/robot_action) 서버 연결 실패"

        goal = RobotMove.Goal()
        goal.command = str(cmd_str)

        gh = await self.robot_client.send_goal_async(goal)
        if not gh.accepted:
            return False, "하위 로봇 Action Goal 거절됨"

        res = await gh.get_result_async()
        return bool(res.result.success), str(res.result.message)

    # ---------------- Tube ----------------
    def tube_goal_callback(self, goal_request: TubeTransport.Goal):
        self.get_logger().info(f"📩 [Tube] Goal: job_id={getattr(goal_request, 'job_id', '')}")
        return GoalResponse.ACCEPT

    def tube_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 [Tube] Cancel")
        return CancelResponse.ACCEPT

    async def handle_tube_command(self, goal_handle):
        req = goal_handle.request

        if self._robot_lock.locked():
            try:
                fb = TubeTransport.Feedback()
                fb.stage = "QUEUED"
                fb.progress = 0.0
                fb.detail = "Waiting for other job"
                goal_handle.publish_feedback(fb)
            except Exception:
                pass

        async with self._robot_lock:
            try:
                fb = TubeTransport.Feedback()
                fb.stage = "RUNNING"
                fb.progress = 0.0
                fb.detail = "Forwarding to /tube_transport"
                goal_handle.publish_feedback(fb)
            except Exception:
                pass

            success, err, msg = await self.call_tube(req, upstream_goal_handle=goal_handle)

        out = TubeTransport.Result()
        out.success = bool(success)
        out.error_code = str(err)
        out.message = str(msg)

        goal_handle.succeed() if success else goal_handle.abort()
        return out

    async def call_tube(self, ui_goal: TubeTransport.Goal, upstream_goal_handle):
        if not self.tube_client.wait_for_server(timeout_sec=2.0):
            return False, "DOWNSTREAM_UNAVAILABLE", "하위 TubeTransport(/tube_transport) 서버 연결 실패"

        goal = TubeTransport.Goal()
        goal.job_id = str(getattr(ui_goal, "job_id", ""))
        goal.pick_posx = list(getattr(ui_goal, "pick_posx", []))
        goal.place_posx = list(getattr(ui_goal, "place_posx", []))

        def _fb_cb(feedback_msg):
            try:
                fb_in = feedback_msg.feedback
                fb_out = TubeTransport.Feedback()
                fb_out.stage = str(getattr(fb_in, "stage", ""))
                fb_out.progress = float(getattr(fb_in, "progress", 0.0))
                fb_out.detail = str(getattr(fb_in, "detail", ""))
                upstream_goal_handle.publish_feedback(fb_out)
            except Exception:
                pass

        gh = await self.tube_client.send_goal_async(goal, feedback_callback=_fb_cb)
        if not gh.accepted:
            return False, "DOWNSTREAM_REJECTED", "하위 TubeTransport Goal 거절됨"

        res = await gh.get_result_async()
        r = res.result
        return bool(getattr(r, "success", False)), str(getattr(r, "error_code", "")), str(getattr(r, "message", ""))


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
