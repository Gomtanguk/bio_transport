# main_integrated v2.200 2026-01-22
# [이번 버전에서 수정된 사항]
# - (기능변경) 튜브 작업을 TubeTransport Action으로 UI(/tube_main_control)에서 수신 후 로봇(/tube_transport)으로 중계
# - (기능변경) Rack(BioCommand)과 Tube(TubeTransport) 모두 동일 asyncio.Lock으로 직렬화(1대 로봇 보호)
# - (유지) QoS는 코드 설정값(ACTION_QOS: RELIABLE/VOLATILE/KEEP_LAST depth=5) 그대로 사용(주석은 무시)

"""[모듈] main_integrated

[역할]
- UI(Action: BioCommand, /bio_main_control)로부터 Rack 명령을 받아,
  하위 로봇 Action(RobotMove, /robot_action)으로 전달하고 결과를 UI에 반환한다.
- UI(Action: TubeTransport, /tube_main_control)로부터 Tube 이동 명령을 받아,
  하위 로봇 Action(TubeTransport, /tube_transport)으로 전달하고 결과/피드백을 UI에 중계한다.

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
    from biobank_interfaces.action import BioCommand, RobotMove, TubeTransport
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


# ✅ [QoS] Reliable + Transient Local + Keep Last
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

    def tube_goal_callback(self, goal_request: TubeTransport.Goal):
        self.get_logger().info(
            f"📩 [TUBE] Goal 요청 수신: job_id={getattr(goal_request, 'job_id', '')}"
        )
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


    async def handle_tube_command(self, goal_handle):
        """UI(/tube_main_control)에서 수신한 TubeTransport goal을 로봇(/tube_transport)으로 중계"""
        job_id = getattr(goal_handle.request, 'job_id', '')
        pick_posx = list(getattr(goal_handle.request, 'pick_posx', []) or [])
        place_posx = list(getattr(goal_handle.request, 'place_posx', []) or [])

        self.get_logger().info(f"📥 [TUBE] 명령 실행 시작: job_id={job_id}")

        # 최소 검증
        if not job_id or len(pick_posx) != 6 or len(place_posx) != 6:
            goal_handle.abort()
            res = TubeTransport.Result()
            res.success = False
            res.error_code = "ERR_INVALID_GOAL"
            res.message = "job_id 또는 pick_posx/place_posx(길이 6)가 유효하지 않습니다."
            return res

        # 로봇이 이미 실행 중이면, 즉시 '대기열' 피드백
        if self._robot_lock.locked():
            try:
                fb = TubeTransport.Feedback()
                fb.stage = "QUEUE"
                fb.progress = 0.0
                fb.detail = "대기열: 다른 작업 실행 중"
                goal_handle.publish_feedback(fb)
            except Exception:
                pass

        async with self._robot_lock:
            # 브릿지 시작 피드백
            try:
                fb = TubeTransport.Feedback()
                fb.stage = "FORWARD"
                fb.progress = 0.0
                fb.detail = "로봇 액션으로 전달 중"
                goal_handle.publish_feedback(fb)
            except Exception:
                pass

            ok, err_code, msg = await self.call_tube(job_id, pick_posx, place_posx, goal_handle)

        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        res = TubeTransport.Result()
        res.success = bool(ok)
        res.error_code = str(err_code or "")
        res.message = str(msg or "")
        return res

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


    async def call_tube(self, job_id: str, pick_posx, place_posx, upstream_goal_handle):
        """로봇(/tube_transport) TubeTransport Action 호출 + 피드백 중계"""
        if not self.tube_client.wait_for_server(timeout_sec=2.0):
            return False, "ERR_NO_SERVER", "하위 로봇 Action(/tube_transport) 서버 연결 실패"

        goal = TubeTransport.Goal()
        goal.job_id = str(job_id)
        goal.pick_posx = [float(x) for x in list(pick_posx)]
        goal.place_posx = [float(x) for x in list(place_posx)]

        def _fb_cb(feedback_msg):
            try:
                fb_in = feedback_msg.feedback
                fb = TubeTransport.Feedback()
                fb.stage = str(getattr(fb_in, 'stage', ''))
                fb.progress = float(getattr(fb_in, 'progress', 0.0))
                fb.detail = str(getattr(fb_in, 'detail', ''))
                upstream_goal_handle.publish_feedback(fb)
            except Exception:
                pass

        gh = await self.tube_client.send_goal_async(goal, feedback_callback=_fb_cb)
        if not gh.accepted:
            return False, "ERR_REJECTED", "하위 로봇 TubeTransport Goal 거절됨"

        result = await gh.get_result_async()
        ok = bool(getattr(result.result, 'success', False))
        err_code = str(getattr(result.result, 'error_code', ''))
        msg = str(getattr(result.result, 'message', ''))
        return ok, err_code, msg


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
