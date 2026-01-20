import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# 인터페이스 정의 (실제 환경에선 biobank_interfaces 사용)
try:
    from biobank_interfaces.action import BioCommand, RobotMove
except ImportError:
    class BioCommand:
        class Goal: command = ""
        class Result: success = True; message = ""
        class Feedback: status = ""
    class RobotMove:
        class Goal: command = ""
        class Result: success = True; message = ""
        class Feedback: status = ""

class MainIntegrated(Node):
    def __init__(self):
        super().__init__('main_integrated')
        
        # 콜백 그룹 (병렬 처리용)
        self.callback_group = ReentrantCallbackGroup()

        # 1. [Server] UI로부터 명령 수신 ('bio_main_control')
        self._ui_server = ActionServer(
            self, BioCommand, 'bio_main_control', 
            self.handle_ui_command, callback_group=self.callback_group
        )

        # 2. [Client] 통합 하위 노드로 명령 전송 ('robot_action')
        self.robot_client = ActionClient(
            self, RobotMove, 'robot_action', callback_group=self.callback_group
        )

        self.get_logger().info("🧠 [Integrated] 통합 메인 노드 시작됨.")

    async def handle_ui_command(self, goal_handle):
        """UI 명령 처리: TUBE/RACK, ACTION, SRC, DEST"""
        raw_cmd = goal_handle.request.command
        self.get_logger().info(f"📥 명령 수신: {raw_cmd}")
        
        # 진행 상황 피드백
        goal_handle.publish_feedback(BioCommand.Feedback(status=f"분석 중: {raw_cmd}"))

        # 명령 파싱 (예: RACK,MOVE,A-1,B-1)
        try:
            parts = raw_cmd.split(',')
            # 하위 노드는 "MOVE,A-1,B-1" 형태만 필요하므로 앞부분(RACK 등) 제외하고 재조립
            sub_cmd = ",".join(parts[1:]) 
        except:
            sub_cmd = raw_cmd # 파싱 실패 시 원본 전송

        # 하위 노드 호출
        success, msg = await self.call_robot(sub_cmd)

        # 결과 반환
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return BioCommand.Result(success=success, message=msg)

    async def call_robot(self, cmd_str):
        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            return False, "하위 노드 연결 실패"
        
        goal = RobotMove.Goal()
        goal.command = cmd_str
        
        future = await self.robot_client.send_goal_async(goal)
        if not future.accepted: return False, "명령 거절됨"
        
        res = await future.get_result_async()
        return res.result.success, res.result.message

def main(args=None):
    rclpy.init(args=args)
    node = MainIntegrated()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()