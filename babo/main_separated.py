import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# 인터페이스 정의
try:
    from biobank_interfaces.action import BioCommand, RobotMove
except ImportError:
    # 가짜 클래스 (테스트용)
    class BioCommand:
        class Goal: command = ""
        class Result: success = True; message = ""
        class Feedback: status = ""
    class RobotMove:
        class Goal: command = ""
        class Result: success = True; message = ""
        class Feedback: status = ""

class MainSeparated(Node):
    def __init__(self):
        super().__init__('main_separated')
        self.callback_group = ReentrantCallbackGroup()

        # 1. [Server] UI 통신 (공통)
        self._ui_server = ActionServer(
            self, BioCommand, 'bio_main_control', 
            self.handle_ui_command, callback_group=self.callback_group
        )

        # 2. [Clients] 3개의 전문 하위 노드 연결
        self.cli_in = ActionClient(self, RobotMove, 'action_inbound', callback_group=self.callback_group)
        self.cli_out = ActionClient(self, RobotMove, 'action_outbound', callback_group=self.callback_group)
        self.cli_move = ActionClient(self, RobotMove, 'action_transport', callback_group=self.callback_group)

        self.get_logger().info("🧠 [Separated] 분리형 메인 노드 시작됨 (3 Clients Ready).")

    async def handle_ui_command(self, goal_handle):
        raw_cmd = goal_handle.request.command # 예: "TUBE,IN,NONE,B-1"
        self.get_logger().info(f"📥 명령 수신: {raw_cmd}")
        goal_handle.publish_feedback(BioCommand.Feedback(status="담당 노드 배정 중..."))

        try:
            parts = raw_cmd.split(',')
            action_type = parts[1] # IN, OUT, MOVE 중 하나
            
            # 하위 노드용 명령 (예: "IN,NONE,B-1")
            sub_cmd = ",".join(parts[1:])
        except:
            goal_handle.abort()
            return BioCommand.Result(success=False, message="명령어 형식 오류")

        success = False
        msg = ""

        # 3. 명령 타입에 따라 다른 클라이언트 호출
        if action_type == "IN":
            goal_handle.publish_feedback(BioCommand.Feedback(status="[입고] 전문 노드 호출"))
            success, msg = await self.call_sub(self.cli_in, sub_cmd)
            
        elif action_type == "OUT":
            goal_handle.publish_feedback(BioCommand.Feedback(status="[출고] 전문 노드 호출"))
            success, msg = await self.call_sub(self.cli_out, sub_cmd)
            
        elif action_type == "MOVE":
            goal_handle.publish_feedback(BioCommand.Feedback(status="[이동] 전문 노드 호출"))
            success, msg = await self.call_sub(self.cli_move, sub_cmd)
            
        else:
            msg = f"알 수 없는 동작 타입: {action_type}"

        # 결과 반환
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return BioCommand.Result(success=success, message=msg)

    async def call_sub(self, client, cmd_str):
        """지정된 클라이언트로 명령 전송"""
        if not client.wait_for_server(timeout_sec=2.0):
            return False, "해당 하위 노드와 연결되지 않았습니다."
        
        goal = RobotMove.Goal()
        goal.command = cmd_str
        
        future = await client.send_goal_async(goal)
        if not future.accepted: return False, "하위 노드가 명령을 거절함"
        
        res = await future.get_result_async()
        return res.result.success, res.result.message

def main(args=None):
    rclpy.init(args=args)
    node = MainSeparated()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()