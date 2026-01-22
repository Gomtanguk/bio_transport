# tube_inbound_node.py v2.000 2026-01-22
# [수정] 랙 Outbound -> 튜브 Pick(고정위치) -> 튜브 Place(WB) -> 랙 Inbound 시퀀스 구현

import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import DR_init

# 기존 모듈 임포트
from .gripper_io import grip_open, grip_close
from .tube_stations import get_tube_loading_station, get_workbench_slot_station

# [가상] 다른 패키지의 Action 인터페이스를 가정
# from my_robot_interfaces.action import TubeInbound, RackTransport

class TubeInboundActionServer(Node):
    def __init__(self):
        super().__init__('tube_inbound_server')
        
        # 1. 내 서버 (TubeInbound)
        self._action_server = ActionServer(
            self,
            None, # TODO: 실제 Action Type (TubeInbound)
            'tube_inbound',
            self.execute_callback
        )
        
        # 2. 하위 클라이언트 (Rack 제어용)
        # 랙 이동 노드(rack_transport 등)가 액션 서버로 떠 있어야 함
        self._rack_out_client = ActionClient(self, None, 'rack_outbound') # TODO: Type
        self._rack_in_client = ActionClient(self, None, 'rack_inbound')   # TODO: Type
        
        DR_init.__dsr__id = "dsr01"
        DR_init.__dsr__model = "m0609"
        DR_init.__dsr__node = self
        
        self.get_logger().info("TubeInbound Node Ready (Sequence: RackOut->Tube->RackIn)")

    def _call_rack_action(self, client, rack_id, action_name):
        """랙 이동 액션 호출 (동기 대기)"""
        self.get_logger().info(f"Waiting for {action_name} server...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{action_name} server not available.")
            return False
            
        # Goal 생성 (가정: rack_id 문자열 전송)
        # goal_msg = RackTransport.Goal()
        # goal_msg.rack_id = rack_id
        # self.get_logger().info(f"Sending Goal to {action_name}: {rack_id}")
        
        # [시뮬레이션] 실제 액션 연결 전까지는 로그만 출력하고 성공 처리
        self.get_logger().warn(f"!!! [SIMULATION] Calling {action_name} for {rack_id} (Wait 2s) !!!")
        time.sleep(2.0) 
        return True

        # 실제 코드:
        # future = client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result().accepted

    def execute_callback(self, goal_handle):
        # target_loc 포맷: "A-2-3" (랙그룹-랙번호-슬롯번호)
        # goal = goal_handle.request
        full_target = "A-2-3" # goal.target_loc (테스트용 하드코딩)
        
        self.get_logger().info(f"[START] Tube Inbound to {full_target}")
        
        # 1. 파싱 (A-2-3 -> Rack: A-2, Slot: 3)
        try:
            parts = full_target.split('-') # ['A', '2', '3']
            rack_id = f"{parts[0]}-{parts[1]}" # "A-2"
            
            # 행/열 인덱스 계산 (예시: 슬롯 번호 '3'이 1행 3열을 의미한다고 가정)
            # 사용자의 네이밍 규칙에 따라 매핑 로직 필요
            # 여기서는 편의상 Row=0(1번째 줄), Col=int(parts[2])-1 로 가정
            target_row = 0 
            target_col = int(parts[2]) - 1 
        except Exception as e:
            self.get_logger().error(f"Invalid Target Format: {full_target}")
            goal_handle.abort()
            return

        import DSR_ROBOT2 as dr

        # ====================================================
        # STEP 1: Rack Outbound (랙을 꺼내옴)
        # ====================================================
        if not self._call_rack_action(self._rack_out_client, rack_id, "rack_outbound"):
            goal_handle.abort()
            return

        # ====================================================
        # STEP 2: Tube Pick (고정 위치에서 튜브 집기)
        # ====================================================
        # 2-1. 스테이션 정보 로드
        pick_st = get_tube_loading_station(dr)
        
        self.get_logger().info("[MOVE] To Tube Loading Zone")
        
        # 2-2. Approach (Z + 200)
        dr.movel(pick_st["approach"], vel=100, acc=100)
        
        # 2-3. 그립 열고 진입
        grip_open(dr)
        dr.movel(pick_st["target"], vel=50, acc=50)
        
        # 2-4. 그립 닫기
        grip_close(dr)
        
        # 2-5. Retract (Z + 200, Approach 위치와 동일)
        dr.movel(pick_st["retract"], vel=100, acc=100)
        
        # ====================================================
        # STEP 3: Tube Place (워크벤치에 있는 랙 A-2-3 위치로)
        # ====================================================
        # 3-1. 워크벤치 위 타겟 좌표 계산
        wb_st = get_workbench_slot_station(dr, target_row, target_col, approach_z_offset=200.0)
        
        self.get_logger().info(f"[MOVE] To Workbench Rack Slot {full_target}")
        
        # 3-2. Approach (Z + 200)
        dr.movel(wb_st["approach"], vel=100, acc=100)
        
        # 3-3. Target (꽂기)
        dr.movel(wb_st["target"], vel=30, acc=30) # 정밀 삽입
        
        # 3-4. 그립 열기
        grip_open(dr)
        
        # 3-5. Retract (Z + 200)
        dr.movel(wb_st["retract"], vel=100, acc=100)
        
        # 3-6. Home (랙 이동 전 안전 위치)
        home_pos = dr.posj(0, 0, 90, 0, 90, 0)
        dr.movej(home_pos, vel=60, acc=60)

        # ====================================================
        # STEP 4: Rack Inbound (랙을 다시 넣음)
        # ====================================================
        if not self._call_rack_action(self._rack_in_client, rack_id, "rack_inbound"):
            goal_handle.abort()
            return

        self.get_logger().info("[FINISH] Tube Inbound Sequence Complete")
        goal_handle.succeed()

def main(args=None):
    rclpy.init(args=args)
    node = TubeInboundActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()