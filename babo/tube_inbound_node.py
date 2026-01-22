# tube_inbound_node.py v1.000 2026-01-21
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import DR_init

from .gripper_io import grip_open, grip_close
from .probe_io import probe_contact_for_rack
from .rel_move import rel_movel_tool, rel_movel_base
from .tube_stations import build_tube_stations, build_workbench_tube_station
from .tube_pick_io import tube_pick_only
from .tube_place_io import tube_place_only

class TubeInboundActionServer(Node):
    def __init__(self):
        super().__init__('tube_inbound_server')
        self._action_server = ActionServer(
            self,
            None, # TODO: Action Type
            'tube_inbound',
            self.execute_callback
        )
        DR_init.__dsr__id = "dsr01"
        DR_init.__dsr__model = "m0609"
        DR_init.__dsr__node = self

    def execute_callback(self, goal_handle):
        # target_loc = goal_handle.request.target_loc
        target_loc = "A-1-1" # 테스트용
        
        import DSR_ROBOT2 as dr
        wb_stations = build_workbench_tube_station(dr)
        rack_stations = build_tube_stations(dr)
        
        # 1. Pick form Workbench
        ok_pick, _ = tube_pick_only(
            self, dr, wb_stations["WB_TUBE"], "PICK_WB",
            probe_contact_for_rack, grip_open, grip_close,
            rel_movel_tool, rel_movel_base
        )
        
        if not ok_pick:
            goal_handle.abort()
            return

        # 2. Place to Rack
        ok_place, _ = tube_place_only(
            self, dr, rack_stations[target_loc], f"PLACE_{target_loc}",
            grip_open
        )
        
        if ok_place:
            goal_handle.succeed()
        else:
            goal_handle.abort()

def main(args=None):
    rclpy.init(args=args)
    node = TubeInboundActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()