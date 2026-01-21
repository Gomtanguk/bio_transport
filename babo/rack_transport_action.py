import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# [Path Settings]
DSR_LIB_PATH = '/home/gom/babo_ws/src/doosan-robot2/dsr_common2/imp'
if DSR_LIB_PATH not in sys.path:
    sys.path.insert(0, DSR_LIB_PATH)

from biobank_interfaces.action import RobotMove

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Create Node
    node = Node('rack_transport_action')
    node.declare_parameter("dry_run", False)
    node.declare_parameter("skip_probe", True)
    
    is_dry_run = node.get_parameter("dry_run").value
    
    node.get_logger().info(f"🚀 [Init] Node Start (DryRun={is_dry_run})")

    # 2. Robot Connection
    dr = None
    rack_stations = {}
    wb_station = None
    home_j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

    if not is_dry_run:
        # Debug Log 1
        node.get_logger().info("⏳ [Connecting] Injecting DR_init settings...")
        
        import DR_init
        DR_init.__dsr__node = node
        DR_init.__dsr__id = "dsr01"
        DR_init.__dsr__model = "m0609"

        # Debug Log 2
        node.get_logger().info("⏳ [Connecting] Importing DSR_ROBOT2...")
        import DSR_ROBOT2
        from DSR_ROBOT2 import CDsrRobot
        
        try:
            # Debug Log 3
            node.get_logger().info("⏳ [Connecting] Creating CDsrRobot object... (Check Emulator Status!)")
            dr = CDsrRobot("dsr01", "m0609")
            
            # Binding
            attrs = ['posx', 'posj', 'movel', 'movej', 'set_ref_coord', 
                     'get_current_posx', 'DR_BASE', 'DR_TOOL', 'DR_MV_MOD_REL', 
                     'OFF', 'ON', 'task_compliance_ctrl', 'release_compliance_ctrl', 
                     'get_tool_force', 'set_robot_mode']
            for a in attrs:
                if not hasattr(dr, a):
                    setattr(dr, a, getattr(DSR_ROBOT2, a))
            
            # Force Mode Set
            node.get_logger().info("⏳ [Setup] Setting Robot Mode to AUTONOMOUS...")
            dr.set_robot_mode(1) 
            node.get_logger().info("✅ Robot Hardware Connected & Mode Set")
            
        except Exception as e:
            node.get_logger().error(f"❌ Robot Connection Failed: {e}")
            return

        # Load Motion Modules
        from babo.rack_stations import build_rack_stations, build_workbench_station_dy
        from babo.gripper_io import grip_open, grip_close, grip_init_open
        from babo.probe_io import probe_contact_for_rack
        from babo.rel_move import rel_movel_tool, rel_movel_base
        from babo.rack_pick_io import rack_pick_only
        from babo.rack_place_io import rack_place_only
        from babo.workbench_place_io import workbench_place_only

        rack_stations = build_rack_stations(dr, approach_dy=-250.0)
        wb_station = build_workbench_station_dy(dr)

    # 3. Action Callback
    def execute_callback(goal_handle):
        raw_cmd = goal_handle.request.command
        node.get_logger().info(f"📥 Command Received: {raw_cmd}")
        
        if is_dry_run:
            goal_handle.succeed()
            return RobotMove.Result(success=True, message="[DRY_RUN] Success")

        parts = [p.strip() for p in raw_cmd.split(',')]
        cmd_type = parts[0]
        src = parts[1] if len(parts) > 1 else "NONE"
        dest = parts[2] if len(parts) > 2 else "NONE"
        
        success, msg = False, ""

        try:
            dr.movej(home_j, vel=60, acc=60)
            grip_init_open(dr)

            if "IN" in cmd_type:
                node.get_logger().info(f"▶ Inbound: WB -> {dest}")
                ok, _ = rack_pick_only(node, dr, wb_station, "WB", probe_contact_for_rack, 
                                       grip_open, grip_close, rel_movel_tool, rel_movel_base)
                if ok:
                    sp_tr = rack_stations.get(dest)
                    ok, _ = rack_place_only(node, dr, sp_tr, dest, grip_open)
                success = ok

            elif "OUT" in cmd_type:
                node.get_logger().info(f"▶ Outbound: {src} -> WB")
                sp_fr = rack_stations.get(src)
                ok, _ = rack_pick_only(node, dr, sp_fr, src, probe_contact_for_rack, 
                                       grip_open, grip_close, rel_movel_tool, rel_movel_base)
                if ok:
                    ok, _ = workbench_place_only(node, dr, wb_station, "WB", grip_open)
                success = ok

            elif "MOVE" in cmd_type:
                node.get_logger().info(f"▶ Move: {src} -> {dest}")
                sp_fr = rack_stations.get(src)
                sp_tr = rack_stations.get(dest)
                ok, _ = rack_pick_only(node, dr, sp_fr, src, probe_contact_for_rack, 
                                       grip_open, grip_close, rel_movel_tool, rel_movel_base)
                if ok:
                    ok, _ = rack_place_only(node, dr, sp_tr, dest, grip_open)
                success = ok

            dr.movej(home_j, vel=60, acc=60)
            msg = "Operation Complete" if success else "Operation Failed"

        except Exception as e:
            success = False
            msg = f"Error: {e}"
            node.get_logger().error(msg)

        if success: goal_handle.succeed()
        else: goal_handle.abort()
        
        return RobotMove.Result(success=success, message=msg)

    # 4. Server Creation
    cb_group = ReentrantCallbackGroup()
    _server = ActionServer(
        node, RobotMove, 'robot_action',
        execute_callback=execute_callback,
        callback_group=cb_group
    )

    # 5. Execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()