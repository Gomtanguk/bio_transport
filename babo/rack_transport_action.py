# rack_transport_action v2.800 2026-01-21
# [수정 사항]
# - ActionServer에 QoS(Reliable, Transient Local, KeepLast=5) 적용
# - DoosanRealBridge.movel: vel/acc 필드를 list[2]로 변환하여 에러 수정

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # ✅ QoS Import

import DR_init
from biobank_interfaces.action import RobotMove

# =========================
# ROBOT 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# =========================
# DEFAULT_ 상수
# =========================
DEFAULT_MOVEJ_VEL = 60.0
DEFAULT_MOVEJ_ACC = 60.0
DEFAULT_MOVEJ_TIME = 2.0

DEFAULT_MOVEL_VEL = 200.0
DEFAULT_MOVEL_ACC = 200.0
DEFAULT_MOVEL_TIME = 2.0

HOME_J_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def _norm(s: Optional[str]) -> Optional[str]:
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t




class DoosanRealBridge:
    ON = 1
    OFF = 0
    DR_BASE = 0
    DR_TOOL = 1

    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.ns = f"/{robot_id}"

        from dsr_msgs2.srv import (
            MoveJoint, MoveLine,
            GetControlMode, GetRobotState, GetRobotMode,
            SetRobotControl, SetRobotMode,
            SetCurrentTool, SetCurrentTcp,
            SetCtrlBoxDigitalOutput, SetToolDigitalOutput,
        )

        self.MoveJoint = MoveJoint
        self.MoveLine = MoveLine
        self.GetControlMode = GetControlMode
        self.GetRobotState = GetRobotState
        self.GetRobotMode = GetRobotMode
        self.SetRobotControl = SetRobotControl
        self.SetRobotMode = SetRobotMode
        self.SetCurrentTool = SetCurrentTool
        self.SetCurrentTcp = SetCurrentTcp
        self.SetCtrlBoxDigitalOutput = SetCtrlBoxDigitalOutput
        self.SetToolDigitalOutput = SetToolDigitalOutput

        self.cli_move_joint = node.create_client(MoveJoint, f"{self.ns}/motion/move_joint")
        self.cli_move_line = node.create_client(MoveLine, f"{self.ns}/motion/move_line")

        self.cli_get_control_mode = node.create_client(GetControlMode, f"{self.ns}/aux_control/get_control_mode")
        self.cli_get_robot_state = node.create_client(GetRobotState, f"{self.ns}/system/get_robot_state")
        self.cli_get_robot_mode = node.create_client(GetRobotMode, f"{self.ns}/system/get_robot_mode")

        self.cli_set_robot_control = node.create_client(SetRobotControl, f"{self.ns}/system/set_robot_control")
        self.cli_set_robot_mode = node.create_client(SetRobotMode, f"{self.ns}/system/set_robot_mode")

        self.cli_set_tool = node.create_client(SetCurrentTool, f"{self.ns}/tool/set_current_tool")
        self.cli_set_tcp = node.create_client(SetCurrentTcp, f"{self.ns}/tcp/set_current_tcp")

        self.cli_set_ctrl_do = node.create_client(SetCtrlBoxDigitalOutput, f"{self.ns}/io/set_ctrl_box_digital_output")
        self.cli_set_tool_do = node.create_client(SetToolDigitalOutput, f"{self.ns}/io/set_tool_digital_output")

    def wait(self, sec: float):
        time.sleep(float(sec))

    def _wait_future(self, fut, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while not fut.done():
            if timeout_sec is not None and (time.monotonic() - t0) > timeout_sec:
                return False
            time.sleep(0.01)
        return True

    def _call(self, client, req, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {client.srv_name}")
        fut = client.call_async(req)
        if not self._wait_future(fut, timeout_sec=timeout_sec):
            raise TimeoutError(f"Service call timeout: {client.srv_name}")
        return fut.result()

    @staticmethod
    def _deg_list_to_rad(j_deg: Sequence[float]) -> Sequence[float]:
        return [math.radians(float(x)) for x in j_deg]

    @staticmethod
    def posj(*args) -> list:
        if len(args) == 1 and isinstance(args[0], (list, tuple)):
            return [float(x) for x in args[0]]
        return [float(x) for x in args]

    @staticmethod
    def posx(*args) -> list:
        if len(args) == 1 and isinstance(args[0], (list, tuple)):
            return [float(x) for x in args[0]]
        return [float(x) for x in args]

    # ---- query ----
    def get_control_mode(self) -> int:
        req = self.GetControlMode.Request()
        resp = self._call(self.cli_get_control_mode, req, timeout_sec=2.0)
        return int(getattr(resp, "control_mode", -1))

    def get_robot_state(self) -> int:
        req = self.GetRobotState.Request()
        resp = self._call(self.cli_get_robot_state, req, timeout_sec=2.0)
        return int(getattr(resp, "robot_state", -1))

    def get_robot_mode(self) -> int:
        req = self.GetRobotMode.Request()
        resp = self._call(self.cli_get_robot_mode, req, timeout_sec=2.0)
        return int(getattr(resp, "robot_mode", -1))

    # ---- optional setters ----
    def set_robot_control(self, value: int):
        req = self.SetRobotControl.Request()
        for k, t in req._fields_and_field_types.items():
            if t in ("int8", "int16", "int32", "uint8", "uint16", "uint32"):
                setattr(req, k, int(value))
                break
        return self._call(self.cli_set_robot_control, req, timeout_sec=2.0)

    def set_robot_mode(self, value: int):
        req = self.SetRobotMode.Request()
        if hasattr(req, "robot_mode"):
            req.robot_mode = int(value)
        elif hasattr(req, "mode"):
            req.mode = int(value)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, int(value))
        return self._call(self.cli_set_robot_mode, req, timeout_sec=2.0)

    def set_tool(self, tool_name: str):
        req = self.SetCurrentTool.Request()
        if hasattr(req, "name"):
            req.name = str(tool_name)
        elif hasattr(req, "tool_name"):
            req.tool_name = str(tool_name)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, str(tool_name))
        return self._call(self.cli_set_tool, req, timeout_sec=2.0)

    def set_tcp(self, tcp_name: str):
        req = self.SetCurrentTcp.Request()
        if hasattr(req, "name"):
            req.name = str(tcp_name)
        elif hasattr(req, "tcp_name"):
            req.tcp_name = str(tcp_name)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, str(tcp_name))
        return self._call(self.cli_set_tcp, req, timeout_sec=2.0)

    # ---- IO ----
    def set_ctrl_box_digital_output(self, index: int, value: int):
        req = self.SetCtrlBoxDigitalOutput.Request()
        if hasattr(req, "index"): req.index = int(index)
        elif hasattr(req, "port"): req.port = int(index)
        elif hasattr(req, "channel"): req.channel = int(index)
        else: setattr(req, req.__slots__[0].lstrip("_"), int(index))

        if hasattr(req, "value"): req.value = int(value)
        elif hasattr(req, "val"): req.val = int(value)
        elif hasattr(req, "state"): req.state = int(value)
        else: setattr(req, req.__slots__[1].lstrip("_"), int(value))
        return self._call(self.cli_set_ctrl_do, req, timeout_sec=2.0)

    def set_tool_digital_output(self, index: int, value: int):
        req = self.SetToolDigitalOutput.Request()
        if hasattr(req, "index"): req.index = int(index)
        elif hasattr(req, "port"): req.port = int(index)
        elif hasattr(req, "channel"): req.channel = int(index)
        else: setattr(req, req.__slots__[0].lstrip("_"), int(index))

        if hasattr(req, "value"): req.value = int(value)
        elif hasattr(req, "val"): req.val = int(value)
        elif hasattr(req, "state"): req.state = int(value)
        else: setattr(req, req.__slots__[1].lstrip("_"), int(value))
        return self._call(self.cli_set_tool_do, req, timeout_sec=2.0)

    def set_digital_output(self, index: int, value: int):
        try:
            resp = self.set_ctrl_box_digital_output(index, value)
            if hasattr(resp, "success") and not bool(resp.success):
                raise RuntimeError("ctrl DO success=False")
            return resp
        except Exception as e:
            self.node.get_logger().warn(f"[IO] ctrl DO failed -> try tool DO ({e})")
            return self.set_tool_digital_output(index, value)

    # ---- motion ----
    def movej(self, j_deg: Sequence[float], vel: float, acc: float, t: float):
        req = self.MoveJoint.Request()
        req.pos = list(self._deg_list_to_rad(j_deg))
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = float(t)
        return self._call(self.cli_move_joint, req, timeout_sec=12.0)

    def movel(self, x: Sequence[float], vel: float, acc: float, t: float):
        req = self.MoveLine.Request()
        fields = req._fields_and_field_types
        if "pos" in fields:
            req.pos = [float(v) for v in x]
        else:
            for k in ("target", "posx", "x"):
                if k in fields:
                    setattr(req, k, [float(v) for v in x])
                    break
        
        # ✅ [Fix] vel/acc must be length 2 (float[2])
        if "vel" in fields:
            req.vel = [float(vel), 0.0] 
        if "acc" in fields:
            req.acc = [float(acc), 0.0]
            
        if "time" in fields:
            req.time = float(t)
        return self._call(self.cli_move_line, req, timeout_sec=20.0)


class RackTransportAction(Node):
    def __init__(self):
        super().__init__("rack_transport_action", namespace=f"/{ROBOT_ID}")

        self.declare_parameter("dry_run", False)
        self.declare_parameter("skip_probe", False)

        self.declare_parameter("movej_vel", float(DEFAULT_MOVEJ_VEL))
        self.declare_parameter("movej_acc", float(DEFAULT_MOVEJ_ACC))
        self.declare_parameter("movej_time", float(DEFAULT_MOVEJ_TIME))

        self.declare_parameter("movel_vel", float(DEFAULT_MOVEL_VEL))
        self.declare_parameter("movel_acc", float(DEFAULT_MOVEL_ACC))
        self.declare_parameter("movel_time", float(DEFAULT_MOVEL_TIME))

        self.declare_parameter("enable_set_robot_control", False)
        self.declare_parameter("robot_control_value", 0)
        self.declare_parameter("enable_set_robot_mode", False)
        self.declare_parameter("robot_mode_value", 1)

        DR_init.__dsr__node = self
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL

        self.dr = DoosanRealBridge(self, ROBOT_ID)

        from babo.rack_stations import build_rack_stations, build_workbench_station_dy
        self.rack_stations: Dict[str, Dict[str, list]] = build_rack_stations(self.dr, approach_dy=-250.0)
        self.wb_station: Dict[str, list] = build_workbench_station_dy(self.dr)

        from babo.gripper_io import grip_open, grip_close, grip_init_open
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.grip_init_open = grip_init_open

        # ✅ [QoS] Reliable + Transient Local 설정
        custom_action_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            # ✅ QoS 적용
            goal_service_qos_profile=custom_action_qos,
            result_service_qos_profile=custom_action_qos,
            cancel_service_qos_profile=custom_action_qos,
            feedback_pub_qos_profile=custom_action_qos,
            status_pub_qos_profile=custom_action_qos
        )

        self._log_robot_status(prefix="[BOOT]")
        self.get_logger().info("✅ [v2.800] rack_transport_action ready (QoS: Transient Local)")

    def _log_robot_status(self, prefix: str):
        try:
            cm = self.dr.get_control_mode()
            rs = self.dr.get_robot_state()
            rm = self.dr.get_robot_mode()
            self.get_logger().info(f"{prefix} control_mode={cm} robot_state={rs} robot_mode={rm}")
        except Exception as e:
            self.get_logger().warn(f"{prefix} status query failed: {e}")

    def _ensure_ready(self) -> Tuple[bool, str]:
        try:
            self._log_robot_status(prefix="[PRE]")

            if bool(self.get_parameter("enable_set_robot_control").value):
                v = int(self.get_parameter("robot_control_value").value)
                self.dr.set_robot_control(v)

            if bool(self.get_parameter("enable_set_robot_mode").value):
                v = int(self.get_parameter("robot_mode_value").value)
                self.dr.set_robot_mode(v)

            try:
                self.dr.set_tool(ROBOT_TOOL)
            except Exception as e:
                self.get_logger().warn(f"[INIT] set_tool failed: {e}")
            try:
                self.dr.set_tcp(ROBOT_TCP)
            except Exception as e:
                self.get_logger().warn(f"[INIT] set_tcp failed: {e}")

            self._log_robot_status(prefix="[POST]")
            return True, "ready"
        except Exception as e:
            return False, f"ensure_ready failed: {e}"

    async def execute_callback(self, goal_handle):
        raw_cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 cmd: {raw_cmd}")

        if bool(self.get_parameter("dry_run").value):
            goal_handle.succeed()
            return RobotMove.Result(success=True, message="[DRY_RUN] success")

        ok, msg = self._ensure_ready()
        if not ok:
            goal_handle.abort()
            return RobotMove.Result(success=False, message=msg)

        parts = [p.strip() for p in raw_cmd.split(",")]
        cmd = parts[0].upper() if parts else ""
        a = _norm(parts[1]) if len(parts) > 1 else None
        b = _norm(parts[2]) if len(parts) > 2 else None

        try:
            if cmd == "IN":
                dest = b if b else a
                if not dest: raise ValueError("IN needs dest")
                ok, m = self._do_in(dest)
            elif cmd == "OUT":
                src = a
                if not src: raise ValueError("OUT needs src")
                ok, m = self._do_out(src)
            elif cmd == "MOVE":
                src, dest = a, b
                if not src or not dest: raise ValueError("MOVE needs src,dest")
                ok, m = self._do_move(src, dest)
            else:
                raise ValueError(f"unknown cmd={cmd}")
        except Exception as e:
            ok, m = False, f"error: {e}"
            self.get_logger().error(m)

        if ok:
            self.get_logger().info(f"[RESULT] ok=True cmd={raw_cmd} msg={m}")
            goal_handle.succeed()
        else:
            self.get_logger().error(f"[RESULT] ok=False cmd={raw_cmd} msg={m}")
            goal_handle.abort()

        return RobotMove.Result(success=ok, message=m)

    def _do_in(self, dest: str) -> Tuple[bool, str]:
        if dest not in self.rack_stations:
            return False, f"unknown rack: {dest}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        tj = float(self.get_parameter("movej_time").value)

        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)
        tl = float(self.get_parameter("movel_time").value)

        st = self.rack_stations[dest]
        wb = self.wb_station

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        self.grip_init_open(self.dr)

        self.dr.movel(wb["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(wb["target"], vel=vl, acc=al, t=tl)
        
        # (Real Probe Logic would go here)
        
        self.grip_close(self.dr)
        self.dr.movel(wb["approach"], vel=vl, acc=al, t=tl)

        self.dr.movel(st["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(st["target"], vel=vl, acc=al, t=tl)
        self.grip_open(self.dr)
        self.dr.movel(st["approach"], vel=vl, acc=al, t=tl)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        return True, f"inbound to {dest} done"

    def _do_out(self, src: str) -> Tuple[bool, str]:
        if src not in self.rack_stations:
            return False, f"unknown rack: {src}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        tj = float(self.get_parameter("movej_time").value)

        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)
        tl = float(self.get_parameter("movel_time").value)

        st = self.rack_stations[src]
        wb = self.wb_station

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        self.grip_init_open(self.dr)

        self.dr.movel(st["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(st["target"], vel=vl, acc=al, t=tl)
        self.grip_close(self.dr)
        self.dr.movel(st["approach"], vel=vl, acc=al, t=tl)

        self.dr.movel(wb["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(wb["target"], vel=vl, acc=al, t=tl)
        self.grip_open(self.dr)
        self.dr.movel(wb["approach"], vel=vl, acc=al, t=tl)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        return True, f"outbound from {src} done"

    def _do_move(self, src: str, dest: str) -> Tuple[bool, str]:
        if src not in self.rack_stations or dest not in self.rack_stations:
            return False, f"unknown rack: {src}->{dest}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        tj = float(self.get_parameter("movej_time").value)

        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)
        tl = float(self.get_parameter("movel_time").value)

        st_src = self.rack_stations[src]
        st_dst = self.rack_stations[dest]

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        self.grip_init_open(self.dr)

        self.dr.movel(st_src["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(st_src["target"], vel=vl, acc=al, t=tl)
        self.grip_close(self.dr)
        self.dr.movel(st_src["approach"], vel=vl, acc=al, t=tl)

        self.dr.movel(st_dst["approach"], vel=vl, acc=al, t=tl)
        self.dr.movel(st_dst["target"], vel=vl, acc=al, t=tl)
        self.grip_open(self.dr)
        self.dr.movel(st_dst["approach"], vel=vl, acc=al, t=tl)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=tj)
        return True, f"move {src}->{dest} done"


def main(args=None):
    rclpy.init(args=args)
    node = RackTransportAction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()