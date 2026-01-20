# rack_transport_action v2.600 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능추가) gripper_io 호환을 위해 DoosanMotionBridge에 set_digital_output / set_tool_digital_output / wait 추가
# - (기능추가) set_robot_mode 호출을 옵션화(set_robot_mode_enable 파라미터). 기본은 False로 하여 "manual mode" 경고 스팸 방지
# - (유지) MoveJoint/MoveLine(서비스) 기반 모션 브릿지 유지(virtual/RViz에서 실제로 움직임)
# - (유지) /dsr01 namespace, /robot_action 절대경로 유지

"""[모듈] rack_transport_action

[역할]
- /robot_action (RobotMove) Action 서버
- UI/메인에서 들어온 IN/OUT/MOVE 명령을 받아 랙 이송 시퀀스를 실행

[핵심]
- virtual/RViz에서 확실히 움직이도록 dsr_controller2가 제공하는 motion 서비스(move_joint/move_line)를 사용
- gripper_io는 dr.set_digital_output을 기대하므로, IO 서비스(set_ctrl_box_digital_output / set_tool_digital_output)로 구현
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import DR_init
from biobank_interfaces.action import RobotMove

# =========================
# ROBOT 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# =========================
# DEFAULT_ 상수
# =========================
DEFAULT_MOVEJ_VEL = 60.0
DEFAULT_MOVEJ_ACC = 60.0

DEFAULT_MOVEL_VEL = 200.0
DEFAULT_MOVEL_ACC = 200.0

HOME_J_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def _norm(s: Optional[str]) -> Optional[str]:
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


class DoosanMotionBridge:
    """dsr_controller2 서비스 기반 모션/IO 브릿지."""

    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.ns = f"/{robot_id}"

        # --- 서비스 타입 import (ros2 interface show가 깨져도 python import는 정상) ---
        from dsr_msgs2.srv import MoveJoint, MoveLine, MoveHome
        from dsr_msgs2.srv import SetRobotMode
        from dsr_msgs2.srv import SetCurrentTool, SetCurrentTcp
        from dsr_msgs2.srv import SetCtrlBoxDigitalOutput, SetToolDigitalOutput

        self.MoveJoint = MoveJoint
        self.MoveLine = MoveLine
        self.MoveHome = MoveHome
        self.SetRobotMode = SetRobotMode
        self.SetCurrentTool = SetCurrentTool
        self.SetCurrentTcp = SetCurrentTcp
        self.SetCtrlBoxDigitalOutput = SetCtrlBoxDigitalOutput
        self.SetToolDigitalOutput = SetToolDigitalOutput

        # --- motion ---
        self.cli_move_joint = node.create_client(MoveJoint, f"{self.ns}/motion/move_joint")
        self.cli_move_line = node.create_client(MoveLine, f"{self.ns}/motion/move_line")
        self.cli_move_home = node.create_client(MoveHome, f"{self.ns}/motion/move_home")

        # --- system/tool/tcp ---
        self.cli_set_robot_mode = node.create_client(SetRobotMode, f"{self.ns}/system/set_robot_mode")
        self.cli_set_tool = node.create_client(SetCurrentTool, f"{self.ns}/tool/set_current_tool")
        self.cli_set_tcp = node.create_client(SetCurrentTcp, f"{self.ns}/tcp/set_current_tcp")

        # --- IO ---
        self.cli_set_ctrl_do = node.create_client(SetCtrlBoxDigitalOutput, f"{self.ns}/io/set_ctrl_box_digital_output")
        self.cli_set_tool_do = node.create_client(SetToolDigitalOutput, f"{self.ns}/io/set_tool_digital_output")

    # -------------------------
    # 내부 유틸
    # -------------------------
    def _wait_future(self, fut, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while not fut.done():
            if timeout_sec is not None and (time.monotonic() - t0) > timeout_sec:
                return False
            time.sleep(0.01)
        return True

    def _call(self, client, req, timeout_sec: float = 5.0):
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

    # -------------------------
    # system/tool/tcp
    # -------------------------
    def set_robot_mode(self, mode_value: int):
        req = self.SetRobotMode.Request()
        # 필드명이 다를 수 있어 후보 처리
        if hasattr(req, "robot_mode"):
            req.robot_mode = int(mode_value)
        elif hasattr(req, "mode"):
            req.mode = int(mode_value)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, int(mode_value))
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

    # -------------------------
    # motion
    # -------------------------
    def movej(self, j_deg: Sequence[float], vel: float, acc: float, t: float = 2.0):
        req = self.MoveJoint.Request()
        # ✅ MoveJoint는 pos(double[6]) rad 기반 동작 확인됨
        req.pos = list(self._deg_list_to_rad(j_deg))
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = float(t)
        return self._call(self.cli_move_joint, req, timeout_sec=10.0)

    def movel(self, x: Sequence[float], vel: float, acc: float, t: float = 2.0):
        req = self.MoveLine.Request()
        fields = req._fields_and_field_types

        if "pos" in fields:
            req.pos = [float(v) for v in x]
        else:
            for k in ("target", "posx", "x"):
                if k in fields:
                    setattr(req, k, [float(v) for v in x])
                    break

        if "vel" in fields:
            req.vel = float(vel)
        if "acc" in fields:
            req.acc = float(acc)
        if "time" in fields:
            req.time = float(t)

        return self._call(self.cli_move_line, req, timeout_sec=12.0)

    # -------------------------
    # IO (gripper_io 호환)
    # -------------------------
    def wait(self, sec: float):
        time.sleep(float(sec))

    def set_ctrl_box_digital_output(self, index: int, value: int):
        req = self.SetCtrlBoxDigitalOutput.Request()
        # 필드 후보 대응
        if hasattr(req, "index"):
            req.index = int(index)
        elif hasattr(req, "port"):
            req.port = int(index)
        elif hasattr(req, "channel"):
            req.channel = int(index)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, int(index))

        # 값 필드
        if hasattr(req, "value"):
            req.value = int(value)
        elif hasattr(req, "val"):
            req.val = int(value)
        elif hasattr(req, "state"):
            req.state = int(value)
        else:
            # 두 번째 슬롯에 넣기 시도
            if len(req.__slots__) >= 2:
                slot2 = req.__slots__[1].lstrip("_")
                setattr(req, slot2, int(value))

        return self._call(self.cli_set_ctrl_do, req, timeout_sec=2.0)

    def set_tool_digital_output(self, index: int, value: int):
        req = self.SetToolDigitalOutput.Request()
        if hasattr(req, "index"):
            req.index = int(index)
        elif hasattr(req, "port"):
            req.port = int(index)
        elif hasattr(req, "channel"):
            req.channel = int(index)
        else:
            slot = req.__slots__[0].lstrip("_")
            setattr(req, slot, int(index))

        if hasattr(req, "value"):
            req.value = int(value)
        elif hasattr(req, "val"):
            req.val = int(value)
        elif hasattr(req, "state"):
            req.state = int(value)
        else:
            if len(req.__slots__) >= 2:
                slot2 = req.__slots__[1].lstrip("_")
                setattr(req, slot2, int(value))

        return self._call(self.cli_set_tool_do, req, timeout_sec=2.0)

    def set_digital_output(self, index: int, value: int):
        """gripper_io가 호출하는 이름. ctrl DO → 실패 시 tool DO로 fallback."""
        try:
            resp = self.set_ctrl_box_digital_output(index, value)
            # 응답에 success 필드가 있으면 체크
            if hasattr(resp, "success") and not bool(resp.success):
                raise RuntimeError("ctrl DO response success=False")
            return resp
        except Exception as e:
            self.node.get_logger().warn(f"[IO] ctrl DO failed -> try tool DO ({e})")
            resp = self.set_tool_digital_output(index, value)
            return resp


class RackTransportAction(Node):
    def __init__(self):
        super().__init__("rack_transport_action", namespace=f"/{ROBOT_ID}")

        # 파라미터
        self.declare_parameter("dry_run", False)
        self.declare_parameter("skip_probe", True)

        self.declare_parameter("movej_vel", float(DEFAULT_MOVEJ_VEL))
        self.declare_parameter("movej_acc", float(DEFAULT_MOVEJ_ACC))
        self.declare_parameter("movel_vel", float(DEFAULT_MOVEL_VEL))
        self.declare_parameter("movel_acc", float(DEFAULT_MOVEL_ACC))

        # ✅ set_robot_mode 경고 방지: 기본은 False
        self.declare_parameter("set_robot_mode_enable", False)
        self.declare_parameter("robot_mode_value", 1)  # 필요 시 바꿔서 사용

        # DR_init 세팅
        DR_init.__dsr__node = self
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL

        # 브릿지
        self.dr = DoosanMotionBridge(self, ROBOT_ID)

        # 스테이션
        self.rack_stations: Dict[str, Dict[str, list]] = {}
        self.wb_station: Optional[Dict[str, list]] = None
        self._load_stations()

        # IO
        from babo.gripper_io import grip_open, grip_close, grip_init_open
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.grip_init_open = grip_init_open

        # Action server
        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("✅ [v2.600] rack_transport_action ready (motion+io bridge enabled)")

    def _load_stations(self):
        try:
            from babo.rack_stations import build_rack_stations, build_workbench_station_dy
            self.rack_stations = build_rack_stations(self.dr, approach_dy=-250.0)
            self.wb_station = build_workbench_station_dy(self.dr)
            self.get_logger().info(f"✅ stations loaded: rack={len(self.rack_stations)} wb={'ok' if self.wb_station else 'none'}")
        except Exception as e:
            self.get_logger().error(f"❌ station load failed: {e}")
            self.rack_stations = {}
            self.wb_station = None

    def _ensure_ready(self) -> Tuple[bool, str]:
        """모드/툴/tcp 보장. set_robot_mode는 옵션."""
        try:
            if bool(self.get_parameter("set_robot_mode_enable").value):
                mode_val = int(self.get_parameter("robot_mode_value").value)
                self.dr.set_robot_mode(mode_val)

            # tool/tcp는 실패해도 모션 자체는 가능하니 경고만
            try:
                self.dr.set_tool(ROBOT_TOOL)
            except Exception as e:
                self.get_logger().warn(f"[INIT] set_tool failed: {e}")
            try:
                self.dr.set_tcp(ROBOT_TCP)
            except Exception as e:
                self.get_logger().warn(f"[INIT] set_tcp failed: {e}")

            return True, "ready"
        except Exception as e:
            return False, f"init failed: {e}"

    def _home(self):
        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)

    async def execute_callback(self, goal_handle):
        raw_cmd = (goal_handle.request.command or "").strip()
        self.get_logger().info(f"📥 cmd: {raw_cmd}")

        if bool(self.get_parameter("dry_run").value):
            goal_handle.publish_feedback(RobotMove.Feedback(status="[DRY_RUN] accepted"))
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
                if not dest:
                    raise ValueError("IN needs dest")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"IN:{dest}"))
                ok, m = self._do_in(dest)

            elif cmd == "OUT":
                src = a
                if not src:
                    raise ValueError("OUT needs src")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"OUT:{src}"))
                ok, m = self._do_out(src)

            elif cmd == "MOVE":
                src, dest = a, b
                if not src or not dest:
                    raise ValueError("MOVE needs src,dest")
                goal_handle.publish_feedback(RobotMove.Feedback(status=f"MOVE:{src}->{dest}"))
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
        if self.wb_station is None:
            return False, "wb station missing"
        st = self.rack_stations.get(dest)
        if not st:
            return False, f"unknown rack: {dest}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)
        self.grip_init_open(self.dr)

        self.dr.movel(self.wb_station["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(self.wb_station["target"], vel=vl, acc=al, t=2.0)

        # probe 스킵(virtual)
        self.grip_close(self.dr)

        self.dr.movel(self.wb_station["approach"], vel=vl, acc=al, t=2.0)

        self.dr.movel(st["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(st["target"], vel=vl, acc=al, t=2.0)

        self.grip_open(self.dr)

        self.dr.movel(st["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)

        return True, f"inbound to {dest} done"

    def _do_out(self, src: str) -> Tuple[bool, str]:
        if self.wb_station is None:
            return False, "wb station missing"
        st = self.rack_stations.get(src)
        if not st:
            return False, f"unknown rack: {src}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)
        self.grip_init_open(self.dr)

        self.dr.movel(st["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(st["target"], vel=vl, acc=al, t=2.0)
        self.grip_close(self.dr)
        self.dr.movel(st["approach"], vel=vl, acc=al, t=2.0)

        self.dr.movel(self.wb_station["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(self.wb_station["target"], vel=vl, acc=al, t=2.0)
        self.grip_open(self.dr)
        self.dr.movel(self.wb_station["approach"], vel=vl, acc=al, t=2.0)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)
        return True, f"outbound from {src} done"

    def _do_move(self, src: str, dest: str) -> Tuple[bool, str]:
        st_src = self.rack_stations.get(src)
        st_dst = self.rack_stations.get(dest)
        if not st_src or not st_dst:
            return False, f"unknown rack: {src}->{dest}"

        vj = float(self.get_parameter("movej_vel").value)
        aj = float(self.get_parameter("movej_acc").value)
        vl = float(self.get_parameter("movel_vel").value)
        al = float(self.get_parameter("movel_acc").value)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)
        self.grip_init_open(self.dr)

        self.dr.movel(st_src["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(st_src["target"], vel=vl, acc=al, t=2.0)
        self.grip_close(self.dr)
        self.dr.movel(st_src["approach"], vel=vl, acc=al, t=2.0)

        self.dr.movel(st_dst["approach"], vel=vl, acc=al, t=2.0)
        self.dr.movel(st_dst["target"], vel=vl, acc=al, t=2.0)
        self.grip_open(self.dr)
        self.dr.movel(st_dst["approach"], vel=vl, acc=al, t=2.0)

        self.dr.movej(HOME_J_DEG, vel=vj, acc=aj, t=2.0)
        return True, f"move {src}->{dest} done"


def main(args=None):
    rclpy.init(args=args)
    node = RackTransportAction()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
