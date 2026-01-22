# rack_transport_action v3.300 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능변경) execute_callback을 async → sync(def)로 고정(이벤트루프/asyncio 혼선 제거)
# - (안정화) 동시 goal 유입 방지: 내부 exec_lock으로 Busy면 즉시 abort 처리
# - (유지) Action 이름 /robot_action 고정, QoS는 서비스/토픽 모두 VOLATILE+RELIABLE

"""[모듈] rack_transport_action

[역할]
- /robot_action (RobotMove) goal.command를 받아 Doosan 로봇 동작을 실행한다.

[Action]
- name: /robot_action
- goal.command 예:
  - MOVE,A-1,B-2
  - IN,NONE,A-1
  - OUT,A-1,NONE
"""

from __future__ import annotations

import re
import threading
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import DR_init

try:
    from biobank_interfaces.action import RobotMove
except ImportError:
    class RobotMove:  # pragma: no cover
        class Goal:
            command = ""
        class Result:
            def __init__(self, success=False, message=""):
                self.success = success
                self.message = message
        class Feedback:
            status = ""


# ==========================================================
# ROBOT 상수
# ==========================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================================
# 기본 모션 파라미터
# ==========================================================
V_J = 60
A_J = 60

V_L = 200.0
A_L = 200.0

V_L_SLOW = 50.0
A_L_SLOW = 50.0

HOME_J_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

MOVE_PICK_APP_DY = -250.0
MOVE_PICK_LIFT_DZ = 30.0
MOVE_PICK_RET_DY = -250.0

MOVE_PLACE_APP_DZ = 30.0
MOVE_PLACE_RET_DY = -250.0

IN_WB_APP_DY = -50.0
IN_RACK_APP_DY = -250.0
IN_TOOL_LIFT_Z = 20.0
IN_BASE_LIFT_Z = 250.0
IN_TARGET_TOP_DZ = 20.0
IN_FINAL_RETRACT_DY = -150.0

OUT_RACK_APP_DY = -100.0
OUT_WB_APP_DZ = 100.0
GRIP_WAIT_SEC = 1.0


def _norm(s: Optional[str]) -> Optional[str]:
    if s is None:
        return None
    t = str(s).strip()
    if t == "" or t.upper() == "NONE":
        return None
    return t


def _normalize_rack_key(raw: Optional[str]) -> str:
    if raw is None:
        return ""
    s = str(raw).strip().upper().replace("_", "-")
    s = re.sub(r"\s+", "", s)

    m = re.match(r"^([A-Z])\-([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.match(r"^([A-Z])([0-9]+)$", s)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    return s


def _apply_offset(dr, pose: Sequence[float], dx=0.0, dy=0.0, dz=0.0):
    return dr.posx(
        float(pose[0]) + float(dx),
        float(pose[1]) + float(dy),
        float(pose[2]) + float(dz),
        float(pose[3]),
        float(pose[4]),
        float(pose[5]),
    )


def initialize_robot(node: Node):
    import DSR_ROBOT2 as dr
    node.get_logger().info("#" * 50)
    node.get_logger().info("Initializing robot with the following settings:")
    node.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
    node.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
    node.get_logger().info(f"ROBOT_TCP: {ROBOT_TCP}")
    node.get_logger().info(f"ROBOT_TOOL: {ROBOT_TOOL}")
    node.get_logger().info("#" * 50)

    dr.set_tool(ROBOT_TOOL)
    dr.set_tcp(ROBOT_TCP)
    return dr


class RackTransportAction(Node):
    def __init__(self):
        super().__init__("rack_transport_action", namespace=ROBOT_ID)

        self.dr = None
        self.declare_parameter("dry_run", False)

        # 동시 실행 방지
        self._exec_lock = threading.Lock()

        from .rack_stations import (
            build_rack_stations,
            build_workbench_station_dy,
            build_workbench_station_top,
            RACK_TARGETS,
        )
        self.RACK_TARGETS = RACK_TARGETS
        self.build_rack_stations = build_rack_stations
        self.build_wb_dy = build_workbench_station_dy
        self.build_wb_top = build_workbench_station_top

        from .gripper_io import grip_open, grip_close, grip_init_open
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.grip_init_open = grip_init_open

        from .rel_move import rel_movel_tool, rel_movel_base
        self.rel_movel_tool = rel_movel_tool
        self.rel_movel_base = rel_movel_base

        srv_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topic_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._server = ActionServer(
            self,
            RobotMove,
            "/robot_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_service_qos_profile=srv_qos,
            result_service_qos_profile=srv_qos,
            cancel_service_qos_profile=srv_qos,
            feedback_pub_qos_profile=topic_qos,
            status_pub_qos_profile=topic_qos,
        )

        self.get_logger().info("✅ [v3.300] /robot_action ready (sync execute)")

    def set_dr(self, dr):
        self.dr = dr

    def _home(self):
        home_j = self.dr.posj(*HOME_J_DEG)
        self.dr.movej(home_j, vel=V_J, acc=A_J)

    def _valid_keys(self):
        keys = list(self.RACK_TARGETS.keys())
        keys.sort()
        return keys

    def execute_callback(self, goal_handle):
        # Busy면 즉시 실패
        if not self._exec_lock.acquire(blocking=False):
            goal_handle.abort()
            return RobotMove.Result(success=False, message="Robot busy (another goal is running)")

        try:
            cmd = (goal_handle.request.command or "").strip()
            self.get_logger().info(f"📥 EXEC: {cmd}")

            if bool(self.get_parameter("dry_run").value):
                goal_handle.succeed()
                return RobotMove.Result(success=True, message="Dry Run Success")

            if self.dr is None:
                goal_handle.abort()
                return RobotMove.Result(success=False, message="Robot not initialized (dr is None)")

            parts = [p.strip() for p in cmd.split(",") if p.strip() != ""]
            op = parts[0].upper() if parts else ""
            a1 = _norm(parts[1]) if len(parts) > 1 else None
            a2 = _norm(parts[2]) if len(parts) > 2 else None

            src = _normalize_rack_key(a1) if a1 else None
            dst = _normalize_rack_key(a2) if a2 else None

            try:
                if op == "MOVE":
                    if not src or not dst:
                        raise ValueError("MOVE requires src & dest")
                    ok, msg = self._do_transport(src, dst)
                elif op == "IN":
                    if not dst:
                        raise ValueError("IN requires dest")
                    ok, msg = self._do_inbound(dst)
                elif op == "OUT":
                    if not src:
                        raise ValueError("OUT requires src")
                    ok, msg = self._do_outbound(src)
                else:
                    ok, msg = False, f"Unknown op: {op}"
            except Exception as e:
                ok, msg = False, f"Error: {e}"

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return RobotMove.Result(success=ok, message=msg)

        finally:
            try:
                self._exec_lock.release()
            except Exception:
                pass

    # --------------------------
    # Robot motions
    # --------------------------
    def _do_transport(self, src: str, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid or dest not in valid:
            return False, f"Invalid rack key: src={src}, dest={dest}"
        if src == dest:
            return False, "src == dest"

        self.get_logger().info(f"[MOVE] {src} -> {dest}")

        rack = self.build_rack_stations(self.dr, approach_dy=MOVE_PICK_APP_DY)
        st_src = rack[src]
        st_dst = rack[dest]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        pick_app = _apply_offset(self.dr, st_src["target"], dy=MOVE_PICK_APP_DY)
        self.dr.movel(pick_app, vel=V_L, acc=A_L)
        self.dr.movel(st_src["target"], vel=V_L, acc=A_L)

        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        pick_lift = _apply_offset(self.dr, st_src["target"], dz=MOVE_PICK_LIFT_DZ)
        self.dr.movel(pick_lift, vel=V_L_SLOW, acc=A_L_SLOW)

        pick_ret = _apply_offset(self.dr, pick_lift, dy=MOVE_PICK_RET_DY)
        self.dr.movel(pick_ret, vel=V_L_SLOW, acc=A_L_SLOW)

        place_app = _apply_offset(self.dr, st_dst["target"], dz=MOVE_PLACE_APP_DZ)
        self.dr.movel(place_app, vel=V_L, acc=A_L)
        self.dr.movel(st_dst["target"], vel=V_L, acc=A_L)

        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        place_ret = _apply_offset(self.dr, st_dst["target"], dy=MOVE_PLACE_RET_DY)
        self.dr.movel(place_ret, vel=V_L_SLOW, acc=A_L_SLOW)

        self._home()
        return True, "Transport Done"

    def _do_inbound(self, dest: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if dest not in valid:
            return False, f"Invalid rack key: dest={dest}"

        self.get_logger().info(f"[IN] WB -> {dest}")

        wb = self.build_wb_dy(self.dr, approach_dy=IN_WB_APP_DY)
        rack = self.build_rack_stations(self.dr, approach_dy=IN_RACK_APP_DY)
        st = rack[dest]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        self.rel_movel_base(self.dr, 0, -180.0, 0, 0, 0, 0, 50.0)
        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        self.rel_movel_tool(self.dr, 0, 0, IN_TOOL_LIFT_Z, 0, 0, 0, 20.0)
        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        self.rel_movel_base(self.dr, 0, 0, IN_BASE_LIFT_Z, 0, 0, 0, 50.0)

        self.dr.movel(st["approach"], vel=V_L, acc=A_L)
        top = _apply_offset(self.dr, st["target"], dz=IN_TARGET_TOP_DZ)
        self.dr.movel(top, vel=V_L, acc=A_L)
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        ret = _apply_offset(self.dr, st["target"], dy=IN_FINAL_RETRACT_DY)
        self.dr.movel(ret, vel=V_L_SLOW, acc=A_L_SLOW)

        self._home()
        return True, "Inbound Done"

    def _do_outbound(self, src: str) -> Tuple[bool, str]:
        valid = self._valid_keys()
        if src not in valid:
            return False, f"Invalid rack key: src={src}"

        self.get_logger().info(f"[OUT] {src} -> WB")

        rack = self.build_rack_stations(self.dr, approach_dy=OUT_RACK_APP_DY)
        wb = self.build_wb_top(self.dr, approach_dz=OUT_WB_APP_DZ)
        st = rack[src]

        self._home()
        self.grip_init_open(self.dr, wait_sec=0.2)

        self.rel_movel_base(self.dr, 0, -100.0, 0, 0, 0, 0, 50.0)
        self.dr.movel(st["approach"], vel=V_L, acc=A_L)
        self.dr.movel(st["target"], vel=V_L, acc=A_L)

        self.grip_close(self.dr, wait_sec=GRIP_WAIT_SEC)

        self.rel_movel_tool(self.dr, 0, 0, IN_TOOL_LIFT_Z, 0, 0, 0, 20.0)
        self.rel_movel_base(self.dr, 0, -250.0, 0, 0, 0, 0, 50.0)

        self.dr.movel(wb["approach"], vel=V_L, acc=A_L)
        self.dr.movel(wb["target"], vel=V_L, acc=A_L)

        self.grip_open(self.dr, wait_sec=GRIP_WAIT_SEC)

        self.rel_movel_base(self.dr, 0, -50.0, 0, 0, 0, 0, 50.0)
        self.rel_movel_base(self.dr, 0, 0, 50.0, 0, 0, 0, 50.0)

        self._home()
        return True, "Outbound Done"


def main(args=None):
    rclpy.init(args=args)
    node = RackTransportAction()

    DR_init.__dsr__node = node
    dr = initialize_robot(node)
    node.set_dr(dr)

    executor = MultiThreadedExecutor(num_threads=4)
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
