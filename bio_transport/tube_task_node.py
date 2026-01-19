# tube_task_node v3.000
# [이번 버전에서 수정된 사항]
# - UI 연동용 /tube_task_cmd(std_msgs/String, JSON) 구독 추가
# - 출고(OUTBOUND) 시퀀스 MVP 구현(더미 튜브 동작 + 기존 rack_transport 토픽 호출)
# - /tube_task_state, /tube_task_result 토픽으로 진행상태/결과를 JSON String으로 발행

import json
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# -------------------------
# Helpers
# -------------------------
def _now_ms() -> int:
    return int(time.time() * 1000)


def parse_tube_addr(addr: str):
    """
    addr: "A-1-1" (StorageSlot-TubeIndex)
    returns: ("A-1", 1)
    """
    parts = addr.split("-")
    if len(parts) != 3:
        raise ValueError(f"Invalid tube addr: {addr}")
    rack_slot = f"{parts[0]}-{parts[1]}"
    tube_idx = int(parts[2])
    if tube_idx < 1 or tube_idx > 4:
        raise ValueError(f"tube_idx must be 1..4: {addr}")
    return rack_slot, tube_idx


@dataclass
class TubeTask:
    mode: str
    src: str | None = None
    dst: str | None = None
    raw: dict | None = None


# -------------------------
# Node
# -------------------------
class TubeTaskNode(Node):
    def __init__(self):
        super().__init__("tube_task_node")

        # ===== Params (설정값은 파라미터로!) =====
        self.declare_parameter("dry_run", True)
        self.declare_parameter("retry_count", 1)

        # UI <-> Tube
        self.declare_parameter("topic_cmd", "/tube_task_cmd")
        self.declare_parameter("topic_state", "/tube_task_state")
        self.declare_parameter("topic_result", "/tube_task_result")

        # Tube -> Rack transport bridge
        # (기존 rack transport 노드가 구독하는 토픽명에 맞춰서 바꿔주면 됨)
        self.declare_parameter("rack_cmd_topic", "/rack_transport_cmd")

        # Workbench IO points (지금은 더미 이름만)
        self.WB_RACK_PRIMARY = "WORKBENCH_A"   # 선택 튜브 랙은 항상 여기로 먼저
        self.WB_OUT = "WORKBENCH_C-2"          # 출고 튜브 내려놓는 위치

        # Pub/Sub
        topic_cmd = self.get_parameter("topic_cmd").value
        topic_state = self.get_parameter("topic_state").value
        topic_result = self.get_parameter("topic_result").value

        self.sub_cmd = self.create_subscription(String, topic_cmd, self._on_cmd, 10)
        self.pub_state = self.create_publisher(String, topic_state, 10)
        self.pub_result = self.create_publisher(String, topic_result, 10)

        rack_cmd_topic = self.get_parameter("rack_cmd_topic").value
        self.pub_rack_cmd = self.create_publisher(String, rack_cmd_topic, 10)

        # Worker control
        self._lock = threading.Lock()
        self._busy = False
        self._current_task: TubeTask | None = None

        self.get_logger().info("TubeTaskNode ready (OUTBOUND MVP).")

    # -------------------------
    # Pub helpers
    # -------------------------
    def publish_state(self, state: str, detail: str = ""):
        msg = {"ts": _now_ms(), "state": state, "detail": detail}
        self.pub_state.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def publish_result(self, ok: bool, mode: str, src: str | None, dst: str | None, message: str, error: str = ""):
        msg = {
            "ts": _now_ms(),
            "ok": ok,
            "mode": mode,
            "src": src,
            "dst": dst,
            "message": message,
            "error": error,
        }
        self.pub_result.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    # -------------------------
    # Rack transport bridge
    # -------------------------
    def rack_checkout(self, rack_slot: str, workbench: str):
        """
        기존 rack_transport 노드가 받을 명령을 publish.
        토픽/포맷은 너희 기존 rack 로직에 맞춰 조정하면 됨.
        """
        # 예시 포맷: {"cmd":"CHECKOUT","slot":"A-1","to":"WORKBENCH_A"}
        cmd = {"cmd": "CHECKOUT", "slot": rack_slot, "to": workbench}
        self.pub_rack_cmd.publish(String(data=json.dumps(cmd, ensure_ascii=False)))
        self.publish_state("RACK_CHECKOUT_CMD", f"{rack_slot} -> {workbench}")

    def rack_checkin(self, rack_slot: str, from_workbench: str):
        cmd = {"cmd": "CHECKIN", "slot": rack_slot, "from": from_workbench}
        self.pub_rack_cmd.publish(String(data=json.dumps(cmd, ensure_ascii=False)))
        self.publish_state("RACK_CHECKIN_CMD", f"{from_workbench} -> {rack_slot}")

    # -------------------------
    # Tube dummy actions (티칭 전이라 로그만)
    # -------------------------
    def tube_pick_from_workbench_rack(self, workbench: str, tube_idx: int):
        # 실제 구현 시: WORKBENCH_A_1..4 포즈로 approach/target + 그립 close
        self.publish_state("TUBE_PICK", f"{workbench}_{tube_idx} (top->bottom 1..4)")
        if self.get_parameter("dry_run").value:
            time.sleep(0.2)  # 더미 딜레이
            return True
        # TODO: DSR 동작 호출
        return True

    def tube_place_to_out(self, out_name: str):
        self.publish_state("TUBE_PLACE_OUT", out_name)
        if self.get_parameter("dry_run").value:
            time.sleep(0.2)
            return True
        # TODO: DSR 동작 호출
        return True

    # -------------------------
    # Command handler
    # -------------------------
    def _on_cmd(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.publish_result(False, "UNKNOWN", None, None, "invalid json", error=str(e))
            return

        mode = str(payload.get("mode", "")).upper().strip()
        src = payload.get("src", None)
        dst = payload.get("dst", None)

        if mode not in ("OUTBOUND",):  # 지금은 출고만
            self.publish_result(False, mode, src, dst, "unsupported mode (MVP: OUTBOUND only)")
            return

        if mode == "OUTBOUND" and (not src or not isinstance(src, str)):
            self.publish_result(False, mode, src, dst, "OUTBOUND requires src like A-1-1")
            return

        task = TubeTask(mode=mode, src=src, dst=dst, raw=payload)

        with self._lock:
            if self._busy:
                self.publish_result(False, mode, src, dst, "busy")
                return
            self._busy = True
            self._current_task = task

        th = threading.Thread(target=self._run_task, args=(task,), daemon=True)
        th.start()

    # -------------------------
    # Task runner (OUTBOUND)
    # -------------------------
    def _run_task(self, task: TubeTask):
        ok = False
        err = ""
        started = _now_ms()

        try:
            self.publish_state("START", f"mode={task.mode}")

            if task.mode == "OUTBOUND":
                ok = self._run_outbound(task.src)
            else:
                ok = False
                err = "unsupported mode"

        except Exception as e:
            ok = False
            err = str(e)

        elapsed = _now_ms() - started
        if ok:
            self.publish_result(True, task.mode, task.src, task.dst, f"done ({elapsed} ms)")
        else:
            self.publish_result(False, task.mode, task.src, task.dst, f"failed ({elapsed} ms)", error=err)

        with self._lock:
            self._busy = False
            self._current_task = None

    def _run_outbound(self, src_addr: str) -> bool:
        rack_slot, tube_idx = parse_tube_addr(src_addr)

        # 1) rack checkout to WORKBENCH_A
        self.publish_state("RACK_CHECKOUT", f"{rack_slot} -> {self.WB_RACK_PRIMARY}")
        self.rack_checkout(rack_slot, self.WB_RACK_PRIMARY)

        # TODO: 여기서 rack_transport 완료 신호(ack/result)를 기다리는 구조로 발전 필요
        # MVP: 딜레이로 대체
        time.sleep(0.5)

        # 2) tube pick from rack on workbench
        if not self.tube_pick_from_workbench_rack(self.WB_RACK_PRIMARY, tube_idx):
            raise RuntimeError("tube pick failed")

        # 3) tube place to OUT position
        if not self.tube_place_to_out(self.WB_OUT):
            raise RuntimeError("tube place out failed")

        # 4) rack checkin back to storage slot
        self.publish_state("RACK_CHECKIN", f"{self.WB_RACK_PRIMARY} -> {rack_slot}")
        self.rack_checkin(rack_slot, self.WB_RACK_PRIMARY)
        time.sleep(0.5)

        self.publish_state("DONE", "OUTBOUND complete")
        return True


def main():
    rclpy.init()
    node = TubeTaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
