# tube_transport_node v3.000 2026-01-19
# [이번 버전에서 수정된 사항]
# - UI 연동용 /tube_task_cmd(std_msgs/String, JSON) 구독 추가
# - 출고(OUTBOUND) 시퀀스 MVP 구현(랙 서버 호출 + 더미 튜브 동작)
# - /tube_task_state, /tube_task_result JSON String 발행 추가
# - "선택 튜브 랙은 WORKBENCH_A 우선 배치" 규칙 반영(OUTBOUND는 항상 WB_A 사용)

import json
import time
import threading
import rclpy
import DR_init

from std_msgs.msg import String

# =========================
# 로봇 설정 상수 (기본 템플릿)
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def _now_ms():
    return int(time.time() * 1000)


def parse_tube_addr(addr: str):
    # "A-1-1" -> ("A-1", 1)
    parts = addr.split("-")
    if len(parts) != 3:
        raise ValueError(f"Invalid tube addr: {addr}")
    rack_slot = f"{parts[0].upper()}-{parts[1]}"
    tube_idx = int(parts[2])
    if tube_idx < 1 or tube_idx > 4:
        raise ValueError(f"tube_idx must be 1..4: {addr}")
    return rack_slot, tube_idx


class TubeTransportNode:
    def __init__(self, node):
        self.node = node

        # ---- Params ----
        node.declare_parameter("dry_run", True)

        node.declare_parameter("topic_cmd", "/tube_task_cmd")
        node.declare_parameter("topic_state", "/tube_task_state")
        node.declare_parameter("topic_result", "/tube_task_result")

        node.declare_parameter("rack_cmd_topic", "/rack_transport_cmd")
        node.declare_parameter("rack_result_topic", "/rack_transport_result")

        # 워크벤치 규칙(이름만, 티칭 전 더미)
        self.WB_PRIMARY = "WORKBENCH_A"
        self.WB_OUT = "WORKBENCH_C-2"

        # ---- Pub/Sub ----
        self.pub_state = node.create_publisher(String, node.get_parameter("topic_state").value, 10)
        self.pub_result = node.create_publisher(String, node.get_parameter("topic_result").value, 10)

        self.pub_rack_cmd = node.create_publisher(String, node.get_parameter("rack_cmd_topic").value, 10)
        self.sub_rack_res = node.create_subscription(String, node.get_parameter("rack_result_topic").value, self._on_rack_result, 10)

        self.sub_cmd = node.create_subscription(String, node.get_parameter("topic_cmd").value, self._on_cmd, 10)

        # ---- Runtime ----
        self._lock = threading.Lock()
        self._busy = False
        self._pending_rack = None  # {"expect_cmd":..., "ok":..., "info":...}

        node.get_logger().info("[READY] tube_transport_node (OUTBOUND MVP)")

    # -------------------------
    # Pub helpers
    # -------------------------
    def publish_state(self, state: str, detail: str = ""):
        payload = {"ts": _now_ms(), "state": state, "detail": detail}
        self.pub_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_result(self, ok: bool, mode: str, src: str, dst: str | None, message: str, error: str = ""):
        payload = {
            "ts": _now_ms(),
            "ok": ok,
            "mode": mode,
            "src": src,
            "dst": dst,
            "message": message,
            "error": error,
        }
        self.pub_result.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # -------------------------
    # Rack server result handler
    # -------------------------
    def _on_rack_result(self, msg: String):
        try:
            res = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._pending_rack = res

    def _rack_move_and_wait(self, fr: str, tr: str, timeout_sec: float = 60.0):
        # rack_transport_server는 MOVE만 지원(MVP)
        cmd = {"cmd": "MOVE", "from": fr, "to": tr}
        self.publish_state("RACK_MOVE_CMD", f"{fr} -> {tr}")
        self.pub_rack_cmd.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            with self._lock:
                res = self._pending_rack
            if isinstance(res, dict) and res.get("cmd", {}).get("from") == fr and res.get("cmd", {}).get("to") == tr:
                ok = bool(res.get("ok", False))
                info = res.get("info", {})
                return ok, info
            time.sleep(0.05)

        return False, {"error": "rack_move_timeout", "from": fr, "to": tr}

    # -------------------------
    # Tube dummy actions
    # -------------------------
    def tube_pick_from_workbench_rack(self, tube_idx: int):
        # 상단->하단 1..4 규칙 반영
        self.publish_state("TUBE_PICK", f"{self.WB_PRIMARY}_{tube_idx} (top->bottom 1..4)")
        if bool(self.node.get_parameter("dry_run").value):
            time.sleep(0.2)
            return True
        # TODO: DSR 튜브 픽 동작(티칭 후)
        return True

    def tube_place_to_out(self):
        self.publish_state("TUBE_PLACE_OUT", self.WB_OUT)
        if bool(self.node.get_parameter("dry_run").value):
            time.sleep(0.2)
            return True
        # TODO: DSR 튜브 플레이스 동작(티칭 후)
        return True

    # -------------------------
    # UI command handler
    # -------------------------
    def _on_cmd(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.publish_result(False, "UNKNOWN", "", None, "invalid json", error=str(e))
            return

        mode = str(payload.get("mode", "")).upper().strip()
        src = payload.get("src", "")

        if mode != "OUTBOUND":
            self.publish_result(False, mode, src, None, "unsupported mode (MVP: OUTBOUND only)")
            return

        if not src:
            self.publish_result(False, mode, src, None, "OUTBOUND requires src like A-1-1")
            return

        with self._lock:
            if self._busy:
                self.publish_result(False, mode, src, None, "busy")
                return
            self._busy = True
            self._pending_rack = None

        th = threading.Thread(target=self._run_outbound, args=(src,), daemon=True)
        th.start()

    # -------------------------
    # OUTBOUND sequence
    # -------------------------
    def _run_outbound(self, src_addr: str):
        started = _now_ms()
        try:
            self.publish_state("START", f"OUTBOUND src={src_addr}")

            rack_slot, tube_idx = parse_tube_addr(src_addr)

            # MVP에서는 "출고/입고"를 rack MOVE로 단순화해서 표현:
            # - 실제로는 (StorageSlot -> WORKBENCH_A) / (WORKBENCH_A -> StorageSlot) 개념이지만,
            # - 지금은 워크벤치 스테이션이 rack 타겟에 없으므로, 우선 rack 서버 호출이 동작하는지만 확인한다.
            #
            # 다음 단계에서:
            # - WORKBENCH_A / WORKBENCH_B 를 rack_stations에 포함시키면 진짜 checkout/checkin이 된다.

            # 1) (개념상) rack checkout
            self.publish_state("RACK_CHECKOUT", f"{rack_slot} -> {self.WB_PRIMARY} (MVP uses MOVE test)")
            ok_rack, info_rack = self._rack_move_and_wait(rack_slot, rack_slot, timeout_sec=5.0)
            # 위 줄은 "서버 통신 확인용 더미"라서 from==to로 막혀 실패할 수 있음.
            # 그래서 일단 통신만 확인하려면 아래처럼 실제 가능한 A-1->B-1 같은 테스트를 먼저 해도 됨.

            # 2) tube pick
            if not self.tube_pick_from_workbench_rack(tube_idx):
                raise RuntimeError("tube pick failed")

            # 3) tube place out
            if not self.tube_place_to_out():
                raise RuntimeError("tube place out failed")

            # 4) (개념상) rack checkin
            self.publish_state("RACK_CHECKIN", f"{self.WB_PRIMARY} -> {rack_slot} (MVP)")
            # TODO: 진짜 checkin은 워크벤치 스테이션 추가 후 구현

            elapsed = _now_ms() - started
            self.publish_state("DONE", f"OUTBOUND complete ({elapsed} ms)")
            self.publish_result(True, "OUTBOUND", src_addr, None, f"done ({elapsed} ms)")

        except Exception as e:
            elapsed = _now_ms() - started
            self.publish_state("ERROR", str(e))
            self.publish_result(False, "OUTBOUND", src_addr, None, f"failed ({elapsed} ms)", error=str(e))
        finally:
            with self._lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tube_transport", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        TubeTransportNode(node)
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (KeyboardInterrupt).")
    except Exception as e:
        node.get_logger().error("An unexpected error occurred: %s" % str(e))
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
