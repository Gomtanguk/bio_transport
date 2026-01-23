# ui_integrated v2.423 2026-01-23
# [수정 사항]
# - 렉(Rack) 작업 시작 시(on_confirm_t2) 선택된 버튼/리스트를 즉시 초기화하도록 수정
#   (버튼이 '체크(주황색)' 상태로 남아있어 완료 후 색상 변화가 안 보이던 문제 해결)
# - 초기 재고 설정(A-2-2... 및 A-2, B-1) 유지

"""[모듈] ui_integrated

[역할]
- PySide6 기반 UI
- Rack 작업: BioCommand ActionClient -> /bio_main_control
- Tube 작업: BioCommand ActionClient -> /tube_main_control
  - UI는 문자열 1줄만 전송
    - IN   : "TUBE,IN,NONE,<dst>"
    - OUT  : "TUBE,OUT,<src>,NONE"
    - MOVE : "TUBE,MOVE,<src>,<dst>"
    - WASTE: "TUBE,WASTE,<src>,NONE"

[연동 흐름]
UI(ui_integrated)
  ├─ Rack: BioCommand(/bio_main_control)  ---> main_integrated ---> RobotMove(/robot_action) ---> rack_transport_action
  └─ Tube: BioCommand(/tube_main_control) ---> main_integrated(파싱/좌표계산) ---> TubeTransport(/tube_transport) ---> rack_transport_action
"""

import sys
import os
import re
import subprocess
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout,
    QLabel, QDialog, QToolButton, QPushButton, QRadioButton, QLineEdit,
    QFormLayout, QTextEdit, QSizePolicy, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from biobank_interfaces.action import BioCommand
except ImportError:  # pragma: no cover
    class BioCommand:
        class Goal:
            command = ""
        class Result:
            def __init__(self, success=True, message=""):
                self.success = success
                self.message = message
        class Feedback:
            def __init__(self, status=""):
                self.status = status


# =========================
# QoS (사용자 설정 유지)
# =========================
ACTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ========================================================
# [스타일시트] (사용자 기존 유지)
# ========================================================
STYLE_SHEET = """QWidget { font-family: "Segoe UI", "Malgun Gothic", sans-serif; color: #000000; }
QMainWindow { background-color: #F1F5F9; }

QRadioButton { font-size: 14px; font-weight: bold; color: #333333; padding: 4px; }
QTabWidget::pane { border: 1px solid #CBD5E1; background: #FFFFFF; border-radius: 6px; }
QTabBar::tab { background: #E2E8F0; color: #64748B; padding: 10px 25px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background: #FFFFFF; color: #2563EB; border-top: 3px solid #2563EB; }

QGroupBox {
    font-weight: bold; font-size: 20px;
    border: 2px solid #334155; border-radius: 8px; margin-top: 35px;
    background-color: #FFFFFF; color: #FFFFFF;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 10px; padding: 5px 15px;
    background-color: #334155; border-radius: 6px;
}

QFrame.RackFrame { background-color: #334155; border-radius: 6px; border: 1px solid #1E293B; }
QLineEdit { border: 1px solid #CBD5E1; border-radius: 4px; padding: 6px; background: #F8FAFC; color: #000000; }
QLineEdit:focus { border: 1px solid #2563EB; background: #FFFFFF; }
QTextEdit { background-color: #1E293B; color: #00FF00; font-family: "Consolas", monospace; font-size: 12px; border-radius: 4px; border: 1px solid #334155; }

QPushButton {
    background-color: #FFFFFF;
    border: 1px solid #CBD5E1;
    color: #333333;
    font-weight: bold;
    border-radius: 4px;
    padding: 8px;
    min-height: 35px;
}
QPushButton:pressed { background-color: #E2E8F0; padding-top: 10px; padding-bottom: 6px; }

QPushButton#btnConfirm {
    background-color: #2563EB;
    color: #000000;
    border: 1px solid #1D4ED8;
    border-bottom: 3px solid #1D4ED8;
    font-weight: bold;
    border-radius: 4px;
}
QPushButton#btnConfirm:hover { background-color: #000000; color: #000000; }
QPushButton#btnConfirm:pressed {
    background-color: #FFFFFF;
    color: #FFFFFF;
    border-bottom: 0px solid;
    border-top: 3px solid transparent;
    padding-top: 10px; padding-bottom: 6px;
}

QToolButton.TubeBtn { background-color: #F8FAFC; border: 2px solid #94A3B8; border-radius: 13px; width: 52px; height: 52px; margin: 4px; }
QToolButton.TubeBtn:checked { background-color: #F59E0B; border-color: #D97706; }

QToolButton.TubeBtnOccupied {
    background-color: #FECACA; border: 2px solid #EF4444;
    border-radius: 13px; width: 52px; height: 52px; margin: 4px;
}
QToolButton.TubeBtnOccupied:checked { background-color: #F59E0B; border-color: #D97706; }

QToolButton.TubeBtnBlocked {
    background-color: #FECACA; border: 2px solid #EF4444;
    border-radius: 13px; width: 52px; height: 52px; margin: 4px;
}

QPushButton.RackSelectBtn {
    background-color: #475569; color: #FFFFFF;
    border: 1px solid #64748B; border-radius: 4px;
    font-size: 18px; font-weight: bold; min-height: 30px;
}
QPushButton.RackSelectBtn:checked { background-color: #F59E0B; border-color: #D97706; color: #FFFFFF; }

QPushButton.RackSelectBtnOccupied {
    background-color: #FECACA; color: #B91C1C;
    border: 2px solid #EF4444; border-radius: 4px;
    font-size: 18px; font-weight: bold; min-height: 30px;
}
QPushButton.RackSelectBtnOccupied:checked { background-color: #F59E0B; border-color: #D97706; color: #FFFFFF; }

QPushButton.RackSelectBtnBlocked {
    background-color: #FECACA; color: #B91C1C;
    border: 2px solid #EF4444; border-radius: 4px;
    font-size: 18px; font-weight: bold; min-height: 30px;
}
"""


class UiActionClientNode(Node):
    """Qt 이벤트 루프와 rclpy를 함께 돌리기 위한 ActionClient 노드."""

    def __init__(self, ui):
        super().__init__("ui_integrated_client")
        self.ui = ui

        self.qos = ACTION_QOS
        cbg = ReentrantCallbackGroup()

        self.client = ActionClient(
            self, BioCommand, "/bio_main_control",
            callback_group=cbg,
            goal_service_qos_profile=self.qos,
            result_service_qos_profile=self.qos,
            cancel_service_qos_profile=self.qos,
            feedback_sub_qos_profile=self.qos,
            status_sub_qos_profile=self.qos,
        )

        self.tube_client = ActionClient(
            self, BioCommand, "/tube_main_control",
            callback_group=cbg,
            goal_service_qos_profile=self.qos,
            result_service_qos_profile=self.qos,
            cancel_service_qos_profile=self.qos,
            feedback_sub_qos_profile=self.qos,
            status_sub_qos_profile=self.qos,
        )

    # -------------------------
    # Rack (BioCommand)
    # -------------------------
    def send_rack_command(self, cmd_type: str, src: str, dest: str) -> bool:
        cmd_type = (cmd_type or "").strip().upper()
        src = (src or "NONE").strip()
        dest = (dest or "NONE").strip()
        final_cmd = f"RACK,{cmd_type},{src},{dest}"

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.ui.log_t2("❌ [Action] /bio_main_control 서버 연결 실패")
            return False

        goal = BioCommand.Goal()
        goal.command = final_cmd

        self.ui.log_t2(f"📤 [Action] 전송: {final_cmd}")
        fut = self.client.send_goal_async(goal, feedback_callback=self._on_rack_feedback)
        fut.add_done_callback(self._on_rack_goal_response)
        return True

    def _on_rack_feedback(self, feedback_msg):
        try:
            st = getattr(feedback_msg.feedback, "status", "")
            if st:
                self.ui.log_t2(f"🟡 [Feedback] {st}")
        except Exception:
            pass

    def _on_rack_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.ui.on_rack_action_result(False, f"Goal exception: {e}")
            return

        if not goal_handle.accepted:
            self.ui.on_rack_action_result(False, "Goal rejected")
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_rack_result)

    def _on_rack_result(self, future):
        try:
            res = future.result().result
            ok = bool(getattr(res, "success", False))
            msg = str(getattr(res, "message", ""))
        except Exception as e:
            ok = False
            msg = f"Result exception: {e}"

        self.ui.on_rack_action_result(ok, msg)

    # -------------------------
    # Tube (BioCommand)
    # -------------------------
    def send_tube_command_line(self, line: str) -> bool:
        if not self.tube_client.wait_for_server(timeout_sec=5.0):
            self.ui.log_t1("❌ [Tube] /tube_main_control 서버 연결 실패")
            return False

        goal = BioCommand.Goal()
        goal.command = str(line)

        self.ui.log_t1(f"📤 [Tube] 전송: {goal.command}")
        fut = self.tube_client.send_goal_async(goal, feedback_callback=self._on_tube_feedback)
        fut.add_done_callback(self._on_tube_goal_response)
        return True

    def _on_tube_feedback(self, feedback_msg):
        try:
            st = getattr(feedback_msg.feedback, "status", "")
            if st:
                self.ui.log_t1(f"🟡 [TubeFeedback] {st}")
        except Exception:
            pass

    def _on_tube_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.ui.on_tube_action_result(False, "GOAL_EXCEPTION", str(e))
            return

        if not goal_handle.accepted:
            self.ui.on_tube_action_result(False, "GOAL_REJECTED", "Goal rejected")
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_tube_result)

    def _on_tube_result(self, future):
        try:
            res = future.result().result
            ok = bool(getattr(res, "success", False))
            msg = str(getattr(res, "message", ""))
            err = "" if ok else "FAIL"
        except Exception as e:
            ok = False
            err = "RESULT_EXCEPTION"
            msg = str(e)

        self.ui.on_tube_action_result(ok, err, msg)


class BusyPopup(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("작동 중")
        self.setModal(True)
        self.setFixedSize(360, 180)
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(12)
        self.lbl = QLabel("작동 중입니다.", self)
        self.lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl.setWordWrap(True)
        self.lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #d32f2f;")
        v.addWidget(self.lbl, stretch=1)
        btn = QPushButton("닫기", self)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.clicked.connect(self.close)
        v.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

    def set_message(self, text: str):
        self.lbl.setText(str(text))


class BioBankApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BioBank System")
        self.resize(1300, 850)
        self.setStyleSheet(STYLE_SHEET)

        self.t1_mode_group = QButtonGroup(self)
        self.t2_mode_group = QButtonGroup(self)

        self.t1_selected_items = set()
        self.t1_dest_items = set()
        self.t1_active_buttons = set()

        self.t2_selected_items = set()
        self.t2_dest_items = set()
        self.t2_active_buttons = set()

        self.blocked_specific = ["A-1", "B-3"]
        self.blocked_prefix = ["C-", "D-"]

        self.inventory = set()
        # [초기 재고 설정]
        # 1. 튜브
        initial_tubes = ["A-3-3", "A-3-1", "B-1-2", "B-1-1"]
        for t in initial_tubes:
            self.inventory.add(t)
        # 2. 렉
        initial_racks = ["A-3", "B-1"]
        for r in initial_racks:
            self.inventory.add(r)
        self.widget_map = {}

        self._pending_rack_change = None
        self._pending_tube_change = None
        self._tube_job_queue = []
        self._tube_job_running = False

        self.ros_node = None

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.setup_tab1()
        self.setup_tab2()

        self.busy_overlay = QLabel("작동중!!!", self)
        self.busy_overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.busy_overlay.setFixedSize(300, 150)
        self.busy_overlay.hide()

        self._rack_job_running = False
        self._busy_reason = ""
        self._busy_popup = BusyPopup(self)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        x = (self.width() - self.busy_overlay.width()) // 2
        y = (self.height() - self.busy_overlay.height()) // 2
        self.busy_overlay.move(x, y)
        self.busy_overlay.raise_()

    def set_busy_state(self, is_busy: bool):
        if is_busy:
            x = (self.width() - self.busy_overlay.width()) // 2
            y = (self.height() - self.busy_overlay.height()) // 2
            self.busy_overlay.move(x, y)
            self.busy_overlay.show()
            self.busy_overlay.raise_()
            QApplication.processEvents()
        else:
            self.busy_overlay.hide()

        # NOTE:
        # - Busy overlay는 시각적 표시만 담당
        # - Busy 상태 플래그(_rack_job_running/_tube_job_running/_tube_job_queue)는
        #   Result/Queue 처리 로직에서만 변경한다.

    def set_ros_node(self, ros_node):
        self.ros_node = ros_node


    def _is_busy_global(self) -> bool:
        return bool(self._rack_job_running or self._tube_job_running or self._tube_job_queue)

    def _set_busy_reason(self, reason: str):
        self._busy_reason = str(reason or "").strip()

    @staticmethod
    def _clean_target_token(tok: str) -> str:
        s = (tok or "").strip()
        if not s:
            return ""
        if s.upper() == "NONE":
            return ""
        s = s.strip().strip("[]").strip().strip("'\"")
        return s

    def _format_busy_reason_rack(self, mode_id: int, sel_list, dest_list) -> str:
        mode_map = {1: "입고", 2: "출고", 3: "이동"}
        mode_txt = mode_map.get(int(mode_id or 0), str(mode_id))

        src = self._clean_target_token(sel_list[0]) if sel_list else ""
        dst = self._clean_target_token(dest_list[0]) if dest_list else ""

        target = ""
        if int(mode_id or 0) == 1:
            target = dst
        elif int(mode_id or 0) == 2:
            target = src
        elif int(mode_id or 0) == 3:
            if src and dst:
                target = f"{src}->{dst}"
            else:
                target = src or dst

        parts = ["RACK", mode_txt]
        if target:
            parts.append(target)
        return "".join([f"[{p}]" for p in parts])

    def show_busy_popup(self, extra: str = ""):
        detail = self._busy_reason or "알 수 없음"
        msg = "작동 중입니다.\n\n현재 작업: " + detail
        if extra:
            msg += "\n\n" + str(extra)
        self._busy_popup.set_message(msg)
        try:
            self._busy_popup.show()
            self._busy_popup.raise_()
            self._busy_popup.activateWindow()
        except Exception:
            pass

    def _auto_close_busy_popup_if_idle(self):
        if self._is_busy_global():
            return
        try:
            if self._busy_popup.isVisible():
                self._busy_popup.close()
        except Exception:
            pass

    def on_rack_action_result(self, success: bool, message: str):
        if success:
            self.log_t2(f"✅ [Result] 성공: {message}")
            if self._pending_rack_change is not None:
                mode, sel_list, dest_list = self._pending_rack_change
                self.process_inventory_change(mode, sel_list, dest_list)
        else:
            self.log_t2(f"❌ [Result] 실패: {message}")

        self._rack_job_running = False
        self._pending_rack_change = None

        if not self._tube_job_running and not self._tube_job_queue:
            self._set_busy_reason("")

        self._auto_close_busy_popup_if_idle()

    def on_tube_action_result(self, success: bool, error_code: str, message: str):
        if success:
            self.log_t1(f"✅ [Result] 성공: {message}")
            if self._pending_tube_change is not None:
                mode, sel_list, dest_list = self._pending_tube_change
                self.process_inventory_change(mode, sel_list, dest_list)
        else:
            self.log_t1(f"❌ [Result] 실패({error_code}): {message}")

        self._pending_tube_change = None
        self._tube_job_running = False
        if not self._rack_job_running and not self._tube_job_queue:
            self._set_busy_reason("")
        self._start_next_tube_job()

        self._auto_close_busy_popup_if_idle()

    def _start_next_tube_job(self):
        if self._tube_job_running:
            return
        if not self._tube_job_queue:
            self.log_t1("✅ [Tube] 모든 작업 완료")
            return

        mode_id, sel_list, dest_list, line = self._tube_job_queue.pop(0)
        self._tube_job_running = True
        self._pending_tube_change = (mode_id, sel_list, dest_list)
        self._set_busy_reason(f"TUBE: {line}")

        self.log_t1(f"📤 [Tube] 전송: {line}")
        if self.ros_node is None:
            self.log_t1("❌ [Tube] ROS 노드 미연동")
            self._tube_job_running = False
            return

        ok = self.ros_node.send_tube_command_line(line)
        if not ok:
            self._tube_job_running = False
            self.log_t1("❌ [Tube] tube_main_control 서버 연결 실패")

    def log_t1(self, msg):
        self.txt_log_t1.append(str(msg))

    def log_t2(self, msg):
        self.txt_log_t2.append(str(msg))

    def is_item_blocked(self, item_id):
        for bad in self.blocked_specific:
            if bad in item_id:
                return True
        for prefix in self.blocked_prefix:
            if item_id.startswith(prefix):
                return True
        return False

    def update_button_style(self, item_id):
        if item_id not in self.widget_map:
            return
        btn, mode = self.widget_map[item_id]

        if self.is_item_blocked(item_id):
            cls = "TubeBtnBlocked" if mode == "tube" else "RackSelectBtnBlocked"
        elif item_id in self.inventory:
            cls = "TubeBtnOccupied" if mode == "tube" else "RackSelectBtnOccupied"
        else:
            cls = "TubeBtn" if mode == "tube" else "RackSelectBtn"

        btn.setProperty("class", cls)
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    def process_inventory_change(self, mode_id, src_list, dest_list):
        if mode_id == 1:  # 입고
            for item in dest_list:
                self.inventory.add(item)
        elif mode_id in (2, 4):  # 출고/폐기
            for item in src_list:
                self.inventory.discard(item)
        elif mode_id == 3:  # 이동
            for item in src_list:
                self.inventory.discard(item)
            for item in dest_list:
                self.inventory.add(item)

        for item in set(src_list) | set(dest_list):
            self.update_button_style(item)

    def reset_selection_t1(self):
        for btn in list(self.t1_active_buttons):
            btn.setChecked(False)
        self.t1_active_buttons.clear()
        self.t1_selected_items.clear()
        self.t1_dest_items.clear()
        self.le_t1_selected.clear()
        self.le_t1_dest.clear()
        self.le_t1_input.clear()
        self.txt_log_t1.setText("[System] Ready...")

    def reset_selection_t2(self):
        for btn in list(self.t2_active_buttons):
            btn.setChecked(False)
        self.t2_active_buttons.clear()
        self.t2_selected_items.clear()
        self.t2_dest_items.clear()
        self.le_t2_selected.clear()
        self.le_t2_dest.clear()
        self.le_t2_input.clear()
        self.txt_log_t2.setText("[System] Ready...")

    def update_text_fields_t1(self):
        self.le_t1_selected.setText(", ".join(sorted(self.t1_selected_items)))
        self.le_t1_dest.setText(", ".join(sorted(self.t1_dest_items)))

    def update_text_fields_t2(self):
        self.le_t2_selected.setText(", ".join(sorted(self.t2_selected_items)))
        self.le_t2_dest.setText(", ".join(sorted(self.t2_dest_items)))

    def on_tube_clicked(self, checked, tube_id, btn_obj):
        if self.is_item_blocked(tube_id):
            btn_obj.setChecked(False)
            self.log_t1(f"⛔ [경고] {tube_id} 위치는 선택할 수 없습니다.")
            return

        mode_id = self.t1_mode_group.checkedId()

        if mode_id == 3:  # 이동: 텍스트필드 기반
            if not checked:
                if self.le_t1_selected.text() == tube_id:
                    self.le_t1_selected.clear()
                elif self.le_t1_dest.text() == tube_id:
                    self.le_t1_dest.clear()
                self.t1_active_buttons.discard(btn_obj)
            else:
                self.t1_active_buttons.add(btn_obj)
                if not self.le_t1_selected.text():
                    self.le_t1_selected.setText(tube_id)
                elif not self.le_t1_dest.text():
                    self.le_t1_dest.setText(tube_id)
                else:
                    btn_obj.setChecked(False)
                    self.t1_active_buttons.discard(btn_obj)
                    self.log_t1("⚠️ [안내] 이동은 2개(sel/dest)만 선택 가능합니다.")
            return

        target_set = self.t1_dest_items if mode_id == 1 else self.t1_selected_items
        if checked:
            target_set.add(tube_id)
            self.t1_active_buttons.add(btn_obj)
        else:
            target_set.discard(tube_id)
            self.t1_active_buttons.discard(btn_obj)

        self.update_text_fields_t1()

    def on_rack_clicked(self, checked, rack_id, btn_obj):
        if self.is_item_blocked(rack_id):
            btn_obj.setChecked(False)
            self.log_t2(f"⛔ [경고] {rack_id} 렉은 선택할 수 없습니다.")
            return

        mode_id = self.t2_mode_group.checkedId()

        if mode_id == 3:
            if not checked:
                if self.le_t2_selected.text() == rack_id:
                    self.le_t2_selected.clear()
                elif self.le_t2_dest.text() == rack_id:
                    self.le_t2_dest.clear()
                self.t2_active_buttons.discard(btn_obj)
            else:
                self.t2_active_buttons.add(btn_obj)
                if not self.le_t2_selected.text():
                    self.le_t2_selected.setText(rack_id)
                else:
                    self.le_t2_dest.setText(rack_id)
            return

        target_set = self.t2_dest_items if mode_id == 1 else self.t2_selected_items
        if checked:
            target_set.add(rack_id)
            self.t2_active_buttons.add(btn_obj)
        else:
            target_set.discard(rack_id)
            self.t2_active_buttons.discard(btn_obj)

        self.update_text_fields_t2()

    def on_confirm_t1(self):
        if self._is_busy_global():
            self.show_busy_popup("현재 작업이 끝난 후 다시 시도하세요.")
            return

        mode_id = self.t1_mode_group.checkedId()
        sel_list = list(self.t1_selected_items)
        dest_list = list(self.t1_dest_items)

        if mode_id == 3:
            src = self.le_t1_selected.text().strip()
            dst = self.le_t1_dest.text().strip()
            if not src or not dst:
                self.log_t1("[경고] 이동: 대상/목적지 필요")
                return
            sel_list = [src]
            dest_list = [dst]
        else:
            if mode_id == 1 and not dest_list:
                self.log_t1("[경고] 입고: 목적지 필요")
                return
            if mode_id in (2, 4) and not sel_list:
                self.log_t1("[경고] 출고/폐기: 대상 필요")
                return

        jobs = []
        try:
            if mode_id == 1:
                for dst in sorted(dest_list):
                    jobs.append((1, [], [dst], f"TUBE,IN,NONE,{dst}"))
            elif mode_id == 2:
                for src in sorted(sel_list):
                    jobs.append((2, [src], [], f"TUBE,OUT,{src},NONE"))
            elif mode_id == 3:
                src = sel_list[0]
                dst = dest_list[0]
                jobs.append((3, [src], [dst], f"TUBE,MOVE,{src},{dst}"))
            elif mode_id == 4:
                for src in sorted(sel_list):
                    jobs.append((4, [src], [], f"TUBE,WASTE,{src},NONE"))
            else:
                self.log_t1("[경고] 알 수 없는 모드")
                return
        except Exception as e:
            self.log_t1(f"❌ [parse] {e}")
            return

        if not jobs:
            self.log_t1("[경고] 실행할 작업이 없습니다.")
            return

        self._tube_job_queue = jobs
        self._tube_job_running = False

        for btn in list(self.t1_active_buttons):
            try:
                btn.setChecked(False)
            except Exception:
                pass
        self.t1_active_buttons.clear()
        self.t1_selected_items.clear()
        self.t1_dest_items.clear()
        self.le_t1_selected.clear()
        self.le_t1_dest.clear()
        self.le_t1_input.clear()

        self._start_next_tube_job()

    def on_confirm_t2(self):
        if self._is_busy_global():
            self.show_busy_popup("현재 작업이 끝난 후 다시 시도하세요.")
            return

        mode_id = self.t2_mode_group.checkedId()
        sel_list = list(self.t2_selected_items)
        dest_list = list(self.t2_dest_items)

        if mode_id == 3:
            src = self.le_t2_selected.text().strip()
            dst = self.le_t2_dest.text().strip()
            if not src or not dst:
                self.log_t2("[경고] 이동: 대상/목적지 필요")
                return
            sel_list = [src]
            dest_list = [dst]
        else:
            if mode_id == 1 and not dest_list:
                self.log_t2("[경고] 입고: 목적지 필요")
                return
            if mode_id == 2 and not sel_list:
                self.log_t2("[경고] 출고: 대상 필요")
                return

        if self.ros_node is None:
            self.log_t2("❌ [Action] ROS 노드 미연동")
            return

        if mode_id == 1:
            dst = sorted(dest_list)[0]
            ok = self.ros_node.send_rack_command("IN", "NONE", dst)
        elif mode_id == 2:
            src = sorted(sel_list)[0]
            ok = self.ros_node.send_rack_command("OUT", src, "NONE")
        else:
            ok = self.ros_node.send_rack_command("MOVE", sel_list[0], dest_list[0])

        if not ok:
            return

        self._rack_job_running = True
        self._set_busy_reason(self._format_busy_reason_rack(mode_id, sel_list, dest_list))
        self._pending_rack_change = (mode_id, sel_list, dest_list)
        # [수정] 렉 버튼 선택 상태 초기화
        # 버튼이 Checked(주황색) 상태로 남지 않도록 즉시 해제
        for btn in list(self.t2_active_buttons):
            btn.setChecked(False)
        self.t2_active_buttons.clear()
        self.t2_selected_items.clear()
        self.t2_dest_items.clear()
        self.le_t2_selected.clear()
        self.le_t2_dest.clear()
        self.le_t2_input.clear()

    def create_rack_widget(self, storage_name, rack_idx, mode="tube"):
        frame = QFrame()
        frame.setProperty("class", "RackFrame")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(frame)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        title = f"{storage_name}-{rack_idx}"
        is_blocked = self.is_item_blocked(title)

        if mode == "tube":
            lbl = QLabel(title)
            lbl.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(lbl)
            layout.addStretch(1)

            for i in range(1, 5):
                btn = QToolButton()
                btn_id = f"{title}-{i}"
                self.widget_map[btn_id] = (btn, "tube")
                btn.setProperty("class", "TubeBtnBlocked" if is_blocked else "TubeBtn")
                if is_blocked:
                    cls = "TubeBtnBlocked"
                elif btn_id in self.inventory:
                    cls = "TubeBtnOccupied"
                else:
                    cls = "TubeBtn"
                btn.setProperty("class", cls)
                btn.setCheckable(True)
                btn.setFixedSize(52, 52)
                btn.setCursor(Qt.CursorShape.PointingHandCursor)
                btn.clicked.connect(lambda checked, bid=btn_id, b_obj=btn: self.on_tube_clicked(checked, bid, b_obj))
                layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)

        else:
            btn_sel = QPushButton(title)
            self.widget_map[title] = (btn_sel, "rack")
            btn_sel.setProperty("class", "RackSelectBtnBlocked" if is_blocked else "RackSelectBtn")
            
            if is_blocked:
                cls = "RackSelectBtnBlocked"
            elif title in self.inventory:
                cls = "RackSelectBtnOccupied"
            else:
                cls = "RackSelectBtn"
            btn_sel.setProperty("class", cls)
            btn_sel.setCheckable(True)
            btn_sel.setCursor(Qt.CursorShape.PointingHandCursor)
            btn_sel.clicked.connect(lambda checked, rid=title, b_obj=btn_sel: self.on_rack_clicked(checked, rid, b_obj))
            layout.addWidget(btn_sel)
            layout.addStretch(1)

            for _ in range(1, 5):
                ind = QLabel()
                ind.setFixedSize(36, 36)
                ind.setStyleSheet("background-color: #64748B; border-radius: 6px;")
                layout.addWidget(ind, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)

        return frame

    def create_storage_grid(self, mode="tube"):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        grid = QGridLayout(content)
        grid.setSpacing(20)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        layout_map = [("C", 0, 0), ("D", 0, 1), ("A", 1, 0), ("B", 1, 1)]
        for name, r, c in layout_map:
            group = QGroupBox(f"Storage {name}")
            hbox = QHBoxLayout(group)
            hbox.setSpacing(10)
            hbox.setContentsMargins(10, 25, 10, 10)
            for i in range(1, 4):
                hbox.addWidget(self.create_rack_widget(name, i, mode))
            grid.addWidget(group, r, c)
        scroll.setWidget(content)
        return scroll

    def create_right_panel(self, title, items, is_tube=True):
        panel = QFrame()
        panel.setMinimumWidth(300)
        panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
        vbox = QVBoxLayout(panel)
        vbox.setContentsMargins(15, 15, 15, 15)
        vbox.setSpacing(10)

        lbl = QLabel(title)
        lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;")
        vbox.addWidget(lbl)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        vbox.addWidget(line)

        grp = QGroupBox("작업 모드")
        v_r = QVBoxLayout(grp)
        v_r.setContentsMargins(10, 15, 10, 10)

        group_obj = self.t1_mode_group if is_tube else self.t2_mode_group
        for i, txt in enumerate(items, 1):
            rb = QRadioButton(txt)
            group_obj.addButton(rb, i)
            if "폐기" in txt:
                rb.setStyleSheet("color: #EF4444; font-weight: bold;")
            if i == 1:
                rb.setChecked(True)
            v_r.addWidget(rb)
        vbox.addWidget(grp)

        form = QFormLayout()
        form.setVerticalSpacing(10)

        le_in = QLineEdit()
        le_in.setPlaceholderText("바코드...")
        le_sel = QLineEdit()
        le_sel.setReadOnly(True)
        le_dest = QLineEdit()
        le_dest.setReadOnly(True)

        form.addRow("바코드 :", le_in)
        form.addRow("선택 객체 :", le_sel)
        form.addRow("목적지 :", le_dest)
        vbox.addLayout(form)

        if is_tube:
            self.le_t1_input = le_in
            self.le_t1_selected = le_sel
            self.le_t1_dest = le_dest
        else:
            self.le_t2_input = le_in
            self.le_t2_selected = le_sel
            self.le_t2_dest = le_dest

        h_btn = QHBoxLayout()
        btn_ok = QPushButton("확인")
        btn_ok.setObjectName("btnConfirm")
        btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_cancel = QPushButton("취소")
        btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_ok.clicked.connect(self.on_confirm_t1 if is_tube else self.on_confirm_t2)
        btn_cancel.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        h_btn.addWidget(btn_ok)
        h_btn.addWidget(btn_cancel)
        vbox.addLayout(h_btn)

        btn_reset = QPushButton("초기화")
        btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        vbox.addWidget(btn_reset)

        grp_log = QGroupBox("로그 (History)")
        grp_log.setFixedHeight(180)
        v_l = QVBoxLayout(grp_log)
        v_l.setContentsMargins(5, 15, 5, 5)
        txt = QTextEdit()
        txt.setReadOnly(True)
        txt.setText("[System] Ready...")
        v_l.addWidget(txt)
        vbox.addWidget(grp_log)
        vbox.addStretch(1)

        if is_tube:
            self.txt_log_t1 = txt
        else:
            self.txt_log_t2 = txt

        return panel

    def setup_tab1(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.create_storage_grid(mode="tube"), stretch=7)
        layout.addWidget(self.create_right_panel("검체 제어 패널", ["입고", "출고", "이동", "폐기"], True), stretch=3)
        self.tabs.addTab(tab, "튜브 관리")

    def setup_tab2(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.create_storage_grid(mode="rack"), stretch=7)
        layout.addWidget(self.create_right_panel("렉(Rack) 제어 패널", ["렉 입고", "렉 출고", "렉 이동"], False), stretch=3)
        self.tabs.addTab(tab, "렉 관리")


def main(args=None):
    if args is None:
        args = sys.argv
    try:
        from rclpy.utilities import remove_ros_args
        qt_argv = remove_ros_args(args)
    except Exception:
        qt_argv = list(args)

    rclpy.init(args=args)

    app = QApplication(qt_argv)
    window = BioBankApp()
    window.showMaximized()

    ros_node = UiActionClientNode(window)
    window.set_ros_node(ros_node)

    timer = QTimer()
    timer.setInterval(10)
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    timer.start()

    try:
        exit_code = app.exec()
    finally:
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
