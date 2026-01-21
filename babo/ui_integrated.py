# ui_integrated v2.102 2026-01-21
# [이번 버전에서 수정된 사항]
# - ui_integrated.py의 ROS2 Action 연동 기능을 이식 (bio_main_control -> main_integrated)
# - 렉 탭(입고/출고/이동) 확인 버튼이 ros2 run 대신 Action 전송으로 동작
# - 로봇 작업 결과(success/fail)에 따라 로그 출력 및 성공 시 재고 상태 반영
# - (버그수정) console_scripts 엔트리포인트를 위한 main() 함수 추가
# - (버그수정) Qt 실행 argv에서 ROS 인자 제거(remove_ros_args) 적용

"""[모듈] ui_integrated

[역할]
- PySide6 기반 UI(스타일 유지)
- Rack 작업(입고/출고/이동)은 ROS2 Action(BioCommand)로 main_integrated에 명령 전달

[연동 흐름]
UI(ui_integrated) --(BioCommand: bio_main_control)--> main_integrated --(RobotMove: /robot_action)--> rack_transport_action
"""

import sys
import os
import subprocess
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout, 
    QLabel, QToolButton, QPushButton, QRadioButton, QLineEdit, 
    QComboBox, QFormLayout, QTextEdit, QSizePolicy, QButtonGroup
)
from PySide6.QtCore import Qt, QProcess, QTimer

# ========================================================
# ROS2 Action 연동 (ui_integrated.py 기능 이식)
# ========================================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# =========================
# QoS (latched)
# =========================
ACTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

try:
    from biobank_interfaces.action import BioCommand
except ImportError:
    # Dummy for environment check
    class BioCommand:
        class Goal:
            command = ""
        class Result:
            success = True
            message = ""
        class Feedback:
            status = ""


# ========================================================
# [스타일시트] 확인 버튼 글씨 가시성 해결 + 빨간색 스타일 유지
# ========================================================
STYLE_SHEET = """
QWidget { font-family: "Segoe UI", "Malgun Gothic", sans-serif; color: #000000; }
QMainWindow { background-color: #F1F5F9; }

QRadioButton { font-size: 14px; font-weight: bold; color: #333333; padding: 4px; }
QTabWidget::pane { border: 1px solid #CBD5E1; background: #FFFFFF; border-radius: 6px; }
QTabBar::tab { background: #E2E8F0; color: #64748B; padding: 10px 25px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background: #FFFFFF; color: #2563EB; border-top: 3px solid #2563EB; }

/* 그룹박스 */
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

/* ------------------------------------------------------------ */
/* 버튼 스타일 수정 (확인 버튼 가시성 확보) */
/* ------------------------------------------------------------ */

/* 1. 일반 버튼 (취소, 초기화) */
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

/* 2. [수정됨] 확인 버튼 (ID Selector 강화) */
QPushButton#btnConfirm { 
    background-color: #2563EB; 
    color: #000000;  /* 검은색 코드 명시 */
    border: 1px solid #1D4ED8; 
    border-bottom: 3px solid #1D4ED8; 
    font-weight: bold;
    border-radius: 4px;
}
/* 호버 및 눌림 상태에서도 글씨색 유지 */
QPushButton#btnConfirm:hover { background-color: #000000; color: #000000; }
QPushButton#btnConfirm:pressed { 
    background-color: #FFFFFF; 
    color: #FFFFFF;
    border-bottom: 0px solid; 
    border-top: 3px solid transparent; 
    padding-top: 10px; padding-bottom: 6px; 
}

/* ------------------------------------------------------------ */
/* 튜브/렉 버튼 스타일 (Occupied = Red) */
/* ------------------------------------------------------------ */

/* 튜브 기본 */
QToolButton.TubeBtn { background-color: #F8FAFC; border: 2px solid #94A3B8; border-radius: 13px; width: 52px; height: 52px; margin: 4px; }
QToolButton.TubeBtn:checked { background-color: #F59E0B; border-color: #D97706; }

/* 튜브 입고됨/차단됨 (빨강) */
QToolButton.TubeBtnOccupied { 
    background-color: #FECACA; border: 2px solid #EF4444; 
    border-radius: 13px; width: 52px; height: 52px; margin: 4px; 
}
QToolButton.TubeBtnOccupied:checked { background-color: #F59E0B; border-color: #D97706; }

QToolButton.TubeBtnBlocked { 
    background-color: #FECACA; border: 2px solid #EF4444; 
    border-radius: 13px; width: 52px; height: 52px; margin: 4px; 
}

/* 렉 선택 기본 */
QPushButton.RackSelectBtn { 
    background-color: #475569; color: #FFFFFF; 
    border: 1px solid #64748B; border-radius: 4px; 
    font-size: 18px; font-weight: bold; min-height: 30px; 
}
QPushButton.RackSelectBtn:checked { background-color: #F59E0B; border-color: #D97706; color: #FFFFFF; }

/* 렉 입고됨/차단됨 (빨강) */
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
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.client = ActionClient(
            self, BioCommand, "bio_main_control",
            goal_service_qos_profile=self.qos,
            result_service_qos_profile=self.qos,
            cancel_service_qos_profile=self.qos,
            feedback_sub_qos_profile=self.qos,
            status_sub_qos_profile=self.qos,
        )

    def send_rack_command(self, cmd_type: str, src: str, dest: str) -> bool:
        """main_integrated로 전달할 최종 문자열을 구성해 전송.

        cmd_type: IN / OUT / MOVE
        src/dest: 없으면 'NONE'
        """
        cmd_type = (cmd_type or "").strip().upper()
        src = (src or "NONE").strip()
        dest = (dest or "NONE").strip()

        final_cmd = f"RACK,{cmd_type},{src},{dest}"

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.ui.log_t2("❌ [Action] bio_main_control 서버 연결 실패")
            return False

        goal = BioCommand.Goal()
        goal.command = final_cmd

        self.ui.log_t2(f"📤 [Action] 전송: {final_cmd}")

        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_feedback(self, feedback_msg):
        try:
            status = getattr(feedback_msg.feedback, "status", "")
            if status:
                self.ui.log_t2(f"🟡 [Feedback] {status}")
        except Exception:
            pass

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.ui.log_t2(f"❌ [Action] Goal 응답 예외: {e}")
            self.ui.on_rack_action_result(False, f"Goal exception: {e}")
            return

        if not goal_handle.accepted:
            self.ui.log_t2("❌ [Action] Goal 거절됨")
            self.ui.on_rack_action_result(False, "Goal rejected")
            return

        self.ui.log_t2("✅ [Action] Goal 수락")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result().result
            success = bool(getattr(res, "success", False))
            msg = str(getattr(res, "message", ""))
        except Exception as e:
            success = False
            msg = f"Result exception: {e}"

        self.ui.on_rack_action_result(success, msg)


class BioBankApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BioBank System v5.7 (Detail Log & Fix Confirm Btn)")
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

        # [차단 목록]
        self.blocked_specific = ["A-1", "B-3"]
        self.blocked_prefix = ["C-", "D-"]

        # [재고 목록] (입고돈 아이템들)
        self.inventory = set()

        # [위젯 맵]
        self.widget_map = {}

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.setup_tab1()
        self.setup_tab2()

        # ========================================================================================
        # 연동을 위한 코딩추가 1
        self.ros_ws = os.environ.get("BABO_WS", os.path.expanduser("~/babo_ws"))
        # ========================================================================================

        # 실행 중인 ROS2 프로세스 참조 유지(가비지 컬렉션 방지)
        self._ros_procs = set()

        # Action 기반 연동용 노드(외부에서 주입)
        self.ros_node = None
        self._pending_rack_change = None  # (mode, sel_list, dest_list)

    # ==========================================================
    # ROS2 Action 연동
    # ==========================================================
    def set_ros_node(self, ros_node):
        self.ros_node = ros_node

    def on_rack_action_result(self, success: bool, message: str):
        """rack 작업 Action 결과를 UI에 반영"""
        if success:
            self.log_t2(f"✅ [Result] 성공: {message}")
            if self._pending_rack_change is not None:
                mode, sel_list, dest_list = self._pending_rack_change
                self.process_inventory_change(mode, sel_list, dest_list)
        else:
            self.log_t2(f"❌ [Result] 실패: {message}")

        self._pending_rack_change = None

    # ==========================================================
    # Helper 상태 확인 및 UI 갱신
    # ==========================================================
    def is_item_blocked(self, item_id):
        for bad in self.blocked_specific:
            if bad in item_id: return True
        for prefix in self.blocked_prefix:
            if item_id.startswith(prefix): return True
        return False

    def update_button_style(self, item_id):
        """특정 버튼의 상태(차단/입고/일반)에 따라 스타일 갱신"""
        if item_id not in self.widget_map: return
        btn, mode = self.widget_map[item_id] 
        
        # 1. 차단 상태 (최우선) -> 빨간색 (선택불가)
        if self.is_item_blocked(item_id):
            cls = "TubeBtnBlocked" if mode == "tube" else "RackSelectBtnBlocked"
        # 2. 입고 상태 (재고 있음) -> 빨간색 (선택가능)
        elif item_id in self.inventory:
            cls = "TubeBtnOccupied" if mode == "tube" else "RackSelectBtnOccupied"
        # 3. 일반 상태 -> 회색
        else:
            cls = "TubeBtn" if mode == "tube" else "RackSelectBtn"
            
        btn.setProperty("class", cls)
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    def process_inventory_change(self, mode_id, src_list, dest_list):
        """작업 완료 후 재고 상태 변경"""
        if mode_id == 1: # 입고
            for item in dest_list: self.inventory.add(item)
        elif mode_id == 2 or mode_id == 4: # 출고/폐기
            for item in src_list: 
                if item in self.inventory: self.inventory.remove(item)
        elif mode_id == 3: # 이동
            for item in src_list:
                if item in self.inventory: self.inventory.remove(item)
            for item in dest_list:
                self.inventory.add(item)

        # 스타일 갱신
        all_changed = set(src_list) | set(dest_list)
        for item in all_changed:
            self.update_button_style(item)

    # ==========================================================
    # 초기화 및 핸들러
    # ==========================================================
    def reset_selection_t1(self):
        for btn in self.t1_active_buttons: btn.setChecked(False)
        self.t1_active_buttons.clear(); self.t1_selected_items.clear(); self.t1_dest_items.clear()
        self.le_t1_selected.clear(); self.le_t1_dest.clear(); self.le_t1_input.clear()
        self.txt_log_t1.setText("[System] Ready...")

    def reset_selection_t2(self):
        for btn in self.t2_active_buttons: btn.setChecked(False)
        self.t2_active_buttons.clear(); self.t2_selected_items.clear(); self.t2_dest_items.clear()
        self.le_t2_selected.clear(); self.le_t2_dest.clear(); self.le_t2_input.clear()
        self.txt_log_t2.setText("[System] Ready...")

    def update_text_fields_t1(self):
        self.le_t1_selected.setText(", ".join(sorted(list(self.t1_selected_items))))
        self.le_t1_dest.setText(", ".join(sorted(list(self.t1_dest_items))))

    def update_text_fields_t2(self):
        self.le_t2_selected.setText(", ".join(sorted(list(self.t2_selected_items))))
        self.le_t2_dest.setText(", ".join(sorted(list(self.t2_dest_items))))

    def on_tube_clicked(self, checked, tube_id, btn_obj):
        # [차단 체크] 차단된 아이템은 선택 불가
        if self.is_item_blocked(tube_id):
            btn_obj.setChecked(False)
            self.log_t1(f"⛔ [경고] {tube_id} 위치는 선택할 수 없습니다.")
            return

        mode_id = self.t1_mode_group.checkedId()
        if mode_id == 3: # 이동
            if not checked:
                if self.le_t1_selected.text() == tube_id: self.le_t1_selected.clear()
                elif self.le_t1_dest.text() == tube_id: self.le_t1_dest.clear()
                if btn_obj in self.t1_active_buttons: self.t1_active_buttons.remove(btn_obj)
            else:
                self.t1_active_buttons.add(btn_obj)
                if not self.le_t1_selected.text(): self.le_t1_selected.setText(tube_id)
                else: self.le_t1_dest.setText(tube_id)
            return

        target_set = self.t1_dest_items if mode_id == 1 else self.t1_selected_items
        if mode_id == 4: self.le_t1_dest.setText("폐기장 (Disposal)")

        if checked: target_set.add(tube_id); self.t1_active_buttons.add(btn_obj)
        else:
            if tube_id in target_set: target_set.remove(tube_id)
            if btn_obj in self.t1_active_buttons: self.t1_active_buttons.remove(btn_obj)
        self.update_text_fields_t1()

    def on_rack_clicked(self, checked, rack_id, btn_obj):
        if self.is_item_blocked(rack_id):
            btn_obj.setChecked(False)
            self.log_t2(f"⛔ [경고] {rack_id} 렉은 선택할 수 없습니다.")
            return

        mode_id = self.t2_mode_group.checkedId()
        if mode_id == 3:
            if not checked:
                if self.le_t2_selected.text() == rack_id: self.le_t2_selected.clear()
                elif self.le_t2_dest.text() == rack_id: self.le_t2_dest.clear()
                if btn_obj in self.t2_active_buttons: self.t2_active_buttons.remove(btn_obj)
            else:
                self.t2_active_buttons.add(btn_obj)
                if not self.le_t2_selected.text(): self.le_t2_selected.setText(rack_id)
                else: self.le_t2_dest.setText(rack_id)
            return

        target_set = self.t2_dest_items if mode_id == 1 else self.t2_selected_items
        if checked: target_set.add(rack_id); self.t2_active_buttons.add(btn_obj)
        else:
            if rack_id in target_set: target_set.remove(rack_id)
            if btn_obj in self.t2_active_buttons: self.t2_active_buttons.remove(btn_obj)
        self.update_text_fields_t2()

    # ==========================================================
    # 확인 버튼 -> 로그 상세화
    # ==========================================================
    def on_confirm_t1(self):
        mode = self.t1_mode_group.checkedId()
        sel_list = list(self.t1_selected_items)
        dest_list = list(self.t1_dest_items)
        
        # [데이터 검증]
        if mode == 3:
            s_txt = self.le_t1_selected.text()
            d_txt = self.le_t1_dest.text()
            if not s_txt or not d_txt: self.log_t1("[경고] 이동: 대상/목적지 필요"); return
            sel_list = [s_txt]
            dest_list = [d_txt]
        else:
            if mode == 1 and not dest_list: self.log_t1("[경고] 입고: 목적지 필요"); return
            if (mode == 2 or mode == 4) and not sel_list: self.log_t1("[경고] 출고/폐기: 대상 필요"); return

        # [로그 메시지 작성]
        action_name = {1:"입고", 2:"출고", 3:"이동", 4:"폐기"}[mode]
        log_msg = f"✅ [{action_name}] "
        
        # 상세 내역 추가
        if mode == 1: # 입고: 바코드 -> 목적지들
            input_bc = self.le_t1_input.text() if self.le_t1_input.text() else "Unknown"
            dest_str = ", ".join(sorted(dest_list))
            log_msg += f"바코드({input_bc}) ➡️ {dest_str}"
        elif mode == 2: # 출고: 선택들 -> 반출
            sel_str = ", ".join(sorted(sel_list))
            log_msg += f"{sel_str} ➡️ 반출"
        elif mode == 3: # 이동: A -> B
            log_msg += f"{sel_list[0]} ➡️ {dest_list[0]}"
        elif mode == 4: # 폐기: 선택들 -> 폐기장
            sel_str = ", ".join(sorted(sel_list))
            log_msg += f"{sel_str} ➡️ 폐기장"
        
        # 로그 출력
        self.log_t1(log_msg)
        
        # 재고 반영 및 초기화 (로그는 유지)
        self.process_inventory_change(mode, sel_list, dest_list)
        
        # 입력값만 초기화
        for btn in self.t1_active_buttons: btn.setChecked(False)
        self.t1_active_buttons.clear(); self.t1_selected_items.clear(); self.t1_dest_items.clear()
        self.le_t1_selected.clear(); self.le_t1_dest.clear(); self.le_t1_input.clear()

    def on_confirm_t2(self):
        mode = self.t2_mode_group.checkedId()
        sel_list = list(self.t2_selected_items)
        dest_list = list(self.t2_dest_items)

        # --- 확정 sel/dest 만들기 ---
        if mode == 3:
            sel = self.le_t2_selected.text().strip()
            dest = self.le_t2_dest.text().strip()
            if not sel or not dest:
                self.log_t2("[경고] 이동: 대상/목적지 필요")
                return
            sel_list = [sel]
            dest_list = [dest]
        else:
            if mode == 1 and not dest_list:
                self.log_t2("[경고] 입고: 목적지 필요")
                return
            if mode == 2 and not sel_list:
                self.log_t2("[경고] 출고: 대상 필요")
                return

        action_name = {1:"렉 입고", 2:"렉 출고", 3:"렉 이동"}[mode]
        log_msg = f"✅ [{action_name}] "

        # UI 로그(기존 형식 유지)
        if mode == 1:
            input_bc = self.le_t2_input.text().strip() if self.le_t2_input.text().strip() else "Unknown"
            dest_str = ", ".join(sorted(dest_list))
            log_msg += f"바코드({input_bc}) ➡️ {dest_str}"
        elif mode == 2:
            sel_str = ", ".join(sorted(sel_list))
            log_msg += f"{sel_str} ➡️ 반출"
        elif mode == 3:
            log_msg += f"{sel_list[0]} ➡️ {dest_list[0]}"
        self.log_t2(log_msg)

        # ------------------------------------------------------
        # UI -> main_integrated(Action) -> rack_transport_action
        # ------------------------------------------------------
        if self.ros_node is None:
            self.log_t2("❌ [Action] ROS 노드 미연동 (UI 실행 시 rclpy/init 필요)")
            return

        # 다중 선택은 현재 Action 포맷상 1개만 지원 (첫 항목만 전송)
        if mode == 1:
            if len(dest_list) > 1:
                self.log_t2("⚠️ [안내] 렉 입고 목적지는 1개만 지원: 첫 항목만 전송")
            dest = sorted(dest_list)[0]
            ok = self.ros_node.send_rack_command("IN", "NONE", dest)
        elif mode == 2:
            if len(sel_list) > 1:
                self.log_t2("⚠️ [안내] 렉 출고 대상은 1개만 지원: 첫 항목만 전송")
            src = sorted(sel_list)[0]
            ok = self.ros_node.send_rack_command("OUT", src, "NONE")
        else:
            ok = self.ros_node.send_rack_command("MOVE", sel_list[0], dest_list[0])

        if not ok:
            return

        # 성공 시에만 재고 반영(결과 콜백에서 처리)
        self._pending_rack_change = (mode, sel_list, dest_list)

        # 입력/선택 UI는 즉시 초기화(기존 UX 유지)
        for btn in self.t2_active_buttons:
            btn.setChecked(False)
        self.t2_active_buttons.clear(); self.t2_selected_items.clear(); self.t2_dest_items.clear()
        self.le_t2_selected.clear(); self.le_t2_dest.clear(); self.le_t2_input.clear()

    def log_t1(self, msg): self.txt_log_t1.append(msg)
    def log_t2(self, msg): self.txt_log_t2.append(msg)

    # ==========================================================
    # ROS2 노드 실행 (UI -> 로봇 동작) 연동을 위한 코드추가 3
    # ==========================================================
    def run_ros2(
        self,
        executable: str,
        extra_ros_args: str = "",
        *,
        start_msg: str | None = None,
        done_msg: str | None = None,
        log_fn=None,
    ):
        """ROS2 실행 + 종료 감지해서 '완료' 로그 찍기.

        ⚠️ 전제: 여기서 '완료'는 **ros2 run으로 실행한 프로세스가 종료될 때**를 의미합니다.
        (노드가 계속 spin()하면서 종료하지 않는 구조면 finished가 안 와서 완료 로그가 안 뜹니다.)
        """

        if log_fn is None:
            log_fn = self.log_t2

        ws_setup = os.path.join(self.ros_ws, "install", "setup.bash")
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"( [ -f '{ws_setup}' ] && source '{ws_setup}' || true ) && "
            f"ros2 run babo {executable} {extra_ros_args}".strip()
        )

        p = QProcess(self)
        p.setProgram("bash")
        p.setArguments(["-lc", cmd])

        # (선택) 노드 출력이 필요하면 로그로 보여주기
        p.readyReadStandardOutput.connect(
            lambda: log_fn(p.readAllStandardOutput().data().decode(errors="ignore").rstrip())
        )
        p.readyReadStandardError.connect(
            lambda: log_fn(p.readAllStandardError().data().decode(errors="ignore").rstrip())
        )

        if start_msg:
            log_fn(start_msg)

        def _on_finished(exit_code, _exit_status):
            if exit_code == 0:
                log_fn(done_msg or f"✅ [{executable}] 완료")
            else:
                log_fn(f"❌ [{executable}] 실패 (exit={exit_code})")
            self._ros_procs.discard(p)
            p.deleteLater()

        p.finished.connect(_on_finished)

        self._ros_procs.add(p)
        p.start()
        return True

    # ==========================================================
    # UI 생성
    # ==========================================================
    def create_rack_widget(self, storage_name, rack_idx, mode="tube"):
        frame = QFrame(); frame.setProperty("class", "RackFrame"); frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(frame); layout.setSpacing(5); layout.setContentsMargins(5, 5, 5, 5)
        
        title = f"{storage_name}-{rack_idx}"
        is_blocked = self.is_item_blocked(title)

        if mode == "tube":
            lbl = QLabel(title); lbl.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;"); lbl.setAlignment(Qt.AlignmentFlag.AlignCenter); layout.addWidget(lbl); layout.addStretch(1)
            for i in range(1, 5):
                btn = QToolButton()
                btn_id = f"{title}-{i}"
                self.widget_map[btn_id] = (btn, "tube")
                if is_blocked: btn.setProperty("class", "TubeBtnBlocked")
                else: btn.setProperty("class", "TubeBtn")
                btn.setCheckable(True); btn.setFixedSize(52, 52); btn.setCursor(Qt.CursorShape.PointingHandCursor)
                btn.clicked.connect(lambda checked, bid=btn_id, b_obj=btn: self.on_tube_clicked(checked, bid, b_obj))
                layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)
        else:
            btn_sel = QPushButton(title)
            self.widget_map[title] = (btn_sel, "rack")
            if is_blocked: btn_sel.setProperty("class", "RackSelectBtnBlocked")
            else: btn_sel.setProperty("class", "RackSelectBtn")
            btn_sel.setCheckable(True); btn_sel.setCursor(Qt.CursorShape.PointingHandCursor)
            btn_sel.clicked.connect(lambda checked, rid=title, b_obj=btn_sel: self.on_rack_clicked(checked, rid, b_obj))
            layout.addWidget(btn_sel); layout.addStretch(1)
            for i in range(1, 5):
                ind = QLabel(); ind.setFixedSize(36, 36); ind.setStyleSheet("background-color: #64748B; border-radius: 6px;"); layout.addWidget(ind, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)
        return frame

    def create_storage_grid(self, mode="tube"):
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget(); grid = QGridLayout(content); grid.setSpacing(20); grid.setContentsMargins(10, 10, 10, 10)
        grid.setRowStretch(0, 1); grid.setRowStretch(1, 1); grid.setColumnStretch(0, 1); grid.setColumnStretch(1, 1)
        
        layout_map = [("C", 0, 0), ("D", 0, 1), ("A", 1, 0), ("B", 1, 1)]

        for name, r, c in layout_map:
            group = QGroupBox(f"Storage {name}"); hbox = QHBoxLayout(group); hbox.setSpacing(10); hbox.setContentsMargins(10, 25, 10, 10)
            for i in range(1, 4): hbox.addWidget(self.create_rack_widget(name, i, mode))
            grid.addWidget(group, r, c)
        scroll.setWidget(content)
        return scroll

    def create_right_panel(self, title, items, is_tube=True):
        panel = QFrame(); panel.setMinimumWidth(300); panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
        vbox = QVBoxLayout(panel); vbox.setContentsMargins(15, 15, 15, 15); vbox.setSpacing(10)
        lbl = QLabel(title); lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;"); vbox.addWidget(lbl)
        line = QFrame(); line.setFrameShape(QFrame.Shape.HLine); line.setFrameShadow(QFrame.Shadow.Sunken); vbox.addWidget(line)
        
        grp = QGroupBox("작업 모드"); v_r = QVBoxLayout(grp); v_r.setContentsMargins(10, 15, 10, 10)
        group_obj = self.t1_mode_group if is_tube else self.t2_mode_group
        for i, txt in enumerate(items, 1):
            rb = QRadioButton(txt); group_obj.addButton(rb, i)
            if "폐기" in txt: rb.setStyleSheet("color: #EF4444; font-weight: bold;")
            if i==1: rb.setChecked(True)
            v_r.addWidget(rb)
        vbox.addWidget(grp)

        form = QFormLayout(); form.setVerticalSpacing(10)
        le_in = QLineEdit(); le_in.setPlaceholderText("바코드..."); 
        le_sel = QLineEdit(); le_sel.setReadOnly(True)
        le_dest = QLineEdit(); le_dest.setReadOnly(True)
        form.addRow("바코드 :", le_in); form.addRow("선택 객체 :", le_sel); form.addRow("목적지 :", le_dest)
        vbox.addLayout(form)
        
        if is_tube: self.le_t1_input=le_in; self.le_t1_selected=le_sel; self.le_t1_dest=le_dest
        else: self.le_t2_input=le_in; self.le_t2_selected=le_sel; self.le_t2_dest=le_dest

        h_btn = QHBoxLayout()
        btn_ok = QPushButton("확인"); btn_ok.setObjectName("btnConfirm"); btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_cancel = QPushButton("취소"); btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_ok.clicked.connect(self.on_confirm_t1 if is_tube else self.on_confirm_t2)
        btn_cancel.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        h_btn.addWidget(btn_ok); h_btn.addWidget(btn_cancel); vbox.addLayout(h_btn)
        
        btn_reset = QPushButton("초기화"); btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        vbox.addWidget(btn_reset); vbox.addWidget(line)
        
        grp_log = QGroupBox("로그 (History)"); grp_log.setFixedHeight(180); v_l = QVBoxLayout(grp_log); v_l.setContentsMargins(5,15,5,5)
        txt = QTextEdit(); txt.setReadOnly(True); txt.setStyleSheet("background-color: #1E293B; color: #00FF00;")
        txt.setText("[System] Ready..."); v_l.addWidget(txt); vbox.addWidget(grp_log); vbox.addStretch(1)
        
        if is_tube: self.txt_log_t1 = txt
        else: self.txt_log_t2 = txt
        return panel

    def setup_tab1(self):
        tab = QWidget(); layout = QHBoxLayout(tab); layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.create_storage_grid(mode="tube"), stretch=7)
        layout.addWidget(self.create_right_panel("검체 제어 패널", ["입고", "출고", "이동", "폐기"], True), stretch=3)
        self.tabs.addTab(tab, "튜브 관리")

    def setup_tab2(self):
        tab = QWidget(); layout = QHBoxLayout(tab); layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.create_storage_grid(mode="rack"), stretch=7)
        layout.addWidget(self.create_right_panel("렉(Rack) 제어 패널", ["렉 입고", "렉 출고", "렉 이동"], False), stretch=3)
        self.tabs.addTab(tab, "렉 관리")

def main(args=None):
    """console_scripts(ros2 run) 엔트리포인트.
    - rclpy는 ROS 인자 포함한 argv로 init
    - Qt는 ROS 인자를 제거한 argv로 실행
    """
    # ROS2 init
    if args is None:
        args = sys.argv

    # Qt에 ROS 인자가 들어가면 오류가 나서 제거
    try:
        from rclpy.utilities import remove_ros_args
        qt_argv = remove_ros_args(args)
    except Exception:
        qt_argv = list(args)

    rclpy.init(args=args)

    # (선택) High-DPI
    try:
        from PySide6.QtGui import QGuiApplication
        QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    except Exception:
        pass

    app = QApplication(qt_argv)
    window = BioBankApp()
    window.showMaximized()

    # Action Client Node 생성 및 UI에 주입
    ros_node = UiActionClientNode(window)
    window.set_ros_node(ros_node)

    # Qt 타이머로 rclpy spin_once 수행(메인 스레드)
    timer = QTimer()
    timer.setInterval(10)  # ms
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