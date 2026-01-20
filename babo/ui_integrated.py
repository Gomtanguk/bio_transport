# ui_integrated.py v2.110 2026-01-21
# [이번 버전에서 수정된 사항]
# - (기능추가) QProcess 기반 'ros2 run' 실행 방식 제거, ActionClient 기반(/bio_main_control) 명령 전송으로 전환
# - (기능추가) UI에서 feedback/status 및 result 메시지를 로그창에 실시간 표시
# - (변수수정) ROS UI 노드 이름에 PID를 포함해 중복 노드 경고를 줄임
# - (유지) 기존 UI 레이아웃/동작(랙 선택/차단/재고 로직) 유지

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout,
    QLabel, QToolButton, QPushButton, QRadioButton, QLineEdit,
    QComboBox, QFormLayout, QTextEdit, QSizePolicy, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer

from biobank_interfaces.action import BioCommand, RobotMove



class UiRosBridge(Node):
    """Qt(UI)에서 Action을 보내기 위한 ROS2 브릿지 노드.

    - 기본 타깃: /bio_main_control (BioCommand)  → main_orchestrator
    - 옵션 타깃: /robot_action (RobotMove)       → sub 노드(직접 제어/디버그)
    """

    def __init__(self, node_name: str = "ui_client"):
        super().__init__(node_name)
        self.bio_client = ActionClient(self, BioCommand, "bio_main_control")
        self.robot_client = ActionClient(self, RobotMove, "robot_action")

    def send_command(
        self,
        command: str,
        *,
        target: str = "bio_main_control",
        on_feedback=None,
        on_done=None,
        server_wait_sec: float = 0.2,
    ):
        """command 문자열을 Action Goal로 전송한다.

        target:
          - "bio_main_control" (권장): main_orchestrator로 전송
          - "robot_action"            : sub로 직접 전송
        """
        client = self.bio_client if target == "bio_main_control" else self.robot_client

        if not client.wait_for_server(timeout_sec=server_wait_sec):
            if on_done:
                on_done(False, f"[ERR] Action server not available: /{target}")
            return

        goal = (BioCommand.Goal() if target == "bio_main_control" else RobotMove.Goal())
        goal.command = command

        def _fb_cb(msg):
            # msg.feedback.status
            if on_feedback:
                try:
                    on_feedback(getattr(msg.feedback, "status", str(msg.feedback)))
                except Exception:
                    pass

        send_future = client.send_goal_async(goal, feedback_callback=_fb_cb)

        def _on_goal_response(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                if on_done:
                    on_done(False, f"[ERR] send_goal failed: {e}")
                return

            if not getattr(goal_handle, "accepted", False):
                if on_done:
                    on_done(False, "[ERR] goal rejected")
                return

            result_future = goal_handle.get_result_async()

            def _on_result(rfut):
                try:
                    res = rfut.result().result
                    ok = bool(getattr(res, "success", False))
                    msg = str(getattr(res, "message", ""))
                except Exception as e:
                    ok, msg = False, f"[ERR] get_result failed: {e}"

                if on_done:
                    on_done(ok, msg)

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)


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

        # ROS2 Action 연동 초기화
        self._init_ros()


        # ========================================================================================
        # 연동을 위한 코딩추가 1
        # ========================================================================================

        # 실행 중인 ROS2 프로세스 참조 유지(가비지 컬렉션 방지)

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

            # 여러 개 선택 가능 → 우선 문자열로 합쳐서 전달
            sel = ", ".join(sorted(sel_list))
            dest = ", ".join(sorted(dest_list))

        action_name = {1:"렉 입고", 2:"렉 출고", 3:"렉 이동"}[mode]
        log_msg = f"✅ [{action_name}] "

        if mode == 1:
            input_bc = self.le_t2_input.text().strip() if self.le_t2_input.text().strip() else "Unknown"
            log_msg += f"바코드({input_bc}) ➡️ {dest}"
        elif mode == 2:
            log_msg += f"{sel} ➡️ 반출"
        elif mode == 3:
            log_msg += f"{sel} ➡️ {dest}"

        self.log_t2(log_msg)
        # ======================================================
        # UI -> 로봇 동작 연동 (Action)
        #  - UI → /bio_main_control (BioCommand)
        #  - main_orchestrator가 하위 /robot_action(RobotMove)로 중계
        # 명령 포맷(권장):
        #   - IN :  RACK,IN,NONE,<DEST>
        #   - OUT:  RACK,OUT,<SRC>,NONE
        #   - MOVE: RACK,MOVE,<SRC>,<DEST>
        # ======================================================
        cmds = []
        if mode == 1:
            # 입고: dest 여러 개면 순차 전송
            for d in sorted(dest_list):
                cmds.append(f"RACK,IN,NONE,{d}")
            self._send_cmd_queue(cmds, start_prefix="[실행 중] 렉 입고", done_prefix="[입고완료]")
        elif mode == 2:
            for s in sorted(sel_list):
                cmds.append(f"RACK,OUT,{s},NONE")
            self._send_cmd_queue(cmds, start_prefix="[실행 중] 렉 출고", done_prefix="[출고완료]")
        elif mode == 3:
            # 이동은 1개만 (sel/dest 확정 입력 기준)
            cmds = [f"RACK,MOVE,{sel},{dest}"]
            self._send_cmd_queue(cmds, start_prefix="[실행 중] 렉 이동", done_prefix="[이동완료]")
        # =======================================================

        self.process_inventory_change(mode, sel_list, dest_list)
        
        for btn in self.t2_active_buttons: btn.setChecked(False)
        self.t2_active_buttons.clear(); self.t2_selected_items.clear(); self.t2_dest_items.clear()
        self.le_t2_selected.clear(); self.le_t2_dest.clear(); self.le_t2_input.clear()

    def log_t1(self, msg): self.txt_log_t1.append(msg)
    def log_t2(self, msg): self.txt_log_t2.append(msg)

    # ==========================================================
    # ROS2 Action 연동 (UI -> main_orchestrator)
    # ==========================================================
    def _init_ros(self):
        if not rclpy.ok():
            rclpy.init(args=None)

        self._ros_node = UiRosBridge(node_name=f"ui_client_{os.getpid()}")

        # Qt 이벤트 루프에서 spin_once로 콜백 처리
        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start(20)  # 50Hz

        # 큐 전송(여러 목적지/선택 지원)
        self._pending_cmds = []
        self._pending_done_prefix = ""
        self._pending_target = "bio_main_control"

        self.log_t2("🟢 ROS2 ActionClient 준비됨: /bio_main_control")

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        except Exception:
            # spin_once 중 예외는 UI를 죽이지 않도록 무시(로그는 필요 시 추가)
            pass

    def _send_cmd_queue(self, cmds, *, start_prefix: str, done_prefix: str, target: str = "bio_main_control"):
        if not cmds:
            return
        self._pending_cmds = list(cmds)
        self._pending_done_prefix = done_prefix
        self._pending_target = target

        # 첫 명령부터 전송
        self.log_t2(f"{start_prefix} ({len(self._pending_cmds)}건)...")
        self._send_next_cmd()

    def _send_next_cmd(self):
        if not self._pending_cmds:
            return

        cmd = self._pending_cmds.pop(0)
        self.log_t2(f"➡️ SEND: {cmd}")

        def _on_feedback(status: str):
            # main_orchestrator에서 올리는 status 문자열 그대로 표시
            self.log_t2(f"… {status}")

        def _on_done(ok: bool, message: str):
            tag = "✅" if ok else "❌"
            self.log_t2(f"{tag} {self._pending_done_prefix}: {message}")

            # 다음 명령이 있으면 연속 전송
            if self._pending_cmds:
                self._send_next_cmd()

        self._ros_node.send_command(
            cmd,
            target=self._pending_target,
            on_feedback=_on_feedback,
            on_done=_on_done,
            server_wait_sec=0.5,
        )

    def closeEvent(self, event):
        # Qt 종료 시 ROS 정리
        try:
            if hasattr(self, "_ros_timer"):
                self._ros_timer.stop()
            if hasattr(self, "_ros_node"):
                self._ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


    # ==========================================================
    # ROS2 노드 실행 (UI -> 로봇 동작) 연동을 위한 코드추가 3
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

# if __name__ == "__main__":
#     try:
#         from PySide6.QtGui import QGuiApplication
#         QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
#             Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
#     except:
#         pass
#     app = QApplication(sys.argv)
#     window = BioBankApp()
#     window.showMaximized()
#     sys.exit(app.exec())

def main(args=None):
    app = QApplication(sys.argv)
    # 기존 UI 클래스 이름에 맞춰 생성 (예: BioBankApp)
    window = BioBankApp() 
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()