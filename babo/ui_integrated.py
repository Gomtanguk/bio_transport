# ui_integrated.py v2.700 (Final Clean)
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout,
    QLabel, QToolButton, QPushButton, QRadioButton, QLineEdit,
    QFormLayout, QTextEdit, QSizePolicy, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer

# 인터페이스 (빌드된 패키지에서 가져옴)
from biobank_interfaces.action import BioCommand, RobotMove


# ========================================================
# [1] ROS 2 통신 브릿지 (UI 스레드와 분리)
# ========================================================
class UiRosBridge(Node):
    """Qt(UI)에서 Action을 보내기 위한 ROS2 브릿지 노드."""

    def __init__(self, node_name: str = "ui_client"):
        super().__init__(node_name)
        # 메인 오케스트레이터로 명령 전송
        self.bio_client = ActionClient(self, BioCommand, "bio_main_control")
        # 필요시 로봇 액션 직접 제어용
        self.robot_client = ActionClient(self, RobotMove, "robot_action")

    def send_command(
        self,
        command: str,
        *,
        target: str = "bio_main_control",
        on_feedback=None,
        on_done=None,
        server_wait_sec: float = 0.5,
    ):
        client = self.bio_client if target == "bio_main_control" else self.robot_client

        # 서버 연결 대기
        if not client.wait_for_server(timeout_sec=server_wait_sec):
            if on_done:
                on_done(False, f"[ERR] 서버 연결 실패: /{target}")
            return

        # Goal 생성 및 전송
        goal = (BioCommand.Goal() if target == "bio_main_control" else RobotMove.Goal())
        goal.command = command

        # 피드백 콜백
        def _fb_cb(msg):
            if on_feedback:
                try:
                    # ROS 버전에 따라 msg.feedback.status 또는 msg.feedback 접근
                    status = getattr(msg.feedback, "status", str(msg.feedback))
                    on_feedback(status)
                except:
                    pass

        send_future = client.send_goal_async(goal, feedback_callback=_fb_cb)

        # 결과 처리 콜백
        def _on_goal_response(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                if on_done: on_done(False, f"[ERR] 전송 실패: {e}")
                return

            if not getattr(goal_handle, "accepted", False):
                if on_done: on_done(False, "[ERR] 서버가 명령 거절")
                return

            result_future = goal_handle.get_result_async()

            def _on_result(rfut):
                try:
                    res = rfut.result().result
                    ok = bool(getattr(res, "success", False))
                    msg = str(getattr(res, "message", ""))
                except Exception as e:
                    ok, msg = False, f"[ERR] 결과 수신 실패: {e}"

                if on_done: on_done(ok, msg)

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)


# ========================================================
# [2] 스타일시트 (사용자 원본 유지)
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

/* 버튼 스타일 */
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

/* 튜브/렉 버튼 스타일 */
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

# ========================================================
# [3] 메인 애플리케이션 클래스
# ========================================================
class BioBankApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BioBank System v6.0 (Integrated Final)")
        self.resize(1300, 850)
        self.setStyleSheet(STYLE_SHEET)

        # 상태 관리
        self.t1_mode_group = QButtonGroup(self)
        self.t2_mode_group = QButtonGroup(self)
        
        self.t1_selected_items = set(); self.t1_dest_items = set(); self.t1_active_buttons = set()
        self.t2_selected_items = set(); self.t2_dest_items = set(); self.t2_active_buttons = set()

        self.blocked_specific = ["A-1", "B-3"]
        self.blocked_prefix = ["C-", "D-"]
        self.inventory = set()
        self.widget_map = {}

        # 메인 레이아웃 구성
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # 탭 구성 (함수 호출)
        self.setup_tab1()
        self.setup_tab2()

        # ROS2 초기화
        self._init_ros()

    # -------------------------------------------------------------------------
    # UI 생성 Helper 함수들 (클래스 내부로 완벽 통합)
    # -------------------------------------------------------------------------
    def create_rack_widget(self, storage_name, rack_idx, mode="tube"):
        frame = QFrame(); frame.setProperty("class", "RackFrame")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(frame); layout.setSpacing(5); layout.setContentsMargins(5, 5, 5, 5)
        
        title = f"{storage_name}-{rack_idx}"
        is_blocked = self.is_item_blocked(title)

        if mode == "tube":
            lbl = QLabel(title); lbl.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter); layout.addWidget(lbl); layout.addStretch(1)
            for i in range(1, 5):
                btn = QToolButton()
                btn_id = f"{title}-{i}"
                self.widget_map[btn_id] = (btn, "tube")
                cls = "TubeBtnBlocked" if is_blocked else "TubeBtn"
                btn.setProperty("class", cls)
                btn.setCheckable(True); btn.setFixedSize(52, 52); btn.setCursor(Qt.CursorShape.PointingHandCursor)
                btn.clicked.connect(lambda checked, bid=btn_id, b_obj=btn: self.on_tube_clicked(checked, bid, b_obj))
                layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)
        else:
            btn_sel = QPushButton(title)
            self.widget_map[title] = (btn_sel, "rack")
            cls = "RackSelectBtnBlocked" if is_blocked else "RackSelectBtn"
            btn_sel.setProperty("class", cls)
            btn_sel.setCheckable(True); btn_sel.setCursor(Qt.CursorShape.PointingHandCursor)
            btn_sel.clicked.connect(lambda checked, rid=title, b_obj=btn_sel: self.on_rack_clicked(checked, rid, b_obj))
            layout.addWidget(btn_sel); layout.addStretch(1)
            for i in range(1, 5):
                ind = QLabel(); ind.setFixedSize(36, 36)
                ind.setStyleSheet("background-color: #64748B; border-radius: 6px;")
                layout.addWidget(ind, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)
        return frame

    def create_storage_grid(self, mode="tube"):
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget(); grid = QGridLayout(content)
        grid.setSpacing(20); grid.setContentsMargins(10, 10, 10, 10)
        grid.setRowStretch(0, 1); grid.setRowStretch(1, 1); grid.setColumnStretch(0, 1); grid.setColumnStretch(1, 1)
        
        for name, r, c in [("C", 0, 0), ("D", 0, 1), ("A", 1, 0), ("B", 1, 1)]:
            group = QGroupBox(f"Storage {name}")
            hbox = QHBoxLayout(group); hbox.setSpacing(10); hbox.setContentsMargins(10, 25, 10, 10)
            for i in range(1, 4): hbox.addWidget(self.create_rack_widget(name, i, mode))
            grid.addWidget(group, r, c)
        
        scroll.setWidget(content)
        return scroll

    def create_right_panel(self, title, items, is_tube=True):
        panel = QFrame(); panel.setMinimumWidth(300)
        panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
        vbox = QVBoxLayout(panel); vbox.setContentsMargins(15, 15, 15, 15); vbox.setSpacing(10)
        
        lbl = QLabel(title); lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;")
        vbox.addWidget(lbl)
        line = QFrame(); line.setFrameShape(QFrame.Shape.HLine); line.setFrameShadow(QFrame.Shadow.Sunken)
        vbox.addWidget(line)
        
        grp = QGroupBox("작업 모드"); v_r = QVBoxLayout(grp); v_r.setContentsMargins(10, 15, 10, 10)
        group_obj = self.t1_mode_group if is_tube else self.t2_mode_group
        for i, txt in enumerate(items, 1):
            rb = QRadioButton(txt); group_obj.addButton(rb, i)
            if "폐기" in txt: rb.setStyleSheet("color: #EF4444; font-weight: bold;")
            if i==1: rb.setChecked(True)
            v_r.addWidget(rb)
        vbox.addWidget(grp)

        form = QFormLayout(); form.setVerticalSpacing(10)
        le_in = QLineEdit(); le_in.setPlaceholderText("바코드...") 
        le_sel = QLineEdit(); le_sel.setReadOnly(True)
        le_dest = QLineEdit(); le_dest.setReadOnly(True)
        form.addRow("바코드 :", le_in); form.addRow("선택 객체 :", le_sel); form.addRow("목적지 :", le_dest)
        vbox.addLayout(form)
        
        if is_tube: self.le_t1_input=le_in; self.le_t1_selected=le_sel; self.le_t1_dest=le_dest
        else: self.le_t2_input=le_in; self.le_t2_selected=le_sel; self.le_t2_dest=le_dest

        h_btn = QHBoxLayout()
        btn_ok = QPushButton("확인"); btn_ok.setObjectName("btnConfirm")
        btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_cancel = QPushButton("취소"); btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        
        btn_ok.clicked.connect(self.on_confirm_t1 if is_tube else self.on_confirm_t2)
        btn_cancel.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        
        h_btn.addWidget(btn_ok); h_btn.addWidget(btn_cancel); vbox.addLayout(h_btn)
        
        btn_reset = QPushButton("초기화"); btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.clicked.connect(self.reset_selection_t1 if is_tube else self.reset_selection_t2)
        vbox.addWidget(btn_reset); vbox.addWidget(line)
        
        grp_log = QGroupBox("로그 (History)"); grp_log.setFixedHeight(180)
        v_l = QVBoxLayout(grp_log); v_l.setContentsMargins(5,15,5,5)
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

    # -------------------------------------------------------------------------
    # 로직 핸들러
    # -------------------------------------------------------------------------
    def is_item_blocked(self, item_id):
        for bad in self.blocked_specific:
            if bad in item_id: return True
        for prefix in self.blocked_prefix:
            if item_id.startswith(prefix): return True
        return False

    def update_button_style(self, item_id):
        if item_id not in self.widget_map: return
        btn, mode = self.widget_map[item_id] 
        
        if self.is_item_blocked(item_id):
            cls = "TubeBtnBlocked" if mode == "tube" else "RackSelectBtnBlocked"
        elif item_id in self.inventory:
            cls = "TubeBtnOccupied" if mode == "tube" else "RackSelectBtnOccupied"
        else:
            cls = "TubeBtn" if mode == "tube" else "RackSelectBtn"
            
        btn.setProperty("class", cls)
        btn.style().unpolish(btn); btn.style().polish(btn)

    def process_inventory_change(self, mode_id, src_list, dest_list):
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
        all_changed = set(src_list) | set(dest_list)
        for item in all_changed: self.update_button_style(item)

    # -------------------------------------------------------------------------
    # 클릭 및 초기화 이벤트
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # [ACTION] 확인 버튼 클릭 및 전송
    # -------------------------------------------------------------------------
    def on_confirm_t1(self):
        self.log_t1("⚠️ 튜브 제어 기능은 아직 서버에 구현되지 않았습니다.")
        # 추후 구현 시 T2와 동일한 방식으로 호출

    def on_confirm_t2(self):
        mode = self.t2_mode_group.checkedId()
        sel_list = list(self.t2_selected_items)
        dest_list = list(self.t2_dest_items)

        # 1. 유효성 검사
        if mode == 3:
            sel = self.le_t2_selected.text().strip()
            dest = self.le_t2_dest.text().strip()
            if not sel or not dest:
                self.log_t2("[경고] 이동: 대상/목적지 필요"); return
            sel_list = [sel]; dest_list = [dest]
        else:
            if mode == 1 and not dest_list:
                self.log_t2("[경고] 입고: 목적지 필요"); return
            if mode == 2 and not sel_list:
                self.log_t2("[경고] 출고: 대상 필요"); return
            
            sel = ", ".join(sorted(sel_list))
            dest = ", ".join(sorted(dest_list))

        # 2. 명령 생성 및 큐 전송
        cmds = []
        if mode == 1:
            # 입고 (IN)
            for d in sorted(dest_list):
                cmds.append(f"RACK,IN,NONE,{d}")
            self._send_cmd_queue(cmds, start_prefix="[실행] 렉 입고", done_prefix="[입고완료]")
        elif mode == 2:
            # 출고 (OUT)
            for s in sorted(sel_list):
                cmds.append(f"RACK,OUT,{s},NONE")
            self._send_cmd_queue(cmds, start_prefix="[실행] 렉 출고", done_prefix="[출고완료]")
        elif mode == 3:
            # 이동 (MOVE)
            cmds = [f"RACK,MOVE,{sel},{dest}"]
            self._send_cmd_queue(cmds, start_prefix="[실행] 렉 이동", done_prefix="[이동완료]")

        # 3. UI 재고 반영
        self.process_inventory_change(mode, sel_list, dest_list)
        
        # 4. 선택 초기화
        for btn in self.t2_active_buttons: btn.setChecked(False)
        self.t2_active_buttons.clear(); self.t2_selected_items.clear(); self.t2_dest_items.clear()
        self.le_t2_selected.clear(); self.le_t2_dest.clear(); self.le_t2_input.clear()

    # 로그 래퍼
    def log_t1(self, msg): self.txt_log_t1.append(msg)
    def log_t2(self, msg): self.txt_log_t2.append(msg)

    # -------------------------------------------------------------------------
    # ROS2 연동 로직
    # -------------------------------------------------------------------------
    def _init_ros(self):
        if not rclpy.ok(): rclpy.init(args=None)
        
        # UI용 ROS 노드 생성 (PID 붙여 중복 방지)
        self._ros_node = UiRosBridge(node_name=f"ui_client_{os.getpid()}")

        # Qt 타이머로 ROS 스핀 (50Hz)
        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start(20)

        # 큐 전송용 변수
        self._pending_cmds = []
        self._pending_done_prefix = ""
        self._pending_target = "bio_main_control"

        self.log_t2("🟢 시스템 준비 완료 (ROS2 Connected)")

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        except:
            pass

    def _send_cmd_queue(self, cmds, *, start_prefix: str, done_prefix: str):
        if not cmds: return
        self._pending_cmds = list(cmds)
        self._pending_done_prefix = done_prefix
        
        self.log_t2(f"{start_prefix} ({len(cmds)}건)...")
        self._send_next_cmd()

    def _send_next_cmd(self):
        if not self._pending_cmds: return
        
        cmd = self._pending_cmds.pop(0)
        self.log_t2(f"➡️ SEND: {cmd}")

        def _on_feedback(status):
            self.log_t2(f"… {status}")

        def _on_done(ok, msg):
            tag = "✅" if ok else "❌"
            self.log_t2(f"{tag} {self._pending_done_prefix}: {msg}")
            if self._pending_cmds:
                self._send_next_cmd()

        # bio_main_control로 전송
        self._ros_node.send_command(
            cmd,
            target="bio_main_control",
            on_feedback=_on_feedback,
            on_done=_on_done
        )

    def closeEvent(self, event):
        try:
            self._ros_timer.stop()
            self._ros_node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        except:
            pass
        super().closeEvent(event)


# ========================================================
# [4] 메인 실행부
# ========================================================
def main(args=None):
    app = QApplication(sys.argv)
    window = BioBankApp()
    window.show()
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()