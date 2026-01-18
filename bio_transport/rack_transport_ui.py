# rack_transport_ui v1.200 2026-01-18
# [이번 버전에서 수정된 사항]
# - (기능추가) 렉 이동(mode==3) 실행 시 rack_transport에 from_rack/to_rack 파라미터 전달
#   - 예: ros2 run bio_transport rack_transport --ros-args -p from_rack:='A-1' -p to_rack:='B-3'
# - (구조개선) UI에 표시되는 랙 목록을 rack_stations.RACK_TARGETS 기반으로 동적 생성(불일치 방지)
# - (유지) 기존 UI/로그/선택 로직은 최대한 유지

import sys
import os
import subprocess

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QScrollArea, QGroupBox, QFrame, QGridLayout,
    QLabel, QToolButton, QPushButton, QRadioButton, QLineEdit,
    QTextEdit, QSizePolicy, QButtonGroup, QFormLayout
)
from PySide6.QtCore import Qt

# ✅ RACK_TARGETS 기반 동적 생성
from .rack_stations import RACK_TARGETS

# ========================================================
# [스타일시트]
# ========================================================
STYLE_SHEET = """
QWidget { font-family: "Segoe UI", "Malgun Gothic", sans-serif; color: #000000; }
QMainWindow { background-color: #F1F5F9; }

/* 라디오 버튼 */
QRadioButton { font-size: 14px; font-weight: bold; color: #333333; padding: 4px; }
QRadioButton::indicator { width: 16px; height: 16px; }
QRadioButton::indicator:checked { background-color: #2563EB; border: 2px solid #BFDBFE; border-radius: 8px; }

/* 탭 */
QTabWidget::pane { border: 1px solid #CBD5E1; background: #FFFFFF; border-radius: 6px; }
QTabBar::tab { background: #E2E8F0; color: #64748B; padding: 10px 25px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background: #FFFFFF; color: #2563EB; border-top: 3px solid #2563EB; }

/* 그룹박스 */
QGroupBox { font-weight: bold; border: 1px solid #E2E8F0; border-radius: 8px; margin-top: 20px; background-color: #FFFFFF; color: #000000; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }

/* 랙 프레임 */
QFrame.RackFrame { background-color: #334155; border-radius: 6px; border: 1px solid #1E293B; }

/* 입력창 */
QLineEdit { border: 1px solid #CBD5E1; border-radius: 4px; padding: 6px; background: #F8FAFC; color: #000000; }
QLineEdit:focus { border: 1px solid #2563EB; background: #FFFFFF; }
QLineEdit[readOnly="true"] { background-color: #E2E8F0; color: #555555; }

/* 로그 창 */
QTextEdit { background-color: #1E293B; color: #00FF00; font-family: "Consolas", monospace; font-size: 12px; border-radius: 4px; border: 1px solid #334155; }

/* 튜브 버튼 */
QToolButton.TubeBtn { background-color: #F8FAFC; border: 2px solid #94A3B8; border-radius: 13px; width: 52px; height: 52px; margin: 4px; }
QToolButton.TubeBtn:checked { background-color: #10B981; border-color: #059669; }

/* 랙 선택 버튼 */
QPushButton.RackSelectBtn {
    background-color: #475569;
    color: #FFFFFF;
    border: 1px solid #64748B;
    border-radius: 4px;
    font-size: 18px;
    font-weight: bold;
    min-height: 30px;
}
QPushButton.RackSelectBtn:checked { background-color: #F59E0B; border-color: #D97706; font-weight: bold; color: #FFFFFF; }

/* 일반 버튼 */
QPushButton { background-color: #FFFFFF; border: 1px solid #CBD5E1; color: #333333; font-weight: bold; border-radius: 4px; padding: 8px; min-height: 35px; }
QPushButton:pressed { background-color: #E2E8F0; padding-top: 10px; padding-bottom: 6px; border-color: #94A3B8; }

/* 확인 버튼 */
QPushButton#btnConfirm { background-color: #2563EB; color: #FFFFFF; border: none; border-bottom: 3px solid #1D4ED8; }
QPushButton#btnConfirm:hover { background-color: #1D4ED8; }
QPushButton#btnConfirm:pressed { background-color: #1E3A8A; border-bottom: 0px solid; border-top: 3px solid transparent; padding-top: 10px; padding-bottom: 6px; }
"""


def _build_storage_map():
    """
    RACK_TARGETS = {"A-1":..., "B-3":...} 형태에서
    storages = ["A","B"], storage_to_idxs["A"]=[1,2,3] 형태로 변환
    """
    storage_to_idxs = {}
    for k in RACK_TARGETS.keys():
        parts = str(k).strip().upper().split("-")
        if len(parts) != 2:
            continue
        sname = parts[0]
        try:
            idx = int(parts[1])
        except Exception:
            continue
        if sname not in storage_to_idxs:
            storage_to_idxs[sname] = []
        storage_to_idxs[sname].append(idx)

    for sname in storage_to_idxs:
        storage_to_idxs[sname] = sorted(list(set(storage_to_idxs[sname])))

    storages = sorted(list(storage_to_idxs.keys()))
    return storages, storage_to_idxs


class BioBankApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.proc = None
        self.setWindowTitle("BioBank System UI v1.200 (Rack Params Linked)")
        self.resize(1300, 850)
        self.setStyleSheet(STYLE_SHEET)

        self.t1_mode_group = QButtonGroup(self)
        self.t2_mode_group = QButtonGroup(self)

        # [다중 선택 저장용 Set]
        self.t1_selected_items = set()
        self.t1_dest_items = set()

        self.t2_selected_items = set()
        self.t2_dest_items = set()

        # 버튼 객체 저장 (불 끄기용)
        self.t1_active_buttons = set()
        self.t2_active_buttons = set()

        # ✅ RACK_TARGETS 기반 storage 구성
        self.storages, self.storage_to_idxs = _build_storage_map()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.setup_tab1()
        self.setup_tab2()
        



        # ROS2 실행에 사용할 워크스페이스(기본값: ~/bio_transport_ws)
        self.ros_ws = os.environ.get("BIO_TRANSPORT_WS", os.path.expanduser("~/bio_transport_ws"))

    # ==========================================================
    # [로직] 초기화 기능
    # ==========================================================
    def reset_selection_t1(self):
        for btn in self.t1_active_buttons:
            btn.setChecked(False)
        self.t1_active_buttons.clear()
        self.t1_selected_items.clear()
        self.t1_dest_items.clear()

        self.le_t1_selected.clear()
        self.le_t1_dest.clear()
        self.le_t1_input.clear()

    def reset_selection_t2(self):
        for btn in self.t2_active_buttons:
            btn.setChecked(False)
        self.t2_active_buttons.clear()
        self.t2_selected_items.clear()
        self.t2_dest_items.clear()

        self.le_t2_selected.clear()
        self.le_t2_dest.clear()
        self.le_t2_input.clear()

    # ==========================================================
    # [핵심] 다중 선택 핸들러
    # ==========================================================
    def update_text_fields_t1(self):
        self.le_t1_selected.setText(", ".join(sorted(list(self.t1_selected_items))))
        self.le_t1_dest.setText(", ".join(sorted(list(self.t1_dest_items))))

    def update_text_fields_t2(self):
        self.le_t2_selected.setText(", ".join(sorted(list(self.t2_selected_items))))
        self.le_t2_dest.setText(", ".join(sorted(list(self.t2_dest_items))))

    def on_tube_clicked(self, checked, tube_id, btn_obj):
        mode_id = self.t1_mode_group.checkedId()

        # [이동 모드]는 1:1 방식 유지
        if mode_id == 3:
            if not checked:
                if self.le_t1_selected.text() == tube_id:
                    self.le_t1_selected.clear()
                elif self.le_t1_dest.text() == tube_id:
                    self.le_t1_dest.clear()
                if btn_obj in self.t1_active_buttons:
                    self.t1_active_buttons.remove(btn_obj)
            else:
                self.t1_active_buttons.add(btn_obj)
                curr_sel = self.le_t1_selected.text()
                if not curr_sel:
                    self.le_t1_selected.setText(tube_id)
                else:
                    self.le_t1_dest.setText(tube_id)
            return

        # [입고/출고/폐기] 다중 선택 로직
        target_set = None
        if mode_id == 1:
            target_set = self.t1_dest_items
        elif mode_id == 2 or mode_id == 4:
            target_set = self.t1_selected_items
            if mode_id == 4:
                self.le_t1_dest.setText("폐기장 (Disposal)")

        if checked:
            target_set.add(tube_id)
            self.t1_active_buttons.add(btn_obj)
        else:
            if tube_id in target_set:
                target_set.remove(tube_id)
            if btn_obj in self.t1_active_buttons:
                self.t1_active_buttons.remove(btn_obj)

        self.update_text_fields_t1()

    def on_rack_clicked(self, checked, rack_id, btn_obj):
        mode_id = self.t2_mode_group.checkedId()

        # [이동 모드] 1:1 유지
        if mode_id == 3:
            if not checked:
                if self.le_t2_selected.text() == rack_id:
                    self.le_t2_selected.clear()
                elif self.le_t2_dest.text() == rack_id:
                    self.le_t2_dest.clear()
                if btn_obj in self.t2_active_buttons:
                    self.t2_active_buttons.remove(btn_obj)
            else:
                self.t2_active_buttons.add(btn_obj)
                if not self.le_t2_selected.text():
                    self.le_t2_selected.setText(rack_id)
                else:
                    self.le_t2_dest.setText(rack_id)
            return

        # [입고/출고] 다중 선택
        target_set = None
        if mode_id == 1:
            target_set = self.t2_dest_items
        elif mode_id == 2:
            target_set = self.t2_selected_items

        if checked:
            target_set.add(rack_id)
            self.t2_active_buttons.add(btn_obj)
        else:
            if rack_id in target_set:
                target_set.remove(rack_id)
            if btn_obj in self.t2_active_buttons:
                self.t2_active_buttons.remove(btn_obj)

        self.update_text_fields_t2()

    # ==========================================================
    # 확인 버튼 핸들러
    # ==========================================================
    def on_confirm_t1(self):
        mode = self.t1_mode_group.checkedId()
        sel = self.le_t1_selected.text()
        dest = self.le_t1_dest.text()

        if mode == 1 and not dest:
            self.log_t1("[경고] 입고할 목적지를 선택하세요.")
            return
        if (mode == 2 or mode == 4) and not sel:
            self.log_t1("[경고] 대상을 선택하세요.")
            return
        if mode == 3 and (not sel or not dest):
            self.log_t1("[경고] 이동 대상과 목적지를 선택하세요.")
            return

        action = {1: "입고", 2: "출고", 3: "이동", 4: "폐기"}[mode]
        msg = f"✅ [{action} 완료] "
        if mode == 1:
            msg += f"바코드:{self.le_t1_input.text()} -> {dest}"
        elif mode == 2:
            msg += f"대상: {sel}"
        elif mode == 3:
            msg += f"{sel} ➡️ {dest}"
        elif mode == 4:
            msg += f"{sel} -> 폐기됨"

        self.log_t1(msg)
        self.reset_selection_t1()

    def on_confirm_t2(self):
        mode = self.t2_mode_group.checkedId()
        sel = self.le_t2_selected.text().strip()
        dest = self.le_t2_dest.text().strip()

        # -------------------------
        # mode 1: inbound (WORKBENCH -> to_rack)
        # -------------------------
        if mode == 1:
            if not dest:
                self.log_t2("[경고] 입고 위치(to_rack) 1개를 선택하세요.")
                return

            self.log_t2(f"[실행] rack_inbound (WORKBENCH -> {dest})")
            extra = f"--ros-args -p to_rack:='{dest}'"
            self.run_ros2("rack_inbound", extra_ros_args=extra)

            self.log_t2(f"✅ [렉 입고 요청] 위치: {dest}")
            self.reset_selection_t2()
            return

        # -------------------------
        # mode 2: outbound (from_rack -> WORKBENCH)
        # -------------------------
        if mode == 2:
            if not sel:
                self.log_t2("[경고] 출고 대상(from_rack) 1개를 선택하세요.")
                return

            self.log_t2(f"[실행] rack_outbound ({sel} -> WORKBENCH)")
            extra = f"--ros-args -p from_rack:='{sel}'"
            self.run_ros2("rack_outbound", extra_ros_args=extra)

            self.log_t2(f"✅ [렉 출고 요청] 대상: {sel}")
            self.reset_selection_t2()
            return

        # -------------------------
        # mode 3: transport (from -> to)
        # -------------------------
        if mode == 3:
            if not sel or not dest:
                self.log_t2("[경고] 이동 대상(sel)과 목적지(dest)를 선택하세요.")
                return
            if sel == dest:
                self.log_t2("[경고] from/to가 같습니다. 다른 랙을 선택하세요.")
                return

            self.log_t2(f"[실행] rack_transport ({sel} -> {dest})")
            extra = f"--ros-args -p from_rack:='{sel}' -p to_rack:='{dest}'"
            self.run_ros2("rack_transport", extra_ros_args=extra)

            self.log_t2(f"✅ [렉 이동 요청] {sel} ➡️ {dest}")
            self.reset_selection_t2()
            return

        # -------------------------
        # fallback
        # -------------------------
        self.log_t2("[경고] 알 수 없는 모드입니다.")

        

    def log_t1(self, msg):
        self.txt_log_t1.append(msg)

    def log_t2(self, msg):
        self.txt_log_t2.append(msg)

    # ==========================================================
    # ROS2 노드 실행 (UI -> 로봇 동작)
    # ==========================================================
    def run_ros2(self, executable: str, extra_ros_args: str = ""):
                # 이전 실행이 끝났으면 proc 참조 정리
        if self.proc is not None and self.proc.poll() is not None:
            self.proc = None

        # ✅ 중복 실행 방지
        if self.proc is not None and self.proc.poll() is None:
            self.log_t2("[WARN] rack_transport already running. (blocked)")
            return False

        ws_setup = os.path.join(self.ros_ws, "install", "setup.bash")

        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"( [ -f '{ws_setup}' ] && source '{ws_setup}' || true ) && "
            f"ros2 run bio_transport {executable} {extra_ros_args}"
        )

        try:
            self.proc = subprocess.Popen(["bash", "-lc", cmd])
            return True
        except Exception as e:
            self.log_t2(f"[ERROR] ros2 실행 실패: {e}")
            self.proc = None
            return False


    # ==========================================================
    # UI 생성
    # ==========================================================
    def create_rack_widget(self, storage_name, rack_idx, mode="tube"):
        frame = QFrame()
        frame.setProperty("class", "RackFrame")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(frame)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        title = f"{storage_name}-{rack_idx}"

        if mode == "tube":
            lbl = QLabel(title)
            lbl.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(lbl)
            layout.addStretch(1)
            for i in range(1, 5):
                btn = QToolButton()
                btn.setProperty("class", "TubeBtn")
                btn.setCheckable(True)
                btn.setFixedSize(52, 52)
                btn.setCursor(Qt.CursorShape.PointingHandCursor)
                btn_id = f"{title}-{i}"
                btn.clicked.connect(lambda checked, bid=btn_id, b_obj=btn: self.on_tube_clicked(checked, bid, b_obj))
                layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            layout.addStretch(1)
        else:
            btn_sel = QPushButton(title)
            btn_sel.setProperty("class", "RackSelectBtn")
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

        # ✅ storages 개수에 맞춰 자동 배치 (2열 기본)
        cols = 2
        for idx, name in enumerate(self.storages):
            r = idx // cols
            c = idx % cols

            group = QGroupBox(f"Storage {name}")
            hbox = QHBoxLayout(group)
            hbox.setSpacing(10)
            hbox.setContentsMargins(10, 25, 10, 10)

            for rack_idx in self.storage_to_idxs.get(name, []):
                hbox.addWidget(self.create_rack_widget(name, rack_idx, mode))

            grid.addWidget(group, r, c)

        scroll.setWidget(content)
        return scroll

    def setup_tab1(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.create_storage_grid(mode="tube"), stretch=7)

        panel = QFrame()
        panel.setMinimumWidth(300)
        panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
        vbox = QVBoxLayout(panel)
        vbox.setContentsMargins(15, 15, 15, 15)
        vbox.setSpacing(10)

        lbl = QLabel("검체 제어 패널")
        lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;")
        vbox.addWidget(lbl)
        vbox.addWidget(self.create_line())

        grp_action = QGroupBox("작업 모드")
        v_radio = QVBoxLayout(grp_action)
        v_radio.setContentsMargins(10, 15, 10, 10)

        rb1 = QRadioButton("입고"); self.t1_mode_group.addButton(rb1, 1); rb1.setChecked(True)
        rb2 = QRadioButton("출고"); self.t1_mode_group.addButton(rb2, 2)
        rb3 = QRadioButton("이동"); self.t1_mode_group.addButton(rb3, 3)
        rb4 = QRadioButton("폐기"); self.t1_mode_group.addButton(rb4, 4)
        rb4.setStyleSheet("color: #EF4444; font-weight: bold;")
        v_radio.addWidget(rb1); v_radio.addWidget(rb2); v_radio.addWidget(rb3); v_radio.addWidget(rb4)
        vbox.addWidget(grp_action)

        form = QFormLayout()
        form.setVerticalSpacing(10)
        self.le_t1_input = QLineEdit(); self.le_t1_input.setPlaceholderText("바코드 입력...")
        self.le_t1_selected = QLineEdit(); self.le_t1_selected.setReadOnly(True)
        self.le_t1_dest = QLineEdit(); self.le_t1_dest.setReadOnly(True)
        form.addRow("바코드 :", self.le_t1_input)
        form.addRow("선택 객체 :", self.le_t1_selected)
        form.addRow("목적지 :", self.le_t1_dest)
        vbox.addLayout(form)

        h_btn = QHBoxLayout()
        btn_ok = QPushButton("확인"); btn_ok.setObjectName("btnConfirm"); btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_cancel = QPushButton("취소"); btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_ok.clicked.connect(self.on_confirm_t1)
        btn_cancel.clicked.connect(self.reset_selection_t1)
        h_btn.addWidget(btn_ok); h_btn.addWidget(btn_cancel)
        vbox.addLayout(h_btn)

        btn_reset = QPushButton("초기화"); btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.clicked.connect(self.reset_selection_t1)
        vbox.addWidget(btn_reset)

        vbox.addWidget(self.create_line())

        grp_log = QGroupBox("로그 (History)")
        grp_log.setFixedHeight(180)
        v_log = QVBoxLayout(grp_log); v_log.setContentsMargins(5, 15, 5, 5)
        self.txt_log_t1 = QTextEdit(); self.txt_log_t1.setReadOnly(True)
        self.txt_log_t1.setStyleSheet("background-color: #1E293B; color: #00FF00;")
        self.txt_log_t1.setText("[System] Tube Management Ready...")
        v_log.addWidget(self.txt_log_t1)
        vbox.addWidget(grp_log)
        vbox.addStretch(1)

        layout.addWidget(panel, stretch=3)
        self.tabs.addTab(tab, "튜브 관리")

    def setup_tab2(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.create_storage_grid(mode="rack"), stretch=7)

        panel = QFrame()
        panel.setMinimumWidth(300)
        panel.setStyleSheet("background-color: #FFFFFF; border-left: 1px solid #E2E8F0;")
        vbox = QVBoxLayout(panel)
        vbox.setContentsMargins(15, 15, 15, 15)
        vbox.setSpacing(10)

        lbl = QLabel("렉(Rack) 제어 패널")
        lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #1E293B;")
        vbox.addWidget(lbl)
        vbox.addWidget(self.create_line())

        grp_action = QGroupBox("작업 모드")
        v_radio = QVBoxLayout(grp_action)
        v_radio.setContentsMargins(10, 15, 10, 10)
        rb1 = QRadioButton("렉 입고"); self.t2_mode_group.addButton(rb1, 1); rb1.setChecked(True)
        rb2 = QRadioButton("렉 출고"); self.t2_mode_group.addButton(rb2, 2)
        rb3 = QRadioButton("렉 이동"); self.t2_mode_group.addButton(rb3, 3)
        v_radio.addWidget(rb1); v_radio.addWidget(rb2); v_radio.addWidget(rb3)
        vbox.addWidget(grp_action)

        form = QFormLayout()
        form.setVerticalSpacing(10)
        self.le_t2_input = QLineEdit(); self.le_t2_input.setPlaceholderText("렉 바코드 입력...")
        self.le_t2_selected = QLineEdit(); self.le_t2_selected.setReadOnly(True)
        self.le_t2_dest = QLineEdit(); self.le_t2_dest.setReadOnly(True)
        form.addRow("바코드 :", self.le_t2_input)
        form.addRow("선택 객체 :", self.le_t2_selected)
        form.addRow("목적지 :", self.le_t2_dest)
        vbox.addLayout(form)

        h_btn = QHBoxLayout()
        btn_ok = QPushButton("확인"); btn_ok.setObjectName("btnConfirm"); btn_ok.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_cancel = QPushButton("취소"); btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_ok.clicked.connect(self.on_confirm_t2)
        btn_cancel.clicked.connect(self.reset_selection_t2)
        h_btn.addWidget(btn_ok); h_btn.addWidget(btn_cancel)
        vbox.addLayout(h_btn)

        btn_reset = QPushButton("초기화"); btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.clicked.connect(self.reset_selection_t2)
        vbox.addWidget(btn_reset)

        vbox.addWidget(self.create_line())

        grp_log = QGroupBox("로그 (History)")
        grp_log.setFixedHeight(180)
        v_log = QVBoxLayout(grp_log); v_log.setContentsMargins(5, 15, 5, 5)
        self.txt_log_t2 = QTextEdit(); self.txt_log_t2.setReadOnly(True)
        self.txt_log_t2.setStyleSheet("background-color: #1E293B; color: #00FF00;")
        self.txt_log_t2.setText("[System] Rack Management Ready...")
        v_log.addWidget(self.txt_log_t2)
        vbox.addWidget(grp_log)
        vbox.addStretch(1)

        layout.addWidget(panel, stretch=3)
        self.tabs.addTab(tab, "렉 관리")

    def create_line(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        return line


def main():
    try:
        from PySide6.QtGui import QGuiApplication
        QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    except Exception:
        pass

    app = QApplication(sys.argv)
    window = BioBankApp()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
