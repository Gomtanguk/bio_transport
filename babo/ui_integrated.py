import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton, 
                             QRadioButton, QButtonGroup, QTextEdit, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QFont

# =========================================================
# 인터페이스 정의 (import 실패 시 더미 클래스 사용)
# =========================================================
try:
    from biobank_interfaces.action import BioCommand
except ImportError:
    class BioCommand:
        class Goal: command = ""
        class Result: success = True; message = ""
        class Feedback: status = ""

# =========================================================
# 1. ROS2 Node 클래스 (Action Client + QoS)
# =========================================================
class BioUINode(Node):
    def __init__(self, ui_window):
        super().__init__('ui_client')
        self.ui = ui_window
        self.get_logger().info("🖥️ [UI] 노드 시작 (QoS: Reliable + Transient Local)")

        # ✅ [QoS] 사용자 요청 프로파일 정의
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ✅ Action Client 생성 (모든 채널에 QoS 적용)
        self._action_client = ActionClient(
            self,
            BioCommand,
            'bio_main_control',
            goal_service_qos_profile=self.custom_qos,
            result_service_qos_profile=self.custom_qos,
            cancel_service_qos_profile=self.custom_qos,
            feedback_sub_qos_profile=self.custom_qos,
            status_sub_qos_profile=self.custom_qos
        )

    def send_command(self, cmd_str):
        """서버로 명령 전송"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.ui.append_log("❌ [Error] 서버 연결 실패 (ROS2 Network 확인 필요)", "red")
            return

        goal_msg = BioCommand.Goal()
        goal_msg.command = cmd_str

        self.ui.append_log(f"➡ SEND: {cmd_str}", "lime")
        
        # 비동기 전송
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.ui.append_log("❌ [Rejected] 서버가 명령을 거절했습니다.", "red")
            return

        self.ui.append_log("🟢 [Accepted] 명령 수행 시작...", "yellow")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.ui.append_log(f"... {feedback.status}", "gray")

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.ui.append_log(f"✅ [Done] {result.message}", "cyan")
        else:
            self.ui.append_log(f"❌ [Fail] {result.message}", "red")

# =========================================================
# 2. PyQt5 GUI 클래스
# =========================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BioBank System v6.0 (Integrated Final)")
        self.resize(1400, 800)
        self.setStyleSheet("background-color: #f0f2f5;") # 전체 배경색

        # 메인 레이아웃
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # -------------------------------------------------
        # [왼쪽] 스토리지 뷰 (A, B, C, D)
        # -------------------------------------------------
        storage_frame = QFrame()
        storage_layout = QGridLayout(storage_frame)
        
        # 스크린샷처럼 Storage C, D가 위, A, B가 아래
        self.create_storage_box(storage_layout, "Storage C", ["C-1", "C-2", "C-3"], 0, 0)
        self.create_storage_box(storage_layout, "Storage D", ["D-1", "D-2", "D-3"], 0, 1)
        self.create_storage_box(storage_layout, "Storage A", ["A-1", "A-2", "A-3"], 1, 0)
        self.create_storage_box(storage_layout, "Storage B", ["B-1", "B-2", "B-3"], 1, 1)
        
        main_layout.addWidget(storage_frame, stretch=2)

        # -------------------------------------------------
        # [오른쪽] 제어 패널 & 로그
        # -------------------------------------------------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_panel.setStyleSheet("background-color: white; border-radius: 10px;")

        # 1. 제어 패널 그룹
        ctrl_group = QGroupBox("렉(Rack) 제어 패널")
        ctrl_group.setFont(QFont("Arial", 12, QFont.Bold))
        ctrl_layout = QVBoxLayout()

        # 작업 모드 (Radio Button)
        mode_group = QGroupBox("작업 모드")
        mode_group.setStyleSheet("background-color: #2c3e50; color: white; font-weight: bold;")
        mode_layout = QVBoxLayout()
        
        self.rb_in = QRadioButton("렉 입고 (IN)")
        self.rb_out = QRadioButton("렉 출고 (OUT)")
        self.rb_move = QRadioButton("렉 이동 (MOVE)")
        self.rb_in.setChecked(True)

        # 라디오 버튼 스타일
        rb_style = "QRadioButton { color: white; margin: 5px; }"
        self.rb_in.setStyleSheet(rb_style)
        self.rb_out.setStyleSheet(rb_style)
        self.rb_move.setStyleSheet(rb_style)

        self.bg_mode = QButtonGroup()
        self.bg_mode.addButton(self.rb_in)
        self.bg_mode.addButton(self.rb_out)
        self.bg_mode.addButton(self.rb_move)

        mode_layout.addWidget(self.rb_in)
        mode_layout.addWidget(self.rb_out)
        mode_layout.addWidget(self.rb_move)
        mode_group.setLayout(mode_layout)
        ctrl_layout.addWidget(mode_group)

        # 입력 필드
        form_layout = QGridLayout()
        form_layout.addWidget(QLabel("바코드 :"), 0, 0)
        self.edit_barcode = QLineEdit()
        self.edit_barcode.setPlaceholderText("바코드...")
        form_layout.addWidget(self.edit_barcode, 0, 1)

        form_layout.addWidget(QLabel("선택 객체(Src) :"), 1, 0)
        self.edit_src = QLineEdit()
        form_layout.addWidget(self.edit_src, 1, 1)

        form_layout.addWidget(QLabel("목적지(Dest) :"), 2, 0)
        self.edit_dest = QLineEdit()
        form_layout.addWidget(self.edit_dest, 2, 1)

        ctrl_layout.addLayout(form_layout)

        # 버튼
        btn_layout = QHBoxLayout()
        self.btn_confirm = QPushButton("확인")
        self.btn_confirm.setStyleSheet("background-color: #3498db; color: white; padding: 10px;")
        self.btn_confirm.clicked.connect(self.on_click_confirm)

        self.btn_cancel = QPushButton("취소")
        self.btn_cancel.setStyleSheet("background-color: #ecf0f1; color: black; padding: 10px;")
        self.btn_cancel.clicked.connect(self.on_click_cancel)

        btn_layout.addWidget(self.btn_confirm)
        btn_layout.addWidget(self.btn_cancel)
        ctrl_layout.addLayout(btn_layout)
        
        self.btn_init = QPushButton("초기화")
        self.btn_init.setStyleSheet("margin-top: 5px; padding: 5px;")
        self.btn_init.clicked.connect(self.on_click_init)
        ctrl_layout.addWidget(self.btn_init)

        ctrl_group.setLayout(ctrl_layout)
        right_layout.addWidget(ctrl_group)

        # 2. 로그 패널
        log_group = QGroupBox("로그 (History)")
        log_group.setFont(QFont("Arial", 12, QFont.Bold))
        log_layout = QVBoxLayout()
        
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setStyleSheet("background-color: #1e272e; color: #00d2d3; font-family: Monospace;")
        
        log_layout.addWidget(self.text_log)
        log_group.setLayout(log_layout)
        
        right_layout.addWidget(log_group, stretch=1)
        main_layout.addWidget(right_panel, stretch=1)

        # ROS2 Node Reference (나중에 연결됨)
        self.ros_node = None

    def create_storage_box(self, parent_layout, title, rack_names, row, col):
        """스토리지 시각화 헬퍼 함수"""
        frame = QFrame()
        frame.setStyleSheet("background-color: #2c3e50; border-radius: 8px;")
        layout = QVBoxLayout(frame)
        
        # 헤더 (Title)
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("color: white; font-weight: bold; background-color: #34495e; padding: 5px;")
        layout.addWidget(lbl_title)

        # 랙 그리드 (A-1, A-2, A-3 등)
        grid = QHBoxLayout()
        for name in rack_names:
            rack_frame = QFrame()
            rack_frame.setStyleSheet("background-color: #34495e; border: 1px solid #7f8c8d;")
            rack_layout = QVBoxLayout(rack_frame)
            
            # 랙 이름 (핑크 배경)
            lbl_name = QLabel(name)
            lbl_name.setAlignment(Qt.AlignCenter)
            lbl_name.setStyleSheet("background-color: #ffcccc; color: black; font-weight: bold;")
            rack_layout.addWidget(lbl_name)

            # 슬롯 (회색 박스들)
            for _ in range(4):
                slot = QLabel()
                slot.setStyleSheet("background-color: #576574; border-radius: 2px; min-height: 20px;")
                rack_layout.addWidget(slot)
            
            grid.addWidget(rack_frame)
        
        layout.addLayout(grid)
        parent_layout.addWidget(frame, row, col)

    def set_ros_node(self, node):
        self.ros_node = node

    def append_log(self, text, color="white"):
        """로그 창에 색상 텍스트 추가"""
        color_map = {
            "white": "#ffffff", "red": "#ff4757", "lime": "#7bed9f", 
            "yellow": "#ffa502", "cyan": "#70a1ff", "gray": "#a4b0be"
        }
        hex_color = color_map.get(color, "#ffffff")
        self.text_log.append(f'<span style="color:{hex_color}">{text}</span>')

    def on_click_confirm(self):
        if not self.ros_node:
            return

        # 명령 생성
        cmd_type = ""
        if self.rb_in.isChecked(): cmd_type = "IN"
        elif self.rb_out.isChecked(): cmd_type = "OUT"
        elif self.rb_move.isChecked(): cmd_type = "MOVE"

        src = self.edit_src.text().strip() or "NONE"
        dest = self.edit_dest.text().strip() or "NONE"
        
        # 포맷: "RACK,IN,NONE,A-2" (Main Node가 파싱하는 형식에 맞춤)
        # Main Node Logic: parts = raw_cmd.split(',') -> sub_cmd = ",".join(parts[1:])
        # 따라서 "PREFIX,CMD,SRC,DEST" 형태로 보내야 Main이 "CMD,SRC,DEST"로 잘라서 씀
        
        final_cmd = f"RACK,{cmd_type},{src},{dest}"
        self.ros_node.send_command(final_cmd)

    def on_click_cancel(self):
        self.append_log("⚠ 작업 취소됨", "yellow")
        self.edit_src.clear()
        self.edit_dest.clear()
        self.edit_barcode.clear()

    def on_click_init(self):
        self.text_log.clear()
        self.append_log("[System] 로그 초기화됨", "white")


# =========================================================
# 3. 메인 실행부
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    
    # GUI 생성
    window = MainWindow()
    
    # ROS 노드 생성 및 GUI 연결
    ros_node = BioUINode(window)
    window.set_ros_node(ros_node)
    
    window.show()

    # ROS Spin을 위한 타이머 설정 (10ms마다 spin)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()