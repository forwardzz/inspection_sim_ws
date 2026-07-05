import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from PyQt5.QtCore import QSettings, Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .config import (
    DEFAULT_IMU_SERIAL_PORT,
    DEFAULT_LIDAR_BAUDRATE,
    DEFAULT_LIDAR_SERIAL_PORT,
    DEFAULT_ROS_SETUP_PATH,
)
from .remote_robot_manager import RemoteRobotManager


DEFAULT_REMOTE_HOST = "192.168.43.21"
DEFAULT_REMOTE_USER = "yy"
DEFAULT_REMOTE_PORT = "22"
DEFAULT_REMOTE_WORKSPACE = "/home/yy/inspection_sim_ws"
DEFAULT_REMOTE_MAP = "/home/yy/inspection_sim_ws/maps/inspection_map.yaml"
DEFAULT_LOCAL_WORKSPACE = "/home/zjy/inspection_sim_ws"
DEFAULT_RVIZ_CONFIG = "/home/zjy/inspection_sim_ws/src/inspection_sim_gui/rviz/remote_robot.rviz"
DEFAULT_ROS_DOMAIN_ID = "0"
DEFAULT_ROS_LOCALHOST_ONLY = "0"


class RemoteRobotWindow(QMainWindow):
    def __init__(self, defaults=None):
        super().__init__()
        defaults = defaults or {}
        self.settings = QSettings("inspection_sim", "remote_robot_gui")
        self.manager = RemoteRobotManager(
            host=defaults.get("host", DEFAULT_REMOTE_HOST),
            user=defaults.get("user", DEFAULT_REMOTE_USER),
            port=defaults.get("port", DEFAULT_REMOTE_PORT),
            workspace=defaults.get("workspace", DEFAULT_REMOTE_WORKSPACE),
            ros_setup=defaults.get("ros_setup", DEFAULT_ROS_SETUP_PATH),
            local_workspace=defaults.get("local_workspace", DEFAULT_LOCAL_WORKSPACE),
        )
        self.manager.log_line.connect(self.append_log)
        self.manager.state_changed.connect(self._launch_state_changed)
        self.manager.rviz_state_changed.connect(self._rviz_state_changed)

        self.setWindowTitle("WUT 远端巡检机器人控制台")
        self.resize(1280, 760)

        self.host_edit = QLineEdit(self._setting("host", defaults.get("host", DEFAULT_REMOTE_HOST)))
        self.user_edit = QLineEdit(self._setting("user", defaults.get("user", DEFAULT_REMOTE_USER)))
        self.port_edit = QLineEdit(str(self._setting("port", defaults.get("port", DEFAULT_REMOTE_PORT))))
        self.local_workspace_edit = QLineEdit(
            self._setting("local_workspace", defaults.get("local_workspace", DEFAULT_LOCAL_WORKSPACE))
        )
        self.remote_workspace_edit = QLineEdit(
            self._setting("remote_workspace", defaults.get("workspace", DEFAULT_REMOTE_WORKSPACE))
        )
        self.ros_setup_edit = QLineEdit(
            self._setting("ros_setup", defaults.get("ros_setup", DEFAULT_ROS_SETUP_PATH))
        )
        self.map_edit = QLineEdit(self._setting("map", defaults.get("map", DEFAULT_REMOTE_MAP)))
        self.rviz_config_edit = QLineEdit(
            self._setting("rviz_config", defaults.get("rviz_config", self._default_rviz_config()))
        )
        self.ros_domain_edit = QLineEdit(self._setting("ros_domain_id", DEFAULT_ROS_DOMAIN_ID))
        self.ros_localhost_edit = QLineEdit(
            self._setting("ros_localhost_only", DEFAULT_ROS_LOCALHOST_ONLY)
        )

        self.sensor_source_combo = QComboBox()
        self.sensor_source_combo.addItem("硬件串口", "hardware")
        self.sensor_source_combo.addItem("仿真传感器", "sim")
        sensor_source = self._setting("sensor_source", defaults.get("sensor_source", "hardware"))
        sensor_index = self.sensor_source_combo.findData(sensor_source)
        self.sensor_source_combo.setCurrentIndex(max(sensor_index, 0))

        self.use_rviz_check = QCheckBox("远端 RViz")
        self.use_rviz_check.setChecked(self._setting("use_rviz", "false") == "true")
        self.local_rviz_check = QCheckBox("本机 RViz")
        self.local_rviz_check.setChecked(self._setting("local_rviz", "true") == "true")
        self.headless_check = QCheckBox("Gazebo headless")
        self.headless_check.setChecked(self._setting("headless", "true") == "true")
        self.use_sim_time_check = QCheckBox("use_sim_time")
        self.use_sim_time_check.setChecked(self._setting("use_sim_time", "false") == "true")

        self.lidar_port_edit = QLineEdit(
            self._setting("lidar_serial_port", DEFAULT_LIDAR_SERIAL_PORT)
        )
        self.lidar_baudrate_edit = QLineEdit(
            self._setting("lidar_baudrate", DEFAULT_LIDAR_BAUDRATE)
        )
        self.imu_port_edit = QLineEdit(
            self._setting("imu_serial_port", DEFAULT_IMU_SERIAL_PORT)
        )

        self.connection_label = QLabel("连接: 未检查")
        self.launch_label = QLabel("远端启动: idle")
        self.rviz_label = QLabel("本机 RViz: idle")
        self.workspace_label = QLabel("远端工作空间: 未检查")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1600)

        self._build_ui()
        self._apply_style()

    def _setting(self, key, default):
        value = self.settings.value(key, default)
        if value is None:
            return default
        return str(value)

    def _default_rviz_config(self):
        candidates = []
        try:
            candidates.append(
                os.path.join(
                    get_package_share_directory("inspection_sim_gui"),
                    "rviz",
                    "remote_robot.rviz",
                )
            )
        except PackageNotFoundError:
            pass
        candidates.append(
            os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "rviz", "remote_robot.rviz")
            )
        )
        candidates.append(DEFAULT_RVIZ_CONFIG)
        for path in candidates:
            if os.path.exists(path):
                return path
        return candidates[-1]

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)
        root.addWidget(self._build_header())

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(10)
        left_layout.addWidget(self._build_launch_group())
        left_layout.addWidget(self._build_mission_group())
        left_layout.addWidget(self._build_status_group(), 1)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setSpacing(10)
        right_layout.addWidget(self._build_connection_group())
        right_layout.addWidget(self._build_log_group(), 1)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([800, 460])

    def _build_header(self):
        header = QWidget()
        header.setObjectName("Header")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(18, 10, 18, 10)
        layout.setSpacing(12)

        logo = QLabel()
        logo.setObjectName("Logo")
        logo.setFixedSize(54, 54)
        logo.setPixmap(self._load_logo_pixmap().scaled(54, 54, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        logo.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(logo)

        title_box = QVBoxLayout()
        title = QLabel("Tracked Robot Control Center")
        title.setObjectName("Title")
        subtitle = QLabel("本机局域网 UI：SSH 启动远端机器人，并同步打开本机 RViz")
        subtitle.setObjectName("Subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box, 1)

        self.connection_label.setObjectName("Badge")
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setMinimumWidth(130)
        layout.addWidget(self.connection_label)
        return header

    def _load_logo_pixmap(self):
        candidates = []
        try:
            candidates.append(
                os.path.join(get_package_share_directory("inspection_sim_gui"), "assets", "wut_logo.png")
            )
        except PackageNotFoundError:
            pass
        candidates.append(
            os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "assets", "wut_logo.png")
            )
        )
        for path in candidates:
            pixmap = QPixmap(path)
            if not pixmap.isNull():
                return pixmap
        fallback = QPixmap(54, 54)
        fallback.fill(Qt.transparent)
        return fallback

    def _build_connection_group(self):
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("地址"), 0, 0)
        layout.addWidget(self.host_edit, 0, 1)
        layout.addWidget(QLabel("用户"), 0, 2)
        layout.addWidget(self.user_edit, 0, 3)
        layout.addWidget(QLabel("端口"), 0, 4)
        layout.addWidget(self.port_edit, 0, 5)

        layout.addWidget(QLabel("本地 workspace"), 1, 0)
        layout.addWidget(self.local_workspace_edit, 1, 1, 1, 5)
        layout.addWidget(QLabel("远端 workspace"), 2, 0)
        layout.addWidget(self.remote_workspace_edit, 2, 1, 1, 5)
        layout.addWidget(QLabel("ROS setup"), 3, 0)
        layout.addWidget(self.ros_setup_edit, 3, 1, 1, 5)

        buttons = QHBoxLayout()
        for text, handler in [
            ("测试连接", self.test_connection),
            ("同步源码", self.sync_workspace),
            ("远端构建", self.build_workspace),
            ("远端状态", self.remote_status),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            buttons.addWidget(button)
        layout.addLayout(buttons, 4, 0, 1, 6)
        return group

    def _build_launch_group(self):
        group = QGroupBox("Mission Control")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("地图 YAML"), 0, 0)
        layout.addWidget(self.map_edit, 0, 1, 1, 5)

        layout.addWidget(QLabel("本机 RViz 配置"), 1, 0)
        layout.addWidget(self.rviz_config_edit, 1, 1, 1, 5)

        layout.addWidget(QLabel("ROS_DOMAIN_ID"), 2, 0)
        layout.addWidget(self.ros_domain_edit, 2, 1)
        layout.addWidget(QLabel("LOCALHOST_ONLY"), 2, 2)
        layout.addWidget(self.ros_localhost_edit, 2, 3)
        layout.addWidget(self.local_rviz_check, 2, 4)
        layout.addWidget(self.use_rviz_check, 2, 5)

        layout.addWidget(QLabel("传感器来源"), 3, 0)
        layout.addWidget(self.sensor_source_combo, 3, 1)
        layout.addWidget(self.headless_check, 3, 2)
        layout.addWidget(self.use_sim_time_check, 3, 3)

        layout.addWidget(QLabel("雷达串口"), 4, 0)
        layout.addWidget(self.lidar_port_edit, 4, 1, 1, 3)
        layout.addWidget(QLabel("波特率"), 4, 4)
        layout.addWidget(self.lidar_baudrate_edit, 4, 5)

        layout.addWidget(QLabel("IMU 串口"), 5, 0)
        layout.addWidget(self.imu_port_edit, 5, 1, 1, 5)

        start_buttons = QHBoxLayout()
        for text, handler in [
            ("启动建图", self.start_mapping),
            ("启动导航", self.start_navigation),
            ("停止远端启动项", self.stop_launch),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            start_buttons.addWidget(button)
        layout.addLayout(start_buttons, 6, 0, 1, 6)

        return group

    def _build_mission_group(self):
        group = QGroupBox("RViz Mission")
        layout = QVBoxLayout(group)
        hint = QLabel(
            "本机 RViz 使用原项目配置启动。用 Publish Point 添加巡检点，"
            "用 2D Goal Pose 发布普通导航目标；任务服务由远端节点提供。"
        )
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        buttons = QHBoxLayout()
        for text, handler in [
            ("检查话题/节点", self.check_topics),
            ("发布 /cmd_vel 停止", self.publish_stop),
            ("查看导航参数", self.show_navigation_args),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        return group

    def _build_status_group(self):
        group = QGroupBox("状态")
        layout = QFormLayout(group)
        self.launch_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.rviz_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.workspace_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addRow("启动", self.launch_label)
        layout.addRow("RViz", self.rviz_label)
        layout.addRow("工作空间", self.workspace_label)
        return group

    def _build_log_group(self):
        group = QGroupBox("远端日志")
        layout = QVBoxLayout(group)
        layout.addWidget(self.log_view)
        return group

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #ebe6dc; color: #202426; }
            QWidget#Header { background: #153243; }
            QLabel#Title { color: #f4efe7; font-size: 22px; font-weight: 700; padding: 0; }
            QLabel#Subtitle { color: #d6cdbf; padding: 0; }
            QLabel#Hint { color: #3e4a4f; font-weight: 500; }
            QLabel#Badge {
                background: #ffc857;
                color: #153243;
                border-radius: 5px;
                padding: 8px 10px;
                font-weight: 700;
            }
            QLabel#Logo { background: transparent; }
            QGroupBox {
                font-weight: 700;
                border: 1px solid #c5cccc;
                border-radius: 6px;
                margin-top: 12px;
                padding: 12px;
                background: #f7f3eb;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLineEdit, QComboBox {
                background: #ffffff;
                border: 1px solid #b8c0c2;
                border-radius: 4px;
                padding: 5px;
            }
            QPlainTextEdit {
                background: #101820;
                color: #d8f3dc;
                border: 1px solid #263644;
                border-radius: 4px;
                padding: 7px;
                font-family: "Courier New", monospace;
            }
            QPushButton {
                background: #2f5f73;
                color: white;
                border: 0;
                border-radius: 5px;
                padding: 8px 10px;
                font-weight: 600;
            }
            QPushButton:hover { background: #39758d; }
            QPushButton:pressed { background: #234758; }
            QPushButton:disabled { background: #9aa3a6; }
            QCheckBox { font-weight: 500; }
            """
        )

    def _apply_connection(self):
        self.manager.update_connection(
            self.host_edit.text().strip() or DEFAULT_REMOTE_HOST,
            self.user_edit.text().strip() or DEFAULT_REMOTE_USER,
            self.port_edit.text().strip() or DEFAULT_REMOTE_PORT,
            self.remote_workspace_edit.text().strip() or DEFAULT_REMOTE_WORKSPACE,
            self.ros_setup_edit.text().strip() or DEFAULT_ROS_SETUP_PATH,
            self.local_workspace_edit.text().strip() or DEFAULT_LOCAL_WORKSPACE,
            self.ros_domain_edit.text().strip() or DEFAULT_ROS_DOMAIN_ID,
            self.ros_localhost_edit.text().strip() or DEFAULT_ROS_LOCALHOST_ONLY,
        )
        self._save_settings()
        target = f"{self.manager.user}@{self.manager.host}"
        self.connection_label.setText(target)
        self.workspace_label.setText(self.manager.workspace)

    def _save_settings(self):
        values = {
            "host": self.host_edit.text().strip(),
            "user": self.user_edit.text().strip(),
            "port": self.port_edit.text().strip(),
            "local_workspace": self.local_workspace_edit.text().strip(),
            "remote_workspace": self.remote_workspace_edit.text().strip(),
            "ros_setup": self.ros_setup_edit.text().strip(),
            "map": self.map_edit.text().strip(),
            "rviz_config": self.rviz_config_edit.text().strip(),
            "ros_domain_id": self.ros_domain_edit.text().strip(),
            "ros_localhost_only": self.ros_localhost_edit.text().strip(),
            "sensor_source": self.sensor_source_combo.currentData() or "hardware",
            "use_rviz": "true" if self.use_rviz_check.isChecked() else "false",
            "local_rviz": "true" if self.local_rviz_check.isChecked() else "false",
            "headless": "true" if self.headless_check.isChecked() else "false",
            "use_sim_time": "true" if self.use_sim_time_check.isChecked() else "false",
            "lidar_serial_port": self.lidar_port_edit.text().strip(),
            "lidar_baudrate": self.lidar_baudrate_edit.text().strip(),
            "imu_serial_port": self.imu_port_edit.text().strip(),
        }
        for key, value in values.items():
            self.settings.setValue(key, value)
        self.settings.sync()

    def _sensor_args(self):
        return {
            "sensor_source": self.sensor_source_combo.currentData() or "hardware",
            "lidar_serial_port": self.lidar_port_edit.text().strip() or DEFAULT_LIDAR_SERIAL_PORT,
            "lidar_baudrate": self.lidar_baudrate_edit.text().strip() or DEFAULT_LIDAR_BAUDRATE,
            "imu_serial_port": self.imu_port_edit.text().strip() or DEFAULT_IMU_SERIAL_PORT,
            "use_rviz": self.use_rviz_check.isChecked(),
            "headless": self.headless_check.isChecked(),
            "use_sim_time": self.use_sim_time_check.isChecked(),
            "start_local_rviz": self.local_rviz_check.isChecked(),
            "local_rviz_config": self.rviz_config_edit.text().strip(),
        }

    def test_connection(self):
        self._apply_connection()
        self.manager.test_connection()

    def sync_workspace(self):
        self._apply_connection()
        self.manager.sync_workspace()

    def build_workspace(self):
        self._apply_connection()
        self.manager.build_workspace()

    def remote_status(self):
        self._apply_connection()
        self.manager.status()

    def show_navigation_args(self):
        self._apply_connection()
        self.manager.show_navigation_args()

    def start_mapping(self):
        self._apply_connection()
        self.manager.start_mapping(**self._sensor_args())

    def start_navigation(self):
        self._apply_connection()
        map_path = self.map_edit.text().strip() or DEFAULT_REMOTE_MAP
        if not map_path:
            QMessageBox.warning(self, "缺少地图路径", "启动导航前需要设置远端地图 YAML。")
            return
        self.manager.start_navigation(map_path, **self._sensor_args())

    def stop_launch(self):
        self._apply_connection()
        self.manager.stop_launch()

    def check_topics(self):
        self._apply_connection()
        self.manager.check_topics()

    def publish_stop(self):
        self._apply_connection()
        self.manager.publish_stop()

    def append_log(self, line):
        if not line:
            return
        self.log_view.appendPlainText(line)
        bar = self.log_view.verticalScrollBar()
        bar.setValue(bar.maximum())
        if "[OK] workspace exists" in line:
            self.workspace_label.setText("远端工作空间: 已找到")
        elif "[WARN] workspace missing" in line:
            self.workspace_label.setText("远端工作空间: 缺失")
        if line.startswith("[REMOTE] host="):
            self.connection_label.setText("连接: 可用")
        elif "Permission denied" in line or "Could not resolve" in line or "Connection timed out" in line:
            self.connection_label.setText("连接: 失败")

    def _launch_state_changed(self, state):
        self.launch_label.setText(f"远端启动: {state}")

    def _rviz_state_changed(self, state):
        self.rviz_label.setText(f"本机 RViz: {state}")

    def closeEvent(self, event):
        self._save_settings()
        if self.manager.is_running():
            reply = QMessageBox.question(
                self,
                "关闭控制台",
                "仍有远端日志跟随或本机 RViz 在运行。关闭窗口会停止本机 RViz 和日志跟随，不会自动停止远端 launch。是否继续？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if reply != QMessageBox.Yes:
                event.ignore()
                return
        self.manager.shutdown()
        event.accept()


def run_app(defaults=None):
    app = QApplication.instance() or QApplication([])
    window = RemoteRobotWindow(defaults)
    window.show()
    return app.exec_()
