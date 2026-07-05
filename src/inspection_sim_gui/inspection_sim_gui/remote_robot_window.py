import math
import os
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from PyQt5.QtCore import QSettings, Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from std_srvs.srv import SetBool, Trigger

from robot_monitor_interfaces.srv import Localize, StartNavigation

from .config import (
    DEFAULT_ANGULAR_SPEED_RADPS,
    DEFAULT_IMU_SERIAL_PORT,
    DEFAULT_LIDAR_BAUDRATE,
    DEFAULT_LIDAR_SERIAL_PORT,
    DEFAULT_LINEAR_SPEED_MPS,
    DEFAULT_ROS_SETUP_PATH,
    MAX_ANGULAR_SPEED_RADPS,
    MAX_LINEAR_SPEED_MPS,
)
from .main_window import GuiSignals, MapView
from .remote_robot_manager import RemoteRobotManager
from .ros_adapter import RosAdapter


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
        self._seed_ros_environment(defaults)

        self.signals = GuiSignals()
        self.ros = RosAdapter(
            status_callback=self.signals.log,
            feedback_callback=self.signals.nav_feedback,
            result_callback=self.signals.nav_result,
        )
        self.signals.log.connect(self.append_log)
        self.signals.nav_feedback.connect(self.append_log)
        self.signals.nav_result.connect(self._nav_result)
        self.signals.service_result.connect(self._service_result)

        self.manager = RemoteRobotManager(
            host=defaults.get("host", DEFAULT_REMOTE_HOST),
            user=defaults.get("user", DEFAULT_REMOTE_USER),
            port=defaults.get("port", DEFAULT_REMOTE_PORT),
            workspace=defaults.get("workspace", DEFAULT_REMOTE_WORKSPACE),
            ros_setup=defaults.get("ros_setup", DEFAULT_ROS_SETUP_PATH),
            local_workspace=defaults.get("local_workspace", DEFAULT_LOCAL_WORKSPACE),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID),
            ros_localhost_only=os.environ.get("ROS_LOCALHOST_ONLY", DEFAULT_ROS_LOCALHOST_ONLY),
        )
        self.manager.log_line.connect(self.append_log)
        self.manager.state_changed.connect(self._launch_state_changed)
        self.manager.rviz_state_changed.connect(self._rviz_state_changed)

        self.pose_history = []
        self.setWindowTitle("WUT 远端巡检机器人控制台")
        self.resize(1280, 780)

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
        self.rviz_config_edit = QLineEdit(
            self._setting("rviz_config", defaults.get("rviz_config", self._default_rviz_config()))
        )
        self.ros_domain_edit = QLineEdit(self._setting("ros_domain_id", DEFAULT_ROS_DOMAIN_ID))
        self.ros_localhost_edit = QLineEdit(
            self._setting("ros_localhost_only", DEFAULT_ROS_LOCALHOST_ONLY)
        )

        self.map_combo = QComboBox()
        self.map_combo.setEditable(True)
        self._set_map_text(self._setting("map", defaults.get("map", DEFAULT_REMOTE_MAP)))

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

        self.linear_spin = self._double_spin(0.0, MAX_LINEAR_SPEED_MPS, DEFAULT_LINEAR_SPEED_MPS, 0.01)
        self.angular_spin = self._double_spin(0.0, MAX_ANGULAR_SPEED_RADPS, DEFAULT_ANGULAR_SPEED_RADPS, 0.01)
        self.linear_slider = self._speed_slider(self.linear_spin, MAX_LINEAR_SPEED_MPS)
        self.angular_slider = self._speed_slider(self.angular_spin, MAX_ANGULAR_SPEED_RADPS)

        self.connection_label = QLabel("连接: 未检查")
        self.launch_label = QLabel("远端启动: idle")
        self.rviz_label = QLabel("本机 RViz: idle")
        self.workspace_label = QLabel("远端工作空间: 未检查")
        self.pose_label = QLabel("里程计: 等待数据")
        self.velocity_label = QLabel("速度: 等待数据")
        self.amcl_label = QLabel("AMCL: 等待数据")
        self.map_label = QLabel("地图: 等待数据")
        self.nav_label = QLabel("导航: idle")
        self.mission_label = QLabel("任务: RViz mode")
        self.safety_label = QLabel("安全: waiting")
        self.power_label = QLabel("电源: waiting")
        self.thermal_label = QLabel("热成像: waiting")
        self.gas_label = QLabel("气体: waiting")
        self.topic_labels = {
            "scan": QLabel("/scan"),
            "imu": QLabel("/imu"),
            "laser_odom": QLabel("/laser_odom"),
            "odom": QLabel("/odom"),
            "map": QLabel("/map"),
            "amcl": QLabel("/amcl_pose"),
            "thermal": QLabel("/thermal_frame"),
            "gas": QLabel("/gas_data"),
            "safety": QLabel("/robot_safety_status"),
        }
        self.region_mode_check = QCheckBox("Region Mode")
        self.map_view = MapView()
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1800)

        self._build_ui()
        self._apply_style()
        self._connect_speed_controls()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(300)

    def _seed_ros_environment(self, defaults):
        domain = self._setting("ros_domain_id", defaults.get("ros_domain_id", DEFAULT_ROS_DOMAIN_ID))
        localhost = self._setting(
            "ros_localhost_only",
            defaults.get("ros_localhost_only", DEFAULT_ROS_LOCALHOST_ONLY),
        )
        os.environ.setdefault("ROS_DOMAIN_ID", str(domain))
        os.environ.setdefault("ROS_LOCALHOST_ONLY", str(localhost))

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
        left_layout.addWidget(self._build_log_group(), 1)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setSpacing(10)
        right_layout.addWidget(self._build_connection_group())
        right_layout.addWidget(self._build_map_group())
        right_layout.addWidget(self._build_drive_group())
        right_layout.addWidget(self._build_status_group(), 1)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([780, 500])

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
        subtitle = QLabel("本机局域网 UI：按原 Tk GUI 控制远端机器人、任务、地图、RViz 和手动行进")
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
        layout.addWidget(self.map_combo, 0, 1, 1, 4)
        refresh_maps = QPushButton("Refresh Maps")
        refresh_maps.clicked.connect(self.refresh_maps)
        layout.addWidget(refresh_maps, 0, 5)

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

        row1 = QHBoxLayout()
        for text, handler in [
            ("Start Mapping", self.start_mapping),
            ("Start Navigation", self.start_navigation),
            ("Save Map", self.save_map),
            ("Stop Launch", self.stop_launch),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            row1.addWidget(button)
        layout.addLayout(row1, 6, 0, 1, 6)

        row2 = QHBoxLayout()
        for text, handler in [
            ("Start Thermal", self.start_thermal),
            ("Stop Thermal", self.stop_thermal),
            ("Reset Safety", self.reset_safety),
            ("Abort Mission", self.abort_mission),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            row2.addWidget(button)
        layout.addLayout(row2, 7, 0, 1, 6)
        return group

    def _build_mission_group(self):
        group = QGroupBox("RViz Mission")
        layout = QVBoxLayout(group)

        hint = QLabel(
            "RViz 操作方式与原 Tk GUI 一致：Publish Point 添加任务点；"
            "2D Goal Pose (Mission Heading) 设置点位朝向；Start Mission 执行点位或区域巡检。"
        )
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        top = QHBoxLayout()
        for text, handler in [
            ("Check Localization", self.check_localization),
            ("Start Mission", self.start_mission),
            ("Clear RViz Points", self.clear_rviz_points),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            top.addWidget(button)
        layout.addLayout(top)

        region = QHBoxLayout()
        self.region_mode_check.clicked.connect(self.set_region_mode)
        region.addWidget(self.region_mode_check)
        for text, handler in [
            ("Save Regions", self.save_regions),
            ("Load Regions", self.load_regions),
            ("Clear Regions", self.clear_regions),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            region.addWidget(button)
        layout.addLayout(region)

        checks = QHBoxLayout()
        for text, handler in [
            ("检查话题/节点", self.check_topics),
            ("发布 /cmd_vel 停止", self.publish_stop),
            ("查看导航参数", self.show_navigation_args),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            checks.addWidget(button)
        layout.addLayout(checks)

        self.mission_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.mission_label.setWordWrap(True)
        layout.addWidget(self.mission_label)
        return group

    def _build_map_group(self):
        group = QGroupBox("Live Map")
        layout = QVBoxLayout(group)
        self.map_view.setMinimumHeight(180)
        layout.addWidget(self.map_view)
        return group

    def _build_drive_group(self):
        group = QGroupBox("Manual Drive")
        layout = QVBoxLayout(group)

        speed_form = QFormLayout()
        speed_form.addRow("线速度 m/s", self.linear_spin)
        speed_form.addRow("", self.linear_slider)
        speed_form.addRow("角速度 rad/s", self.angular_spin)
        speed_form.addRow("", self.angular_slider)
        layout.addLayout(speed_form)

        pad = QGridLayout()
        buttons = {
            (0, 1): ("W / ↑", self.drive_forward),
            (1, 0): ("A / ←", self.turn_left),
            (1, 1): ("STOP", self.stop_robot),
            (1, 2): ("D / →", self.turn_right),
            (2, 1): ("S / ↓", self.drive_backward),
        }
        for (row, col), (text, handler) in buttons.items():
            button = QPushButton(text)
            button.setMinimumHeight(42)
            button.clicked.connect(handler)
            pad.addWidget(button, row, col)
        layout.addLayout(pad)

        arc = QHBoxLayout()
        left = QPushButton("Forward Left")
        right = QPushButton("Forward Right")
        left.clicked.connect(self.forward_left)
        right.clicked.connect(self.forward_right)
        arc.addWidget(left)
        arc.addWidget(right)
        layout.addLayout(arc)
        return group

    def _build_status_group(self):
        group = QGroupBox("Robot Status")
        layout = QVBoxLayout(group)

        topics = QGridLayout()
        for index, label in enumerate(self.topic_labels.values()):
            row = index // 3
            col = index % 3
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(28)
            topics.addWidget(label, row, col)
        layout.addLayout(topics)

        for label in [
            self.pose_label,
            self.velocity_label,
            self.amcl_label,
            self.map_label,
            self.nav_label,
            self.mission_label,
            self.safety_label,
            self.power_label,
            self.thermal_label,
            self.gas_label,
            self.launch_label,
            self.rviz_label,
            self.workspace_label,
        ]:
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addStretch(1)
        return group

    def _build_log_group(self):
        group = QGroupBox("Runtime Log")
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
            QLineEdit, QComboBox, QDoubleSpinBox {
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

    def _connect_speed_controls(self):
        self.linear_spin.valueChanged.connect(
            lambda value: self.linear_slider.setValue(int(value / MAX_LINEAR_SPEED_MPS * 100))
        )
        self.angular_spin.valueChanged.connect(
            lambda value: self.angular_slider.setValue(int(value / MAX_ANGULAR_SPEED_RADPS * 100))
        )
        self.linear_slider.valueChanged.connect(
            lambda value: self.linear_spin.setValue(MAX_LINEAR_SPEED_MPS * value / 100.0)
        )
        self.angular_slider.valueChanged.connect(
            lambda value: self.angular_spin.setValue(MAX_ANGULAR_SPEED_RADPS * value / 100.0)
        )

    def _double_spin(self, minimum, maximum, value, step):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(3 if step < 0.1 else 2)
        spin.setButtonSymbols(QAbstractSpinBox.PlusMinus)
        return spin

    def _speed_slider(self, spin, max_value):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(int(spin.value() / max_value * 100))
        return slider

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
            "map": self._map_text(),
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

    def refresh_maps(self):
        self._apply_connection()
        self.manager.list_maps()

    def save_map(self):
        self._apply_connection()
        map_path = self._normalize_map_path(self._map_text())
        if not map_path:
            QMessageBox.warning(self, "Missing Map", "Map YAML path is required.")
            return
        self._set_map_text(map_path)
        self.manager.save_map(map_path)

    def start_mapping(self):
        self._apply_connection()
        self.manager.start_mapping(**self._sensor_args())

    def start_navigation(self):
        self._apply_connection()
        map_path = self._normalize_map_path(self._map_text())
        if not map_path:
            QMessageBox.warning(self, "Missing Map", "Map YAML path is required.")
            return
        self._set_map_text(map_path)
        self.manager.start_navigation(map_path, **self._sensor_args())

    def stop_launch(self):
        self._apply_connection()
        self.manager.stop_launch()

    def start_thermal(self):
        self._apply_connection()
        self.manager.start_thermal()

    def stop_thermal(self):
        self._apply_connection()
        self.manager.stop_thermal()

    def check_topics(self):
        self._apply_connection()
        self.manager.check_topics()

    def publish_stop(self):
        self._apply_connection()
        self.manager.publish_stop()
        self.stop_robot()

    def abort_mission(self):
        self.append_log("[MISSION] abort mission")
        self.ros.call_service_async(
            self.ros.abort_mission_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("Abort Mission", result, error),
        )

    def reset_safety(self):
        self.append_log("[SAFETY] reset requested")
        self.ros.call_service_async(
            self.ros.reset_safety_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("Reset Safety", result, error),
        )

    def check_localization(self):
        self.append_log("[MISSION] check localization")
        self.ros.call_service_async(
            self.ros.localize_client,
            Localize.Request(),
            lambda result, error: self._emit_service_result("Check Localization", result, error),
        )

    def start_mission(self):
        snap = self.ros.snapshot()
        if snap.get("safety_level") == "FAULT":
            QMessageBox.warning(
                self,
                "Start Mission",
                f"Safety fault is latched: {snap.get('safety_message', '')}",
            )
            return
        self.append_log("[MISSION] start mission")
        self.ros.call_service_async(
            self.ros.start_navigation_client,
            StartNavigation.Request(),
            lambda result, error: self._emit_service_result("Start Mission", result, error),
            timeout_sec=10.0,
        )

    def clear_rviz_points(self):
        self.append_log("[MISSION] clear RViz points")
        self.ros.call_service_async(
            self.ros.clear_rviz_points_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("Clear RViz Points", result, error),
        )

    def set_region_mode(self):
        request = SetBool.Request()
        request.data = bool(self.region_mode_check.isChecked())
        self.append_log(f"[MISSION] region mode {request.data}")
        self.ros.call_service_async(
            self.ros.set_region_mode_client,
            request,
            lambda result, error: self._emit_service_result("Region Mode", result, error),
        )

    def save_regions(self):
        self._call_region_trigger(
            self.ros.save_inspection_regions_client,
            "Save Regions",
            "[MISSION] save regions",
        )

    def load_regions(self):
        self._call_region_trigger(
            self.ros.load_inspection_regions_client,
            "Load Regions",
            "[MISSION] load regions",
        )

    def clear_regions(self):
        self._call_region_trigger(
            self.ros.clear_inspection_regions_client,
            "Clear Regions",
            "[MISSION] clear regions",
        )

    def _call_region_trigger(self, client, title, log_line):
        self.append_log(log_line)
        self.ros.call_service_async(
            client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result(title, result, error),
        )

    def _emit_service_result(self, title, result, error):
        if error is not None:
            self.signals.service_result.emit(title, False, "", error)
            return
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))
        self.signals.service_result.emit(title, success, message, "")

    def _service_result(self, title, success, message, error):
        if error:
            self.append_log(f"[ERROR] {title}: {error}")
            if title == "Region Mode":
                self._revert_region_mode()
            QMessageBox.warning(self, title, error)
            return
        prefix = "[MISSION]" if success else "[WARN]"
        self.append_log(f"{prefix} {title}: {message}")
        self.mission_label.setText(f"任务: {message or title}")
        if title == "Region Mode" and not success:
            self._revert_region_mode()
        if success:
            QMessageBox.information(self, title, message or "Done")
        else:
            QMessageBox.warning(self, title, message or "Failed")

    def _revert_region_mode(self):
        self.region_mode_check.blockSignals(True)
        self.region_mode_check.setChecked(not self.region_mode_check.isChecked())
        self.region_mode_check.blockSignals(False)

    def drive_forward(self):
        self.ros.publish_cmd_vel(self.linear_spin.value(), 0.0)

    def drive_backward(self):
        self.ros.publish_cmd_vel(-self.linear_spin.value(), 0.0)

    def turn_left(self):
        self.ros.publish_cmd_vel(0.0, self.angular_spin.value())

    def turn_right(self):
        self.ros.publish_cmd_vel(0.0, -self.angular_spin.value())

    def forward_left(self):
        self.ros.publish_cmd_vel(self.linear_spin.value(), self.angular_spin.value() * 0.5)

    def forward_right(self):
        self.ros.publish_cmd_vel(self.linear_spin.value(), -self.angular_spin.value() * 0.5)

    def stop_robot(self):
        self.ros.publish_cmd_vel(0.0, 0.0)

    def keyPressEvent(self, event):
        key = event.key()
        if key in (Qt.Key_W, Qt.Key_Up):
            self.drive_forward()
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.drive_backward()
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.turn_left()
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.turn_right()
        elif key == Qt.Key_Space:
            self.stop_robot()
        else:
            super().keyPressEvent(event)

    def append_log(self, line):
        if not line:
            return
        self.log_view.appendPlainText(line)
        bar = self.log_view.verticalScrollBar()
        bar.setValue(bar.maximum())
        if line.startswith("/") and line.endswith(".yaml"):
            self._add_map_choice(line)
        if "[OK] workspace exists" in line:
            self.workspace_label.setText("远端工作空间: 已找到")
        elif "[WARN] workspace missing" in line:
            self.workspace_label.setText("远端工作空间: 缺失")
        if line.startswith("[REMOTE] host="):
            self.connection_label.setText("连接: 可用")
        elif "Permission denied" in line or "Could not resolve" in line or "Connection timed out" in line:
            self.connection_label.setText("连接: 失败")

    def _nav_result(self, line):
        self.append_log(line)
        self.nav_label.setText("导航: " + line)

    def _launch_state_changed(self, state):
        self.launch_label.setText(f"远端启动: {state}")

    def _rviz_state_changed(self, state):
        self.rviz_label.setText(f"本机 RViz: {state}")

    def _refresh_status(self):
        snap = self.ros.snapshot()
        now = time.monotonic()
        self._record_pose(snap["x"], snap["y"])
        self._set_topic("scan", now - snap["last_scan"], f"scan {snap['scan_count']}")
        self._set_topic("imu", now - snap["last_imu"], f"imu {snap['imu_frame'] or '--'}")
        self._set_topic("laser_odom", now - snap["last_laser_odom"], "laser odom")
        self._set_topic("odom", now - snap["last_odom"], "odom")
        self._set_topic("map", now - snap["last_map"], "map")
        self._set_topic("amcl", now - snap["last_amcl"], "amcl")
        self._set_topic("thermal", now - snap["last_thermal"], "thermal")
        self._set_topic("gas", now - snap["last_gas"], "gas")
        self._set_topic("safety", now - snap["last_safety"], snap["safety_level"])

        self.pose_label.setText(
            f"里程计: x={snap['x']:.3f}  y={snap['y']:.3f}  yaw={math.degrees(snap['yaw']):.1f} deg"
        )
        self.velocity_label.setText(
            f"速度: 线={snap['vx']:.3f} m/s  角={snap['wz']:.3f} rad/s"
        )
        self.amcl_label.setText(
            f"AMCL: x={snap['amcl_x']:.3f}  y={snap['amcl_y']:.3f}  yaw={math.degrees(snap['amcl_yaw']):.1f} deg"
        )
        if snap["map_width"] > 0:
            self.map_label.setText(
                f"地图: {snap['map_width']} x {snap['map_height']} @ {snap['map_resolution']:.3f} m/cell"
            )
        else:
            self.map_label.setText("地图: 等待数据")
        distance = snap["nav_distance_remaining"]
        if distance is None:
            self.nav_label.setText(f"导航: {snap['nav_status']}")
        else:
            self.nav_label.setText(f"导航: {snap['nav_status']}  剩余={distance:.2f} m")
        if snap["safety_mission_active"]:
            self.mission_label.setText("任务: running from RViz")
        elif self.region_mode_check.isChecked():
            self.mission_label.setText("任务: region mode")
        else:
            self.mission_label.setText("任务: RViz point mode")
        self.safety_label.setText(
            f"安全: {snap['safety_level']} {snap['safety_code']} {snap['safety_message']}"
        )
        if snap["safety_voltage_available"]:
            voltage = f"{snap['safety_measured_voltage_v']:.3f}V"
        else:
            voltage = "n/a"
        self.power_label.setText(
            f"电源: V={voltage} uv_now={int(snap['safety_undervoltage_now'])} "
            f"uv_seen={int(snap['safety_undervoltage_seen'])} "
            f"throttled=0x{snap['safety_throttled_flags']:x}"
        )
        self.thermal_label.setText(
            f"热成像: min={snap['thermal_min']:.1f}C max={snap['thermal_max']:.1f}C avg={snap['thermal_avg']:.1f}C"
        )
        self.gas_label.setText(
            f"气体: H2={snap['gas_h2']:.1f} CO={snap['gas_co']:.1f} "
            f"VOC={snap['gas_voc']:.1f} Smoke={snap['gas_smoke']:.1f}"
        )
        self.map_view.set_state(
            snap.get("map_msg"),
            snap["x"],
            snap["y"],
            snap["yaw"],
            self.pose_history,
        )

    def _record_pose(self, x, y):
        point = (round(float(x), 2), round(float(y), 2))
        if self.pose_history and self.pose_history[-1] == point:
            return
        self.pose_history.append(point)
        if len(self.pose_history) > 160:
            self.pose_history.pop(0)

    def _set_topic(self, name, age, text):
        label = self.topic_labels[name]
        label.setText(text)
        if age < 1.5:
            label.setStyleSheet("background: #237a57; color: white; border-radius: 4px;")
        elif age < 5.0:
            label.setStyleSheet("background: #b88728; color: white; border-radius: 4px;")
        else:
            label.setStyleSheet("background: #7c8588; color: white; border-radius: 4px;")

    def _map_text(self):
        return self.map_combo.currentText().strip()

    def _set_map_text(self, path):
        if not path:
            path = DEFAULT_REMOTE_MAP
        index = self.map_combo.findText(path)
        if index < 0:
            self.map_combo.addItem(path)
            index = self.map_combo.findText(path)
        self.map_combo.setCurrentIndex(index)

    def _add_map_choice(self, path):
        if self.map_combo.findText(path) < 0:
            self.map_combo.addItem(path)
        if self._map_text() in ("", DEFAULT_REMOTE_MAP):
            self._set_map_text(path)

    def _normalize_map_path(self, path):
        maps_dir = os.path.join(
            self.remote_workspace_edit.text().strip() or DEFAULT_REMOTE_WORKSPACE,
            "maps",
        )
        if not path:
            return os.path.join(maps_dir, "inspection_map.yaml")
        if os.path.basename(path) == path:
            return os.path.join(maps_dir, path)
        return path

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
        self.ros.shutdown()
        event.accept()


def run_app(defaults=None):
    app = QApplication.instance() or QApplication([])
    window = RemoteRobotWindow(defaults)
    window.show()
    return app.exec_()
