import os


DEFAULT_WORKSPACE_PATH = os.path.expanduser("~/inspection_sim_ws")
DEFAULT_ROS_SETUP_PATH = "/opt/ros/jazzy/setup.bash"
DEFAULT_MAP_PATH = os.path.join(DEFAULT_WORKSPACE_PATH, "maps", "inspection_map.yaml")
DEFAULT_LIDAR_SERIAL_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"
DEFAULT_LIDAR_BAUDRATE = "115200"
DEFAULT_IMU_SERIAL_PORT = "/dev/ttyAMA0"

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_INITIAL_POSE = "/initialpose"
TOPIC_GOAL_POSE = "/goal_pose"
TOPIC_ODOM = "/odom"
TOPIC_LASER_ODOM = "/laser_odom"
TOPIC_SCAN = "/scan"
TOPIC_IMU_RAW = "/imu/data_raw"
TOPIC_MAP = "/map"
TOPIC_AMCL_POSE = "/amcl_pose"
TOPIC_CLICKED_POINT = "/clicked_point"
TOPIC_MISSION_GOAL_POSE = "/mission_goal_pose"
TOPIC_MISSION_PREVIEW_PATH = "/mission_preview_path"
TOPIC_MISSION_POINTS_MARKERS = "/mission_points_markers"

ACTION_NAVIGATE_TO_POSE = "/navigate_to_pose"

SERVICE_LOCALIZE_ROBOT = "/localize_robot"
SERVICE_START_NAVIGATION = "/start_navigation"
SERVICE_CLEAR_RVIZ_POINTS = "/clear_rviz_points"
SERVICE_SET_REGION_MODE = "/set_region_mode"
SERVICE_CLEAR_INSPECTION_REGIONS = "/clear_inspection_regions"
SERVICE_SAVE_INSPECTION_REGIONS = "/save_inspection_regions"
SERVICE_LOAD_INSPECTION_REGIONS = "/load_inspection_regions"

MAX_LINEAR_SPEED_MPS = 0.18
MAX_ANGULAR_SPEED_RADPS = 0.55
DEFAULT_LINEAR_SPEED_MPS = 0.10
DEFAULT_ANGULAR_SPEED_RADPS = 0.35
