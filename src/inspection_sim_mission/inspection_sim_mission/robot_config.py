import os


FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE_LINK = "base_link"
FRAME_LASER = "laser"
FRAME_IMU = "imu_link"

TOPIC_SCAN = "/scan"
TOPIC_ODOM = "/odom"
TOPIC_LASER_ODOM = "/laser_odom"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_CMD_VEL_NAV = "/cmd_vel_nav"
TOPIC_MAP = "/map"
TOPIC_AMCL_POSE = "/amcl_pose"
TOPIC_CLICKED_POINT = "/clicked_point"
TOPIC_MISSION_GOAL_POSE = "/mission_goal_pose"
TOPIC_GOAL_POSE = "/goal_pose"
TOPIC_MISSION_PREVIEW_PATH = "/mission_preview_path"
TOPIC_MISSION_POINTS_MARKERS = "/mission_points_markers"
TOPIC_ROBOT_SAFETY_STATUS = "/robot_safety_status"
TOPIC_THERMAL_FRAME = "/thermal_frame"
TOPIC_GAS_DATA = "/gas_data"
TOPIC_IMU_RAW = "/imu/data_raw"
TOPIC_SIM_IMU_RAW = "/sim/imu/data_raw"

ACTION_NAVIGATE_TO_POSE = "/navigate_to_pose"
ACTION_NAVIGATE_THROUGH_POSES = "/navigate_through_poses"

SERVICE_LOCALIZE_ROBOT = "/localize_robot"
SERVICE_CONFIRM_INSPECTION_POINTS = "/confirm_inspection_points"
SERVICE_START_NAVIGATION = "/start_navigation"
SERVICE_CLEAR_RVIZ_POINTS = "/clear_rviz_points"
SERVICE_SET_REGION_MODE = "/set_region_mode"
SERVICE_CLEAR_INSPECTION_REGIONS = "/clear_inspection_regions"
SERVICE_SAVE_INSPECTION_REGIONS = "/save_inspection_regions"
SERVICE_LOAD_INSPECTION_REGIONS = "/load_inspection_regions"
SERVICE_ABORT_MISSION = "/abort_mission"
SERVICE_RESET_SAFETY_MONITOR = "/reset_safety_monitor"

LIDAR_SERIAL_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"
LIDAR_BAUDRATE = "115200"
LIDAR_Z = "0.10"
LIDAR_YAW = "3.1415926"

IMU_SERIAL_PORT = "/dev/ttyAMA0"
IMU_X = "-0.125"
IMU_Y = "-0.031"
IMU_Z = "0.040"

INSPECTION_REGIONS_PATH = os.path.join(
    os.path.expanduser("~/inspection_sim_ws"),
    "maps",
    "inspection_regions.yaml",
)

MAX_LINEAR_SPEED_MPS = 0.18
MAX_ANGULAR_SPEED_RADPS = 0.55
