import argparse
import sys

from PyQt5.QtWidgets import QApplication

from .remote_robot_window import (
    DEFAULT_LOCAL_WORKSPACE,
    DEFAULT_REMOTE_HOST,
    DEFAULT_REMOTE_MAP,
    DEFAULT_REMOTE_PORT,
    DEFAULT_REMOTE_USER,
    DEFAULT_REMOTE_WORKSPACE,
    DEFAULT_RVIZ_CONFIG,
    RemoteRobotWindow,
)
from .config import DEFAULT_ROS_SETUP_PATH


def main():
    parser = argparse.ArgumentParser(description="Local LAN UI for starting the remote inspection robot.")
    parser.add_argument("--host", default=DEFAULT_REMOTE_HOST)
    parser.add_argument("--user", default=DEFAULT_REMOTE_USER)
    parser.add_argument("--port", default=DEFAULT_REMOTE_PORT)
    parser.add_argument("--workspace", default=DEFAULT_REMOTE_WORKSPACE)
    parser.add_argument("--local-workspace", default=DEFAULT_LOCAL_WORKSPACE)
    parser.add_argument("--ros-setup", default=DEFAULT_ROS_SETUP_PATH)
    parser.add_argument("--map", default=DEFAULT_REMOTE_MAP)
    parser.add_argument("--rviz-config", default=DEFAULT_RVIZ_CONFIG)
    args, _unknown = parser.parse_known_args()

    app = QApplication(sys.argv)
    window = RemoteRobotWindow(vars(args))
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
