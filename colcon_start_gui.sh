#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

exec ros2 launch inspection_sim_gui gui.launch.py
/