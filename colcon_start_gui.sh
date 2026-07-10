#!/usr/bin/env bash
cd "$(dirname "$0")"

killros

colcon build

source install/setup.bash

ros2 launch inspection_sim_gui gui.launch.py