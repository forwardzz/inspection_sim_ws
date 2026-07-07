# AGENTS.md

## Repo overview

ROS 2 Jazzy simulation-only workspace (`/home/zjy/inspection_sim_ws`) for a tracked inspection robot. Packages under `src/`:

- **`inspection_sim_bringup/`** — ament_cmake package for Gazebo Sim, SLAM, EKF, Nav2. All assets (launch, config, urdf, models, worlds, rviz, behavior_trees).
- **`inspection_sim_gui/`** — ament_python PyQt5 control panel. Entrypoint: `inspection_sim_gui/main.py:main` → console script `inspection_sim_gui`.
- **`inspection_sim_mission/`** — ament_python mission, region, and simulation sensor helper nodes.
- **`rf2o_laser_odometry/`** — laser odometry (third-party, do not edit).
- **`robot_mission_utils/`** — pure Python mission planning utilities.
- **`robot_monitor_interfaces/`** — custom mission message/service interfaces.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Never hand-edit `install/` or `build/`.

## Launch hierarchy

| Command | What it starts |
|---------|---------------|
| `sim.launch.py` | Gazebo Sim + RSP + ros_gz_bridge. Optional `headless:=true` `use_rviz:=true`. |
| `mapping.launch.py` | Includes sim + rf2o_laser_odometry + EKF + slam_toolbox. |
| `navigation.launch.py map:=…` | Includes sim + rf2o_laser_odometry + EKF + Nav2 (AMCL, planner, controller, BT, lifecycle manager). |
| `teleop.launch.py` | Keyboard teleop (`teleop_twist_keyboard`). |
| `gui.launch.py` | PyQt5 GUI. Parameters: `workspace`, `map`, `ros_setup`. |

Quick-check without GUI: `navigation.launch.py use_rviz:=false headless:=true`.

## Odometry chain

`/scan` → `rf2o_laser_odometry` → `/laser_odom` → `robot_localization::ekf` (+ `/imu/data_raw`) → `/odom`

The EKF uses two inputs: `/laser_odom` (x, y, yaw, vx, vy, wz) and `/imu/data_raw` (yaw only). Config: `config/ekf.yaml`.

## Gazebo -> ROS bridge

Topics bridged via `ros_gz_bridge` parameter_bridge: `/clock`, `/cmd_vel`, `/scan`, `/sim/imu/data_raw`.
`inspection_sim_mission/sim_imu_adapter` republishes `/sim/imu/data_raw` as `/imu/data_raw`.

## Nav2 specifics

- **Planner**: SmacPlanner2D. **Controller**: RotationShimController → DWBLocalPlanner.
- **Custom BT**: `behavior_trees/navigate_replan_if_path_invalid.xml` — replans only when path invalidates.
- **Velocity smoothing**: `controller_server` publishes to `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel`.
- **Lifecycle nodes**: map_server, amcl, planner_server, controller_server, bt_navigator, behavior_server, smoother_server, velocity_smoother.
- **Map**: saved to `maps/inspection_map.yaml` (trinary, 0.03m/cell). Save: `ros2 run nav2_map_server map_saver_cli -f maps/inspection_map -t /map`.

## GUI internals

The GUI (`gui.launch.py`) spawns a node that reads `workspace_path`, `map_path`, `ros_setup_path` parameters. It runs launches as child processes via `QProcess` using `setsid bash -lc '<source ros && source install && command>'` — never use nested `ros2 launch` calls manually inside the GUI's workspace.

Manual drive via `/cmd_vel` (keyboard or WASD buttons). Nav2 goal via NavigateToPose action server. Initial pose via `/initialpose`.

## Config files

- `config/ekf.yaml` — EKF fusion of laser_odom + IMU
- `config/slam.yaml` — slam_toolbox mapping mode params
- `config/nav2_params.yaml` — all Nav2 nodes (AMCL, controller, costmaps, BT, smoother, behaviors, waypoint_follower)

## Testing

No test framework. Validate with `colcon build --symlink-install` and short-run the affected launch file. Use headless mode when possible.
