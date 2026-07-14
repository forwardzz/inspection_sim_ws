# AGENTS.md

## Repo overview

ROS 2 Jazzy simulation-only workspace (`/home/zjy/inspection_sim_ws`) for a tracked inspection robot. Packages under `src/`:

- **`inspection_sim_bringup/`** — Gazebo Sim, SLAM, EKF and Nav2 launch/config/assets.
- **`inspection_sim_dwa_controller/`** — custom Nav2 DWA controller plugin (`inspection_sim_dwa_controller::DWAController`).
- **`inspection_sim_gui/`** — PyQt5 control panel; console script `inspection_sim_gui`.
- **`inspection_sim_mission/`** — mission, region and simulation sensor helper nodes.
- **`rf2o_laser_odometry/`** — third-party laser odometry; do not edit.
- **`robot_mission_utils/`** — pure Python mission planning utilities.
- **`robot_monitor_interfaces/`** — custom mission messages and services.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjy/inspection_sim_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Never hand-edit `install/`, `build/` or `log/`. Use `colcon_start_gui.sh` for the build-and-launch shortcut; it does not terminate unrelated ROS processes.

## Launch hierarchy

| Command | What it starts |
|---------|----------------|
| `sim.launch.py` | Gazebo Sim, robot state publisher, bridges and IMU adapter. Optional `headless:=true` and `use_rviz:=true`. |
| `mapping.launch.py` | Simulation, rf2o laser odometry, EKF and slam_toolbox. |
| `navigation.launch.py map:=… regions:=…` | Simulation, odometry, mission manager and the complete Nav2 stack. |
| `teleop.launch.py` | Keyboard teleoperation. |
| `gui.launch.py` | PyQt5 GUI. Arguments: `workspace`, `map`, `ros_setup`, `scene_catalog`. |

Quick runtime check:

```bash
ros2 launch inspection_sim_bringup navigation.launch.py use_rviz:=false headless:=true
```

## Simulation and odometry

Gazebo topics bridged to ROS are `/clock`, `/cmd_vel`, `/scan`, `/sim/imu/data_raw` and `/wheel_odom`. `sim_imu_adapter` republishes the simulated IMU on `/imu/data_raw`.

Odometry chain:

`/scan` → `rf2o_laser_odometry` → `/laser_odom` → `robot_localization::ekf` (+ `/wheel_odom`, `/imu/data_raw`) → `/odom`

The robot model, Xacro, worlds and scene catalog live under `inspection_sim_bringup`. Scene-specific maps and region YAML files live in the workspace-level `maps/` directory.

## Nav2 and mission behavior

- Planner: `nav2_smac_planner::SmacPlanner2D`.
- Controller: custom `inspection_sim_dwa_controller::DWAController` registered as `FollowPath`.
- Velocity chain: `controller_server`/mission manager → `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel`.
- Custom behavior trees replan when the current path becomes invalid.
- Lifecycle nodes: map server, AMCL, planner, controller, BT navigator, behavior server, smoother and velocity smoother.
- Region missions use Nav2 to reach a staging point, then use the mission manager for low-speed coverage motion.
- RViz multi-point missions use an open TSP from the latest AMCL pose: exact for 2–10 points and map-cost nearest-neighbor plus 2-opt above 10. Marker numbers, preview paths and dispatched goals follow the optimized order; request, confirmed and region point order remains unchanged.
- RViz TSP path searches run on one background worker. `/start_navigation` only accepts the planning request and keeps the robot stopped; a generation-matched result starts the mission later. Point/map changes and abort operations must invalidate stale results.
- `/start_navigation` can request return-to-start for RViz, confirmed, external and region missions. The fixed home pose is the scene `initial_pose_*` in `map`, passed to the mission manager as `mission_home_*`; it is a separate final Nav2 phase and must run only after successful inspection. Failure, blocked-region and abort paths stop without returning.
- Multi-region missions jointly optimize region order and forward/reverse coverage direction using map-path cost. They use exact search for 2–10 regions and nearest-neighbor plus 2-opt above 10; return-home cost is included when requested. Planning runs on the shared background worker, while `REGION_n` names and YAML order remain unchanged.
- Nav2 uses the explicitly selected `progress_checker`; failure to move 0.10 m within 20 seconds enters the configured costmap-clear/spin/wait/backup recovery tree. Goal completion remains position-only.
- Dynamic obstacles during low-speed region coverage stop the robot and enter a bounded clearance wait. Ten consecutive clear frames resume the same target, with at most two recoveries per target; persistent blockage skips the region. A terminal Nav2 staging failure also marks that region blocked and continues with the remaining optimized regions.
- During region motion, stale laser data or unavailable `map → base_link` TF must publish zero velocity immediately; five consecutive failures skip the current region.

## GUI internals

The GUI launches child processes with `QProcess` using `setsid bash -lc`, sourcing both the ROS installation and workspace. Do not add nested manual launch shells inside the GUI.

Manual drive publishes `/cmd_vel`; mission start uses `/start_navigation`; initial pose uses `/initialpose`. GUI shutdown must stop its child launch group and shut down its ROS executor cleanly. Scene refresh must not add duplicate signal connections.

## Important configuration

- `config/ekf.yaml` — laser, wheel and IMU fusion.
- `config/slam.yaml` — slam_toolbox mapping configuration.
- `config/nav2_params.yaml` — Nav2, custom DWA and velocity smoother parameters.
- `scenes/scene_catalog.json` — selectable worlds, maps, region files and initial poses.
- `behavior_trees/` — navigation behavior trees.

## Validation expectations

For code changes, run the affected pytest tests plus the full `colcon build` and `colcon test`. For launch/config/controller changes, short-run the affected launch file in headless mode and verify that all Nav2 lifecycle nodes become active. A successful plan is not enough: confirm that commands reach `/cmd_vel_nav` and `/cmd_vel`, and that sensor/TF failures stop the robot.
