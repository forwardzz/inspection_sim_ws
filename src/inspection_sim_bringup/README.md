# Inspection Simulation Bringup

This workspace is a standalone ROS 2 Jazzy simulation for the tracked inspection robot.
Gazebo publishes `/scan`, `/sim/imu/data_raw`, `/wheel_odom`, `/clock`, and accepts `/cmd_vel`.
`sim_imu_adapter` republishes the Gazebo IMU data as `/imu/data_raw`.
The ROS odometry chain is `rf2o_laser_odometry + /wheel_odom + /imu/data_raw -> robot_localization EKF -> /odom`.

## Build

```bash
cd ~/inspection_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Mapping

```bash
ros2 launch inspection_sim_bringup mapping.launch.py
ros2 launch inspection_sim_bringup teleop.launch.py
ros2 run nav2_map_server map_saver_cli -f "$HOME/inspection_sim_ws/maps/inspection_map" -t /map
```

## Navigation

```bash
ros2 launch inspection_sim_bringup navigation.launch.py map:="$HOME/inspection_sim_ws/maps/inspection_map.yaml"
```
