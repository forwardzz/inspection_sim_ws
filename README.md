# Inspection Simulation Workspace

这是一个 ROS 2 Jazzy 仿真工作区，用于履带式巡检小车的 Gazebo 仿真、建图、定位导航和巡检任务验证。当前项目只保留仿真链路，不包含真实雷达、真实 IMU、GPIO 电机或远程实机控制代码。

## 功能

- Gazebo Sim 中加载履带小车、激光雷达、IMU 和轮速/编码器里程计。
- 通过 `ros_gz_bridge` 桥接 `/scan`、`/sim/imu/data_raw`、`/wheel_odom`、`/clock` 和 `/cmd_vel`。
- 使用 `rf2o_laser_odometry` 和 `robot_localization` EKF 融合 `/laser_odom`、`/wheel_odom`、`/imu/data_raw` 生成 `/odom`。
- 使用 `slam_toolbox` 建图，使用 Nav2 进行 AMCL 定位和路径导航。
- 提供 PyQt5 本机控制面板，支持启动仿真、建图、导航、保存地图、手动遥控、可调停留时间、巡检点/区域任务和主机状态监控。

## 实现路径

- `src/inspection_sim_bringup`：Gazebo、SLAM、Nav2 的 launch、配置、URDF、SDF、world、RViz 和行为树。
- `src/inspection_sim_mission`：仿真 IMU 适配、巡检任务管理、区域点生成和任务服务。
- `src/inspection_sim_gui`：本机 PyQt5 控制面板。
- `src/rf2o_laser_odometry`：基于激光扫描的 2D 里程计。
- `src/robot_mission_utils`：巡检路径规划工具。
- `src/robot_monitor_interfaces`：巡检任务消息和服务接口。

## 快速启动

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjy/inspection_sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_sim_bringup sim.launch.py
```

更多启动方式见 [start.md](start.md)。
