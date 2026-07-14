# Inspection Simulation Workspace

这是一个 ROS 2 Jazzy 纯仿真工作区，用于履带式巡检小车的 Gazebo 仿真、建图、定位导航和区域巡检验证。项目不包含真实雷达、真实 IMU、GPIO 电机或远程实机控制代码。

## 主要功能

- Gazebo Sim 中的履带小车、2D 激光雷达、IMU 和轮速里程计。
- `rf2o_laser_odometry`、轮速和 IMU 经 EKF 融合生成 `/odom`。
- slam_toolbox 建图，AMCL 定位，SmacPlanner2D 全局规划。
- 自研 `inspection_sim_dwa_controller::DWAController` 局部控制器。
- 速度链路：`/cmd_vel_nav` → Nav2 velocity smoother → `/cmd_vel`。
- PyQt5 控制面板，支持场景选择、启动仿真/建图/导航、地图保存、初始位姿、手动驾驶和巡检任务。
- 巡检点与多区域任务，包含 RViz 预览、区域入口检查、障碍检测和传感器失效安全停车。

## RViz 多点导航排序

RViz `Publish Point` 的点击顺序只用于记录点位和生成 `RVIZ_n` 名称。系统以任务启动时最新的 AMCL 位姿为固定起点，按地图可通行路径代价生成开放式 TSP 路线；数字标记、路径预览和实际逐点派发均使用优化后的顺序，各点原有朝向保持不变。

- 1 个点直接导航；2–10 个点使用精确求解；超过 10 个点使用最近邻初始解和 2-opt。
- 排序代价包含地图最短路径长度与转向惩罚。膨胀地图无完整路线时会尝试基础地图，仍不可达则拒绝任务，不按点击顺序兜底。
- 点位或地图变化时更新预览；AMCL 微小更新不会反复求解，点击“开始任务”时会基于最新位置做最终重算。
- TSP 路径搜索在后台单工作线程执行；启动服务会先确认规划请求并保持停车，只有当前地图和点位对应的最新结果才能派发任务。取消、清空、撤销或地图变化都会废弃迟到结果。
- `/start_navigation` 显式传入的点、已确认点和区域巡检点继续保持调用方给定顺序。

## 任务完成后返回起点

GUI 的“任务完成后返回起点”选项适用于 RViz 点、确认点、外部请求点和区域任务。启用后，系统只在巡检正常完成时再发送一个独立的 Nav2 返航目标；任务失败、区域阻塞或用户取消时保持停车，不执行返航。返航不参与 TSP 排序、点位编号或点位停留。

固定起点使用当前场景的 `initial_pose_x/y/yaw`（`map` 坐标系），不会在任务启动时重新采集 AMCL 位姿。Gazebo 的 `spawn_*` 属于世界坐标，仅用于生成模型，不能直接作为 Nav2 目标。

## 多区域巡检排序

多个巡检区域按地图可通行代价自动排序，不再由区域录入或 YAML 顺序决定。系统会同时选择每个区域的正向或反向扫掠入口：1 个区域直接选择入口，2–10 个区域精确求解，超过 10 个区域使用最近邻加 2-opt。勾选返航时，最后区域到固定起点的路径也计入排序代价。

区域 TSP 使用后台单工作线程。区域或地图变化时更新开放路线预览；点击“开始任务”后使用最新 AMCL 位姿和本次返航选项最终重算。RViz 数字表示本次动态访问顺序，原始 `REGION_n` 名称和区域 YAML 保存顺序保持不变。无法形成完整可通行路线时拒绝任务，不回退到录入顺序。

## 工作区结构

- `src/inspection_sim_bringup`：launch、Nav2/SLAM/EKF 配置、URDF/SDF、world、RViz、场景目录和行为树。
- `src/inspection_sim_dwa_controller`：Nav2 DWA 控制器插件。
- `src/inspection_sim_mission`：任务管理、区域覆盖控制和仿真 IMU 适配。
- `src/inspection_sim_gui`：PyQt5 控制面板。
- `src/robot_mission_utils`：地图、路径搜索、平滑和任务排序工具。
- `src/robot_monitor_interfaces`：自定义消息与服务。
- `src/rf2o_laser_odometry`：第三方激光里程计。
- `maps/`：场景地图和巡检区域配置。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjy/inspection_sim_ws
colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --verbose
```

不要直接修改 `build/`、`install/` 或 `log/`。

## 快速启动

启动 GUI 控制台：

```bash
./colcon_start_gui.sh
```

或在已经构建并加载环境后手动启动：

```bash
ros2 launch inspection_sim_gui gui.launch.py
```

只启动基础仿真：

```bash
ros2 launch inspection_sim_bringup sim.launch.py
```

无界面导航快速检查：

```bash
ros2 launch inspection_sim_bringup navigation.launch.py \
  map:="$PWD/maps/inspection_map.yaml" \
  regions:="$PWD/maps/inspection_regions.yaml" \
  use_rviz:=false headless:=true
```

完整命令、场景参数和验证方法见 [start.md](start.md)。

## 运行安全

Nav2 使用显式的 `progress_checker` 监测导航进展：20 秒内未移动至少 0.10 m 时触发路径跟踪失败，行为树依次尝试清理代价地图、旋转、等待和后退恢复。目标到达仍只检查位置，不强制最终朝向。

区域巡检进入自主底盘覆盖阶段后，任务管理器持续检查 `/scan` 和 `map → base_link` TF。任一数据失效时立即向 `/cmd_vel_nav` 发布零速度；连续 5 次失败后跳过当前区域并通过 `/mission_status` 上报安全事件。检测到动态障碍时也会立即停车，最多观察 3 秒；通道连续 10 帧安全后重试当前覆盖目标，每个目标最多恢复两次，持续阻塞才跳过区域。区域入口的 Nav2 恢复最终失败时会把该区域标记为阻塞并继续剩余 TSP 区域；任务最终报告部分完成且不执行返航。

关闭 GUI 或按 `Ctrl+C` 会停止该 GUI 启动的子 launch 进程并关闭 ROS executor，不会主动终止其他终端中的 ROS 进程。
