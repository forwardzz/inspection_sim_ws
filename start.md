# 启动与验证

## 首次构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjy/inspection_sim_ws
colcon build --symlink-install
source install/setup.bash
```

新终端必须重新执行前三行中的 `source`、`cd` 和工作区 `source`。

## GUI 控制台

构建后直接启动：

```bash
ros2 launch inspection_sim_gui gui.launch.py
```

也可以使用自动构建并启动的脚本：

```bash
./colcon_start_gui.sh
```

GUI 场景下拉框读取 `inspection_sim_bringup/scenes/scene_catalog.json`。当前包含默认巡检场景和新能源车辆运输船甲板场景，并自动选择对应的 world、地图、区域文件和初始位姿。

## 基础仿真

```bash
# Gazebo GUI
ros2 launch inspection_sim_bringup sim.launch.py

# 无界面
ros2 launch inspection_sim_bringup sim.launch.py headless:=true

# Gazebo + RViz
ros2 launch inspection_sim_bringup sim.launch.py use_rviz:=true
```

## 建图

```bash
ros2 launch inspection_sim_bringup mapping.launch.py
```

另一个已加载工作区环境的终端中启动键盘遥控：

```bash
ros2 launch inspection_sim_bringup teleop.launch.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/zjy/inspection_sim_ws/maps/inspection_map -t /map
```

## 定位导航

默认场景：

```bash
ros2 launch inspection_sim_bringup navigation.launch.py \
  map:=/home/zjy/inspection_sim_ws/maps/inspection_map.yaml \
  regions:=/home/zjy/inspection_sim_ws/maps/inspection_regions.yaml
```

车辆运输船甲板场景：

```bash
ros2 launch inspection_sim_bringup navigation.launch.py \
  world:=/home/zjy/inspection_sim_ws/src/inspection_sim_bringup/worlds/ev_car_carrier_deck.sdf \
  world_name:=ev_car_carrier_deck_world \
  map:=/home/zjy/inspection_sim_ws/maps/ev_car_carrier_deck_map.yaml \
  regions:=/home/zjy/inspection_sim_ws/maps/ev_car_carrier_deck_regions.yaml \
  spawn_x:=-13.2 spawn_z:=0.25
```

无界面快速检查：

```bash
ros2 launch inspection_sim_bringup navigation.launch.py \
  use_rviz:=false headless:=true
```

## 运行验证

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

完整自动测试：

```bash
colcon test
colcon test-result --verbose
```

使用 `Ctrl+C` 结束当前 launch。GUI 只清理自己启动的进程，不会执行全局 ROS 进程终止命令。
