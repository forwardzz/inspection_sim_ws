# 启动仿真

## 环境设置

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zjy/inspection_sim_ws
source install/setup.bash
```

## 启动仿真

### 基础仿真（带 GUI）

```bash
ros2 launch inspection_sim_bringup sim.launch.py
```

### 无头模式（无 GUI）

```bash
ros2 launch inspection_sim_bringup sim.launch.py headless:=true
```

### 带 RViz 可视化

```bash
ros2 launch inspection_sim_bringup sim.launch.py use_rviz:=true
```

### 无头 + RViz（推荐用于调试）

```bash
ros2 launch inspection_sim_bringup sim.launch.py headless:=true use_rviz:=true
```

## 启动 Qt GUI 控制面板

```bash
ros2 launch inspection_sim_gui gui.launch.py
```

## 验证

检查仿真是否正常运行：

```bash
# 查看话题列表
ros2 topic list

# 查看 /scan 话题（激光雷达数据）
ros2 topic hz /scan

# 查看 /cmd_vel 话题（速度指令）
ros2 topic echo /cmd_vel --once
```
