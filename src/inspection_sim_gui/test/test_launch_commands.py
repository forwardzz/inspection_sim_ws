from inspection_sim_gui.launch_manager import (
    build_mapping_command,
    build_navigation_command,
    build_sim_command,
)


def test_build_sim_command_without_dynamic_obstacles():
    command = build_sim_command(
        use_rviz=False,
        headless=True,
        world="/tmp/some world.sdf",
        world_name="w",
        spawn_x=1.0,
        spawn_y=2.0,
        spawn_z=0.05,
        spawn_yaw=0.3,
    )
    assert command == (
        "ros2 launch inspection_sim_bringup sim.launch.py "
        "use_rviz:=false headless:=true world:='/tmp/some world.sdf' "
        "world_name:=w spawn_x:=1.0 spawn_y:=2.0 spawn_z:=0.05 spawn_yaw:=0.3"
    )


def test_build_sim_command_passes_dynamic_obstacle_fields():
    command = build_sim_command(
        dynamic_obstacles_config="/tmp/dynamic_obstacles_complex_inspection.yaml",
        dynamic_obstacle_seed=42,
    )
    assert "dynamic_obstacles_config:=/tmp/dynamic_obstacles_complex_inspection.yaml" in command
    assert "dynamic_obstacle_seed:=42" in command


def test_build_sim_command_dynamic_config_without_seed():
    command = build_sim_command(dynamic_obstacles_config="/tmp/obstacles.yaml")
    assert "dynamic_obstacles_config:=/tmp/obstacles.yaml" in command
    assert "dynamic_obstacle_seed" not in command


def test_build_mapping_command_passes_dynamic_obstacle_fields():
    command = build_mapping_command(
        headless=True,
        dynamic_obstacles_config="/tmp/obstacles.yaml",
        dynamic_obstacle_seed=-1,
    )
    assert command.startswith(
        "ros2 launch inspection_sim_bringup mapping.launch.py "
        "use_rviz:=true headless:=true"
    )
    assert "dynamic_obstacles_config:=/tmp/obstacles.yaml" in command
    assert "dynamic_obstacle_seed:=-1" in command


def test_build_commands_without_dynamic_fields_keep_legacy_format():
    for command in (build_sim_command(), build_mapping_command()):
        assert "dynamic_obstacles_config" not in command
        assert "dynamic_obstacle_seed" not in command


def test_build_navigation_command_passes_dynamic_obstacle_fields():
    command = build_navigation_command(
        map_path="/tmp/map.yaml",
        use_rviz=False,
        headless=True,
        world="/tmp/world.sdf",
        world_name="w",
        spawn_x=0.0,
        spawn_y=0.0,
        spawn_z=0.05,
        spawn_yaw=0.0,
        initial_pose_x=1.0,
        initial_pose_y=2.0,
        initial_pose_yaw=0.3,
        dynamic_obstacles_config="/tmp/dynamic_obstacles_complex_inspection.yaml",
        dynamic_obstacle_seed=42,
    )
    assert command.startswith(
        "ros2 launch inspection_sim_bringup navigation.launch.py "
        "map:=/tmp/map.yaml use_rviz:=false headless:=true"
    )
    assert "dynamic_obstacles_config:=/tmp/dynamic_obstacles_complex_inspection.yaml" in command
    assert "dynamic_obstacle_seed:=42" in command


def test_build_navigation_command_without_dynamic_config_omits_fields():
    command = build_navigation_command(
        map_path="/tmp/map.yaml",
        use_rviz=False,
        headless=True,
        initial_pose_x=0.0,
        initial_pose_y=0.0,
        initial_pose_yaw=0.0,
    )
    assert "dynamic_obstacles_config" not in command
    assert "dynamic_obstacle_seed" not in command


def test_build_navigation_command_config_without_seed():
    command = build_navigation_command(
        map_path="/tmp/map.yaml",
        use_rviz=False,
        headless=True,
        initial_pose_x=0.0,
        initial_pose_y=0.0,
        initial_pose_yaw=0.0,
        dynamic_obstacles_config="/tmp/obstacles.yaml",
    )
    assert "dynamic_obstacles_config:=/tmp/obstacles.yaml" in command
    assert "dynamic_obstacle_seed" not in command


def test_build_navigation_command_quotes_config_path_with_spaces():
    command = build_navigation_command(
        map_path="/tmp/map.yaml",
        use_rviz=False,
        headless=True,
        initial_pose_x=0.0,
        initial_pose_y=0.0,
        initial_pose_yaw=0.0,
        dynamic_obstacles_config="/tmp/obstacle config.yaml",
        dynamic_obstacle_seed=7,
    )
    assert "dynamic_obstacles_config:='/tmp/obstacle config.yaml'" in command
