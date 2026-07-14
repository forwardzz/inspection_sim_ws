from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("inspection_sim_bringup")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world_name = LaunchConfiguration("world_name")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    regions = LaunchConfiguration("regions")

    sim_launch = PathJoinSubstitution(
        [pkg_share, "launch", "sim.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "inspection_world.sdf"]
    )
    nav2_params = PathJoinSubstitution(
        [pkg_share, "config", "nav2_params.yaml"]
    )
    ekf_config = PathJoinSubstitution(
        [pkg_share, "config", "ekf.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", "inspection_sim.rviz"]
    )
    nav_to_pose_bt = PathJoinSubstitution(
        [pkg_share, "behavior_trees", "navigate_replan_if_path_invalid.xml"]
    )
    nav_through_poses_bt = PathJoinSubstitution(
        [pkg_share, "behavior_trees", "navigate_through_poses_replan_if_invalid.xml"]
    )
    default_map = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "inspection_sim_ws", "maps", "inspection_map.yaml"]
    )
    default_regions = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "inspection_sim_ws", "maps", "inspection_regions.yaml"]
    )

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "smoother_server",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to a saved Nav2 map YAML",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Gazebo Sim world file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz for navigation",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo Sim server without the GUI",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="inspection_world",
            description="Gazebo world name used for model spawning",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Robot spawn X position in Gazebo world coordinates",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Robot spawn Y position in Gazebo world coordinates",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.05",
            description="Robot spawn Z position in Gazebo world coordinates",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Robot spawn yaw in Gazebo world coordinates (radians)",
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="0.0",
            description="AMCL initial pose X in map frame",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="0.0",
            description="AMCL initial pose Y in map frame",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="0.0",
            description="AMCL initial pose yaw in radians",
        ),
        DeclareLaunchArgument(
            "regions",
            default_value=default_regions,
            description="Inspection regions YAML file path",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "world": world,
                "use_sim_time": use_sim_time,
                "use_rviz": use_rviz,
                "headless": headless,
                "world_name": world_name,
                "rviz_config": rviz_config,
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_z": spawn_z,
                "spawn_yaw": spawn_yaw,
            }.items(),
        ),

        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            arguments=["--ros-args", "--log-level", "error"],
            parameters=[{
                "use_sim_time": use_sim_time,
                "laser_scan_topic": "/scan",
                "odom_topic": "/laser_odom",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            }],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "/odom")],
        ),

        Node(
            package="inspection_sim_mission",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "inspection_regions_path": regions,
                "mission_home_x": initial_pose_x,
                "mission_home_y": initial_pose_y,
                "mission_home_yaw": initial_pose_yaw,
            }],
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[nav2_params, {"yaml_filename": map_file}, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_params, {
                "use_sim_time": use_sim_time,
                "initial_pose.x": initial_pose_x,
                "initial_pose.y": initial_pose_y,
                "initial_pose.yaw": initial_pose_yaw,
            }],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_params,
                {"use_sim_time": use_sim_time},
                {"default_nav_to_pose_bt_xml": nav_to_pose_bt},
                {"default_nav_through_poses_bt_xml": nav_through_poses_bt},
            ],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/cmd_vel", "/cmd_vel_nav"),
                ("/cmd_vel_smoothed", "/cmd_vel"),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "bond_timeout": 6.0,
                        "node_names": lifecycle_nodes,
                    }],
                ),
            ],
        ),
    ])
