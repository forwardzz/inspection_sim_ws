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
    sensor_source = LaunchConfiguration("sensor_source")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    imu_serial_port = LaunchConfiguration("imu_serial_port")

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
            "sensor_source",
            default_value="sim",
            description="Use sim Gazebo sensors or hardware serial sensors",
            choices=["sim", "hardware"],
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0",
            description="SLLIDAR serial port when sensor_source:=hardware",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="115200",
            description="SLLIDAR serial baudrate when sensor_source:=hardware",
        ),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="/dev/ttyAMA0",
            description="YB IMU serial port when sensor_source:=hardware",
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
                "sensor_source": sensor_source,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "imu_serial_port": imu_serial_port,
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
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "/odom")],
        ),

        Node(
            package="inspection_sim_mission",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "inspection_regions_path": default_regions,
            }],
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[nav2_params, {"yaml_filename": map_file}],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_params,
                {"default_nav_to_pose_bt_xml": nav_to_pose_bt},
                {"default_nav_through_poses_bt_xml": nav_through_poses_bt},
            ],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
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
                        "use_sim_time": False,
                        "autostart": True,
                        "bond_timeout": 6.0,
                        "node_names": lifecycle_nodes,
                    }],
                ),
            ],
        ),
    ])
