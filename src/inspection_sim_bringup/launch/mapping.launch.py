from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("inspection_sim_bringup")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world_name = LaunchConfiguration("world_name")
    world = LaunchConfiguration("world")
    sensor_source = LaunchConfiguration("sensor_source")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    imu_serial_port = LaunchConfiguration("imu_serial_port")
    sim_condition = IfCondition(PythonExpression(["'", sensor_source, "' == 'sim'"]))
    hardware_condition = IfCondition(PythonExpression(["'", sensor_source, "' == 'hardware'"]))

    sim_launch = PathJoinSubstitution(
        [pkg_share, "launch", "sim.launch.py"]
    )
    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "inspection_world.sdf"]
    )
    ekf_config = PathJoinSubstitution(
        [pkg_share, "config", "ekf.yaml"]
    )
    robot_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "inspection_tracked_robot.urdf.xacro"]
    )
    slam_config = PathJoinSubstitution(
        [pkg_share, "config", "slam.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", "inspection_sim.rviz"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    robot_description = Command(["xacro ", robot_xacro])

    return LaunchDescription([
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
            description="Start RViz for mapping",
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
            condition=sim_condition,
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
            condition=hardware_condition,
        ),

        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }],
            respawn=True,
            respawn_delay=5.0,
            condition=hardware_condition,
        ),

        Node(
            package="inspection_sim_mission",
            executable="tracked_motor_driver",
            name="tracked_motor_driver",
            output="screen",
            condition=hardware_condition,
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
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            }],
            condition=hardware_condition,
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
            condition=sim_condition,
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "/odom")],
            condition=sim_condition,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": slam_config,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ])
