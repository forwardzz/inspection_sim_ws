from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("inspection_sim_bringup")
    world_path = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world_name = LaunchConfiguration("world_name")
    rviz_config = LaunchConfiguration("rviz_config")

    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "inspection_world.sdf"]
    )
    robot_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "inspection_tracked_robot.urdf.xacro"]
    )
    robot_sdf = PathJoinSubstitution(
        [pkg_share, "models", "inspection_tracked_robot", "model.sdf"]
    )
    default_rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", "inspection_sim_only.rviz"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
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
            default_value="false",
            description="Start RViz with the inspection simulation config",
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
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                "gz_args": [
                    PythonExpression([
                        "'-s -r ' if '", headless, "' == 'true' else '-r '"
                    ]),
                    world_path,
                ],
                "gz_version": "8",
            }.items(),
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
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_inspection_robot",
                    output="screen",
                    arguments=[
                        "-world", world_name,
                        "-file", robot_sdf,
                        "-name", "inspection_tracked_robot",
                        "-allow_renaming", "false",
                        "-x", "0.0",
                        "-y", "0.0",
                        "-z", "0.05",
                    ],
                ),
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_sensor_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/sim/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
        ),

        Node(
            package="inspection_sim_mission",
            executable="sim_imu_adapter",
            name="sim_imu_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/sim/imu/data_raw",
                "output_topic": "/imu/data_raw",
                "frame_id": "imu_link",
                "angular_velocity_z_sign": 1.0,
                "orientation_z_sign": 1.0,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
