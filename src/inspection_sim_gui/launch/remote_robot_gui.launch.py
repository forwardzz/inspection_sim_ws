from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("inspection_sim_gui"), "rviz", "remote_robot.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "host",
            default_value="192.168.43.21",
            description="Remote robot IP or hostname on the local network",
        ),
        DeclareLaunchArgument(
            "user",
            default_value="yy",
            description="Remote SSH user",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="22",
            description="Remote SSH port",
        ),
        DeclareLaunchArgument(
            "workspace",
            default_value="/home/yy/inspection_sim_ws",
            description="Remote inspection simulation workspace",
        ),
        DeclareLaunchArgument(
            "local_workspace",
            default_value="/home/zjy/inspection_sim_ws",
            description="Local inspection simulation workspace used for source sync",
        ),
        DeclareLaunchArgument(
            "ros_setup",
            default_value="/opt/ros/jazzy/setup.bash",
            description="Remote ROS setup.bash",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="/home/yy/inspection_sim_ws/maps/inspection_map.yaml",
            description="Remote map YAML path used by navigation launch",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Local RViz config started with mapping/navigation",
        ),
        Node(
            package="inspection_sim_gui",
            executable="remote_robot_gui",
            name="remote_robot_gui",
            output="screen",
            arguments=[
                "--host", LaunchConfiguration("host"),
                "--user", LaunchConfiguration("user"),
                "--port", LaunchConfiguration("port"),
                "--workspace", LaunchConfiguration("workspace"),
                "--local-workspace", LaunchConfiguration("local_workspace"),
                "--ros-setup", LaunchConfiguration("ros_setup"),
                "--map", LaunchConfiguration("map"),
                "--rviz-config", LaunchConfiguration("rviz_config"),
            ],
        ),
    ])
