from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_workspace = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "inspection_sim_ws"]
    )
    default_map = PathJoinSubstitution(
        [default_workspace, "maps", "inspection_map.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace",
            default_value=default_workspace,
            description="Workspace used by the GUI launch controls",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Default map YAML used by the navigation controls",
        ),
        DeclareLaunchArgument(
            "ros_setup",
            default_value="/opt/ros/jazzy/setup.bash",
            description="ROS setup.bash sourced by GUI launch controls",
        ),
        Node(
            package="inspection_sim_gui",
            executable="inspection_sim_gui",
            name="inspection_sim_gui",
            output="screen",
            parameters=[{
                "workspace_path": LaunchConfiguration("workspace"),
                "map_path": LaunchConfiguration("map"),
                "ros_setup_path": LaunchConfiguration("ros_setup"),
            }],
        ),
    ])
