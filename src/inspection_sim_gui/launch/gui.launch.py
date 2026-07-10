from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_workspace = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "inspection_sim_ws"]
    )
    default_map = PathJoinSubstitution(
        [default_workspace, "maps", "inspection_map.yaml"]
    )
    default_scene_catalog = PathJoinSubstitution(
        [FindPackageShare("inspection_sim_bringup"), "scenes", "scene_catalog.json"]
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
        DeclareLaunchArgument(
            "scene_catalog",
            default_value=default_scene_catalog,
            description="Path to the shared scene catalog JSON",
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
                "scene_catalog_path": LaunchConfiguration("scene_catalog"),
            }],
        ),
    ])
