from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    terminal_prefix = LaunchConfiguration("terminal_prefix")

    return LaunchDescription([
        DeclareLaunchArgument(
            "terminal_prefix",
            default_value="",
            description="Optional terminal prefix, for example: xterm -e",
        ),
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            output="screen",
            emulate_tty=True,
            prefix=terminal_prefix,
            remappings=[("/cmd_vel", "/cmd_vel")],
        ),
    ])
