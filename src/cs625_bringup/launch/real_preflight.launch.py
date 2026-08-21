"""P6 R0 safety preflight; intentionally does not start drivers or controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "p6_real_safety.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("preflight_config", default_value=config),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("require_confirmation", default_value="true"),
        DeclareLaunchArgument("exit_after_publish", default_value="false"),
        Node(
            package="cs625_motion_adapter",
            executable="real_preflight",
            name="cs625_real_preflight",
            output="screen",
            parameters=[
                ParameterFile(LaunchConfiguration("preflight_config"), allow_substs=True),
                {
                    "execute": LaunchConfiguration("execute"),
                    "require_confirmation": LaunchConfiguration("require_confirmation"),
                    "exit_after_publish": LaunchConfiguration("exit_after_publish"),
                },
            ],
        ),
    ])
