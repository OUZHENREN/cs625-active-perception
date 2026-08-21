"""P6 R1--R3 read-only evidence collection; no driver or controller is launched."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("cs625_bringup"), "config", "p6_readiness.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("readiness_config", default_value=config),
        DeclareLaunchArgument("exit_after_assessment", default_value="false"),
        Node(
            package="cs625_motion_adapter",
            executable="real_readiness_monitor",
            name="cs625_real_readiness_monitor",
            output="screen",
            parameters=[
                ParameterFile(LaunchConfiguration("readiness_config"), allow_substs=True),
                {"exit_after_assessment": LaunchConfiguration("exit_after_assessment")},
            ],
        ),
    ])
