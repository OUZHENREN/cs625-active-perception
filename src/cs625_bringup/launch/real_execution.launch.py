"""Explicit R4 entry: inactive unless `start_executor:=true` is supplied."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("cs625_bringup"), "config", "p6_real_execution.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("execution_config", default_value=config),
        DeclareLaunchArgument("start_executor", default_value="false"),
        Node(
            condition=IfCondition(LaunchConfiguration("start_executor")),
            package="cs625_motion_adapter",
            executable="real_view_executor",
            name="cs625_real_view_executor",
            output="screen",
            parameters=[ParameterFile(LaunchConfiguration("execution_config"), allow_substs=True)],
        ),
    ])
