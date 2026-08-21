"""P4 baseline selector; consumes the retained P3 reachable-candidate set."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "view_evaluation_sim.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("strategy", default_value="fixed_view"),
        DeclareLaunchArgument("evaluation_config", default_value=default_config),
        Node(
            package="cs625_view_evaluation",
            executable="baseline_selector",
            name="cs625_baseline_selector",
            output="screen",
            parameters=[
                ParameterFile(LaunchConfiguration("evaluation_config"), allow_substs=True),
                {"strategy": LaunchConfiguration("strategy")},
            ],
        ),
    ])
