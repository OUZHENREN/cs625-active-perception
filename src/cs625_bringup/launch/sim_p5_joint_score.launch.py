"""P5 planning-only selector; it never starts a trajectory executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "p5_joint_score_sim.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("score_config", default_value=config),
        # Deliberately no execute argument and no motion adapter executor.
        Node(
            package="cs625_view_evaluation",
            executable="baseline_selector",
            name="cs625_baseline_selector",
            output="screen",
            parameters=[ParameterFile(LaunchConfiguration("score_config"), allow_substs=True)],
        ),
    ])
