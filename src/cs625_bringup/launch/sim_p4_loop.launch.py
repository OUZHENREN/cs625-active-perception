"""P4 closed-loop composition; motion is opt-in and sim-only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("cs625_bringup"), "config", "p4_sim_loop.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("execute_sim_motion", default_value="false"),
        DeclareLaunchArgument("require_confirmation", default_value="true"),
        DeclareLaunchArgument("loop_config", default_value=config),
        # These overrides make the recorded P4 loop usable as a controlled
        # matrix experiment without maintaining one YAML file per cell.
        DeclareLaunchArgument("strategy", default_value="predefined_scan"),
        DeclareLaunchArgument("random_seed", default_value="17"),
        DeclareLaunchArgument("scene_id", default_value="minimal_occlusion"),
        DeclareLaunchArgument("episode_log_directory", default_value="/tmp/cs625_active_perception/episodes"),
        Node(
            package="cs625_view_evaluation",
            executable="sim_episode_coordinator",
            name="cs625_sim_episode_coordinator",
            output="screen",
            parameters=[
                ParameterFile(LaunchConfiguration("loop_config"), allow_substs=True),
                {
                    "strategy": LaunchConfiguration("strategy"),
                    "random_seed": LaunchConfiguration("random_seed"),
                    "scene_id": LaunchConfiguration("scene_id"),
                    "episode_log_directory": LaunchConfiguration("episode_log_directory"),
                },
            ],
        ),
        Node(package="cs625_motion_adapter", executable="sim_view_executor", name="cs625_sim_view_executor", output="screen", parameters=[ParameterFile(LaunchConfiguration("loop_config"), allow_substs=True), {"execute": LaunchConfiguration("execute_sim_motion"), "require_confirmation": LaunchConfiguration("require_confirmation")}]),
    ])
