"""P3 simulation composition: candidate generation and planning-only filtering.

The target pose is intentionally an input to the common perception contract.
For a synthetic P3 run, publish a stamped ``/perception/target_pose`` in
``base_link`` or start the separately verified target-pose source.  This launch
never sends a trajectory, regardless of the ``execute`` argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "launch", "sim_base.launch.py"]
    )
    config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "view_planning_sim.yaml"]
    )
    world_default = PathJoinSubstitution(
        [FindPackageShare("cs625_simulation"), "worlds", "minimal_occlusion.sdf"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("require_confirmation", default_value="true"),
            DeclareLaunchArgument("planning_config", default_value=config_default),
            DeclareLaunchArgument("launch_sim", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument(
                "gazebo_model_file",
                default_value="/tmp/cs625_active_perception_gazebo_model.urdf",
                description="Per-run Gazebo-only model path; avoid sharing this file across concurrent simulations.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "execute": LaunchConfiguration("execute"),
                    "require_confirmation": LaunchConfiguration("require_confirmation"),
                    "launch_official_sim": LaunchConfiguration("launch_sim"),
                    "launch_moveit": LaunchConfiguration("launch_sim"),
                    "launch_fixture_world": "false",
                    "launch_sensor_adapter": LaunchConfiguration("launch_sim"),
                    "headless": LaunchConfiguration("headless"),
                    "world": LaunchConfiguration("world"),
                    "gazebo_model_file": LaunchConfiguration("gazebo_model_file"),
                }.items(),
            ),
            Node(
                package="cs625_view_generation",
                executable="candidate_generator",
                name="cs625_candidate_generator",
                output="screen",
                parameters=[ParameterFile(LaunchConfiguration("planning_config"), allow_substs=True)],
            ),
            Node(
                package="cs625_motion_adapter",
                executable="reachability_filter",
                name="cs625_reachability_filter",
                output="screen",
                parameters=[ParameterFile(LaunchConfiguration("planning_config"), allow_substs=True)],
            ),
        ]
    )
