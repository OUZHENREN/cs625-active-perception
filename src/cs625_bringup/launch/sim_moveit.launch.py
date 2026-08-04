"""MoveIt composition using the application robot description.

The senior MoveIt package remains the source of SRDF, kinematics, joint-limit,
planning and controller configuration. Its legacy move_group launcher is not
included because it reconstructs the vendor URDF from setup-assistant metadata
and would omit the application eye-in-hand extension.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_share_file(package_name: str, relative_or_absolute: str) -> str:
    candidate = Path(relative_or_absolute)
    if candidate.is_absolute():
        return str(candidate)
    return str(Path(get_package_share_directory(package_name)) / candidate)


def _compose(context):
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    moveit_package = LaunchConfiguration("moveit_config_package").perform(context)
    srdf_file = LaunchConfiguration("moveit_config_file").perform(context)
    controllers_file = LaunchConfiguration("moveit_controllers_file").perform(context)
    sensors_file = LaunchConfiguration("sensors_config").perform(context)

    description_path = _resolve_share_file(
        description_package, f"urdf/{description_file}"
    )
    moveit_share = Path(get_package_share_directory(moveit_package))
    srdf_path = _resolve_share_file(moveit_package, f"config/{srdf_file}")
    kinematics_path = str(moveit_share / "config" / "kinematics.yaml")
    joint_limits_path = str(moveit_share / "config" / "joint_limits.yaml")
    trajectory_path = _resolve_share_file(
        moveit_package, f"config/{controllers_file}"
    )

    robot_mappings = {
        "name": "cs625",
        "cs_type": LaunchConfiguration("cs_type").perform(context),
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
        "prefix": LaunchConfiguration("prefix").perform(context),
        "safety_limits": LaunchConfiguration("safety_limits").perform(context),
        "safety_pos_margin": LaunchConfiguration("safety_pos_margin").perform(context),
        "safety_k_position": LaunchConfiguration("safety_k_position").perform(context),
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
        "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands").perform(context),
        "sim_gazebo": "false",
        "sim_ignition": "false",
    }
    semantic_mappings = {
        "name": "cs625",
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
    }

    moveit_config = (
        MoveItConfigsBuilder("cs625", package_name=moveit_package)
        .robot_description(file_path=description_path, mappings=robot_mappings)
        .robot_description_semantic(file_path=srdf_path, mappings=semantic_mappings)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=trajectory_path)
        .planning_pipelines()
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .sensors_3d(file_path=sensors_file)
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                # Preserve the senior CS625 MoveIt runtime contract. The
                # normalized point-cloud topic still comes from sensors_file.
                "octomap_frame": "base_link",
                "octomap_resolution": 0.02,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cs_type", default_value="cs625"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("description_package", default_value="cs625_ap_description"),
            DeclareLaunchArgument(
                "description_file", default_value="cs625_active_perception.urdf.xacro"
            ),
            DeclareLaunchArgument(
                "moveit_config_package", default_value="elite_cs625_moveit_config"
            ),
            DeclareLaunchArgument("moveit_config_file", default_value="cs625.srdf.xacro"),
            DeclareLaunchArgument(
                "moveit_controllers_file",
                default_value="cs625_moveit_controllers.yaml",
            ),
            DeclareLaunchArgument(
                "sensors_config",
                default_value=str(
                    Path(get_package_share_directory("cs625_bringup"))
                    / "config"
                    / "moveit_sensors_3d.yaml"
                ),
            ),
            OpaqueFunction(function=_compose),
        ]
    )
