"""Real-profile MoveIt composition using the application robot description.

This is the real-profile counterpart of ``sim_moveit.launch.py``: it reuses the
same underlay MoveIt package (``elite_cs625_moveit_config``) for SRDF,
kinematics, joint limits and planning pipelines, and the same application
Xacro, so the planning model does not drift between simulation and hardware.

It deliberately starts MoveIt only.  The real driver launch
(``eli_cs_robot_driver/elite_control.launch.py``) already owns
``robot_state_publisher`` and the controller manager, so this file must not
publish a second robot description or a second TF tree.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_share_file(
    package_name: str, relative_or_absolute: str, subdirectory: str = ""
) -> str:
    """Resolve a description/SRDF path.

    The real profile passes the application Xacro as an absolute path because
    the driver launch must keep ``description_package`` pointed at the vendor
    parameter package.  Absoluteness therefore has to be checked before any
    ``urdf/`` prefix is added, otherwise the prefix turns an absolute path into
    a package-relative one.
    """
    candidate = Path(relative_or_absolute)
    if candidate.is_absolute():
        return str(candidate)
    base = Path(get_package_share_directory(package_name))
    if subdirectory:
        base = base / subdirectory
    return str(base / candidate)


def _compose(context):
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    moveit_package = LaunchConfiguration("moveit_config_package").perform(context)
    semantic_package = LaunchConfiguration("semantic_package").perform(context)
    semantic_file = LaunchConfiguration("semantic_file").perform(context)
    controllers_file = LaunchConfiguration("moveit_controllers_file").perform(context)
    sensors_file = LaunchConfiguration("sensors_config").perform(context)
    rviz_config = LaunchConfiguration("moveit_rviz_config").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    description_path = _resolve_share_file(
        description_package, description_file, "urdf"
    )
    moveit_share = Path(get_package_share_directory(moveit_package))
    srdf_path = _resolve_share_file(semantic_package, semantic_file)
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
        "initial_positions_file": LaunchConfiguration("initial_positions_file").perform(context),
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
                # Hardware runs on wall clock; no Gazebo /clock is available.
                "use_sim_time": use_sim_time,
                # Keep the same planning/octomap frame as the simulation
                # profile so view planning and collision checking agree.
                "octomap_frame": "world",
                "octomap_resolution": 0.02,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    return [
        LogInfo(msg=f"real_moveit description_file={description_path}"),
        LogInfo(msg=f"real_moveit semantic_file={srdf_path}"),
        move_group,
        rviz,
    ]


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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("description_package", default_value="cs625_ap_description"),
            DeclareLaunchArgument(
                "description_file", default_value="cs625_active_perception.urdf.xacro"
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=str(
                    Path(get_package_share_directory("eli_cs_robot_description"))
                    / "config"
                    / "initial_positions.yaml"
                ),
                description="Joint positions shared with the real robot description.",
            ),
            DeclareLaunchArgument(
                "moveit_config_package", default_value="elite_cs625_moveit_config"
            ),
            DeclareLaunchArgument(
                "semantic_package", default_value="cs625_bringup",
                description="Package containing the application semantic overlay.",
            ),
            DeclareLaunchArgument(
                "semantic_file", default_value="config/cs625_active_perception.srdf",
                description="Application SRDF preserving the senior arm semantics plus gripper pairs.",
            ),
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
            DeclareLaunchArgument(
                "moveit_rviz_config",
                default_value=str(
                    Path(get_package_share_directory("cs625_bringup"))
                    / "config"
                    / "cs625_moveit.rviz"
                ),
                description=(
                    "Application RViz layout shared with the simulation profile; "
                    "vendor layout minus the unavailable elite_dashboard_* panels."
                ),
            ),
            OpaqueFunction(function=_compose),
        ]
    )
