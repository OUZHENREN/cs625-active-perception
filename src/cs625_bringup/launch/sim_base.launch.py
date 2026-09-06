"""Safe simulation composition entry point.

The fixture world remains available for interface-only checks. When the
verified senior/official underlay is present, ``launch_official_sim:=true``
composes its control launch and the thin application MoveIt launch while
supplying this repository's xacro wrapper. The underlay is never edited here.
"""

import os
from pathlib import Path

import time

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _compose(context):
    actions = [
        LogInfo(msg=["CS625 sim_base profile: ", LaunchConfiguration("cs_type")]),
        LogInfo(msg=["execute=", LaunchConfiguration("execute"), " (safe default is false)\n"]),
    ]

    launch_fixture = LaunchConfiguration("launch_fixture_world").perform(context).lower()
    launch_official = LaunchConfiguration("launch_official_sim").perform(context).lower()
    if launch_fixture == "true" and launch_official != "true":
        fixture_launch = PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
        )
        world = LaunchConfiguration("fixture_world")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fixture_launch),
                launch_arguments={"gz_args": [world, " -r"]}.items(),
            )
        )

    official_launch = LaunchConfiguration("official_sim_launch").perform(context)
    if launch_official == "true":
        official_package = LaunchConfiguration("official_sim_package").perform(context)
        official_file = LaunchConfiguration("official_sim_launch_file").perform(context)
        official_launch_path = PathJoinSubstitution(
            [FindPackageShare(official_package), "launch", official_file]
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(official_launch_path),
                launch_arguments={
                    "cs_type": LaunchConfiguration("cs_type"),
                    "runtime_config_package": LaunchConfiguration("runtime_config_package"),
                    "controllers_file": LaunchConfiguration("controllers_file"),
                    "description_package": LaunchConfiguration("description_package"),
                    "description_file": LaunchConfiguration("description_file"),
                    "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                    "initial_detach": LaunchConfiguration("initial_detach"),
                    "initial_detach_topic": LaunchConfiguration("initial_detach_topic"),
                    "prefix": LaunchConfiguration("prefix"),
                    "world_file": LaunchConfiguration("world"),
                    "launch_rviz": "false",
                    "headless": LaunchConfiguration("headless"),
                    "camera_image_width": LaunchConfiguration("camera_image_width"),
                    "camera_image_height": LaunchConfiguration("camera_image_height"),
                    "camera_update_rate": LaunchConfiguration("camera_update_rate"),
                    "camera_enabled": LaunchConfiguration("camera_enabled"),
                    "gazebo_model_file": LaunchConfiguration("gazebo_model_file"),
                }.items(),
            )
        )
        if LaunchConfiguration("launch_moveit").perform(context).lower() == "true":
            moveit_launch_path = PathJoinSubstitution(
                [FindPackageShare("cs625_bringup"), "launch", "sim_moveit.launch.py"]
            )
            # MoveIt publishes a global robot_description.  Start it after
            # the spawned Gazebo model has initialized gz_ros2_control; if it
            # publishes first, controller_manager can load MoveIt's mock
            # hardware description instead of the model's GazeboSimSystem.
            actions.append(
                TimerAction(
                    period=12.0,
                    actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_launch_path),
                    launch_arguments={
                        "cs_type": LaunchConfiguration("cs_type"),
                        "tf_prefix": LaunchConfiguration("prefix"),
                        "prefix": LaunchConfiguration("prefix"),
                        "safety_limits": LaunchConfiguration("safety_limits"),
                        "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
                        "safety_k_position": LaunchConfiguration("safety_k_position"),
                        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                        "launch_rviz": LaunchConfiguration("launch_rviz"),
                        "description_package": LaunchConfiguration("description_package"),
                        "description_file": LaunchConfiguration("description_file"),
                        "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                        "moveit_config_package": LaunchConfiguration("moveit_config_package"),
                        "semantic_package": LaunchConfiguration("semantic_package"),
                        "semantic_file": LaunchConfiguration("semantic_file"),
                        "moveit_controllers_file": LaunchConfiguration("moveit_controllers_file"),
                        "sensors_config": LaunchConfiguration("moveit_sensors_config"),
                    }.items(),
                    )],
                )
            )
    elif official_launch:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(official_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                }.items(),
            )
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                "[WARN] official CS625 sim is disabled: use launch_official_sim:=true "
                "after the underlay revision and controller contract are verified."
            )
        )
        )

    camera_enabled = LaunchConfiguration("camera_enabled").perform(context).lower()
    launch_adapter = LaunchConfiguration("launch_sensor_adapter").perform(context).lower()
    if camera_enabled == "true" and launch_adapter == "true":
        bridge_launch_path = PathJoinSubstitution(
            [FindPackageShare("cs625_bringup"), "launch", "sim_sensor_bridge.launch.py"]
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch_path),
                launch_arguments={
                    "bridge_config": LaunchConfiguration("sensor_bridge_config"),
                }.items(),
            )
        )
        adapter_launch_path = PathJoinSubstitution(
            [FindPackageShare("cs625_bringup"), "launch", "sensor_adapter.launch.py"]
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(adapter_launch_path),
                launch_arguments={
                    "common_config": LaunchConfiguration("sensor_common_config"),
                    "profile_config": LaunchConfiguration("sensor_profile_config"),
                    "input_color_topic": LaunchConfiguration("input_color_topic"),
                    "input_depth_topic": LaunchConfiguration("input_depth_topic"),
                    "input_camera_info_topic": LaunchConfiguration(
                        "input_camera_info_topic"
                    ),
                    "input_points_topic": LaunchConfiguration("input_points_topic"),
                }.items(),
            )
        )
    elif launch_adapter == "true":
        actions.append(
            LogInfo(msg="[WARN] launch_sensor_adapter=true but camera_enabled=false; skipped.")
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                    "simulation sensor adapter is disabled until Gazebo source "
                    "topics are verified and supplied."
                )
            )
        )
    return actions


def generate_launch_description():
    gz_control_lib_dir = Path(get_package_prefix("gz_ros2_control")) / "lib"
    ycb_resource_dir = Path(get_package_prefix("cs625_simulation")) / "share" / "cs625_simulation" / "assets"
    gz_control_library = gz_control_lib_dir / "libgz_ros2_control-system.so"
    if not gz_control_library.is_file():
        raise RuntimeError(
            "gz_ros2_control package was found, but its Gazebo system plugin is missing: "
            f"{gz_control_library}"
        )

    # Humble uses Gazebo Fortress. Some package revisions consult the legacy
    # IGN variable while newer revisions consult the GZ variable. Prepend the
    # actual ament package lib directory to both before the senior launch
    # starts Gazebo, avoiding a silent model-plugin lookup failure.
    # Unique IGN_PARTITION prevents stale server connections across runs
    # Keep the historical unique default, while allowing an explicitly named
    # partition for repeatable headless evidence collection and Gazebo topic
    # inspection in a single simulation session.
    ign_partition = os.environ.get("CS625_GZ_PARTITION", f"cs625_{int(time.time())}")
    plugin_environment = [
        SetEnvironmentVariable(
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            [
                str(gz_control_lib_dir),
                os.pathsep,
                EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            [
                str(gz_control_lib_dir),
                os.pathsep,
                EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                str(ycb_resource_dir),
                os.pathsep,
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable("IGN_PARTITION", ign_partition),
        SetEnvironmentVariable("GZ_PARTITION", ign_partition),
        # WSLg Mesa D3D12 → Ogre-Next 2.2.5 GL3PlusTextureGpu::copyTo
        # is unimplemented (abort at material init).  llvmpipe is slow
        # but stable on this path.  Remove when Ogre-Next ≥ 2.3.3.
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        LogInfo(msg=f"gz_ros2_control_plugin={gz_control_library}"),
        LogInfo(msg=f"ign_partition={ign_partition}"),
    ]

    world_default = PathJoinSubstitution(
        [
            FindPackageShare("cs625_simulation"),
            "worlds",
            "minimal_occlusion.sdf",
        ]
    )
    fixture_world_default = PathJoinSubstitution(
        [
            FindPackageShare("cs625_simulation"),
            "worlds",
            "rgbd_fixture.sdf",
        ]
    )
    common_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "common.yaml"]
    )
    sim_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "sim.yaml"]
    )
    bridge_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "sim_gz_bridge.yaml"]
    )
    moveit_sensors_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "moveit_sensors_3d.yaml"]
    )
    arguments = [
        DeclareLaunchArgument("cs_type", default_value="cs625", description="Robot type selector."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo simulation time."),
        DeclareLaunchArgument("world", default_value=world_default, description="SDF world path."),
        DeclareLaunchArgument("fixture_world", default_value=fixture_world_default, description="Interface-only fixed RGB-D fixture world."),
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Reserved RViz profile switch."),
        DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use fake hardware when supported."),
        DeclareLaunchArgument("camera_enabled", default_value="true", description="Enable the profile camera path."),
        DeclareLaunchArgument("launch_sensor_adapter", default_value="true", description="Start the common normalized RGB-D adapter after source topics are verified."),
        DeclareLaunchArgument("sensor_common_config", default_value=common_config_default, description="Shared normalized sensor configuration."),
        DeclareLaunchArgument("sensor_profile_config", default_value=sim_config_default, description="Simulation sensor profile configuration."),
        DeclareLaunchArgument("sensor_bridge_config", default_value=bridge_config_default, description="Gazebo RGB-D bridge configuration."),
        DeclareLaunchArgument("input_color_topic", default_value="", description="Verified Gazebo color topic."),
        DeclareLaunchArgument("input_depth_topic", default_value="", description="Verified Gazebo depth topic."),
        DeclareLaunchArgument("input_camera_info_topic", default_value="", description="Verified Gazebo camera-info topic."),
        DeclareLaunchArgument("input_points_topic", default_value="", description="Verified Gazebo point-cloud topic."),
        DeclareLaunchArgument("execute", default_value="false", description="Execution gate; false is mandatory outside explicit sim tests."),
        DeclareLaunchArgument("require_confirmation", default_value="true", description="Require an explicit execution confirmation."),
        DeclareLaunchArgument("launch_fixture_world", default_value="true", description="Launch the generic RGB-D fixture world."),
        DeclareLaunchArgument("launch_official_sim", default_value="true", description="Compose the verified senior/official CS625 control + MoveIt launch."),
        DeclareLaunchArgument("official_sim_package", default_value="cs625_bringup", description="Package containing the deterministic CS625 simulation control composition."),
        DeclareLaunchArgument("official_sim_launch_file", default_value="sim_control.launch.py", description="Application simulation control launch; senior CS625 model/configuration remain reused."),
        DeclareLaunchArgument("launch_moveit", default_value="true", description="Launch MoveIt with the application robot description when official simulation is enabled."),
        DeclareLaunchArgument("runtime_config_package", default_value="cs625_bringup", description="Simulation controller configuration package; override with the verified senior package when its SDK-backed controllers are available."),
        DeclareLaunchArgument("controllers_file", default_value="sim_controllers.yaml", description="Simulation controller YAML file using standard Humble ros2_controllers."),
        DeclareLaunchArgument("description_package", default_value="cs625_ap_description", description="Application description wrapper package."),
        DeclareLaunchArgument("description_file", default_value="cs625_active_perception.urdf.xacro", description="Application xacro wrapper file."),
        DeclareLaunchArgument("initial_positions_file", default_value=PathJoinSubstitution([FindPackageShare("eli_cs_robot_description"), "config", "initial_positions.yaml"]), description="Joint positions used to initialize the Gazebo and MoveIt robot descriptions."),
        DeclareLaunchArgument("initial_detach", default_value="false", description="Publish a one-shot Gazebo DetachableJoint detach command after robot spawn."),
        DeclareLaunchArgument("initial_detach_topic", default_value="/p7/attachment/detach", description="Absolute Gazebo transport detach topic used when initial_detach is true."),
        DeclareLaunchArgument("prefix", default_value="", description="Optional TF and joint-name prefix; must match the underlay controller configuration."),
        DeclareLaunchArgument("moveit_config_package", default_value="elite_cs625_moveit_config", description="Underlay MoveIt configuration package."),
        DeclareLaunchArgument("semantic_package", default_value="cs625_bringup", description="Package containing the application SRDF overlay."),
        DeclareLaunchArgument("semantic_file", default_value="config/cs625_active_perception.srdf", description="Application SRDF that preserves the senior arm semantics and adds gripper-adjacency exemptions."),
        DeclareLaunchArgument("moveit_controllers_file", default_value="cs625_moveit_controllers.yaml", description="Reused MoveIt trajectory/controller configuration."),
        DeclareLaunchArgument("moveit_sensors_config", default_value=moveit_sensors_default, description="Normalized point-cloud configuration for MoveIt scene updates."),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo without GUI. WSL2/GPU: false; VMware headless: true (Xvfb+ogre1 fallback)."),
        DeclareLaunchArgument("camera_image_width", default_value="320", description="Eye-in-hand RGB-D image width for the sim profile."),
        DeclareLaunchArgument("camera_image_height", default_value="240", description="Eye-in-hand RGB-D image height for the sim profile."),
        DeclareLaunchArgument("camera_update_rate", default_value="10.0", description="Eye-in-hand RGB-D update rate in Hz for the sim profile."),
        DeclareLaunchArgument("gazebo_model_file", default_value="/tmp/cs625_active_perception_gazebo_model.urdf", description="Generated visual-mesh-free URDF used only by Gazebo."),
        DeclareLaunchArgument("official_sim_launch", default_value="", description="Pinned official CS625 simulation launch path; empty means not connected."),
    ]
    return LaunchDescription(
        plugin_environment + arguments + [OpaqueFunction(function=_compose)]
    )
