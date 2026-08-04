"""Phase 1 active-localization framework entry point with strategy disabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _log_strategy(context):
    strategy = LaunchConfiguration("strategy").perform(context).strip()
    return [
        LogInfo(
            msg=(
                "Phase 1 active-localization framework: strategy='"
                + strategy
                + "'; no target-perception/NBV algorithm is started."
            )
        )
    ]


def generate_launch_description():
    base_launch = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "launch", "sim_base.launch.py"]
    )
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
    arguments = [
        DeclareLaunchArgument("strategy", default_value="disabled", description="Phase 1 strategy selector; only disabled is implemented."),
        DeclareLaunchArgument("cs_type", default_value="cs625", description="Robot type selector."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo simulation time."),
        DeclareLaunchArgument("world", default_value=world_default, description="SDF world path."),
        DeclareLaunchArgument("fixture_world", default_value=fixture_world_default, description="Interface-only fixed RGB-D fixture world."),
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Reserved RViz profile switch."),
        DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use fake hardware when supported."),
        DeclareLaunchArgument("camera_enabled", default_value="true", description="Enable the profile camera path."),
        DeclareLaunchArgument("launch_sensor_adapter", default_value="false", description="Start the common normalized RGB-D adapter after source topics are verified."),
        DeclareLaunchArgument("sensor_common_config", default_value=common_config_default, description="Shared normalized sensor configuration."),
        DeclareLaunchArgument("sensor_profile_config", default_value=sim_config_default, description="Simulation sensor profile configuration."),
        DeclareLaunchArgument("input_color_topic", default_value="", description="Verified Gazebo color topic."),
        DeclareLaunchArgument("input_depth_topic", default_value="", description="Verified Gazebo depth topic."),
        DeclareLaunchArgument("input_camera_info_topic", default_value="", description="Verified Gazebo camera-info topic."),
        DeclareLaunchArgument("input_points_topic", default_value="", description="Verified Gazebo point-cloud topic."),
        DeclareLaunchArgument("execute", default_value="false", description="Execution gate."),
        DeclareLaunchArgument("require_confirmation", default_value="true", description="Require explicit confirmation."),
        DeclareLaunchArgument("launch_fixture_world", default_value="true", description="Launch the generic RGB-D fixture world."),
        DeclareLaunchArgument("launch_official_sim", default_value="false", description="Compose the verified senior/official CS625 control + MoveIt launch."),
        DeclareLaunchArgument("official_sim_package", default_value="cs625_bringup", description="Package containing the deterministic CS625 simulation control composition."),
        DeclareLaunchArgument("official_sim_launch_file", default_value="sim_control.launch.py", description="Application control launch reusing the senior CS625 model and configuration."),
        DeclareLaunchArgument("runtime_config_package", default_value="cs625_bringup", description="Application simulation controller profile package."),
        DeclareLaunchArgument("controllers_file", default_value="sim_controllers.yaml", description="Standard Humble simulation controller YAML."),
        DeclareLaunchArgument("description_package", default_value="cs625_ap_description", description="Application description wrapper package."),
        DeclareLaunchArgument("description_file", default_value="cs625_active_perception.urdf.xacro", description="Application xacro wrapper file."),
        DeclareLaunchArgument("moveit_config_package", default_value="elite_cs625_moveit_config", description="Underlay MoveIt configuration package."),
        DeclareLaunchArgument("moveit_config_file", default_value="cs625.srdf.xacro", description="Underlay MoveIt SRDF xacro file."),
        DeclareLaunchArgument("headless", default_value="true", description="Run Gazebo without GUI when composing the official launch."),
        DeclareLaunchArgument("official_sim_launch", default_value="", description="Pinned official CS625 simulation launch path."),
    ]

    include_args = {
        "cs_type": LaunchConfiguration("cs_type"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "world": LaunchConfiguration("world"),
        "fixture_world": LaunchConfiguration("fixture_world"),
        "launch_rviz": LaunchConfiguration("launch_rviz"),
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        "camera_enabled": LaunchConfiguration("camera_enabled"),
        "launch_sensor_adapter": LaunchConfiguration("launch_sensor_adapter"),
        "sensor_common_config": LaunchConfiguration("sensor_common_config"),
        "sensor_profile_config": LaunchConfiguration("sensor_profile_config"),
        "input_color_topic": LaunchConfiguration("input_color_topic"),
        "input_depth_topic": LaunchConfiguration("input_depth_topic"),
        "input_camera_info_topic": LaunchConfiguration("input_camera_info_topic"),
        "input_points_topic": LaunchConfiguration("input_points_topic"),
        "execute": LaunchConfiguration("execute"),
        "require_confirmation": LaunchConfiguration("require_confirmation"),
        "launch_fixture_world": LaunchConfiguration("launch_fixture_world"),
        "launch_official_sim": LaunchConfiguration("launch_official_sim"),
        "official_sim_package": LaunchConfiguration("official_sim_package"),
        "official_sim_launch_file": LaunchConfiguration("official_sim_launch_file"),
        "runtime_config_package": LaunchConfiguration("runtime_config_package"),
        "controllers_file": LaunchConfiguration("controllers_file"),
        "description_package": LaunchConfiguration("description_package"),
        "description_file": LaunchConfiguration("description_file"),
        "moveit_config_package": LaunchConfiguration("moveit_config_package"),
        "moveit_config_file": LaunchConfiguration("moveit_config_file"),
        "headless": LaunchConfiguration("headless"),
        "official_sim_launch": LaunchConfiguration("official_sim_launch"),
    }
    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=_log_strategy),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments=include_args.items(),
            )
        ]
    )
