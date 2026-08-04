"""Safe real-profile composition using the verified underlay package boundary.

The robot and camera drivers remain underlay dependencies. The profile reuses
the senior/official Elite control launch, requires an explicit robot address,
and keeps the motion controller inactive by default. Camera-brand topics end at
the common sensor adapter launch.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _compose(context):
    actions = [
        LogInfo(msg="CS625 real profile: common application interfaces are active."),
        LogInfo(msg="execute=false and the arm controller is inactive by default."),
    ]

    launch_driver = LaunchConfiguration("launch_driver").perform(context).lower()
    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    if launch_driver == "true" and robot_ip:
        driver_launch_path = PathJoinSubstitution(
            [
                FindPackageShare(LaunchConfiguration("driver_package")),
                "launch",
                LaunchConfiguration("driver_launch_file"),
            ]
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(driver_launch_path),
                launch_arguments={
                    "cs_type": LaunchConfiguration("cs_type"),
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "runtime_config_package": LaunchConfiguration(
                        "runtime_config_package"
                    ),
                    "controllers_file": LaunchConfiguration("controllers_file"),
                    "description_package": LaunchConfiguration(
                        "description_package"
                    ),
                    "description_file": LaunchConfiguration("description_file"),
                    "tf_prefix": LaunchConfiguration("tf_prefix"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                    "fake_sensor_commands": LaunchConfiguration(
                        "fake_sensor_commands"
                    ),
                    "controller_spawner_timeout": LaunchConfiguration(
                        "controller_spawner_timeout"
                    ),
                    "initial_joint_controller": LaunchConfiguration(
                        "initial_joint_controller"
                    ),
                    "activate_joint_controller": LaunchConfiguration(
                        "activate_joint_controller"
                    ),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "headless_mode": LaunchConfiguration("headless_mode"),
                    "safety_limits": LaunchConfiguration("safety_limits"),
                    "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
                    "safety_k_position": LaunchConfiguration("safety_k_position"),
                }.items(),
            )
        )
    elif launch_driver == "true":
        actions.append(
            LogInfo(
                msg=(
                    "[WARN] launch_driver=true but robot_ip is empty; the official "
                    "driver was not started."
                )
            )
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                    "[WARN] real robot driver is disabled. Set launch_driver=true and "
                    "provide robot_ip only after the underlay revision is verified."
                )
            )
        )

    camera_enabled = LaunchConfiguration("camera_enabled").perform(context).lower()
    launch_adapter = LaunchConfiguration("launch_sensor_adapter").perform(context).lower()
    if camera_enabled == "true" and launch_adapter == "true":
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
                    "[WARN] real sensor adapter is disabled. Verify the camera driver "
                    "and source topics before enabling it."
                )
            )
        )

    return actions


def generate_launch_description():
    common_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "common.yaml"]
    )
    real_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "real.yaml"]
    )
    application_description_default = PathJoinSubstitution(
        [
            FindPackageShare("cs625_ap_description"),
            "urdf",
            "cs625_active_perception.urdf.xacro",
        ]
    )

    arguments = [
        DeclareLaunchArgument("cs_type", default_value="cs625"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("require_confirmation", default_value="true"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("launch_driver", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="eli_cs_robot_driver"),
        DeclareLaunchArgument(
            "driver_launch_file", default_value="elite_control.launch.py"
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="Required real robot address; intentionally empty by default.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="eli_cs_robot_driver"
        ),
        DeclareLaunchArgument(
            "controllers_file", default_value="cs625_controllers.yaml"
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description=(
                "Underlay package that owns the CS625 parameter YAML files."
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=application_description_default,
            description=(
                "Absolute application wrapper path; the underlay launch keeps "
                "using description_package for its vendor parameter YAML files."
            ),
        ),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="30"),
        DeclareLaunchArgument(
            "initial_joint_controller", default_value="arm_controller"
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="false",
            description="Keep the arm controller inactive until explicitly confirmed.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("launch_sensor_adapter", default_value="false"),
        DeclareLaunchArgument(
            "sensor_common_config", default_value=common_config_default
        ),
        DeclareLaunchArgument(
            "sensor_profile_config", default_value=real_config_default
        ),
        DeclareLaunchArgument("input_color_topic", default_value=""),
        DeclareLaunchArgument("input_depth_topic", default_value=""),
        DeclareLaunchArgument("input_camera_info_topic", default_value=""),
        DeclareLaunchArgument("input_points_topic", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_compose)])
