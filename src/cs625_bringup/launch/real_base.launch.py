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
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _compose(context):
    actions = [
        # Capture the application RViz switch before the driver include.  That
        # include declares launch_rviz and ROS 2 launch writes include arguments
        # into the global launch configurations, so reading the user value after
        # the driver include would observe the suppressed value instead.
        SetLaunchConfiguration("app_launch_rviz", LaunchConfiguration("launch_rviz")),
        LogInfo(msg="CS625 real profile: common application interfaces are active."),
        LogInfo(msg="execute=false and the arm controller is inactive by default."),
    ]

    launch_driver = LaunchConfiguration("launch_driver").perform(context).lower()
    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    execute = LaunchConfiguration("execute").perform(context).lower()
    require_confirmation = LaunchConfiguration("require_confirmation").perform(context).lower()
    requested_controller = LaunchConfiguration("activate_joint_controller").perform(context).lower()
    controller_activation_allowed = (
        requested_controller == "true" and execute == "true" and require_confirmation == "false"
    )
    if requested_controller == "true" and not controller_activation_allowed:
        actions.append(
            LogInfo(
                msg=(
                    "[WARN] requested real controller activation was blocked; it requires "
                    "execute=true and require_confirmation=false after external R4 review."
                )
            )
        )
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
                    "activate_joint_controller": "true" if controller_activation_allowed else "false",
                    # The driver launch would otherwise start its own RViz from
                    # the vendor description.  The application owns RViz through
                    # real_moveit.launch.py, and the user value is preserved in
                    # app_launch_rviz above.
                    "launch_rviz": "false",
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

    launch_moveit = LaunchConfiguration("launch_moveit").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    if launch_moveit:
        if launch_driver == "true" and robot_ip:
            moveit_launch_path = PathJoinSubstitution(
                [FindPackageShare("cs625_bringup"), "launch", "real_moveit.launch.py"]
            )
            # The real driver launch owns robot_state_publisher and the
            # controller manager; give it a moment to publish the description
            # before MoveIt builds its planning scene model.  This mirrors the
            # senior cs625_full_system ordering.
            actions.append(
                TimerAction(
                    period=3.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(moveit_launch_path),
                            launch_arguments={
                                "cs_type": LaunchConfiguration("cs_type"),
                                "tf_prefix": LaunchConfiguration("tf_prefix"),
                                # The driver builds the description with prefix
                                # defaulting to tf_prefix; keep MoveIt identical.
                                "prefix": LaunchConfiguration("tf_prefix"),
                                "safety_limits": LaunchConfiguration("safety_limits"),
                                "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
                                "safety_k_position": LaunchConfiguration("safety_k_position"),
                                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                                "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
                                "use_sim_time": "false",
                                "launch_rviz": LaunchConfiguration("app_launch_rviz"),
                                "description_package": LaunchConfiguration("description_package"),
                                "description_file": LaunchConfiguration("description_file"),
                                "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                                "moveit_config_package": LaunchConfiguration("moveit_config_package"),
                                "semantic_package": LaunchConfiguration("semantic_package"),
                                "semantic_file": LaunchConfiguration("semantic_file"),
                                "moveit_controllers_file": LaunchConfiguration("moveit_controllers_file"),
                                "sensors_config": LaunchConfiguration("moveit_sensors_config"),
                            }.items(),
                        )
                    ],
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        "[WARN] launch_moveit=true but the real driver is not running; "
                        "MoveIt was not started because no robot description is published."
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
    moveit_sensors_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "moveit_sensors_3d.yaml"]
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
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("eli_cs_robot_description"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description=(
                "Joint positions for the real description.  Must stay equal to "
                "the driver xacro default so MoveIt and the hardware interface "
                "share one model."
            ),
        ),
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
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description=(
                "Start the real-profile MoveIt base after the driver.  It is "
                "skipped when the driver is not running."
            ),
        ),
        DeclareLaunchArgument(
            "moveit_config_package", default_value="elite_cs625_moveit_config",
            description="Underlay MoveIt configuration package shared with the simulation profile.",
        ),
        DeclareLaunchArgument(
            "semantic_package", default_value="cs625_bringup",
            description="Package containing the application semantic overlay.",
        ),
        DeclareLaunchArgument(
            "semantic_file", default_value="config/cs625_active_perception.srdf",
            description="Application SRDF that preserves the senior arm semantics and adds gripper-adjacency exemptions.",
        ),
        DeclareLaunchArgument(
            "moveit_controllers_file", default_value="cs625_moveit_controllers.yaml",
            description="Reused MoveIt trajectory/controller configuration.",
        ),
        DeclareLaunchArgument(
            "moveit_sensors_config", default_value=moveit_sensors_default,
            description="Normalized point-cloud configuration for MoveIt scene updates.",
        ),
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
