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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

# robot_description 的注入器与 sim_control.launch.py 共用同一个脚本。仓库用
# colcon --symlink-install，launch 文件在源码树里，所以这里的相对路径能解析到
# <仓库根>/scripts；解析不到就直接报错，不做静默回退（否则 MoveIt 会悄悄用回
# 厂商原始描述，和仿真实体悄悄不一致）。
REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_DESCRIPTION_INJECTOR = REPO_ROOT / "scripts" / "sim_robot_description.py"


def _resolve_share_file(package_name: str, relative_or_absolute: str) -> str:
    candidate = Path(relative_or_absolute)
    if candidate.is_absolute():
        return str(candidate)
    return str(Path(get_package_share_directory(package_name)) / candidate)


def _compose(context):
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    moveit_package = LaunchConfiguration("moveit_config_package").perform(context)
    semantic_package = LaunchConfiguration("semantic_package").perform(context)
    semantic_file = LaunchConfiguration("semantic_file").perform(context)
    controllers_file = LaunchConfiguration("moveit_controllers_file").perform(context)
    sensors_file = LaunchConfiguration("sensors_config").perform(context)
    rviz_config = LaunchConfiguration("moveit_rviz_config").perform(context)

    description_path = _resolve_share_file(
        description_package, f"urdf/{description_file}"
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
        # 规划侧描述里的 <ros2_control> 块对 MoveIt 没有用处，但**会被
        # gz_ros2_control 抢着读**（控制器管理器订阅 /robot_description，谁先发就
        # 读谁的；move_group 也会发一份）。如果这一份写的是 mock_components，
        # 控制器管理器就会去加载 mock 硬件并失败
        # （实测：class mock_components/GenericSystem ... does not exist）。
        # 所以规划侧也用 Gazebo 硬件块，谁被读到都正确。
        "use_fake_hardware": "false",
        "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands").perform(context),
        "initial_positions_file": LaunchConfiguration("initial_positions_file").perform(context),
        "sim_gazebo": "false",
        "sim_ignition": "true",
    }
    semantic_mappings = {
        "name": "cs625",
        "tf_prefix": LaunchConfiguration("tf_prefix").perform(context),
    }

    # 为什么规划侧也要过注入器：
    #   1) 厂商 cs625.urdf.xacro 的 <hardware> 块里没有 position_proportional_gain，
    #      而 gz_ros2_control 的位置环静差只能靠这个参数压下去（详见脚本头部）；
    #   2) 厂商的 tool0_vision.STL 偏在工具轴一侧，注入器会换成关于工具轴对称的
    #      gripper_0920.stl。
    #   sim_control.launch.py 已经用同一个注入器生成 Gazebo 实体的描述；如果 MoveIt
    #   仍用原始描述，规划用的工具几何/碰撞体就会和真实执行机构不同，规划结论不可信。
    #   这里沿用 sim_control 的写法：python3 <注入器> <xacro> <urdf> key:=value ...
    if not ROBOT_DESCRIPTION_INJECTOR.is_file():
        raise RuntimeError(
            f"robot description injector missing: {ROBOT_DESCRIPTION_INJECTOR}; "
            "sim_moveit must not fall back to the vendor description"
        )
    # Command 是拼接式 substitution（相邻 token 之间不会自动加空格），所以每个
    # token 后面都要显式跟一个 " "，否则 xacro 会收到 "name:=cs625cs_type:=..."
    # 这样一个畸形参数并直接报错。
    robot_description_command: list = [
        FindExecutable(name="python3"),
        " ",
        str(ROBOT_DESCRIPTION_INJECTOR),
        " ",
        FindExecutable(name="xacro"),
        " ",
        description_path,
        " ",
    ]
    for key, value in robot_mappings.items():
        robot_description_command += [f"{key}:=", value, " "]
    injected_robot_description = ParameterValue(
        Command(robot_description_command), value_type=str
    )

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
    # 仍保留 .robot_description() 调用：builder 需要它来完成描述文件解析与存在性检查，
    # 否则 to_moveit_configs() 会退回 .setup_assistant 里的厂商默认 URDF 并告警。
    # 但真正交给 move_group / RViz 的参数必须是注入版，所以这里替换掉唯一一项。
    moveit_config.robot_description = {
        "robot_description": injected_robot_description
    }
    # 两个 Node 共用同一份参数字典，保证 move_group 规划用的模型和 RViz 显示的一致。
    moveit_parameters = moveit_config.to_dict()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_parameters,
            {
                # Point clouds and TF are stamped by Gazebo.  Keep the
                # occupancy-map message filter on the same clock so it can
                # resolve camera-frame transforms at cloud timestamps.
                "use_sim_time": True,
                # Jazzy's PlanningSceneMonitor uses the planning frame as the
                # effective Octomap frame.  This model's planning frame is
                # world, so keep the explicit parameter aligned with runtime.
                # The normalized point-cloud topic still comes from sensors_file.
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
        parameters=[moveit_parameters],
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
                "initial_positions_file",
                default_value=str(
                    Path(get_package_share_directory("eli_cs_robot_description"))
                    / "config"
                    / "initial_positions.yaml"
                ),
                description="Joint positions shared with the simulated robot description.",
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
                    "Application RViz layout.  Application-owned copy of the "
                    "vendor layout with the unavailable elite_dashboard_* "
                    "panels removed; the vendor file is never edited."
                ),
            ),
            OpaqueFunction(function=_compose),
        ]
    )
