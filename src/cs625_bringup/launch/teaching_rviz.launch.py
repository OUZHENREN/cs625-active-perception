"""只启动 RViz（复用 MoveIt 的显示参数），供已运行的驱动栈打开第二个 3D 窗口。

为什么需要它：`rviz2 -d <config>` 单独启动时拿不到 `robot_description`，
RobotModel 显示会报 "Could not find parameter robot_description ... within 10 s"，
场景里因此只有一个空坐标系。RViz 必须带着 MoveIt 的参数启动，
这正是 `real_moveit.launch.py` 里那个 rviz Node 做的事。

本文件不重复那些设置，只是把同一份 launch 以 `rviz_only:=true` 复用：
  · move_group 被跳过（避免和已在跑的实例冲突）
  · rviz2 仍然拿到 robot_description / SRDF / kinematics 等参数
  · 显示配置默认用 config/rviz/cs625_teaching.rviz（MoveIt 显示 + 点云）

用法：
    ros2 launch cs625_bringup teaching_rviz.launch.py
    ros2 launch cs625_bringup teaching_rviz.launch.py \\
        moveit_rviz_config:=$PWD/config/rviz/cs625_teaching.rviz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 默认显示配置 = 仓库自带的 MoveIt 配置（MotionPlanning/TF/RobotModel/Marker），
    # 由调用方通过 moveit_rviz_config 覆盖（面板默认传 config/rviz/cs625_teaching.rviz）。
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "cs625_moveit.rviz"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "moveit_rviz_config",
                default_value=default_rviz_config,
                description="RViz display config (default: the MoveIt config in this package).",
            ),
            DeclareLaunchArgument("description_package", default_value="eli_cs_robot_description"),
            DeclareLaunchArgument("description_file", default_value="cs625.urdf.xacro"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("cs625_bringup"), "/launch/real_moveit.launch.py"]
                ),
                launch_arguments={
                    "rviz_only": "true",
                    "launch_rviz": "true",
                    "moveit_rviz_config": LaunchConfiguration("moveit_rviz_config"),
                    "description_package": LaunchConfiguration("description_package"),
                    "description_file": LaunchConfiguration("description_file"),
                }.items(),
            ),
        ]
    )
