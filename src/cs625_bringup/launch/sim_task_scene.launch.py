"""Bring up the CS625 shielding-module insertion task scene.

This composes the existing simulation base with the task world, and mirrors the
static slot fixture into MoveIt's planning scene once move_group is up.  It adds
no new robot, controller or MoveIt configuration: those still come from
``sim_base.launch.py``.

    ros2 launch cs625_bringup sim_task_scene.launch.py

Relevant overrides:

    task_world:=<path>              SDF world (default cs625_insertion_scene.sdf)
    task_scene_config:=<path>       cs625_task_scene.yaml, the geometry authority
    apply_task_scene:=false         skip the planning-scene mirror
    task_scene_seated_module:=false only mirror the fixture, not the seated module
    gazebo_visuals:=true            draw the arm meshes in the Gazebo GUI
"""

from __future__ import annotations


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


TRUTHY = ("true", "1", "yes", "on")
# move_group has to be serving /apply_planning_scene before the mirror runs.
FIXTURE_SCENE_DELAY_SEC = 20.0


def generate_launch_description() -> LaunchDescription:
    task_world_default = PathJoinSubstitution(
        [
            FindPackageShare("cs625_simulation"),
            "worlds",
            "cs625_insertion_scene.sdf",
        ]
    )
    task_scene_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "cs625_task_scene.yaml"]
    )
    apply_scene_script = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "scripts", "apply_task_scene.py"]
    )
    sim_base_launch = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "launch", "sim_base.launch.py"]
    )

    task_world = LaunchConfiguration("task_world")
    task_scene_config = LaunchConfiguration("task_scene_config")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_visuals = LaunchConfiguration("gazebo_visuals")
    apply_task_scene = LaunchConfiguration("apply_task_scene")
    seated_module = LaunchConfiguration("task_scene_seated_module")

    def launch_setup(context, *_args, **_kwargs):
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_base_launch),
                launch_arguments={
                    "world": task_world,
                    # The generic RGB-D fixture world is a separate rendering
                    # session; starting it alongside the task world gives two
                    # Gazebo instances and no benefit.
                    "launch_fixture_world": "false",
                    "launch_rviz": launch_rviz,
                    "gazebo_visuals": gazebo_visuals,
                }.items(),
            )
        ]

        if apply_task_scene.perform(context).lower() in TRUTHY:
            command = [
                FindExecutable(name="python3"),
                apply_scene_script,
                "--task-scene-config",
                task_scene_config,
            ]
            if seated_module.perform(context).lower() in TRUTHY:
                command.append("--seated-module")
            actions.append(
                LogInfo(
                    msg=(
                        "task scene mirror scheduled in "
                        f"{FIXTURE_SCENE_DELAY_SEC:.0f} s: {' '.join(str(part) for part in command)}"
                    )
                )
            )
            actions.append(
                TimerAction(
                    period=FIXTURE_SCENE_DELAY_SEC,
                    actions=[ExecuteProcess(cmd=command, output="screen")],
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg="task scene mirror disabled; run `ros2 run cs625_bringup "
                    "apply_task_scene.py` once move_group is serving"
                )
            )
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_world",
                default_value=task_world_default,
                description="Task SDF world containing the fixture and the module.",
            ),
            DeclareLaunchArgument(
                "task_scene_config",
                default_value=task_scene_config_default,
                description="Task scene YAML; the authority for the task geometry.",
            ),
            DeclareLaunchArgument(
                "apply_task_scene",
                default_value="true",
                description="Mirror the fixture into MoveIt's planning scene.",
            ),
            DeclareLaunchArgument(
                "task_scene_seated_module",
                default_value="false",
                description=(
                    "Also mirror the module at its seated pose.  Off by default "
                    "because that body is a planning goal, not fixed world geometry."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Start RViz with the application layout.",
            ),
            DeclareLaunchArgument(
                "gazebo_visuals",
                default_value="true",
                description="Keep the CS625 mesh visuals so the arm is drawn.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
