"""One-command simulation cell for the six task modes.

    ros2 launch cs625_bringup sim_cell.launch.py \
        scope:=load mode:=safe headless:=false launch_rviz:=true

This composes, once:

    sim_base.launch.py     Gazebo + robot + ros2_control + camera bridge/adapter
    sim_moveit.launch.py   move_group, plus the only RViz instance
    cs625_real_cell.sdf    the real-cell-aligned world

``scope`` and ``mode`` are reporting arguments only: they print the exact
``run_task.py`` command to run next and never start a task from a launch file.
The six modes therefore execute through the same entry point as on the real
arm; only the world and the controller action name differ.

The live real-cell world is produced by a separate session.  When
``cs625_real_cell.sdf`` is not on disk yet, this launch says so and falls back
to the existing insertion-scene world so the plumbing can still be exercised;
the task poses will only match the real cell after the new world is generated.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


DEFAULT_WORLD_NAME = "cs625_real_cell.sdf"
FALLBACK_WORLD_NAME = "cs625_insertion_scene.sdf"
# sim_base documents why MoveIt must start after the Gazebo entity: MoveIt
# publishes a global robot_description, and publishing it before gz_ros2_control
# initializes makes controller_manager load the mock-hardware description.
MOVEIT_START_DELAY_SEC = 12.0

SCOPES = ("load", "unload", "full")
MODES = ("safe", "efficient")


def _package_world(name: str) -> Path | None:
    try:
        return Path(get_package_share_directory("cs625_simulation")) / "worlds" / name
    except Exception:  # package lookup only fails in a broken environment
        return None


def _source_world(name: str) -> Path | None:
    """Find a world in the source checkout.

    The installed launch file is a symlink into ``src/``, so resolving
    ``__file__`` reaches the repository even when a newly generated world has
    not been installed by colcon yet.  That keeps the new world usable without
    rebuilding cs625_simulation.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "src" / "cs625_simulation" / "worlds" / name
        if candidate.is_file():
            return candidate
    return None


def _resolve_world(configured: str) -> tuple[str, str]:
    """Return ``(world_path, note)``; the note is empty when no fallback was used."""
    if Path(configured).is_file():
        return configured, ""
    if Path(configured).name != DEFAULT_WORLD_NAME:
        # An explicit override must not be silently replaced by another world.
        return configured, f"explicit world not found: {configured}"
    source = _source_world(DEFAULT_WORLD_NAME)
    if source is not None:
        return str(source), (
            f"{DEFAULT_WORLD_NAME} is not installed yet; using the source "
            f"checkout: {source}"
        )
    fallback = _package_world(FALLBACK_WORLD_NAME)
    if fallback is not None and fallback.is_file():
        return str(fallback), (
            f"{DEFAULT_WORLD_NAME} does not exist yet; falling back to "
            f"{FALLBACK_WORLD_NAME}. The task poses will not match the real "
            "cell until the new world is generated."
        )
    return configured, f"no usable world found (configured: {configured})"


def _next_command(scope: str, mode: str) -> str:
    return (
        "\n[sim_cell] Gazebo + MoveIt are starting; wait ~20 s before planning.\n"
        "[sim_cell] Next step (same phases/speeds/config as the real arm):\n"
        f"[sim_cell]   python3 scripts/run_task.py --scope {scope} --mode {mode} "
        "--world sim --dry-run\n"
        "[sim_cell] After checking the plan, replace --dry-run with --confirm.\n"
        "[sim_cell] No task is started by this launch file."
    )


def _compose(context):
    scope = LaunchConfiguration("scope").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    configured_world = LaunchConfiguration("world").perform(context)
    world, note = _resolve_world(configured_world)

    actions = [LogInfo(msg=_next_command(scope, mode))]
    if note:
        actions.insert(0, LogInfo(msg=f"[sim_cell] {note}"))
    if scope not in SCOPES:
        actions.insert(
            0,
            LogInfo(msg=f"[sim_cell][WARN] unknown scope '{scope}'; "
                        f"expected one of {', '.join(SCOPES)}"),
        )
    if mode not in MODES:
        actions.insert(
            0,
            LogInfo(msg=f"[sim_cell][WARN] unknown mode '{mode}'; "
                        f"expected one of {', '.join(MODES)}"),
        )

    sim_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cs625_bringup"), "launch", "sim_base.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            # The generic RGB-D fixture is a second Gazebo world; the cell needs
            # exactly one simulation instance.
            "launch_fixture_world": "false",
            # sim_cell owns the single sim_moveit include below, so sim_base must
            # not start a second move_group/RViz.
            "launch_moveit": "false",
            "headless": LaunchConfiguration("headless"),
            "gazebo_visuals": LaunchConfiguration("gazebo_visuals"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cs625_bringup"), "launch", "sim_moveit.launch.py"]
            )
        ),
        launch_arguments={
            # The only RViz instance in this composition: sim_moveit owns it and
            # only starts it when launch_rviz is true.
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            # Same value sim_base forwards, so the planning-side description
            # matches the accepted simulation composition.
            "use_fake_hardware": "true",
        }.items(),
    )

    actions.append(sim_base)
    actions.append(
        TimerAction(period=MOVEIT_START_DELAY_SEC, actions=[sim_moveit])
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scope",
                default_value="load",
                description="Task scope used only to print the next run_task command.",
                choices=list(SCOPES),
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="safe",
                description="Task mode used only to print the next run_task command.",
                choices=list(MODES),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without a GUI (true keeps only the server).",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start MoveIt's RViz instance (the only RViz here).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("cs625_simulation"),
                        "worlds",
                        DEFAULT_WORLD_NAME,
                    ]
                ),
                description=(
                    "SDF world; defaults to the real-cell-aligned world and falls "
                    "back to the insertion scene when that file does not exist yet."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_visuals",
                default_value="true",
                description="Keep the CS625 mesh visuals in the Gazebo GUI.",
            ),
            OpaqueFunction(function=_compose),
        ]
    )
