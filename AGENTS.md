# CS625 Active Perception Development Rules

## Project goal

This repository implements the ROS 2 framework for the thesis:

**面向遮挡目标定位的机械臂可达性规划方法研究**

The system has two deployment profiles with one shared application core:

- simulation: Elite CS625 + Gazebo + simulated RGB-D camera;
- real: official Elite CS625 ROS 2 driver + real RGB-D camera.

The research core is reachability-aware observation/view planning for occluded target localization. NBV is a pluggable strategy, not the system architecture.

## Supported baseline

- current accepted simulation: WSL2 Ubuntu 24.04;
- ROS 2 Jazzy;
- MoveIt 2 Jazzy;
- Gazebo Harmonic through ros_gz;
- C++17, Python 3, colcon and rosdep.

Do not mix ROS distributions within one sourced shell.  The older Humble/VM
notes remain historical project context only; the active P4/P7 simulation
evidence in this repository is Jazzy/Harmonic.

## Environment entry (mandatory)

Every build, test, launch and `ros2` command must load the environment through
the single supported entry point before doing anything else:

```bash
source scripts/source_dev_env.sh
```

The chain is fixed and must not be reordered, shortened or mixed with another
distribution:

```text
/opt/ros/jazzy/setup.bash
  -> $HOME/cs625_underlay_jazzy/install/setup.bash
  -> <repository>/install/setup.bash
```

Supported variants:

```bash
source scripts/source_dev_env.sh --full         # complete chain (explicit form required in scripts)
source scripts/source_dev_env.sh --no-overlay   # first build: ROS Jazzy + vendor underlay only
source scripts/source_dev_env.sh --verify       # assert ROS_DISTRO=jazzy and resolve packages
```

Sourcing only affects the current shell, so the entry point and the command must
run in the same shell:

```bash
source scripts/source_dev_env.sh && <command>
```

Inside a script, always pass an explicit mode flag.  Bash makes `source <file>`
with no arguments inherit the calling script's positional parameters, so a bare
`source` inside a script that takes arguments would feed those arguments to the
entry point as flags:

```bash
source "$repo_root/scripts/source_dev_env.sh" --full
```

Prohibited:

- relying on an implicit ROS source from `~/.bashrc` or any other ambient shell
  state;
- ROS 2 Humble (`/opt/ros/humble`, a Humble underlay) for build, test or launch;
- any pre-Jazzy or temporary external overlay left over from early bring-up.

The entry point fails fast instead of degrading silently: a missing setup file,
an already-sourced non-Jazzy distribution, a missing overlay, or an
unresolvable package under `--verify` all abort before the real command starts.
`scripts/build.sh`, `scripts/test.sh`, `scripts/run_minimum_showcase_matrix.sh`
and the `test/` runners already route through it.

## Current package boundary

The repository has moved beyond the Phase 0--1 bootstrap.  Keep the existing
application packages separated by responsibility:

- `cs625_ap_interfaces`: ROS messages, services and actions only;
- `cs625_ap_description`: camera/tool extensions to the official CS625 description;
- `cs625_sensor_adapter`: normalized simulated and real camera inputs;
- `cs625_target_perception`: target pose and localization quality interface layer;
- `cs625_view_generation`: candidate view generation;
- `cs625_view_evaluation`: strategy scoring and episode coordination;
- `cs625_motion_adapter`: TF conversion, IK, MoveIt planning and execution adapters;
- `cs625_task_orchestrator`: task-level evidence and grasp episode orchestration;
- `cs625_experiment_tools`: episode summaries and experiment metrics;
- `cs625_simulation`: Gazebo-only worlds, sensors and ground truth;
- `cs625_bringup`: launch composition and profiles only.

That list is the complete current package set (11 packages) and must match
`colcon list`.  [`docs/project_layout.md`](docs/project_layout.md) is the single
detailed source for package structure; do not maintain a different list here.

Do not create a mega-node that owns perception, planning, execution and logging.

## Third-party code

Elite official ROS 2 Driver, Elite CS SDK, MoveIt 2, ros2_control, ros_gz, gz_ros2_control and camera drivers are upstream dependencies.

- Do not edit vendor packages in place.
- Prefer `.repos` files and an underlay workspace.
- If an upstream modification is unavoidable, use a dedicated fork and pin the commit.
- Record URL, branch/tag/commit, purpose, vendor status and modification policy in `docs/dependencies.md`.
- Never commit `build*`, `install*`, `log*`, rosbag or generated database files.
- Keep large raw RGB-D/point-cloud captures in the local evidence archive unless
  the user explicitly requests a data-release mechanism such as Git LFS.

## Sim/real contract

Required frames:

```text
base_link
tool0
camera_link
camera_depth_optical_frame
target_frame (when available)
```

Required normalized topics:

```text
/sensors/camera/points
/perception/target_pose
/perception/localization_quality
/scene/fused_cloud
/view_planner/raw_candidates
/view_planner/reachable_candidates
/view_planner/selected_view
/motion/status
/active_localization/state
```

Camera-brand topic names must not escape `cs625_sensor_adapter`. Every pose and cloud needs a valid timestamp and `frame_id`. Simulation and real profiles must expose the same application-facing contract.

## Safety and configuration

Real execution defaults are always:

```text
execute:=false
require_confirmation:=true
```

Use conservative velocity/acceleration scaling, bounded workspace, explicit timeout and explicit failure codes. Never bypass MoveIt collision checking or use undocumented raw TCP motion when the official interface is available.

Do not hardcode IP addresses, absolute user paths, topic names, frame names, workspace bounds, camera intrinsics, hand-eye transforms, score weights, controller names or velocity limits. Put shared values in `config/common` and profile values in `config/sim` or `config/real`.

## View-planning order

The eventual pipeline is:

```text
observe → estimate target → check termination → generate candidates
→ transform camera poses → filter hard reachability constraints
→ score reachable candidates → plan → optionally execute
→ wait for a new settled sensor frame → repeat
```

Hard constraints include IK, joint limits, self collision, environment collision and required TF. Fixed, random, predefined and NBV strategies must use one common interface and candidate set.

## Task workflow and verification

Before editing, read this file and the relevant `docs/` files, inspect the current package and tests, and state the package/interface impact. Make the smallest coherent change, add/update tests, and update documentation when behavior or interfaces change.

Minimum checks for a changed package when the environment budget allows:

```bash
colcon build --symlink-install --packages-up-to <changed_package>
colcon test --packages-select <changed_package>
colcon test-result --verbose
```

For launch and TF changes, also use the commands documented in `docs/simulation.md`
and `docs/frames_and_topics.md`.  During time-boxed report/evidence work, run
focused unit tests and cite the last runtime evidence instead of rebuilding the
entire workspace.

Every work log must state changed files, exact commands, pass/fail results, remaining limitations, whether fake hardware/Gazebo/real hardware was used, and whether real motion occurred.

### Work-log storage and mirroring

Tracked work logs live in `docs/worklogs/` and are named **date first**:

```text
YYYY-MM-DD_<TYPE>_<TOPIC>.md
```

`<TYPE>` is one of `WORKLOG`, `HANDOFF`, `PLAN`, `SPEC`, `REVIEW`.
`docs/worklogs/README.md` holds the detailed convention; do not create a second
log directory.

Saving a work log is not finished until it is mirrored to the personal Obsidian
vault, in the same turn:

```bash
scripts/sync_worklog.sh docs/worklogs/<file>.md
```

The vault location is machine-local configuration, not a repository value: the
script reads `CS625_OBSIDIAN_WORKLOG_DIR` from the environment or from
`$HOME/.cs625_local.env`.  When the vault drive is not mounted the script fails
with instructions — report that to the user instead of silently dropping the
mirror step.

### Capability-gate work-log requirements

Every work log must additionally make the current system maturity and next allowed step explicit. Use this structure, adapting it to the task:

```text
Capability layer
- environment and dependencies
- robot model and simulation
- kinematics, control and planning
- vision, hand-eye calibration and TF
- active perception / NBV
- real-hardware integration and safety

Acceptance target
- objective
- pass criteria
- runtime evidence: commands, topics, TF, logs and/or screenshots

Critical-chain status
- URDF -> Gazebo entity
- ros2_control -> joint_states
- base_link -> camera optical TF
- RGB-D -> normalized sensor topics
- point cloud -> MoveIt planning scene
- NBV decision -> robot execution

Sim/real alignment
- reused common core
- sim entry point
- real entry point
- whether real execution is allowed
- execute and confirmation gate state

Intentionally not done
- deferred layers/features and the gate that blocks them
- confirmation that vendor code was not modified, when applicable
```

For every critical-chain item, record `PASS`, `BLOCKED`, `NOT ACCEPTED`, or `NOT STARTED` and cite the concrete evidence or blocking condition. Do not treat a topic name in the ROS graph as proof of a working publisher, and do not treat a process that exited after a timeout as runtime validation.

Advance to a higher capability layer only after the prerequisite layer has passed its runtime gate. In particular, do not begin NBV, active-vision, reinforcement-learning, imitation-learning, or VLA work while the simulation camera, point-cloud, TF and MoveIt input chain is not accepted. Record infrastructure blockers at their actual layer instead of attributing their downstream symptoms to planning or perception algorithms.

## Prohibited patterns

- Do not use `pkill ros2` for lifecycle management.
- Do not create backup or duplicate source scripts.
- Do not commit generated artifacts.
- Do not add a mega-node owning perception, planning, execution and logging.
- Do not publish the same TF from multiple nodes.
- Do not consume stale point clouds after robot motion.
- Do not label synthetic smoke tests as real perception validation.
- Do not modify several architecture layers in one task without explicit scope.
