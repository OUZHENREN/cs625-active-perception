# CS625 Active Perception Development Rules

## 可理解开发模式（本项目底层工作方式，优先级最高）

项目所有者设定的工作方式。它约束**每一轮怎么做事和怎么解释**，优先级高于本文件
其余部分的流程要求；与本文件其他章节冲突时，以本节为准。

目标不只是让代码运行，而是让所有者**理解系统**。

1. **一轮只解决一个明确问题**，不主动扩展新功能或新架构。
2. **改代码前先用中文说清楚**：当前问题是什么；涉及 ROS 2 哪一层；数据从哪里来、
   到哪里去；为什么需要改；预计改哪些文件。
3. **给出任何 ROS 2 命令时都要说明**：这条命令在做什么；正常情况下应该看到什么；
   输出里的关键字段是什么；失败分别意味着什么。
4. **不要用 P7.x、gate、contract、preflight 这类内部工程术语作为主要解释方式。**
   先用机器人系统的语言解释，再说明它对应仓库里哪个内部脚本/测试。
5. **除非所有者明确要求，否则不新增**：自动化 gate；contract check；evidence capture；
   新 package；新抽象层。
6. **每完成一个功能，给一张简化的数据流**：`输入 → Node/模块 → Topic/TF → 输出`。
7. **改了 launch 文件就说明**：它启动了哪些主要 Node，参数如何传递。
8. **改了 TF / URDF / 相机外参 / 坐标系，必须写出** `parent_frame → child_frame`，
   并解释这个变换的物理含义。
9. **每轮结束只给四样**：本轮解决了什么；所有者必须理解的 3 个知识点；所有者应该亲手
   运行的验证命令；下一步唯一推荐任务。

不要为了"工程完整性"一次性推进很多未来功能。

## Project goal

This repository implements the ROS 2 framework for the thesis:

**面向遮挡目标定位的机械臂可达性规划方法研究**

The system has two deployment profiles with one shared application core:

- simulation: Elite CS625 + Gazebo + simulated RGB-D camera（**与真机 1:1 对齐的场景**）;
- real: official Elite CS625 ROS 2 driver + real RGB-D camera.

The research core is reachability-aware observation/view planning for occluded target
localization. NBV is a pluggable strategy, not the system architecture.

## Supported baseline

- current accepted environment: WSL2 Ubuntu 24.04;
- ROS 2 Jazzy;
- MoveIt 2 Jazzy;
- Gazebo Harmonic through ros_gz;
- C++17, Python 3, colcon and rosdep.

Do not mix ROS distributions within one sourced shell. The older Humble/VM notes are
historical context only; they live in `docs/archive/`.

## Environment entry (mandatory)

Every build, test, launch and `ros2` command must load the environment through the single
supported entry point before doing anything else:

```bash
source scripts/source_dev_env.sh
```

The chain is fixed and must not be reordered, shortened or mixed with another distribution:

```text
/opt/ros/jazzy/setup.bash
  -> $HOME/cs625_underlay_jazzy/install/setup.bash
  -> <repository>/install/setup.bash
```

Supported variants: `--full`, `--no-overlay`, `--verify` (see the script header).
Inside a script always pass an explicit mode flag, because a bare `source` inherits the
caller's positional parameters:

```bash
source "$repo_root/scripts/source_dev_env.sh" --full
```

Prohibited: relying on ambient `~/.bashrc` state; any ROS 2 Humble or pre-Jazzy overlay;
leaving a temporary external overlay in the chain. The entry point fails fast instead of
degrading silently.

The camera driver lives in a **separate** workspace: `~/cs625_camport_ws/install/setup.bash`
must be sourced *after* the entry point when a command needs `percipio_camera`.

## Current package boundary

The repository has 12 ROS 2 packages. The list is maintained in
[`docs/project_layout.md`](docs/project_layout.md) and must match `colcon list`:

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
- `cs625_bringup`: launch composition and profiles only;
- `cs625_rviz_panel`: RViz2 dockable panel with the six task-mode buttons (2026-10-24).

Do not create a mega-node that owns perception, planning, execution and logging.

## Application layers that are NOT packages

Two layers deliberately stay outside `src/`, because they compose packages instead of
owning runtime state:

| Layer | Where | Responsibility |
|---|---|---|
| Task entry | `scripts/run_task.py` | `--scope {load,unload,full} --mode {safe,efficient} --world {real,sim}`; merges config, loads the environment model, runs the phase runner. **No motion logic.** |
| Phase runner | `scripts/run_pick_and_place.py` | The 13 load phases and the unload D1–D9 sequence, phase speeds, verification, plan-only chaining. |
| Environment model | `scripts/model_environment.py` | Offline measurement of tables/slot/workpiece → `model.json` + gate. |
| Experiment data | `scripts/experiment_recorder.py` + `archive_experiment.py` | 20 Hz timeseries, phase markers, camera frames, trimming, archive to `~/cs625_experiment_archive`. |

A new capability belongs in one of these scripts, or in a package, **not in both**.

## Six task modes (the application contract)

```text
scope  load    = 抓取工件 → 装入插槽
scope  unload  = 从插槽取出 → 放回工件桌
scope  full    = 环境建模 → 装入 → 取出 → 回到停车观测位
mode   safe       = 分段多、dwell、time_scale 2.0、每阶段实测验收
mode   efficient  = 单段执行、无 dwell、巡航 ≤0.12 m/s；槽内紧间隙仍 0.005 m/s
```

Every flow starts with an observation prefix (workpiece coarse/fine, table scan, slot
coarse/fine) and ends at the next flow's start pose. Configuration layers:

```text
config/tasks/common.yaml          world constants + reusable poses + observe waypoints
config/tasks/<scope>_<mode>.yaml  6 independent mode files (phases + phase_speeds)
config/tasks/world_sim.yaml       minimal sim overlay (control action name)
```

Hard rules baked into `scripts/run_pick_and_place.py` (do not relax them):

- tight-clearance phases (`insert`, `straighten`, `d2c`, `d4`, `d5_extract`) stay at
  **0.005 m/s** and must not use "fast cruise + slow tail";
- per-phase speeds are validated against per-mode caps; the caps are the values already
  proven on the real cell (approach 0.08, assembly 0.03, tight 0.005);
- `d2b_entrance_tilted` is **not** a tight phase (empty gripper, proven at 0.03);
- gripper-only retreat/entry is slow **only inside the handle band** (`slow_band_mm`);
- gripper release always requires `--allow-release`.

## Environment model is mandatory before motion

No flow may execute without a fresh environment model:

```text
runtime/environment/<ts>/model.json  ← scripts/model_environment.py
gate: slot >1 mm/0.5°, workpiece >3 mm/1°, <20k points, older than 2 h → REFUSE
```

`run_task.py` prints the model path, age and deviations before every step and exits with
code 2 when the gate rejects. `--dry-run` shows the verdict but does not move.

## Experiment data is collected automatically

A real run (`--confirm`) starts the recorder, marks every phase boundary, captures camera
frames, then trims and archives the run:

```text
runtime/experiments/<id>/              raw: timeseries.csv, phases.json, images/
~/cs625_experiment_archive/<id>/       trimmed + README + summary
docs/evidence/experiment_index.md      one line per archived run
```

Do not delete archived experiment data. Fill the summary honestly: missing channels (for
example no torque topic) are reported as "未采集", never as zero.

## Third-party code

Elite official ROS 2 driver, Elite CS SDK, MoveIt 2, ros2_control, ros_gz,
gz_ros2_control and camera drivers are upstream dependencies.

- Do not edit vendor packages in place.
- Prefer `.repos` files and an underlay workspace.
- If an upstream modification is unavoidable, use a dedicated fork and pin the commit.
- Record URL, branch/tag/commit, purpose, vendor status and modification policy in
  [`docs/dependencies.md`](docs/dependencies.md).

The senior's automation stack is frozen at `~/cs625_legacy_underlay_jazzy/` (read-only)
with provenance in [`docs/decisions/ADR-004-legacy-senior-underlay.md`](docs/decisions/ADR-004-legacy-senior-underlay.md).

Never commit `build*`, `install*`, `log*`, `runtime/`, rosbag or generated database files.

## Sim/real contract

Required frames: `base_link`, `tool0`, `camera_link`, `camera_depth_optical_frame`,
`target_frame` (when available).

Required normalized topics:

```text
/sensors/camera/points          /perception/target_pose
/perception/localization_quality  /scene/fused_cloud
/view_planner/raw_candidates    /view_planner/reachable_candidates
/view_planner/selected_view     /motion/status
/active_localization/state
```

Camera-brand topic names must not escape `cs625_sensor_adapter`. Simulation and real
profiles must expose the same application-facing contract.

## Simulation must mirror the real cell

The Gazebo world is **generated from the measured real cell**, not designed by hand:

```text
runtime/environment/<ts>/model.json + config/tasks/common.yaml
  → scripts/build_sim_world.py → cs625_real_cell.sdf + cs625_real_cell_scene.yaml
  → scripts/apply_real_cell_scene.py → MoveIt planning scene
```

Rules:

- table heights/extents and the fixture pose come from measurements, never from guesses;
- the workpiece pose may change (it is the task variable), everything else must match;
- the old hand-made worlds are archived in `~/cs625_legacy_sim_snapshot_20261024/` and in
  `docs/archive/`; they are read-only and must not be launched by new code;
- when the real cell changes, regenerate; never hand-edit the generated files.

## Safety and configuration

Real execution defaults are always `execute:=false`, `require_confirmation:=true`.
Conservative velocity/acceleration scaling, bounded workspace, explicit timeout and
explicit failure codes. Never bypass MoveIt collision checking or use raw TCP motion when
the official interface is available.

Before any motion, all four must hold:

```text
safety_mode = 1 (NORMAL)      robot_mode = 7 (RUNNING)
arm_controller active         python3 scripts/control_health.py  → sockets/controller/joint_states all True
```

A driver that is half dead keeps publishing *stale* status topics: a plausible-looking
`safety_mode` is not proof. When in doubt, restart the driver stack and re-check.

Do not hardcode IP addresses, absolute user paths, topic names, frame names, workspace
bounds, camera intrinsics, hand-eye transforms, score weights, controller names or
velocity limits. Shared values live in `config/common` / `config/tasks`, profile values in
`config/sim` or `config/real`, machine-local secrets in `~/.cs625_local.env`.

## View-planning order

```text
observe → estimate target → check termination → generate candidates
→ transform camera poses → filter hard reachability constraints
→ score reachable candidates → plan → optionally execute
→ wait for a new settled sensor frame → repeat
```

Fixed, random, predefined and NBV strategies must use one common interface and one
candidate set.

## Task workflow and verification

Before editing: read this file and the relevant `docs/` files, inspect the current scripts
and tests, and state the package/interface impact. Make the smallest coherent change,
add/update tests, and update documentation when behaviour changes.

The verification ladder, cheapest first — use the highest rung that answers the question:

```bash
python3 -m pytest test/ -q                                     # offline
python3 scripts/run_task.py ... --dry-run                      # config + gate + phase table
python3 scripts/run_task.py ... --plan-only                     # reachability, no motion
python3 scripts/run_task.py ... --world sim --confirm           # Gazebo, no real arm
python3 scripts/run_task.py ... --world real --confirm           # real arm, owner present
```

Work logs must state changed files, exact commands, pass/fail results, remaining
limitations, whether fake hardware/Gazebo/real hardware was used, and whether real motion
occurred.

### Work-log storage and mirroring

Tracked work logs live in `docs/worklogs/`, named **date first**:

```text
YYYY-MM-DD_<TYPE>_<TOPIC>.md      TYPE ∈ WORKLOG | HANDOFF | PLAN | SPEC | REVIEW
```

Saving a work log is not finished until it is mirrored to the personal Obsidian vault in
the same turn:

```bash
scripts/sync_worklog.sh docs/worklogs/<file>.md
```

The vault location is machine-local (`CS625_OBSIDIAN_WORKLOG_DIR` in
`$HOME/.cs625_local.env`). If the mirror fails, report it instead of silently dropping it.

### Document archiving and supersession (owner approval required)

Moving a document into an archive, or letting a newer version replace an older one, is an
owner decision. The rule covers repository documents and the Obsidian vault alike.

- Never move/delete/rename a plan, work log, review or design document into an archive
  without the owner's explicit written approval.
- A new version never supersedes an old one by itself: write a new date-stamped file next
  to the old one.
- Ask with a proposal (source path, target path, reason, replacing file, affected links,
  reversibility). Silence is not consent.
- Generated artifacts (build output, screenshots, `.bak`) are the only exception, and they
  still get a log line.

## Source preservation and authority

Nothing is deleted to "clean up". Old code, old docs and old simulation assets are kept
readable:

```text
~/cs625_legacy_underlay_jazzy/          senior's ROS stack (read-only)
~/cs625_legacy_sim_snapshot_20261024/   old simulation worlds/configs + SHA256SUMS
docs/archive/2026-10-24-pre-six-modes/  old docs (protocols, phase reviews)
```

[`docs/source_authority.md`](docs/source_authority.md) defines which numbers are
authoritative (A: 2026-10-22..24 real measurements, B: senior interfaces, C: old simulation
parameters, kept for reference only).

## Capability-gate work-log requirements

Every work log must make the system maturity and the next allowed step explicit:

```text
Capability layer
- environment and dependencies
- robot model and simulation
- kinematics, control and planning
- vision, hand-eye calibration and TF
- active perception / NBV
- real-hardware integration and safety

Acceptance target
- objective / pass criteria / runtime evidence (commands, topics, TF, logs, screenshots)

Critical-chain status
- URDF -> Gazebo entity
- ros2_control -> joint_states
- base_link -> camera optical TF
- RGB-D -> normalized sensor topics
- point cloud -> MoveIt planning scene
- NBV decision -> robot execution

Sim/real alignment
- reused common core / sim entry / real entry / real execution allowed / gate state

Intentionally not done
- deferred layers and the gate that blocks them
- confirmation that vendor code was not modified
```

For every critical-chain item record `PASS`, `BLOCKED`, `NOT ACCEPTED` or `NOT STARTED`
with evidence. A topic name in the ROS graph is not proof of a working publisher, and a
process that exited after a timeout is not runtime validation.

Advance to a higher capability layer only after the prerequisite layer passed its runtime
gate. In particular, do not begin NBV/active-vision/RL/imitation/VLA work while the
camera, point-cloud, TF and MoveIt input chain is not accepted. Record infrastructure
blockers at their actual layer instead of attributing downstream symptoms to algorithms.

## Prohibited patterns

- Do not use `pkill ros2` or wildcard kills for lifecycle management; stop the launch
  process group with SIGINT, or kill a specific PID you verified.
- Do not create backup or duplicate source scripts.
- Do not commit generated artifacts or experiment raw data.
- Do not add a mega-node owning perception, planning, execution and logging.
- Do not publish the same TF from multiple nodes, or the same collision object from two
  publishers.
- Do not consume stale point clouds or stale joint states after robot motion.
- Do not label synthetic smoke tests as real perception validation.
- Do not modify several architecture layers in one task without explicit scope.
- Do not hand-edit files that a generator owns (`cs625_real_cell.sdf`,
  `cs625_real_cell_scene.yaml`); change the generator and regenerate.
