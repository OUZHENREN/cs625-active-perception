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
10. **适当并行、自动分派（所有者 2026-11-13 定）**：遇到大范围盘点、只读审计、独立调研、
   或互不重叠的多文件改动时，**主动**调用子智能体/队友并行处理，不要串行硬扛。
   分派必须写清：范围、只读还是可写、验收标准与输出格式。可写任务要给**互不重叠**的
   写作用域（共享同一个工作目录，改动立刻互相可见）；子智能体的结论必须由主负责人
   汇总、抽样交叉验证后才算数，不能直接转述。**不拆分**的情况：规模小、强顺序依赖、
   或需要同一份上下文连续推理的任务。
11. **收尾必须换角度复盘（所有者 2026-11-13 定，起因是我把"六模式"错当成当前主线）**：
   每轮工作完成后，**自动**做一次与主视角不同的复核，专门找逻辑错误与幻觉——不是把
   自己的结论重读一遍，而是换证据链重验：
   - **溯源**：每条"当前状态"结论都回到权威源（ADR / 代码常量 / 运行日志 / git 历史）
     核对；不采信二手摘要（文档标题、旧 README、交接文档的转述）。
   - **反证**：对最关键的 1–2 条结论主动找反例——是否存在相反写法、是否还有别的入口
     在用、是否已被更新的决策取代。
   - **口径**：同一事实在不同文件里的数字/名称是否一致（速度上限、日期、测试数、
     入口脚本名、包名）。
   - **可执行**：给出的每条命令是否真能跑（`--help`、`--dry-run`），路径是否存在。
   复盘发现错误要**先改正再交付**；结论与未解决问题写进交付说明，不能只声明"应该没问题"。

12. **真机执行与汇报的节奏（所有者 2026-11-25 定，优先级高于"解释清楚"）**：
    - **一次工具调用干完一件事**，禁止"先看状态 → 再看日志 → 再确认"式的三次往返；
    - 所有与机械臂真机有关的动作**一律后台跑**：下发后立刻回一句"已下发"，
      跑完立刻报数字，绝不空转等待；
    - 任何真机动作**超过 1 分钟没有反馈，不允许继续等待**，必须停下来报告；
    - 回复**只给结论与数字**，不写长篇解释；确需解释时单独一段、不超过 5 行；
    - 汇报固定格式：**指标表 + 下一步一句话**；
    - 我自己的推理延迟（每轮 1–2 分钟）与机械臂无关，必须如实区分，
      不能把"我在想"说成"机械臂慢"。

13. **日期归属（所有者 2026-11-26 定，必须自动遵守）**：
    从前一天晚上**连续做到凌晨**的所有工作，一律归到**开始那一天**，不要算到凌晨那天。
    例：2026-11-26 00:30 完成的事，写进 `docs/worklogs/2026-11/2026-11-25_*.md`，
    新建日志时若当前时间是 00:00–06:00，默认沿用前一天（除非所有者明确说这是新的一天）。
    判断依据永远是"这一段工作的起点日期"，不是提交时刻或收尾时刻。

14. **待办体系（所有者 2026-11-26 定）**：待办只写在 `docs/TODO.md`（当前周），
    **独立于工作日志**——日志记"发生了什么"，TODO 记"要做什么"。
    · 每周用 `python3 scripts/todo_week.py --rollover` 归档到 `docs/todos/<年>-W<周>_<起>_<止>.md`，
      新一周只保留未完成/阻塞项；
    · 标记：`- [ ]` 未完成 / `- [x]` 本周完成 / `- [!]` 阻塞（必须写清阻塞原因）；
    · 每次开工先看 TODO，每完成一项立刻勾掉；新增事项必须进 TODO，不许只留在对话里。
    · **需求即时登记（所有者 2026-11-26）**：所有者提出任何需求，先当场写进 TODO
      （写清要什么 + 验收标准），再动手；不许只在对话里记着，否则会被后续需求挤掉。
    · **结构固定（所有者 2026-11-26 指出太乱之后定）**：本文件按
      `⚠️ 下一步就做 → 一、真机主线 → 二、纯视觉实验 → 三、力控 → 四、数据通道 →
       五、论文文献 → 六、视觉计划 → 七、遗留缺口 → 八、已定稿(只读) → 九、本周已完成` 组织；
      **禁止追加式编号**（六b/六k/六l 这种），新事项必须放进已有分组或新增语义分组。
    · **格式由机器强制**：`scripts/check_docs.py` 会校验 TODO 的分组结构与顺序
      （缺「下一步就做」、出现追加式编号、分组乱序都会 FAIL），以及工作日志的时间戳
      **必须单调递增且不重复**。pre-commit hook 每次提交都会跑，不通过就提交不了。
    · **TODO 不镜像到 Obsidian**（所有者 2026-11-26）：Obsidian 只做工作日志的每日记录，
      待办实时变化，复制过去必然过期，反而制造不一致。

不要为了"工程完整性"一次性推进很多未来功能。

## Project goal

This repository implements the ROS 2 framework for the thesis:

**面向遮挡光学元件精密装配的可达观测规划与视触柔顺控制研究**

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
[`docs/reference/project_layout.md`](docs/reference/project_layout.md) and must match `colcon list`:

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
- `cs625_rviz_panel`: RViz2 dockable panel; its six task-mode buttons are frozen with the
  six-mode matrix (ADR-005, 2026-10-24 / 2026-11-13).

Do not create a mega-node that owns perception, planning, execution and logging.

## Application layers that are NOT packages

Four layers deliberately stay outside `src/`, because they compose packages instead of
owning runtime state:

| Layer | Where | Responsibility |
|---|---|---|
| Six-mode entry (frozen, ADR-005) | `scripts/run_task.py` | `--scope {load,unload,full} --mode {safe,efficient} --world {real,sim}`; merges config, loads the environment model, runs the phase runner. **No motion logic.** |
| Phase runner | `scripts/run_pick_and_place.py` | The 13 load phases and the unload D1–D9 sequence, phase speeds, verification, plan-only chaining. |
| Environment model | `scripts/model_environment.py` | Offline measurement of tables/slot/workpiece → `model.json` + gate. |
| Experiment data | `scripts/experiment_recorder.py` + `archive_experiment.py` | 20 Hz timeseries, phase markers, camera frames, trimming, archive to `~/cs625_experiment_archive`. |

A new capability belongs in one of these scripts, or in a package, **not in both**.

## Six task modes (frozen design record — NOT the daily entry)

> Six-mode matrix is **frozen/classified** (ADR-005, 2026-10-24 / 2026-11-13), kept
> runnable but no longer the entry. Hard rules that still apply whenever that code path
> executes: tight-clearance phases (`insert`, `straighten`, `d2c`, `d4`, `d5_extract`)
> stay at **0.005 m/s**; per-phase speeds stay under the proven caps (approach 0.08,
> assembly 0.03); `d2b_entrance_tilted` is not a tight phase; gripper-only motion is slow
> only inside `slow_band_mm`; release always needs `--allow-release`.
> Full design record and the mode matrix: [`docs/decisions/ADR-005-two-modes-and-insert-authority.md`](docs/decisions/ADR-005-two-modes-and-insert-authority.md).

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
  [`docs/reference/dependencies.md`](docs/reference/dependencies.md).

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
velocity limits. Shared values live in `config/tasks/` (task constants), the hand-eye
calibration in `config/handeye/`, simulation/real profile values in
`src/cs625_bringup/config/` and `config/real/`, machine-local secrets in
`~/.cs625_local.env`.

## View-planning order

### Real-machine mainline (方案 A: discrete station selection, 2026-11-13)

On the real cell, changing the view is a **lookup in a verified station table plus a
cross-check**, not a search:

```text
粗观测 workpiece pose in base_link + yaw angle
→ 查已验证站位表 config/tasks/common.yaml:workpiece_views
     (nearest station by yaw, tolerance 15°, strict cross-check within 12°;
      `validated: false` entries are excluded)
→ 与 0° 相对视角推导交叉核对（position delta > 200 mm ⇒ 工件已移动，拒绝）
→ plan_pose_ik 安全闸（工作空间、当前状态接触、轨迹无碰撞、安全构型区间）
→ 换位 change view → 带先验精观测 fine observation with the prior
```

`scripts/nbv_from_reference.py --auto-station` prints the decision; add `--execute` to
move, and the motion always goes through `scripts/plan_pose_ik.py`. The 0° relative-view
derivation is only the cross-check — execution targets the stored, verified station
pose. Stations marked `family: taught` (hand-placed by the owner) skip the cross-check;
for them the derivation is diagnostic information only.  The gate is **200 mm**, not 40 mm
(owner 2026-11-14): a station 12-15 deg away from the measured yaw legitimately shifts the
target by 90-120 mm, so a 40 mm gate would reject valid far stations.  A workpiece moved
beyond 200 mm is still refused.

Do **not** change view automatically — stay in place for a fine measurement, or have the
owner place the arm manually — when:

- no verified station covers the measured yaw (outside the 15° tolerance, or not
  `validated`);
- the derivation and the stored station disagree by more than 200 mm (workpiece moved);
- the move needs a large reconfiguration (flip IK solution, J2 too low, J6 beyond
  ±180°).

### Search-based flow (simulation / research only — not used on the real cell)

```text
observe → estimate target → check termination → generate candidates
→ transform camera poses → filter hard reachability constraints
→ score reachable candidates → plan → optionally execute
→ wait for a new settled sensor frame → repeat
```

This loop is kept for simulation and research. It is **not used on the real cell**: the
camera sits about 0.3 m behind the TCP, so rotating the wrist swings the camera on a
large arc, the wrist's cabling makes extreme travel dangerous (absolute joint range ±150°, but per-plan relative change defaults to 45° and J4 is recommended within ±120°), and table/fixture interference
makes "generate → score → pick the best" candidates mostly unreachable in hardware.

Fixed, random, predefined and NBV strategies must use one common interface and one
common candidate set. That is a research-layer constraint; the real-machine mainline
above implements the **predefined/fixed baseline** (a verified station table), not an
NBV search.

## Task workflow and verification

Before editing: read this file and the relevant `docs/` files, inspect the current scripts
and tests, and state the package/interface impact. Make the smallest coherent change,
add/update tests, and update documentation when behaviour changes.

The verification ladder, cheapest first — use the highest rung that answers the question:

```bash
python3 -m pytest test/ -q                                     # offline
python3 scripts/check_docs.py                                  # docs consistency
python3 scripts/teaching/teaching_debug.py --dry-run           # mainline: print, do not move
python3 scripts/run_task.py ... --dry-run                      # frozen six-mode: config + gate
python3 scripts/run_task.py ... --plan-only                     # reachability, no motion
python3 scripts/run_task.py ... --world sim --confirm           # Gazebo, no real arm
python3 scripts/run_task.py ... --world real --confirm           # real arm, owner present
```

Work logs must state changed files, exact commands, pass/fail results, remaining
limitations, whether fake hardware/Gazebo/real hardware was used, and whether real motion
occurred.

Commits go through `.git/hooks/pre-commit`, which runs `scripts/pre_commit_check.sh`
automatically (full three-checks when staged files include code, `--fast` for doc-only).
The hook is machine-local: a new clone installs it with `bash scripts/install_hooks.sh`.

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

## 文档分工与写入边界（所有者 2026-11-20 定，必须自动遵守）

仓库里的文档**各有唯一归属**。同一件事只写在它的归属文件里，其它地方用链接引用，
不允许把整段叙事复制到多个文件。

| 文件 | 只放什么 | 不放什么 |
|---|---|---|
| `docs/README.md` | **唯一导航入口**：当前状态一句话、"下次开工先看"清单、各文档的用途 | 具体经验、事故细节 |
| `docs/agent_memory.md` | **每次开工必读**的当前权威值 + 硬规则 + 安全红线（控制在 300 行内） | 事故经过、调试流水、历史口径 |
| `docs/agent_memory_history.md` | 旧条目**只读存档**，供溯源；编号与标题保持不变 | 新内容（新内容进 agent_memory.md） |
| `docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md` | **可复用的真机经验**，一条一个编号：`症状 → 真因 → 确认命令 → 处置 → 预防`；含排查顺序与诊断流程 | 逐日流水、"今天 19:40 我们…"、只对一次会话有意义的内容 |
| `docs/manuals/debug/` | **看不到数据时**用的命令速查（相机、ROS 图），正常输出与失败含义 | 系统性经验（进手册正文） |
| `docs/worklogs/YYYY-MM/YYYY-MM-DD_*.md` | **当天的时间线**：每节以 `## HH:MM` 开头，写清"那个时间点在做什么、命令、结果、实测数字" | 长期规则；一个月后还重要的内容 |
| `docs/plans/` | 月度/专题计划：做什么、验收线、明确不做 | 已完成事实的流水账 |
| `docs/decisions/ADR-*.md` | 架构与范围**决策**：背景、选项、结论、日期 | 代码里已经写清的实现细节 |
| `docs/reference/` | **参考资料**：坐标系与话题、位姿口径、状态码、真机工位、接口契约 | 临时观察、未验证结论 |
| `docs/evidence/experiment_index.md` | 每次归档实验一行 | 分析与解释（放工作日志） |
| `docs/literature/` | 文献阅读主表 | 项目结论 |
| Obsidian 库 | **工作日志的镜像**（`scripts/sync_worklog.sh`） | 作为唯一副本 |

### 什么时候自动写、写到哪里（不需要所有者再提醒）

```text
1. 事故处理完、真因确认后：
   · 立刻在真机手册追加**一条**编号条目（症状/真因/确认命令/处置/预防），
     标准是"没见过这次事故的人照着也能用"；
   · 同时在当天工作日志里记下排查过程与实测数字。
2. 流程或常量发生变化（速度上限、恢复步骤、开工清单）：
   · 代码 + 测试 + 真机手册 + agent_memory.md **在同一个提交里**一起改，
     绝不允许文档还写着旧数字。
3. **工作日志必须"边做边写"，不能等所有者提醒**：
   · 每完成一个可交付动作（拍到一张合格示教位、修掉一个缺陷、跑通一次验证）
     就立刻在当天日志里补一行，**带上时间 `HH:MM`**；
   · 日志的每一节都要有"## HH:MM 做了什么"，让读者能看出一天的时间分布；
   · 事故发生时先写"发生时间 + 现象 + 当时在做什么"，真因查明后再补一节交叉引用；
   · 每轮结束检查：本轮每一条经验是否已进手册、每一个事实是否已进带时间的日志；
   · 工作日志必须镜像到 Obsidian（`scripts/sync_worklog.sh`），失败要报告不能静默丢弃。
4. 展示成果（新站位、新脚本、新能力）：
   · 写进工作日志与对应手册条目；涉及数值的门槛（如覆盖率 0.95）写进 agent_memory.md。
```

### 边界（违反即视为写错）

```text
· 手册不是草稿本：只写"我们试了几种办法"而不写清真因与证据的条目不算数。
· 工作日志只追加：不要用今天的理解改写昨天的日志，新增一节并交叉引用。
· 归档、改名、让新文档取代旧文档，必须事先取得所有者书面同意（见上一节）。
· 手册条目编号永久占用；新经验取下一个编号，索引表必须在同一次编辑里补行。
· 同一事实在不同文件里出现时，数字必须完全一致（速度上限、日期、测试数、脚本名）。
```

## Branch policy (frozen 2026-11-13, owner decision)

- `main` is the only active branch. **Every commit goes directly to `main`** and is
  pushed to `origin/main`.
- `teaching-rvs-mode` is **frozen**: it only served the 2026-10 teaching-mode / RVS
  viewpoint update phase. Do not commit to it and do not push it; the remote branch
  stays exactly as the owner left it.

## Source preservation and authority

Nothing is deleted to "clean up". Old code, old docs and old simulation assets are kept
readable:

```text
~/cs625_legacy_underlay_jazzy/          senior's ROS stack (read-only)
~/cs625_legacy_sim_snapshot_20261024/   old simulation worlds/configs + SHA256SUMS
docs/archive/2026-10-24-pre-six-modes/  old docs (protocols, phase reviews)
```

[`docs/archive/2026-11-07-pre-reorg/source_authority.md`](docs/archive/2026-11-07-pre-reorg/source_authority.md) defines which numbers are
authoritative (A: 2026-10-22..24 real measurements, B: senior interfaces, C: old simulation
parameters, kept for reference only).

## Execution safety contract (2026-11-19, owner directive — never bypass)

The owner's standing instruction: **speed, joint travel and workspace limits are enforced by
code, not by good intentions. No deviation is acceptable.** This exists because on 2026-11-18 a
47.6 deg wrist reconfiguration was executed, the arm ended at twice the commanded travel
(J1 128.7 deg, J6 156.8 deg, both outside the safe ranges), the driver then lost contact, and
the same class of automated motion had already damaged the J4 brake on 2026-11-13.

Hard rules, already implemented in `scripts/execute_assembly_chunks.py` and
`scripts/plan_pose_ik.py`:

- **The executor never trusts the plan's timestamps.** It re-times every trajectory so no
  joint exceeds `MAX_JOINT_SPEED_RAD_S = 0.10 rad/s` (~5.7 deg/s; raised from 0.05 on 2026-11-20 by the owner because 2.86 deg/s sat at the servo deadband and caused jitter plus 'state tolerance' aborts), and only ever slows a plan
  (`time_scale < 1` is refused; the governor applies `max(1.0, time_scale)`).
- **Single-joint travel is capped at `MAX_JOINT_DELTA_RAD = 0.44 rad` (25 deg) per plan.**
  A plan that exceeds it is refused (`REFUSED_LARGE_MOVE`) unless the six joint angles have
  been printed and the owner has explicitly consented (`--allow-large-move`). Large
  reconfigurations must be split into smaller segments or planned with MoveIt.
- Before sending anything, print the governed duration, the resulting maximum joint speed and
  the per-joint travel; a trajectory that still exceeds the cap is refused.
- The arm's state must be re-verified after any abort before the next command
  (`ensure_robot_ready.py --force` + a 2 degree probe).
- Workspace bounds, absolute joint ranges, wrist limits and the "current state is already in
  contact" check stay mandatory; they are never relaxed to make a plan pass.

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
- Do not make a plan pass by ignoring contact pairs or suppressing collision warnings;
  contact is contact.
- Do not move while the current state already reports contact — stop and hand over to
  the owner first.
- Do not accept a flipped configuration (J2 too low; `|wrist_3|` — the same joint as J6
  — beyond 170°); before any large reconfiguration, print all 6 joint angles and get the
  owner's explicit consent.

(originally agent_memory §133–§135; after the 2026-11-13 split they are the
"安全红线" section of `docs/agent_memory.md`, with the full history in
`docs/agent_memory_history.md`.  Never relax them.)
