# CS625 Active Perception

面向遮挡目标定位的 CS625 机械臂主动感知与视点规划研究。项目以统一的
sim/real 应用接口组织候选视点生成、可达性筛选、策略选择、MoveIt 规划和执行
状态记录；当前已验收范围为 **ROS 2 + Gazebo 仿真**。

> 真机当前未连接。本仓库中的 real-profile、预检和安全配置仅是软件准备，不能
> 被解释为真实抓取、真实识别或真实轨迹执行已经通过。

## 当前状态

| 能力层 | 当前状态 | 证据边界 |
| --- | --- | --- |
| CS625 URDF、Gazebo、ros2_control、MoveIt | 仿真通过 | 使用 underlay 中复用的 CS625 模型与配置 |
| 候选视点与硬可达性筛选 | 仿真通过 | IK、关节限位、碰撞与规划服务筛选 |
| P4 主动感知闭环 | 仿真通过 | 选点、规划、Gazebo 运动、传感器稳定状态、失败码记录 |
| P5 联合评分基础 | 仿真通过 | 可复现评分与 paired-experiment 工具 |
| P7 五级门禁 | P7.1--P7.4 `PASS / FROZEN`；P7.5 单场景 `PASS`；正式矩阵 `NOT ACCEPTED` | r12 severe-v5 番茄罐完成一次感知条件下的仿真接触抓取闭环：NBV 后 3/3 位姿更新、MoveIt 预抓取/接近/抬升、双指接触、抬升 0.140928 m、保持 2.599 s；这不是 3 遮挡 × 5 seed × 多基线正式抓取矩阵，也不是真机结论 |
| P6 真机底座 | 仅软件组装通过 | `real_base.launch.py` 已能组装 driver + 描述 + MoveIt，并用 `use_fake_hardware:=true` 通过冒烟；真机 R1--R4 尚未验收 |

## 最低可展示实验（仿真）

已冻结的 P4 矩阵包含 45 个独立 episode：

| 因子 | 取值 |
| --- | --- |
| 遮挡场景 | `occlusion_light`、`occlusion_medium`、`occlusion_severe` |
| 随机种子 | 17、18、19、20、21 |
| 基线 | `fixed_view`、`random_reachable`、`coverage_nbv` |
| 总量 | 3 × 5 × 3 = 45 |

每个单元启动独立的 Gazebo + MoveIt 进程，并导出可达率、规划时间、运动代价、
成功观测率、失败码和终止原因。2026-09-06 的完整仿真运行得到 45/45 个有效
episode；三种基线各 15 个样本。本轮矩阵全部以 `MAX_FAILED_ATTEMPTS` 终止，
因此只能作为可达性、规划时间和执行失败码的可审计数据，不能写成成功观测率
对比实验。对应聚合结果为：

| 基线 | 平均可达率 | 平均规划时间总量 (s) | 平均墙钟时间 (s) | 平均成功观测率 |
| --- | ---: | ---: | ---: | ---: |
| `coverage_nbv` | 0.752778 | 0.195406 | 102.413204 | 0.0 |
| `fixed_view` | 0.758333 | 0.187946 | 101.152591 | 0.0 |
| `random_reachable` | 0.744444 | 0.175477 | 90.748936 | 0.0 |

这是当前 Gazebo 配置下的描述性仿真结果，不是对真实机器人性能的结论。GitHub
仓库保留源码、协议、汇总 CSV/JSON 和必要小型日志；大体积原始 RGB-D/点云帧
继续保留在本地实验归档。

## 日常启动

两个 profile 是**并列的底座**，只负责把硬件/仿真、描述和 MoveIt 组装起来；
主动感知流水线用单独的 launch 叠加，不折进任一底座。

### 仿真底座

```bash
source scripts/source_dev_env.sh

# 日常手工看机械臂：Gazebo 只跑物理与传感器，RViz 显示机械臂
ros2 launch cs625_bringup sim_base.launch.py headless:=true launch_rviz:=true

# 想在 Gazebo 窗口里也看到机械臂（软件渲染下点云约 2 Hz）
ros2 launch cs625_bringup sim_base.launch.py \
  headless:=false gazebo_visuals:=true launch_rviz:=true
```

Gazebo 实体默认由**无网格**模型生成，因此 Gazebo 窗口里看不到机械臂，这是
为了避开 WSL2 软件渲染路径下的卡死；`gazebo_visuals:=true` 才保留网格。
细节见 [`docs/simulation.md`](docs/simulation.md) 的 “Visualization and manual run”。

### 主动感知流水线

```bash
source scripts/source_dev_env.sh
ros2 launch cs625_bringup sim_active_localization.launch.py strategy:=disabled launch_rviz:=false
```

### 真机底座

```bash
source scripts/source_dev_env.sh

# 不接机器人，先验证 driver + 描述 + MoveIt 能组装
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  use_fake_hardware:=true robot_ip:=127.0.0.1 launch_moveit:=true launch_rviz:=false
```

真驱动需要控制器 IP。该地址只保存在**本机**，不写进仓库（仓库禁止硬编码 IP，
`test/contract_checks.py` 会拦截）：

```bash
printf 'CS625_ROBOT_IP=<控制器IP>\n' > ~/.cs625_local.env   # 只做一次
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  robot_ip:="$CS625_ROBOT_IP" launch_moveit:=true
```

`real_base.launch.py` 的默认值是 `launch_driver:=false`、`robot_ip` 为空、
`activate_joint_controller:=false`、`execute:=false`；不显式给出 IP 时它不会启动
驱动，也不会用错误的地址去连。这个 IP 是机器人控制器地址，**夹爪没有独立 IP**
（走机械臂的 tool IO / 工具通讯）。真机前置条件与 R1--R4 门禁见
[`docs/real_hardware_readiness.md`](docs/real_hardware_readiness.md)。

### 可选：VS Code 一键任务

本机 `.vscode/tasks.json` 把上面的命令（以及构建、契约检查、启动 agent harness）
做成了任务，`Ctrl+Shift+P` → **Tasks: Run Task** 即可选择。注意 **`.vscode/` 被
`.gitignore` 忽略，不随仓库分发**，新克隆的仓库里没有这个文件，请直接用上面的
终端命令。

## 快速复现

当前验证环境为 Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2 与 Gazebo Harmonic。所有
build/test/launch 命令都必须先加载唯一环境入口，固定链为 ROS 2 Jazzy → CS625
vendor underlay → 本仓库 overlay：

```text
/opt/ros/jazzy/setup.bash
  -> $HOME/cs625_underlay_jazzy/install/setup.bash
  -> $PWD/install/setup.bash
```

vendor underlay 位置可用 `CS625_UNDERLAY_SETUP` 覆盖；入口会在任一段 setup.bash
缺失时直接失败，不会静默跳过。真机 profile 还需要 Elite CS SDK：入口在检测到
SDK 前缀存在时会一并导出其 include / lib 路径（`ELITE_CS_SDK_PREFIX`，默认
`$HOME/elite-sdk-1.2.0`），仿真不受影响。来源与固定 commit 见
[`docs/dependencies.md`](docs/dependencies.md)。

首次构建时 overlay 尚不存在，因此先只加载 ROS 与 vendor underlay。`scripts/build.sh`
等价于 `colcon build --symlink-install`，产物固定落在本仓库 `build/`、`install/`、
`log/`：

```bash
source scripts/source_dev_env.sh --no-overlay
bash scripts/build.sh
```

构建完成后校验完整链条（断言 `ROS_DISTRO=jazzy`，并解析 underlay 与 overlay 中的包）：

```bash
source scripts/source_dev_env.sh --verify
```

运行完整 45-cell 矩阵时，指定一个新的空输出目录：

```bash
scripts/run_minimum_showcase_matrix.sh \
  "$PWD/install/setup.bash" \
  "$HOME/cs625_matrix_run_$(date +%Y%m%d_%H%M%S)"
```

脚本完成后在输出目录的 `summary/` 生成：

- `matrix_validation.md`：45-cell 完整性校验；
- `episode_metrics.csv`：逐 episode 的全部指标、失败码与终止原因；
- `strategy_summary.csv`：三条基线的 15-cell 聚合指标；
- `failure_codes.csv`：按基线统计的失败码。

仅运行一个非证据 smoke test：

```bash
scripts/run_minimum_showcase_matrix.sh \
  "$PWD/install/setup.bash" "$HOME/cs625_matrix_smoke" \
  --scene occlusion_medium --seed 17 --strategy fixed_view
```

## 验证

先加载唯一环境入口，再运行静态契约检查与单元测试：

```bash
source scripts/source_dev_env.sh --verify
python3 test/contract_checks.py
pytest -q \
  src/cs625_experiment_tools/tests/test_paired_experiment.py \
  src/cs625_experiment_tools/tests/test_p4_matrix_summary.py \
  src/cs625_view_evaluation/tests/test_joint_score.py
```

`test/contract_checks.py` 除既有静态契约外，还会校验环境本身：`ROS_DISTRO`
必须是 `jazzy`，`scripts/source_dev_env.sh` 必须存在，`scripts/` 与 `test/`
中不得出现旧环境引用，`AMENT_PREFIX_PATH` 必须按“本仓库 overlay → CS625
underlay → `/opt/ros/jazzy`”排序，并禁止仓库里出现机器相关的 IP / 绝对路径
字面量。它还断言 sim/real 两个 MoveIt 入口复用同一套配置标记。

P7 的当前串行门禁、停止条件与复现入口见
[`docs/p7_five_gate_protocol.md`](docs/p7_five_gate_protocol.md)。历史临时验收见
[`docs/p7_simulation_temporary_acceptance_2026-08-31.md`](docs/p7_simulation_temporary_acceptance_2026-08-31.md)。
2026-09-06 r12 证明 P7.5 在一个 severe-v5 番茄罐场景中完成感知条件下的
仿真接触抓取闭环；正式多场景、多 seed、多策略抓取实验尚未完成。

## 工作日志

工作日志在 [`docs/worklogs/`](docs/worklogs/) 内，文件名**日期在前**：

```text
YYYY-MM-DD_<TYPE>_<TOPIC>.md
```

`<TYPE>` 取 `WORKLOG` / `HANDOFF` / `PLAN` / `SPEC` / `REVIEW`。`AGENTS.md` 规定
每条日志必须写明变更文件、确切命令、通过/失败结果、遗留限制，以及是否使用了
fake hardware / Gazebo / 真实硬件、是否发生真实运动；涉及 sim/real 链路时还要写明
能力层与关键链条状态。日志写完后用 `scripts/sync_worklog.sh` 镜像到本地 Obsidian
保管库（保管库路径是机器本地配置，不在仓库里）。

## 仓库结构

应用层共 **11 个 ROS 2 package**，与 `colcon list` 一致。package 的完整职责、构建
类型与文件层级以 [`docs/project_layout.md`](docs/project_layout.md) 为唯一详细来源；
本 README 与 `AGENTS.md` 只保留简要列表，不再各自维护不同版本。

```text
src/
├── cs625_ap_description/       # 应用层 Eye-in-Hand 描述扩展
├── cs625_ap_interfaces/        # 候选、选择与状态 ROS 接口
├── cs625_bringup/              # sim/real 启动、配置与矩阵清单
├── cs625_experiment_tools/     # 矩阵收集与 paired-experiment 工具
├── cs625_motion_adapter/       # 可达性、规划与仿真/真机执行适配器
├── cs625_sensor_adapter/       # 仿真/真机相机话题归一化
├── cs625_simulation/           # 仅 Gazebo：worlds、YCB 资产与真值
├── cs625_target_perception/    # 目标位姿与定位质量接口层
├── cs625_task_orchestrator/    # P7 抓取证据与 episode 编排
├── cs625_view_evaluation/      # 策略、P4 协调器与联合评分
└── cs625_view_generation/      # 候选视点生成
.repos/                         # underlay 依赖来源固定（real.repos 含 SDK 与师兄驱动快照）
docs/                           # 实验协议、接口、仿真与真机边界说明
├── evidence/                   # 已筛选的 JSON/CSV/PNG/TXT 证据；原始帧本地归档
├── diagnostics/                # 诊断输出与人工审计辅助材料
└── worklogs/                   # 工作日志（文件名日期在前，YYYY-MM-DD_TYPE_TOPIC.md）
scripts/                        # 环境入口、构建/测试、矩阵运行与工作日志镜像脚本
test/                           # 仓库级 contract check、P7 运行/采集工具与固定测试数据
```

`build*`、`install*`、`log*`、`.vscode/` 与 rosbag 均不入库。

## 复用与边界

- 官方 Elite CS625 模型、驱动、MoveIt 配置与 Gazebo 控制能力通过 underlay 复用；
  不在本仓库直接修改 vendor 代码。
- 真机驱动与 Elite CS SDK 由 underlay 提供：SDK 用普通 CMake 单独构建安装，驱动包
  按固定 commit 选择性复制进共享 underlay，来源与修改策略记录在
  [`docs/dependencies.md`](docs/dependencies.md)。
- 本仓库只维护主动感知应用层及 sim/real 的共同接口；仿真与真机入口不复制第二套
  核心逻辑，两个底座共用同一套 MoveIt 配置，仅时钟不同。
- 真机连接、相机标定、实时点云、抓取与安全停机测试必须按
  [`docs/p6_on_site_runbook.md`](docs/p6_on_site_runbook.md) 另行完成，并产生新的
  实物证据。

## 文档入口

- [项目文件层级与 package 结构（唯一详细来源）](docs/project_layout.md)
- [架构说明（当前实现 vs 未来目标）](docs/architecture.md)
- [仿真基线与运行边界（含可视化）](docs/simulation.md)
- [帧与话题契约](docs/frames_and_topics.md)
- [接口与话题契约](docs/interfaces.md)
- [真机就绪度与安全门禁（含两个底座的职责边界）](docs/real_hardware_readiness.md)
- [依赖、来源与复用规则](docs/dependencies.md)
- [最低可展示实验](docs/minimum_showcase_experiment.md)
- [P5 实验协议](docs/p5_experiment_protocol.md)
- [P7 五级门禁协议](docs/p7_five_gate_protocol.md)
- [P7 抓取闭环仿真协议](docs/p7_attachment_assisted_grasp_protocol.md)
- [P7 仿真临时验收（2026-08-31）](docs/p7_simulation_temporary_acceptance_2026-08-31.md)
- [P6 仿真临时验收](docs/p6_simulation_temporary_acceptance.md)
- [P6 真机现场运行手册](docs/p6_on_site_runbook.md)
- [工作日志与命名约定](docs/worklogs/README.md)
- [P7 工具索引](test/README.md)
