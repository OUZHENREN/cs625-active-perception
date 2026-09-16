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
| P6 真机准备 | 暂时仅软件验收 | 真机 R1--R4 尚未验收 |

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

## 日常启动（快速入口）

在 VS Code 里打开本仓库后，`Ctrl+Shift+P` → **Tasks: Run Task** 即可选择下面这些
任务；它们都会先加载 `scripts/source_dev_env.sh` 再执行，不需要手打命令。
`Ctrl+Shift+B` 直接跑默认的构建任务。

| 任务 | 作用 |
| --- | --- |
| `0. 环境校验` | `source_dev_env.sh --verify`，确认三段环境链完整 |
| `1. 构建 overlay` | `colcon build --symlink-install`（默认构建任务） |
| `2. 契约检查 + 单元测试` | 静态契约检查与三个核心单测 |
| `3. 仿真底座（无头 Gazebo + RViz）` | 日常手工看机械臂：Gazebo 无头跑物理/传感器，RViz 显示机械臂 |
| `4. 仿真底座（Gazebo GUI + 网格 + RViz）` | Gazebo 窗口里也能看到机械臂（软件渲染下点云约 2 Hz） |
| `5. 仿真主动感知流水线` | `sim_active_localization.launch.py`（`strategy:=disabled`） |
| `6. 真机底座 - fake hardware 冒烟` | 不接机器人，验证 driver + 描述 + MoveIt 能组装 |
| `7. 真机底座 - 真驱动` | 启动真驱动；控制器保持 `inactive`、`execute:=false`。IP 从本机文件读取，见下 |
| `8. 启动 DSH Harness` | 在本仓库目录下启动本 GUI（`npx @deepseek-ai/dsh web`） |

真机 IP 只在**本机**保存一次，不写进仓库（仓库禁止硬编码 IP，契约检查会拦）：

```bash
printf 'CS625_ROBOT_IP=<控制器IP>\n' > ~/.cs625_local.env   # 只做一次
```

任务 `7` 会先 source 这个文件；文件缺失时命令会立即失败并提示，不会用错误的地址去连。
这个 IP 就是机器人控制器地址，**夹爪没有独立 IP**：它通过机械臂的 tool IO / 工具通讯控制，
驱动里 `remote_ip` / `local_port` / `remote_port` 虽然声明了但代码并未使用。

不用 VS Code 时，直接在终端里跑等价命令（必须在**同一个 shell** 里 source）：

```bash
source scripts/source_dev_env.sh
ros2 launch cs625_bringup sim_base.launch.py headless:=true launch_rviz:=true
```

真机与仿真是两个并列底座，主动感知由单独的 launch 叠加，见
[`docs/real_hardware_readiness.md`](docs/real_hardware_readiness.md)。

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
缺失时直接失败，不会静默跳过。

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
中不得出现旧环境引用，且 `AMENT_PREFIX_PATH` 必须按“本仓库 overlay → CS625
underlay → `/opt/ros/jazzy`”排序。

P7 的当前串行门禁、停止条件与复现入口见
[`docs/p7_five_gate_protocol.md`](docs/p7_five_gate_protocol.md)。历史临时验收见
[`docs/p7_simulation_temporary_acceptance_2026-08-31.md`](docs/p7_simulation_temporary_acceptance_2026-08-31.md)。
2026-09-06 r12 证明 P7.5 在一个 severe-v5 番茄罐场景中完成感知条件下的
仿真接触抓取闭环；正式多场景、多 seed、多策略抓取实验尚未完成。

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
docs/                           # 实验协议、接口、仿真与真机边界说明
├── evidence/                   # 已筛选的 JSON/CSV/PNG/TXT 证据；原始帧本地归档
├── diagnostics/                # 诊断输出与人工审计辅助材料
└── worklogs/                   # 仓库内早期工作日志归档
scripts/                        # 环境入口、构建/测试与可复现矩阵运行脚本
test/                           # 仓库级 contract check、P7 运行/采集工具与固定测试数据
```

## 复用与边界

- 官方 Elite CS625 模型、驱动、MoveIt 配置与 Gazebo 控制能力通过 underlay 复用；
  不在本仓库直接修改 vendor 代码。
- 本仓库只维护主动感知应用层及 sim/real 的共同接口；仿真与真机入口不复制第二套
  核心逻辑。
- 真机连接、相机标定、实时点云、抓取与安全停机测试必须按
  [`docs/p6_on_site_runbook.md`](docs/p6_on_site_runbook.md) 另行完成，并产生新的
  实物证据。

## 文档入口

- [最低可展示实验](docs/minimum_showcase_experiment.md)
- [项目文件层级与 package 结构（唯一详细来源）](docs/project_layout.md)
- [仿真基线与运行边界](docs/simulation.md)
- [接口与话题契约](docs/interfaces.md)
- [P5 实验协议](docs/p5_experiment_protocol.md)
- [P7 抓取闭环仿真协议](docs/p7_attachment_assisted_grasp_protocol.md)
- [P7 仿真临时验收（2026-08-31）](docs/p7_simulation_temporary_acceptance_2026-08-31.md)
- [P6 仿真临时验收](docs/p6_simulation_temporary_acceptance.md)
- [P6 真机现场运行手册](docs/p6_on_site_runbook.md)
- [依赖、来源与复用规则](docs/dependencies.md)
- [P7 工具索引](test/README.md)
