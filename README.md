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

## 快速复现

当前验证环境为 Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2 与 Gazebo。需先准备官方
Elite/CS625、MoveIt 与仿真依赖 underlay，再构建本应用工作区：

```bash
source /opt/ros/jazzy/setup.bash
source <underlay>/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行完整 45-cell 矩阵时，指定一个新的空输出目录：

```bash
scripts/run_minimum_showcase_matrix.sh \
  <application-install>/setup.bash \
  <new-output-directory>
```

脚本完成后在 `<new-output-directory>/summary/` 生成：

- `matrix_validation.md`：45-cell 完整性校验；
- `episode_metrics.csv`：逐 episode 的全部指标、失败码与终止原因；
- `strategy_summary.csv`：三条基线的 15-cell 聚合指标；
- `failure_codes.csv`：按基线统计的失败码。

仅运行一个非证据 smoke test：

```bash
scripts/run_minimum_showcase_matrix.sh \
  <application-install>/setup.bash <temporary-output-directory> \
  --scene occlusion_medium --seed 17 --strategy fixed_view
```

## 验证

在已 source 的 Jazzy、underlay 和本项目 install 环境中运行：

```bash
python3 test/contract_checks.py
pytest -q \
  src/cs625_experiment_tools/tests/test_paired_experiment.py \
  src/cs625_experiment_tools/tests/test_p4_matrix_summary.py \
  src/cs625_view_evaluation/tests/test_joint_score.py
```

P7 的当前串行门禁、停止条件与复现入口见
[`docs/p7_five_gate_protocol.md`](docs/p7_five_gate_protocol.md)。历史临时验收见
[`docs/p7_simulation_temporary_acceptance_2026-08-31.md`](docs/p7_simulation_temporary_acceptance_2026-08-31.md)。
2026-09-06 r12 证明 P7.5 在一个 severe-v5 番茄罐场景中完成感知条件下的
仿真接触抓取闭环；正式多场景、多 seed、多策略抓取实验尚未完成。

## 仓库结构

```text
src/
├── cs625_ap_description/       # 应用层 Eye-in-Hand 描述扩展
├── cs625_ap_interfaces/        # 候选、选择与状态 ROS 接口
├── cs625_bringup/              # sim/real 启动、配置与矩阵清单
├── cs625_view_generation/      # 候选视点生成
├── cs625_motion_adapter/       # 可达性、规划与仿真/真机执行适配器
├── cs625_view_evaluation/      # 策略、P4 协调器与联合评分
└── cs625_experiment_tools/     # 矩阵收集与 paired-experiment 工具
docs/                           # 实验协议、接口、仿真与真机边界说明
scripts/                        # 可复现矩阵运行脚本
test/                           # 静态契约检查与固定测试数据
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
- [仿真基线与运行边界](docs/simulation.md)
- [接口与话题契约](docs/interfaces.md)
- [P5 实验协议](docs/p5_experiment_protocol.md)
- [P7 抓取闭环仿真协议](docs/p7_attachment_assisted_grasp_protocol.md)
- [P7 仿真临时验收（2026-08-31）](docs/p7_simulation_temporary_acceptance_2026-08-31.md)
- [P6 仿真临时验收](docs/p6_simulation_temporary_acceptance.md)
- [P6 真机现场运行手册](docs/p6_on_site_runbook.md)
- [依赖、来源与复用规则](docs/dependencies.md)
