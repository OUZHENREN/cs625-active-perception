# 2026-10-20 日冻结与下一阶段决策（HANDOFF）

## 0. 冻结状态

- 时间：2026-10-20 21:25（Asia/Shanghai）。
- 机器人：停在 **pre-grasp**，未执行 grasp/插入。
  - 最终 TCP：`0.715, 0.552, -0.058` m；四元数 `(-0.482, 0.876, 0.007, 0.001)`。
  - 最终关节（rad）：`[-2.01999, -2.01774, 0.83280, -0.66856, 1.58325, 1.39686]`。
- WSL ROS：真实驱动、MoveIt、规划客户端、`tcp_server`、RVS 位姿 logger 已全部停止；
  无待执行轨迹，未再发运动指令。
- Windows RVS：未动，保留现场。
- 仓库：HEAD `18ee409`；24 个未提交变更（文档/P4 代码/测试），**未擅自提交**。
- 冻结证据：`~/cs625_legacy_snapshot_20261020/p4_evidence_20261020/`
  - `FREEZE_20261020.txt`
  - `joint_samples3.log`、`real_driver_ports.log`、`senior_moveit_real3.log`
  - `p4_exec_success_20261020.txt`
- 今日运行中曾残留多个 MoveIt launch 实例（158698/164751/169266），冻结时一并清理；
  后续每次启动前要先确认只有一个 `move_group` / 一个规划客户端。

## 1. 今日成果（可复述给导师）

1. 真实 CS625 驱动在 WSL Jazzy 下启动成功（`headless_mode=true`）。
2. 修正驱动 launch 的三个错误默认端口，机器人外部控制回连 50001/50003/50004 成功。
3. 新增 Windows 防火墙入站规则，只放行机器人 `机器人局域网地址` 到 50001–50004。
4. 修复 `robot_description` 的 YAML 参数解析、MoveIt 控制器名 `arm_controller` 映射。
5. 真实机器人执行 pre-grasp 到位，最终 TCP 与目标完全一致。
6. RVS 视觉输出 → 法兰/TCP 坐标 → MoveIt 规划链路已跑通。

## 2. 师兄源码完整性判断

### 2.1 现有复制情况

- **vendor underlay** `~/cs625_underlay_jazzy/src`：8 个包
  （description、driver、controllers、simulation_gz、common_interface、
  dashboard_interface、moveit_config、io_rviz_plugin）。
- **legacy automation underlay** `~/cs625_legacy_underlay_jazzy/src`：14 个包
  （compliant placement、state monitor、task manager、trajectory tools、kinematics、
  tcp_force_probe、calibration、环境点云/碰撞、model_visualizer、tcp_bridge、
  vision_bridge、pointcloud interfaces、workspace sampler）。
- D 盘工作区 `.../newfolder/elite_ros_ws/src` 共 22 个包。当前两套 underlay 已覆盖
  其中 20 个，并从 `Model/newfolder` 根目录补了新版 `cs625_compliant_placement`
  和 `cs625_tcp_force_probe`。

### 2.2 没有复制的部分

| 未复制项 | 判断 |
|---|---|
| `cs625_full_system` | 大杂烩 launch，硬编码端点；**不建议整体复制**，以后需要时只抽具体 launch 逻辑 |
| `elite_dashboard_rviz_plugin` | `package.xml` 缺 build_type，已知不可用；不复制 |
| `cs625_nbv` / 旧 NBV 工程 | 不在 D 盘工作区；只存在于旧 NBV 项目/文档引用中。这是**最值得后续选择性审查**的源码，不是当前真机链必需 |
| RVS 插件 C++（`Plug-In-Interface-master`）、RVS `.rvs` 工程、手眼文件 | Windows 侧资产，不能直接进 ROS 仓库；已登记/备份，当前不复制 |
| RoboDK API、rs-opw-kinematics 等 | 第三方参考，不作为本项目源码 |

### 2.3 结论

当前**真机 ROS 链路不缺关键包**。除上述“有意不复制”的项外，没有发现其他必须立刻
补进来的师兄 ROS 包。下一步若做仿真/NBV 实验，再单独审查旧 NBV 代码，按职责抽取，
不整仓复制。

## 3. 是否现在合并师兄代码

**建议：现在不合并，先跑完真机完整流程，再按职责合并。**

理由：

1. 师兄代码是 real-only、22 个包、没有仿真模块；当前仓库有自己的仿真 profile 和
   应用接口。现在合并会把两套运行时强行塞进一个图，破坏今天刚跑通的真机链。
2. 当前冲突主要在 **launch / topic / 控制器名 / 坐标系接口**，不是算法本身；
   用 underlay + adapter 分层比源码合并更稳。
3. 今天刚得到可复现的 real baseline，应该先冻结、补数据，再做重构。
4. 真机 E2E 还未完成（grasp、抬起/验证、插入、失败重试、主动视觉闭环）。

**建议的合并时机**：真机完整流程连续成功（建议 ≥3 次）并留下数据后，再按职责逐包吸收：

```text
cs625_state_monitor      → 自己的状态/实验数据层
cs625_trajectory_tools   → cs625_motion_adapter（保留接口）
cs625_kinematics         → 可达性/IK 约束层
cs625_compliant_placement→ 接触/插入阶段
旧 NBV 资产              → cs625_view_generation / cs625_view_evaluation（先审查）
```

师兄 underlay 始终保留为可回退参考；不做 big-bang 合并。

## 4. 仿真 / 真机双模式（必须保留）

- real：`cs625_bringup/real_base.launch.py` + vendor/legacy underlay；无 Gazebo。
- sim：`cs625_simulation` + `cs625_bringup` 的 Gazebo profile；不依赖师兄 real-only 包。
- 共享：消息/服务接口、`cs625_motion_adapter`、`cs625_task_orchestrator`、
  实验数据 schema。
- 原则：师兄代码只能出现在 real profile；仿真 profile 不被 real-only 依赖污染。

## 5. 仿真实验安排

有必要，但**放在真机 E2E 之后**，定位是“前期验证/消融/可重复实验”，不是为了凑工作量：

1. 真机完整流程先跑通并稳定记录数据；
2. 仿真复用同一套应用接口，做遮挡等级、视角数量、固定/随机/NBV 策略对比；
3. 小规模、可复现：一个场景、3 个遮挡等级、基线 vs NBV，足以支撑论文前期验证；
4. 不把仿真结果标成真实感知证据。

## 6. 实验数据实时记录（从下一次实验开始执行）

每次实验建一个 run 目录：

```text
~/cs625_legacy_snapshot_20261020/runs/YYYYMMDD_HHMMSS_<profile>_<task>/
  meta.json
  joint_states.csv
  tcp.csv
  plan.json
  logs/
```

`meta.json` 至少包含：

- run_id、时间、操作者、仓库 commit、underlay 版本；
- profile（sim/real）、机器人 IP、RVS 工程版本、手眼文件哈希；
- 目标来源（RVS T1/T3/人工）、原始 S3/T1/T3 行、目标位姿与 frame；
- pre-grasp/grasp/插入位姿、`T_b_tcp`/`T_s_tcp` 版本；
- 规划：planner、success、planning_time、轨迹点数、planned TCP path、路径长度、
  超调指标、是否加载碰撞环境；
- 执行：execute 服务返回、controller goal 结果、实际时长、速度缩放；
- 完整 `/joint_states` 与 TCP 采样（整段运动，不只前 10 s）；
- 最终 TCP/关节误差、失败码、是否急停、现场备注/照片。
- 大文件不进 git；worklog 只放路径和摘要。

今天的 pre-grasp 数据已作为第一个 run 记录，见
`~/cs625_legacy_snapshot_20261020/runs/20261020_real_pregrasp/`。

## 7. 超调（越过后回伸）解决方案

今天已确认：最终 TCP 精确到位；`execute_last_plan=false` 是 MoveIt 6.4 s 等待超时，
controller 仍 RUNNING 并在 12.9 s 完成，不是 preempt 导致。超调更可能来自
**OMPL 关节空间轨迹的中间 TCP 绕行**。

下一次按下面做，不再用 OMPL 直接冲抓取点：

1. **长距离段**：OMPL/PTP 先到目标上方 5–10 cm 的 approach waypoint；
2. **接近段**：用 Cartesian 直线（MoveIt `computeCartesianPath` 或师兄
   `cs625_trajectory_tools` 的 `/cs625/cartesian_plan_to_frame`）走到 pre-grasp；
3. **pre-grasp → grasp**：120 mm 固定偏移用 Cartesian 直线，不用 OMPL；
4. 规划后先检查 planned TCP path：若明显偏离直线/越界，拒绝执行；
5. 完整记录整段 joint/TCP，区分“规划超调”还是“跟踪超调”；
6. 调大 MoveIt 等待/时长余量，避免运动完成但服务返回 false；
7. 执行抓取前加载环境碰撞体（今天按用户要求暂时未加载）。

## 8. 下一步唯一推荐任务

先做 **plan-only**：从当前 pre-grasp 规划到 grasp 的 Cartesian 直线路径，检查
planned TCP path 是否单调、无大绕行；不执行。确认后再执行 grasp。
