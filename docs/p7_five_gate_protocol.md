# P7 五级串行门禁协议

## 1. 总原则

P7 不再作为单一抓取任务验收，而按以下顺序逐级冻结：

```text
P7.1 感知输入
  → P7.2 位姿估计
  → P7.3 NBV
  → P7.4 MoveIt 可执行性
  → P7.5 抓取闭环
```

前一门未形成可复核的运行证据时，后一门保持 `NOT STARTED`。任何门失败均停止后续集成，并按“问题 → 根因证据 → 修改 → 单元测试 → 回归测试 → Gate 结果”记录。历史 P4/P6/P7 结果只保留其原证据含义，不自动升级为新门禁的通过证据。

本协议仅使用既有 WSL2 Ubuntu 24.04、ROS 2 Jazzy 与 Gazebo Harmonic 环境；真机未连接，不迁移 ROS 版本，不修改 Elite/师兄 vendor underlay。

## 2. P7.1 感知输入 Gate

### 2.1 冻结输入接口

| 输入 | 规范化话题 | 最低检查 |
| --- | --- | --- |
| RGB | `/sensors/camera/color/image` | 非零时间戳、有效 frame、尺寸/编码/步长一致 |
| Depth | `/sensors/camera/depth/image` | 非零时间戳、有效 frame、尺寸/编码/步长一致 |
| CameraInfo | `/sensors/camera/depth/camera_info` | `fx, fy > 0`，尺寸有效 |
| PointCloud | `/sensors/camera/points` | 含 `x/y/z`，存在有限点，数据布局完整 |
| TF | `base_link → camera_color_optical_frame` 与 `base_link → camera_depth_optical_frame` | 在各消息原始时间戳可查询 |

### 2.2 本轮预登记验收

- 使用 P7 YCB `005_tomato_soup_can` 场景和眼在手 Gazebo RGB-D 相机；不启动 MoveIt、NBV、轨迹、夹爪或抓取编排器。静态观测生成态由版本化 URDF 的离线 FK/look-at 求解并写入 `p7_1_observation_initial_positions.yaml`；Gazebo 确定性稳态另以 `p7_1_observation_settled_positions.yaml` 冻结，预检先等待 `5.0 s` 仿真时间，再以 `0.002 rad` 阈值检查终值及稳定性。两者分开记录，不把请求态冒充实际态。
- 在一个全新隔离仿真会话中连续采集 5 个窗口，窗口间仿真时间至少相隔 `0.50 s`。
- 每窗四路消息最大时间戳差不得超过 `0.15 s`。
- 四路原始消息均以 ROS 2 CDR 保存；另存 RGB PPM、深度 NumPy 和 XYZ NumPy 便于人工审计。
- Gazebo 目标真值只用于 P7.1 可见性审计：目标质心必须投影到成像范围内，且 PointCloud 中至少 12 个有限点落入带 `8 mm` 审计余量的版本化目标圆柱代理。该真值不得发布为算法位姿，也不得进入 P7.2 输出。
- 5/5 窗口全部满足接口、同步、时间戳精确 TF 和目标可见性后，P7.1 才能记为 `PASS` 并冻结 sensor interface。

仅“话题存在”“收到一帧 PointCloud2”“adapter 状态为 fresh”或 Gazebo 日志显示 advertised，均不是 P7.1 通过证据。

复现入口：

```bash
test/run_p7_1_sensor_sim.sh
test/run_p7_1_sensor_gate_capture.sh <new-evidence-directory>
```

## 3. P7.2 位姿估计 Gate

输入只能来自已冻结的 P7.1 原始观测。输出必须包含目标 `SE(3)` 位姿、协方差和质量诊断；不得把 Gazebo GT 直接作为算法输出。验收至少报告平移误差、旋转误差、ADD/ADD-S、pose success rate，并完成轻/中/重遮挡分层。通过后冻结 `pose_estimator` 接口。

当前状态必须区分两类结论。非/轻/中遮挡已经形成完整单帧 `SE(3)` 估计证据；重遮挡 `v5` 仍因罐体纹理 yaw 不可观而安全拒绝，输出 `POSE_YAW_UNOBSERVABLE` 与保守协方差。交接验证器已确认该拒绝不会把不可抓取的位姿交给后续门，因此 P7.2 的**安全接口交接**已冻结为 `P7.3_REQUIRED_REOBSERVATION`；但重遮挡的完整单视图 `SE(3)` Gate 仍不通过。禁止以 GT、场景固定朝向或 identity quaternion 倒填旋转。

## 4. P7.3 NBV Gate

输入为当前位姿、协方差、融合点云和候选视点。正式实现必须使用真实几何 coverage、pose information gain、候选生成和停止准则；`coverage_proxy=sin(elevation)` 只能作为历史工程代理。不同观测状态应产生可解释的不同 NBV，移动后必须从新观测重新计算位姿不确定性。候选评分还必须剔除末端自遮挡的取景，并以冻结 PointCloud 的非目标表面做外遮挡射线审计；静态 FK 预检仅筛除明显不可构型，不构成 P7.4 的 MoveIt 通过证据。

2026-09-04 已冻结 severe-v5 番茄罐仿真验收链：修复目标中心 schema 的重复偏移后，几何/信息 NBV 从 240 个候选中得到 25 个严格静态可行候选，并选择 `geometry_nbv_e75_v-110_a22`。新视点的同步感知为 5/5，复位姿估计为 5/5 完整 SE(3) 成功，平移误差 1.124 mm、旋转误差 1.0°、ADD-S 1.008 mm，实测 yaw 标准差由不可观状态降至 2.5°，满足 5°停止阈值。正式结论为 `PASS / FROZEN`；该视点在本门中通过 Gazebo 静态初始状态实例化，MoveIt 轨迹执行仍归 P7.4。

## 5. P7.4 MoveIt Gate

每个候选依次执行 `FOV → workspace → IK → joint limit → collision → planning`。逐候选保留 feasible view、规划时间、关节路径长度、Cartesian 路径长度、重规划次数和失败原因。MoveIt 状态碰撞拒绝率与 Gazebo 物理接触碰撞率必须分开统计。

2026-09-05 已完成本门在 severe-v5 番茄罐场景的验收。25 个严格静态候选逐个进入 MoveIt 六级过滤，单 IK 分支 22 个碰撞拒绝、3 个规划通过；所选 a22 使用另一个独立验证的冻结关节分支，r9 实际规划与执行成功。相机终点误差 0.688 mm / 0.080°，规划 0.078895 s、重规划 0 次。原生 Gazebo 环境接触记录覆盖运动全程，未观察到机器人–地面/遮挡器/目标的非预期接触；物理自接触未被该传感器组观测，不作零碰撞声明。移动后 640×480 感知 3/3，完整 SE(3) 3/3（平移 0.377 mm、旋转 1°、ADD-S 0.574 mm）。完整索引见 `docs/evidence/p7_4_gate_summary_20260904/`。历史同名 r9 的 E1 失败属于早期抓取记录，不与本次 2026-09-05 的 r9 混用。

## 6. P7.5 抓取闭环 Gate

闭环顺序固定为：

```text
Observe → Pose → NBV → Move → Observe → Pose update
→ Grasp generation → Feasibility → Pregrasp → Close → Lift → Hold
```

最终成功必须同时满足：抓取目标、无非预期物理碰撞、抬升不少于 `0.10 m`、保持不少于 `2 s`。attachment-assisted kinematic simulation 只能作为机制诊断；没有接触/碰撞证据时不得声明“P7 主动视觉可靠抓取闭环通过”。

2026-09-06 r12 已完成 severe-v5 番茄罐单场景抓取闭环：NBV 后新观测 3/3，完整
SE(3) 位姿更新 3/3（平移误差 1.092 mm、旋转误差 1.0°、ADD-S 0.995 mm），
MoveIt 预抓取、接近和反向预抓取抬升均执行成功；双指接触持续被观测，物体
中心上升 0.140928 m，仿真时间保持 2.599 s，高度漂移 0.000000672 m。该结论
仅冻结为“单 frozen 场景 P7.5 仿真 PASS”，不替代正式抓取主实验。

## 7. 当前状态（2026-09-06）

| 门禁 | 状态 | 依据与下一步 |
| --- | --- | --- |
| P7.1 感知输入 | `PASS / FROZEN` | 正式 YCB r7：5/5 窗口通过，目标点数 110--112（门槛 12），四路布局、同步、消息时刻 TF、目标投影与 RGB ROI 均有效；`trajectory_commands_sent=0`，全部校验和通过 |
| P7.2 位姿估计 | `PASS / INTERFACE FROZEN` | 非/轻/中遮挡单视图完整估计正常；重遮挡 v5 初始视图 5/5 以 `POSE_YAW_UNOBSERVABLE` 拒绝并触发 P7.3，r13 主动复观测后 5/5 完整 SE(3) 成功。GT 仅用于冻结后独立评估 |
| P7.3 NBV | `PASS / FROZEN` | 使用真实 CAD coverage、yaw Fisher 信息、末端自遮挡与实测外遮挡；未使用历史 `sin(elevation)` 代理。r13 在 240 个候选中筛得 25 个严格静态可行候选，所选 e75/v-110/a22 复观测 5/5，平移误差 1.124 mm、旋转误差 1.0°、ADD-S 1.008 mm，yaw 标准差 2.5°并触发停止准则。权威证据：`docs/evidence/p7_3_gate_summary_20260904/` |
| P7.4 MoveIt | `PASS / FROZEN` | severe-v5 单场景：25 候选批筛、冻结分支实际执行、命令至回执环境接触观测、执行后感知/位姿更新全部完成。规划端→实际相机端误差 0.688 mm / 0.080°；不是多场景可靠性结论 |
| P7.5 抓取闭环 | `PASS / SINGLE-SCENE` | r12 severe-v5 番茄罐：感知条件下生成 top-grasp，MoveIt 预抓取/接近/抬升成功，双指接触持续观测，抬升 0.140928 m，保持 2.599 s；不是正式多场景抓取矩阵，也不是真机结论 |
