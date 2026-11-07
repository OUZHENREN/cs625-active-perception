---
title: "CS625 精密光学元件可靠操作：10 月中旬任务总览与仿真实验设计 v2"
date: 2026-09-19
version: v2
status: active
supersedes: 2026-09-19_CS625_10月中旬任务总览与仿真实验设计_v1.md
project: cs625-active-perception
type: PLAN
tags:
  - CS625
  - 精密光学模块
  - RGB-D
  - 主动视觉
  - MoveIt
  - Gazebo
  - RVS
  - 仿真实验
  - 可靠性评价
  - 力觉
---

# CS625 精密光学元件可靠操作：10 月中旬任务总览与仿真实验设计 v2

> **近期唯一主线（不变）：**
>
> **10 月中旬前，把「新光学元件任务场景 → 仿真 RGB-D/点云 → 固定视角定位与抓取基线 → 小规模主动重观测验证」这条主链从"单场景手工脚本可跑"升级为"可配置、可批量、可无人值守"的活链路；真机完成只读→规划→空载→已知位姿→固定视角视觉抓取；力觉按情形 B 处理，只做接口核查 + 仿真侧接触状态机，不做力控开发。**

---

## v1 → v2 变更摘要

| # | v1 的说法 | v2 的修正 | 依据 |
|---|---|---|---|
| 1 | 「P7.5 单场景成功、45 个矩阵全部失败」当成同一个谜题 | **这是两套代码、两个执行器、两种目标**；45 矩阵属已废弃的遗留链路 | 45 矩阵走 `sim_p4_loop` → `sim_view_executor`；P7.5 走 P7 链。同一天（2026-09-06） |
| 2 | §3.1 用「3 场景 × 15 层」排查 45/45 共性故障 | **失败已在执行层定位，且修复已在工作树中**；先做半天复跑验证 | 45 集全部 `EXECUTION_FAILED`，`sensor_settle_wait_time_sec=0.0`；`sim_controllers.yaml` 的 `constraints` 与证据同一 commit |
| 3 | Gate 3/4/5 是"待做目标" | **P7.1–P7.5 已在仓库中冻结通过**；真正缺的是"活链路化" | `docs/p7_five_gate_protocol.md` 状态表 |
| 4 | Gate 0–6 独立命名 | 与既有 P7.x 做**显式映射**，重命名为 M1–M4 迁移任务 | 避免两套门禁并存 |
| 5 | 「CS625 自带六维力传感器」 | **按工作假设 B：只有关节电流估计，无腕部六维力**；现场判定 + 决策时限 | 见 §2 |
| 6 | 主实验 30/50 seeds，未给功效核算 | **主比较冻结在 O2**；给出最小可检出差异（MDD）表 | 见 §9 |
| 7 | S1/S2 视点预算未对齐 | 新增**实验公平性约束**（共享 `max_views`） | 见 §6 |
| 8 | 未处理多重比较 | 预注册 ≤3 个确证性检验 + Holm 校正 | 见 §9.3 |
| 9 | `experiments/` 放仓库根目录 | 配置落 `src/cs625_bringup/config/`；`runs/`、`plots/` 入 `.gitignore` | AGENTS.md 配置约定 + 禁止提交生成物 |
| 10 | 新对象建模压在 9/19–9/22 四天内 | CAD 今晚到；建模与链路集成**解耦并行** | 见 §14 |

**v1 中被完整保留的部分**（这些是 v1 的强项，不改）：三种实验的区分（工程调试 / Pilot / 正式）、配对 scene seed 设计、"不要用确定性场景凑样本量"、失败码字典、`重观测恢复率 RR` 作为核心指标、失败结果不得删除、拒绝把单次成功当可靠性、图 A–F 的数据预留。

---

# 0. 本文档解决什么问题

统一 2026-09-19 至 10 月中旬的开发、实验和数据记录：

1. 近期仿真实验怎么安排；
2. 哪些指标必须记录，哪些只是诊断量；
3. 需要跑多少组才能从"工程调通"升级到"论文可用数据"；
4. 力觉在情形 B 下怎么落地（v2 新增核心）；
5. 近期仿真、RVS、PS800-E1、CS625、RViz、力觉分别做到什么程度；
6. 如何从现在开始保存数据，避免后续改日志格式。

> **实验数量不能替代方法贡献。** 论文能否成立取决于：研究问题是否清楚、对比是否合理、条件是否可复现、指标是否与任务相关、统计是否可信、失败是否保留、仿真与真机是否形成证据闭环。

---

# 1. 对 v1 的事实性修正（必读）

## 1.1 修正一：45/45 与 P7.5 不是同一条链

| | 45 矩阵 | P7.5 |
|---|---|---|
| 日期 | 2026-09-06 | 2026-09-06 |
| 入口 | `sim_p4_loop.launch.py` + `sim_view_planning.launch.py` | P7 测试脚本链 |
| 执行器 | `cs625_motion_adapter/sim_view_executor.py` | `cs625_motion_adapter/p7_arm_motion_adapter.py` |
| 候选指标 | `coverage_proxy = sin(elevation)` | 真实几何 coverage + yaw Fisher 信息 + 自遮挡审计 |
| 终点 | 三次视角观测 | 物理接触抓取闭环 |
| 结果 | 45/45 `MAX_FAILED_ATTEMPTS` | 抬升 0.140928 m、保持 2.599 s |

仓库自己的 P7.3 协议已明确：`coverage_proxy=sin(elevation)` **只能作为历史工程代理**。

> **结论：45 矩阵是已被取代的遗留链路，不是"当前系统的批量失败"。** v1 把它列为 10 月头号诊断任务，方向错误。

## 1.2 修正二：45/45 的失败原因已定位，修复已在工作树中

逐条证据（`docs/evidence/minimum_showcase_matrix_20260906_r1/`）：

```json
// episodes/occlusion_light_coverage_nbv_seed17_p4_loop.json
"termination_reason": "MAX_FAILED_ATTEMPTS",
"view_records": [
  {"code": "EXECUTION_FAILED", "trajectory_execution_time_sec": 58.9997, "sensor_settle_wait_time_sec": 0.0},
  {"code": "EXECUTION_FAILED", "trajectory_execution_time_sec": 24.3499, "sensor_settle_wait_time_sec": 0.0},
  {"code": "EXECUTION_FAILED", "trajectory_execution_time_sec": 50.3498, "sensor_settle_wait_time_sec": 0.0}
]
```

```text
summary/failure_codes.csv
coverage_nbv    EXECUTION_FAILED 36   PLANNING_FAILED 9
fixed_view      EXECUTION_FAILED 32   PLANNING_FAILED 13
random_reachable EXECUTION_FAILED 27  PLANNING_FAILED 16  RESULT_TIMEOUT 2
```

```text
summary/strategy_summary.csv（三种策略一致）
mean_successful_observation_rate = 0.0
mean_sensor_settle_wait_time_sec_total = 0.0
```

推理链：

1. `sim_view_executor.py:228` 的 `EXECUTION_FAILED` 只有一个触发条件——`FollowJointTrajectory` 结果码 ≠ `SUCCESSFUL`，即**控制器自行 abort**。
2. `execution_timeout_sec=90`、`max_velocity_scale=1.0`，单次却跑了 24–59 s → 不是超时、不是速度缩放，是**跟踪判定**问题。
3. `sensor_settle_wait_time_sec` 全为 0 → **整批 45 集从未进入传感器稳定阶段，也就从未产生任何一次观测**。这批证据里没有感知结论，只有执行结论。
4. `src/cs625_bringup/config/sim_controllers.yaml` 现存这段（注释几乎就是故障描述）：

```yaml
constraints:
  stopped_velocity_tolerance: 0.01
  goal_time: 5.0          # Gazebo 的 action monitor 会晚一个仿真秒才看到到位
  shoulder_pan_joint: {trajectory: 10.0, goal: 0.01}
  ...
```

5. `git log -S "goal_time" -- src/cs625_bringup/config/sim_controllers.yaml` → 该段由 **`d79713b`（2026-09-06）** 引入，**与 45 矩阵证据同一 commit**。

> **动作：半天复跑 3 个代表 cell（约 2 分钟/cell）验证"约束修复已生效"这一工作假设。不要按 v1 做 3 场景 × 15 层。**
>
> **附带纪律：** `matrix_manifest.json` 只有 scene/seed/strategy，**没有 git revision、没有 config hash**，导致这批证据无法严格复现。以后所有批量证据的 manifest 必须写入 `git HEAD` + 配置文件 sha256（P7 系列已做到，P4 系列没有）。

## 1.3 修正三：P7.1–P7.5 已经冻结通过，缺的是"活链路"

`docs/p7_five_gate_protocol.md` 当前状态：

| 门禁 | 状态 | 已达成 |
|---|---|---|
| P7.1 感知输入 | `PASS / FROZEN` | 5/5 窗口；四路布局、同步、消息时刻 TF、目标投影与 RGB ROI 全部有效 |
| P7.2 位姿估计 | `PASS / INTERFACE FROZEN` | 非/轻/中遮挡完整单帧 SE(3)；重遮挡安全拒绝 `POSE_YAW_UNOBSERVABLE` 并触发重观测 |
| P7.3 NBV | `PASS / FROZEN` | 240 候选 → 25 严格静态可行 → 选中 `e75/v-110/a22`；复观测 5/5，平移 1.124 mm、旋转 1.0°、ADD-S 1.008 mm，yaw 标准差 2.5° 触发停止准则 |
| P7.4 MoveIt | `PASS / FROZEN` | 25 候选六级过滤、冻结分支实际执行；相机终点误差 0.688 mm / 0.080° |
| P7.5 抓取闭环 | `PASS / SINGLE-SCENE` | r12 severe-v5 番茄罐：抬升 0.140928 m、保持 2.599 s、双指接触持续观测 |

**但这里有一个真实且关键的缺口，也是 v1 唯一说对了的地方：**

> `cs625_target_perception` 包里**只有 `target_pose_relay.py`（纯转发，无估计器）**。真正的估计器在 `test/p7_2_estimate_textured_pose.py`（955 行）等**离线脚本**中，靠 40+ 个 `test/run_p7_*.sh` 手工串起来。
>
> **能力已验证，链路是"手工脚本级"，不是"可配置 pipeline 级"。**

## 1.4 门禁映射（取代 v1 的 Gate 0–6）

```text
Gate 0  机械任务定义        → 新增；v2 保留             [本次]
Gate 1  感知输入            → P7.1   PASS/FROZEN        [沿用，不重做]
Gate 2  已知位姿执行        → P7.4/P7.5 部分覆盖        [需补 transport→preassembly]
Gate 3  固定视角定位        → P7.2   PASS/INTERFACE     [缺"活节点化"]
Gate 4  固定视角视觉抓取    → P7.5   PASS/SINGLE-SCENE  [缺"多场景 / 可配置"]
Gate 5  单次主动重观测      → P7.3+P7.5 组合已单场景通过 [缺"活链路闭环"]
Gate 6  Pilot batch         → 新增                      [缺"自动化 + 无人值守"]
```

迁移任务重命名（v2 用它作为执行单元）：

```text
M1  活链路化      P7.2 估计器 + P7.3 NBV 从 test/ 脚本提升为 ROS 节点，
                  接口沿用已冻结 schema
M2  已知位姿执行基线（含 transport → preassembly）
M3  固定视角视觉抓取活链路（estimated pose → grasp → 完整任务）
M4  单次主动重观测活链路闭环（first view reject → 候选 → 移动 → 新帧 → 重新定位 → 抓取）
```

---

# 2. 力觉专项结论与预案（v2 新增核心）

## 2.1 工作假设

> **本文档按情形 B 制定：CS625 无腕部六维力传感器，`get_tcp_force` 类读数来自关节电流的通用力估计（或关节力矩反解）。**
>
> 这是**工作假设，不是已确认事实**。现场判定流程见 §2.4，判定结果写入 `docs/real_hardware_readiness.md`。

## 2.2 核实证据

### 已确认（本地可复核）

| 位置 | 内容 |
|---|---|
| `~/elite-sdk-1.2.0/include/Elite/RtsiIOInterface.hpp:210` | `getAcutalTCPForce()` — "Generalized forces in the TCP. (Subtract the force data caused by the load.)" |
| `RtsiIOInterface.hpp:128` | `setExternalForceTorque(vector6d_t)` — "Used to input **external** force sensor data… takes effect when `ft_rtsi_input_enable` is set to true" |
| `EliteDriver.hpp:259` | `zeroFTSensor()` — 去皮；注释说明数值来自 `get_tcp_force(True)`，已做负载补偿 |
| `EliteDriver.hpp:305/314` | `startForceMode()` / `endForceMode()` — 内置力控模式（参考系、selection_vector、目标力、速度上限、`ForceMode` 枚举） |
| `EliteDriver.hpp:276` | `setPayload(mass, cog)` — 负载质量与质心 |
| `eli_cs_robot_driver/src/hardware_interface.cpp:474` | 每周期 `ft_sensor_measurements_ = rtsi_interface_->getAcutalTCPForce();` |
| `hardware_interface.cpp:141-151` | 注册状态接口 `tcp_fts_sensor/force.x…torque.z` |
| `hardware_interface.cpp:776` | `transformForceTorque()` — 基座系力旋量旋到 TCP 系 |
| `eli_cs_robot_driver/launch/elite_control.launch.py:305` | `force_torque_sensor_broadcaster` **默认 active 启动** |
| `eli_cs_robot_description/urdf/cs.ros2_control.xacro:133` | `<sensor name="${tf_prefix}tcp_fts_sensor">`；生成 URDF 含 `ft_frame` |

### 一个必须先修的接口错误

同一个传感器在三处有**三个不同的名字**：

```text
URDF / ros2_control xacro                      →  tcp_fts_sensor
eli_cs_robot_driver/config/cs625_controllers.yaml →  sensor_name: cs_ft_sensor
eli_cs_robot_driver/config/force_torque_sensor_broadcaster.yaml → sensor_name: "ft_sensor"
```

`force_torque_sensor_broadcaster` 按 `<sensor_name>/force.x` 拼接口名，**三者最多一个能对上**。

> **推论：`/ft_data` 极大概率从未真正跑通。"配置里写了" ≠ "数据流过"。**

**修法（不原地改 vendor）：** 由本仓库 `real_base.launch.py` 用自己的参数启动 broadcaster，`sensor_name: tcp_fts_sensor`，并登记到 `docs/dependencies.md`。这符合 AGENTS.md 的 "profile data, not vendor code"。

### 厂商定位（弱证据，仅作旁证）

艾利特官方产品线将 **CS 系列**（CS63…CS630）与 **CSF 力控系列**（CS63F…CS630F）并列为两个系列；其力控技术文章称 CSF "在各关节内嵌高精度六维力/力矩传感器（CS66F：±0.75 N / ±0.05 Nm），全量程精度 0.1 %"，并称碰撞检测灵敏度"相比**非力控机型**提升一个数量级"。

> 证据强度说明：该文本来自 `elibot.com/tideflow/` 内容营销子站，页脚声明"内容由网络用户投稿"，**不能当规格书使用**，只能说明厂商的产品定位。官方选型手册与 CS 用户手册均为 PDF，本轮未能取到正文。

## 2.3 需求分层：你实际需要哪一级

| 层级 | 回答什么问题 | 力量程 | 力分辨率/噪声 | 力矩 | 采样率 |
|---|---|---|---|---|---|
| **L1 接触/碰撞检测** | 碰没碰到 | 50–200 N | 噪声 ≤0.5 N，阈值 1–5 N | 粗 | ≥100 Hz |
| **L2 接触状态识别** | 单指/双指、卡阻、偏斜、粘滞 | 50–200 N | ≤0.2 N | ≤0.01 Nm | ≥100–200 Hz |
| **L3 柔顺插装** | 0.05–0.1 mm 间隙对中 | 50–100 N | 0.05–0.2 N | 0.005–0.02 Nm | ≥500 Hz |

现状：`cs625_controllers.yaml` 的 `update_rate: 125` → **125 Hz**，满足 L1、勉强 L2、**不够 L3**。

**本方案的任务定义是"抓取 → 抬升 → 搬运至装配预备位置"**（§3.1），因此真实需求是 **L1 + L2**：

- 夹爪闭合时的接触检测（是否真的夹住）；
- 搬运过程中的掉落/滑移检测（力/力矩突变）；
- 碰撞安全停止；
- 到达装配预备位置时的接触/贴合检测。

**L3 的"精密插装"已被主动排除**，所以近期不需要 500 Hz 级力控。

## 2.4 现场判定流程（9/28 第一件事，10 分钟）

做成 `test/real_ft_probe.sh`，现场照着跑：

```bash
# 1) 铭牌与示教器（拍照留证）
#    - 本体铭牌：CS625 还是 CS625F
#    - 示教器 设置→力/力矩：是否存在"力传感器""零点标定"项

# 2) 接口存在性（一定会存在，不构成证据）
ros2 control list_hardware_interfaces | grep -i tcp_fts

# 3) 用正确 sensor_name 起 broadcaster
ros2 run controller_manager spawner force_torque_sensor_broadcaster \
  --controller-manager /eli_ros2_control_node \
  --param-file <本仓库 yaml, sensor_name: tcp_fts_sensor>

# 4) ★静止 10 s 噪声底：力与力矩的 std、峰峰值
ros2 topic hz /ft_data

# 5) ★挂已知质量（500 g）→ Fz 应 ≈ 4.90 N
#    setPayload(gripper_mass, cog) 前后各测一次

# 6) ★纯力矩测试：夹爪侧向加力臂 L、加已知力 F → Mz 应 ≈ F·L
#    关节电流估计在这一步会明显失真或线性度差

# 7) zeroFTSensor() 前后读数变化；控制器重启后是否复位
```

**判据：**

| 观察 | 结论 |
|---|---|
| 步骤 5 偏差 <5 % 且步骤 6 线性 R²>0.95 | 真实腕部六维力 → 情形 A/C，L1+L2 可用 |
| 步骤 5 偏差 >20 %，或步骤 6 严重非线性/噪声大 | **关节电流估计 → 情形 B（当前工作假设）**，只有 L1 勉强可用 |
| `/ft_data` 全零且 step 3 报接口找不到 | 先修 §2.2 的 `sensor_name` 不一致，再重测 |

## 2.5 预案与决策时限

### 情形 B（当前假设）：只有关节电流估计

- L1 可用（碰撞/接触事件），L2 降级，**L3 不可做**。
- **首选补救：法兰外置六维力 + `setExternalForceTorque` 回灌。**
  - SDK 已支持：`ft_rtsi_input_enable = true` + `setExternalForceTorque()` → 控制器内部的 `get_tcp_force` 与 `force_mode` 均改用外部数据。
  - 即：外置传感器不仅能读数，**还能直接驱动机器人内置力控模式**。这是技术上最干净的一条路，且不需要自研阻抗控制器。
  - **采购周期通常 4–8 周**（宇立 SRI / 坤维 / ATI / 鑫精诚）。
- **不采购的降级方案：** 研究内容三 收窄为**"接触事件检测与安全停止"**——用「关节力矩突变 + 夹爪电流/位置误差 + 轨迹跟踪偏差」做接触检测，柔顺降级为"基于位姿修正的准柔顺"。论文口径相应改为"接触感知"而非"力觉柔顺"。

> **决策时限（硬）：**
>
> ```text
> 9/29  现场判定出结论
> 9/30  完成询价（避开国庆假期）
> 10/09 前下单，否则研究内容三滑出本学期
> ```
>
> **若不采购，须在 10 月中旬前与导师确认研究内容三的口径降级。**

### 情形 A/C（若现场判定推翻假设）

- 按 L1+L2 推进；力控用 `startForceMode()`，不自己写阻抗控制器（符合 09-18 计划"不以复杂学习型力控为目标"）。
- 仍需完成 §2.2 的 `sensor_name` 修正。

## 2.6 不依赖现场结论就能做的工作（现在就做）

> world 里已有 Gazebo contact 传感器（`p7_ycb_tomato_occlusion_severe_v5.sdf`）：
> `p7_target_contact` / `p7_occluder_contact` / `p7_ground_contact`，`update_rate=100`，
> 并有 `test/p7_gazebo_contact_monitor.cpp` 与 `p7_episode_contract.py` 中的
> `UNEXPECTED_COLLISION` 失败码。

因此可以**先把接触状态机的算法与归一化接口在仿真里跑通**，真实 F/T 只做为可替换后端：

```text
新增 cs625_force_adapter（与 cs625_sensor_adapter 对称）
  归一化话题 /sensors/force/wrench
  后端 1（仿真）：Gazebo contact sensors
  后端 2（真机）：RTSI F/T（无论是不是真实传感器）
  状态机：CONTACT_NONE / CONTACT_LEFT / CONTACT_RIGHT / CONTACT_BOTH
          / CONTACT_UNEXPECTED / SLIP_DETECTED / OVERLOAD
```

**收益：9/28 的判定结果不会阻塞 10 月的工作。**

---

# 3. 10 月中旬应达到的状态

## 3.1 必须完成

1. 光学元件、夹爪、装配/放置位置和遮挡物进入 Gazebo 与 MoveIt 场景（Visual / Collision / Inertial 齐备，mm→m 正确）。
2. 仿真相机稳定发布 RGB / Depth / CameraInfo / PointCloud2，且镜头移动后产生新帧。
3. 已知位姿机械执行基线跑通并**明确命名**：

```text
预抓取 → 接近 → 闭合 → 抬升 → 搬运 → 放置/装配预备位置
```

4. **M1**：把 P7.2 估计器与 P7.3 NBV 从 `test/` 脚本提升为 ROS 节点，跑通固定视角定位并送入抓取规划。
5. **M4**：完成一次最简单的主动重观测活链路闭环。
6. 完成真实 PS800-E1 / RVS / CS625 的接口盘点（含 §2.4 力觉判定）。
7. 两趟行程完成：只读连接 → planning-only → 低速空载 → 已知位姿最小抓取 → 固定视角视觉最小抓取。

## 3.2 尽量完成

- M4 的多 seed 稳定性；单 seed 无人值守端到端；
- Pilot 前 10 个配对 seed；
- 对比固定视角 / 预设补充视角 / 任务约束补充视角；
- 真实 RGB-D/点云离线跑定位算法；
- RVS 输出与 ROS 实验记录一一对应。

## 3.3 明确不做

- 完整视觉—力觉柔顺装配、复杂阻抗/导纳控制（**情形 B 下不可做**）；
- 自研深度学习位姿估计网络；
- 完整 RViz 操作面板、CS625 示教器插件；
- RVS 内部大规模自定义视觉算法；
- 大规模正式论文实验矩阵；
- 动态目标抓取、多次连续 NBV、完整数字孪生闭环。

---

# 4. 三种实验的区分

> 后续最大的风险是把三种性质完全不同的实验混在一起。

| | 工程调试 | Pilot 预实验 | 正式论文实验 |
|---|---|---|---|
| **目的** | 验证链路有没有坏 | 定阈值、方差、失败分布、样本量 | 回答方法是否稳定提高成功率并解释原因 |
| **统计结论** | 不做 | 不做强结论 | 做 |
| **人工选场景** | 允许 | 不允许 | 不允许 |
| **查看 GT** | 允许 | 仅评价 | 仅评价 |
| **随机 seed** | 不用 | 用 | 用（预生成，冻结） |
| **日志格式** | 任意 | **必须与正式一致** | 冻结 |
| **失败处理** | 定位故障 | 不得删除 | 不得删除 |

**正式论文实验的额外要求：** 冻结代码版本、冻结参数阈值、预生成 scene seed、各方法共享同一组 seed、不因方法失败临时换场景、不静默删除异常、主要终点与统计方法在实验前确定。

---

# 5. 近期仿真实验安排

## 5.1 阶段 A：45/45 复跑验证（半天，原为 3–5 天）

不再做逐层排查。直接执行：

```text
1. 重跑 3 个代表 cell（1 已知曾观测成功 + 1 轻遮挡 + 1 重遮挡）
2. 判据：是否仍有 EXECUTION_FAILED
   - 无 → 假设成立，45 矩阵归档为"遗留链路已修复"，不再投入
   - 有 → 才进入执行层定向诊断（对比 p7_arm_motion_adapter 的成功路径）
3. 记录本次 git HEAD + 配置文件 sha256 到 manifest
```

**验收：** 得出"遗留链路是否已修复"的明确结论，并补齐 manifest 的版本信息。

## 5.2 阶段 B：3 × 3 小矩阵

```text
遮挡等级：O0 无遮挡 / O1 轻遮挡 / O2 重遮挡
目标初始布局：P0 中央标准 / P1 平移扰动 / P2 姿态扰动
```

先固定一种策略跑 9 scenes；9 个均可完整结束后，加入三种策略 → 27 episodes。

**本阶段仍属工程验收，不用于正式论文统计。**

## 5.3 阶段 C：Pilot 预实验

三种策略（接口统一、候选集共享）：

| | 定义 | 作用 |
|---|---|---|
| **S0 Fixed View** | 固定初始视角 → 定位 → 达标则抓取，否则失败退出 | 最低基线 |
| **S1 Preset Re-observation** | 定位不达标 → 移动到**预先人工设定的第二视点** → 重新定位 → 抓取 | 证明"多看一次"本身是否已足够 |
| **S2 Task-constrained Re-observation** | 定位不达标 → 生成候选 → FOV/workspace/IK/joint limit/collision/planning 过滤 → 按任务相关评分选点 → 重新观察 → 定位 → 抓取 | 本文真正的方法增量 |

**Pilot 数量：**

```text
1 个光学元件 × 3 遮挡等级 × 3 策略 × 10 配对 scene seed = 90 episodes
另加 5~10 个专用于失败诊断的极端 seed
合计约 90~120 个完整 episode
```

> 10 个 seed 是 **pilot 数量，不是最终论文样本量**。

**S1 的预设第二视点按遮挡等级定义（O0/O1/O2 各一个），不按 seed 定义**，规则在实验前写入协议，避免事后挑选。

**Pilot 启动的前提（Gate M5）：**

```text
单 seed 无人值守端到端跑通
  → 无人工介入
  → 所有失败有 failure code
  → 可自动汇总 CSV
```

这是 Pilot 的真正难点，不是集数。

---

# 6. 实验公平性约束（v2 新增）

> 这是 v1 缺失、且是 NBV 类工作最容易被审阅者攻击的一点。

**S1 与 S2 必须共享同一视点预算。** 如果 S2 允许 3 次重观测而 S1 只有 1 次，测到的是"看得多"而不是"看得准"。

```text
硬约束：
  max_views            = 3   （三策略一致）
  max_failed_attempts  = 3   （三策略一致）
  候选集                = 完全相同的 source candidate set
  质量门禁阈值          = 完全一致
  任务成功判据          = 完全一致
```

复用 `cs625_view_evaluation/sim_episode_coordinator.py` 中既有的 `max_views` / `max_failed_attempts` 机制。

**同时记录"单次恢复代价"**，用于回答"多看很多次当然更容易成功"：

```text
C_recover = extra_motion_cost / number_of_recovered_tasks
mean_views_per_episode / median_views_per_episode
```

---

# 7. 场景随机化与遮挡等级

## 7.1 不要用"重复同一个确定性场景"凑样本量

Gazebo 完全确定时，重复 50 次相同输入只是重复，不是 50 个样本。**每个 seed 必须真正改变至少一部分受控变量。**

| 随机化项 | 内容 |
|---|---|
| 对象位姿扰动 | x / y 平移小范围、yaw，必要时 pitch / roll 小范围（范围由真实工装允许范围决定，不随意设置） |
| 遮挡物参数 | 位置、高度、横向偏移、与目标距离、遮挡关键抓取区域的比例 |
| 传感噪声 | depth noise、少量 missing depth、点云离群点、相机外参小扰动（**正式鲁棒性实验再加，10 月中旬前不必全开**） |

## 7.2 配对设计

所有策略使用**完全相同**的 scene seed：

```text
scene_seed = 001
  S0 fixed_view
  S1 preset_reobserve
  S2 task_constrained_reobserve
```

比较的是"同一个困难场景下不同策略的差异"，比各自随机生成更有统计效率。

## 7.3 遮挡等级必须有可重复定义

至少记录：

```text
visible_ratio_target        目标整体可见比例
visible_ratio_grasp_roi     关键抓取区域可见比例
valid_depth_ratio           目标投影区域内有效深度比例
```

第一版先根据几何真值离线计算。等级暂定 `O0 高可见 / O1 中等遮挡 / O2 重遮挡`，**正式阈值需依据实际数据分布冻结，不要现在写死百分比**。论文应报告实际 `visible_ratio` 分布，而不只写"轻/中/重"。

---

# 8. 指标体系

## 8.1 主终点：最终任务成功率

```text
定位得到对象位姿
+ 生成有效抓取
+ MoveIt 规划成功
+ 无非预期碰撞
+ 夹爪成功抓取
+ 抬升 ≥ 0.10 m
+ 保持 ≥ 2 s
+ 若包含搬运，则到达装配预备位置容差内
```

```text
SR_task = N_task_success / N_all_episodes
```

> **主分母必须是全部有效启动的 episode。** 不能只对"定位成功的样本"计算成功率。

## 8.2 感知/定位指标

```text
平移误差  e_t = || t_hat - t_gt ||_2          单位 mm
姿态误差  e_R = arccos((tr(R_hat R_gt^T) - 1)/2)  单位 degree
```

**对称对象：** 不只报告普通旋转误差，额外使用 ADD-S / MSSD，或自定义 symmetry-aware task pose error。BOP 评估体系使用 VSD、MSSD、MSPD 处理可见表面与对称性，其中 MSSD 的"最大表面偏差"对机械操作尤其有解释意义。

**定位成功率必须由任务容差定义，而不是 ICP RMSE：**

```text
pose_success =
    translational_error <= task_position_tolerance
AND rotational_error    <= task_orientation_tolerance
AND grasp_pose_feasible == true
```

阈值未冻结前，完整保存误差并绘制多阈值结果（平移 2/5/10 mm，旋转 2/5/10 deg），最终依据夹爪与装配预备位置容差冻结。

## 8.3 点云/配准质量指标

```text
target_point_count
valid_depth_ratio
inlier_ratio
icp_rmse
fitness
multi_init_consistency
pose_temporal_stability
```

**ICP RMSE 只作诊断量，不能独立决定"抓取可用"。** 对称对象的错误姿态也可能获得很小的 ICP RMSE。

## 8.4 主动重观测指标

```text
初次通过率  P_first-pass = N_first_view_pass / N
重观测恢复率 RR = N_reobserve_recovered / N_first_view_failed     ← 核心指标
成功率增益  ΔSR = SR_active - SR_fixed    （必须同时给 95% CI）
平均观察次数 mean / median views_per_episode
单次恢复代价 C_recover = extra_motion_cost / N_recovered
```

`RR` 直接回答：**主动视觉到底救回了多少原本无法抓取的场景？** 这是本课题最合适的核心指标。

## 8.5 候选视点机制指标

```text
S(v) = w_v·V_task(v) + w_g·G_new(v) - w_m·C_motion(v)
       - w_r·C_grasp(v) - w_c·R_collision(v)
```

每个 candidate 记录：

```text
candidate_id / camera_pose
task_visibility_score / new_information_score
motion_cost / grasp_transition_cost / collision_risk
ik_ok / collision_ok / planning_ok
final_score / selected
```

若要证明评分有效，后续做「预测评分 vs 真实执行后获得的定位/抓取改善」，可用 Spearman rank correlation、Top-1/Top-k 命中。**10 月中旬前先把字段留好。**

> **权重纪律：** 权重在 Pilot 阶段确定后即冻结，正式实验不得再调；并用消融 A1/A2 报告敏感性。

## 8.6 MoveIt / 可执行性指标

```text
IK_success / joint_limit_pass / self_collision_pass / environment_collision_pass
planning_success / planning_time
joint_path_length / cartesian_path_length / minimum_clearance / replanning_count
```

**必须区分 `MoveIt collision rejection` 与 `Gazebo physical collision`，两者不是一回事。**

## 8.7 任务级指标

```text
grasp_plan_success / approach_success / gripper_close_success
lift_success / hold_success / transport_success / preassembly_reached
task_success / total_task_time
```

若暂时只做到搬运至装配预备位置，主任务成功定义到 `preassembly_reached` 为止。**不要为了显得完整强行写成精密插接成功。**

## 8.8 力觉与接触指标（v2 新增）

```text
ft_available               （bool；现场判定结果）
ft_source                  （wrist_6axis / joint_current_estimation / external_rtsi / sim_contact）
ft_noise_std_force         （N，静止 10 s）
ft_noise_std_torque        （Nm）
ft_rate_hz                 （实测）
ft_payload_compensated     （是否在夹持前后切换 setPayload）
ft_zero_drift              （zeroFTSensor 后 60 s 漂移）
contact_event_count
contact_state_timeline     （CONTACT_NONE → CONTACT_BOTH → …）
slip_detected              （bool）
unexpected_collision       （bool，来自 contact monitor）
peak_contact_force         （仿真可用 Gazebo contact；真机按情形）
```

**区分声明：** 仿真侧接触指标来自 Gazebo 物理接触传感器，是**物理接触观测**；真机侧若为关节电流估计，**不得把估计值标注为力传感器测量**。这是 AGENTS.md 明确禁止的"把合成冒烟测试当作真实感知验证"。

## 8.9 失败模式（固定失败码）

```text
CAPTURE_FAILED / STALE_FRAME / TF_FAILED / TARGET_NOT_VISIBLE
LOCALIZATION_FAILED / QUALITY_REJECTED / NO_VIEW_CANDIDATE
IK_FAILED / COLLISION_REJECTED / PLANNING_FAILED
VIEW_EXECUTION_FAILED / REOBSERVATION_FAILED
GRASP_PLAN_FAILED / GRASP_EXECUTION_FAILED / GRASP_FAILED
LIFT_FAILED / HOLD_FAILED / TRANSPORT_FAILED
CONTACT_OVERLOAD / SLIP_DETECTED            ← v2 新增
TIMEOUT
```

每个 episode **只能有一个主失败码**，同时可有辅助诊断字段。失败结果不得静默删除。

---

# 9. 正式论文实验规模与统计功效

## 9.1 主比较冻结在 O2

> **确证性比较只做 O2（重遮挡）。** O0/O1 作为对照与趋势展示，不做确证性检验。

理由：本方法的物理假设就是"遮挡严重时主动重观测才有价值"。O2 的预期效应量远大于 20 pp，n=50/条件足够；O0 预期零效应，本来就不该用来撑统计。这同时使叙事更干净，避免"O0 上三策略差不多，你的贡献在哪"的质疑。

## 9.2 最小可检出差异（MDD）

配对 McNemar，80 % power，α=0.05 双侧（基线约 80 %，不一致率按 25 % / 35 % 两档）：

| 每条件 n | 不一致率 25 % | 不一致率 35 % |
|---:|---:|---:|
| 10 | 25.0 pp（退化，≈不可用） | 35.0 pp |
| 30 | 24.6 pp | 29.1 pp |
| **50** | **19.3 pp** | 22.9 pp |
| 100 | 13.8 pp | 16.4 pp |
| 200 | 9.8 pp | 11.7 pp |

不配对的两比例检验（每组 n，基线 80 %）：n=30 → 19.9 pp；n=50 → 17.5 pp；n=100 → 13.4 pp；n=200 → 10.0 pp。

Wilson 95 % 区间（真实成功率 80 %）：

| n | 95 % 区间 | 半宽 |
|---:|---|---:|
| 10 | 49.0 % ~ 94.3 % | ±22.7 % |
| 20 | 58.4 % ~ 91.9 % | ±16.8 % |
| 30 | 62.7 % ~ 90.5 % | ±13.9 % |
| 50 | 67.0 % ~ 88.8 % | ±10.9 % |
| 100 | 71.1 % ~ 86.7 % | ±7.8 % |

> **结论：**
> - n=10 只能做 Pilot，检不出任何小于 ~25 pp 的差异；
> - **n=50/条件 能可靠检出 ≈19 pp 及以上的差异** —— 这是 O2 主比较的下限；
> - 若要主张 10 pp 的提升，需要 **n≈200/条件**（总量 ~1800），是 v1 推荐量的 2 倍；
> - **不要现在把 30 或 50 说成"统计学上已经充分"。** 最终样本量应由 Pilot 得到的基线成功率、预期最小有意义提升、连续指标方差、配对不一致比例再做 power analysis 决定。

## 9.3 规模建议（修正后）

| 阶段 | 目的 | 数量 | 用于论文主统计 |
|---|---|---:|---|
| 45/45 复跑 | 关闭遗留链路 | 3 cells | 否 |
| 3×3 小矩阵 | 不同遮挡/布局均能跑通 | 9~27 | 否 |
| Pilot | 定阈值、成功率、失败分布、样本量 | **90** | 仅预实验 |
| 正式主实验·最低版 | 2 targets × 3 occlusion × 3 strategies × 30 seeds | 540 | 是（仅 O2 确证） |
| 正式主实验·推荐版 | 2 targets × 3 occlusion × 3 strategies × **50 seeds** | 900 | 是（仅 O2 确证） |
| 消融 | 2 ablations × 3 occlusion × 30 seeds × 2 targets | 360 | 是 |

**若只有单个目标：** 建议每条件提高到 50 个配对 seed，并补更系统的遮挡/位姿/噪声鲁棒性分析。

## 9.4 多重比较（v2 新增）

预注册 **≤3 个确证性检验**（全部在 O2）：

```text
C1: S2 vs S0   （方法 vs 最低基线）    ← 主检验
C2: S2 vs S1   （任务约束 vs 仅多看一次）← 方法增量的真正证明
C3: S1 vs S0   （重观测本身的价值）
```

其余（O0/O1 的比较、各消融、候选评分相关性）**全部标为探索性**，不参与主结论。确证性检验用 **Holm 校正**控制族错误率。

## 9.5 消融（等 S2 稳定以后再做）

| | 保留 | 去掉 | 回答 |
|---|---|---|---|
| **A1** | 可见性、新信息、运动代价 | 观察位姿→抓取位姿的衔接代价、抓取可执行性评分 | 任务约束是否真的必要？ |
| **A2** | 视觉/任务质量 | 运动/执行代价项 | 方法是否靠"大幅多走路"换来成功率？ |

```text
2 ablations × 3 occlusion × 30 paired seeds × 2 targets = 360 episodes
```

---

# 10. 统计分析方法

## 10.1 二项结果

task success / pose success / planning success / recovery success：

```text
成功数 / 总数、成功率、95 % Wilson CI
```

同一 scene seed 下比较两个策略**优先用配对分析（McNemar test）**。

> **不要只报告 p 值。** 还要报告：绝对成功率差、相对改善、95 % CI。

## 10.2 连续结果

translation error / rotation error / path length / task time / number of views：

```text
原始数据点 + 中位数/均值 + 95 % CI
```

分布偏态明显时用 median、IQR、paired Wilcoxon。条件与模型规范后可进一步做 repeated-measures ANOVA 或 mixed-effects model（**非近期必须项**）。

---

# 11. 数据保存

## 11.1 每个 episode 必须保存的结构化元数据

```text
episode_id / scene_id / seed / target_id / strategy / occlusion_level
target_gt_pose / occluder_gt_pose / initial_robot_pose
view_id / robot_pose / camera_pose / capture_timestamp
estimated_pose / pose_frame
translation_error / rotation_error / localization_quality
selected_view / candidate_scores
planning_result / execution_result / grasp_result
contact_state_timeline / ft_source / unexpected_collision      ← v2 新增
failure_code / task_success / timestamps
software_version / config_hash / git_head                       ← v2 新增
```

传感数据：RGB、Depth、CameraInfo、PointCloud / 可重建点云的数据。

## 11.2 不要每帧保存完整 PointCloud2

XYZRGB 每点约 16 bytes：640×480 一帧约 4.9 MB；1280×720 约 14.7 MB。

```text
1000 episodes × 3 views × 15 MB ≈ 45 GB
```

（还没算 RGB、Depth 和 rosbag。）

- **仿真：** 所有正式 episode 保存 RGB / Depth / CameraInfo / 相机位姿 / 必要关键帧点云。若点云可由 Depth + CameraInfo 完整重建，不必对所有中间帧重复保存 PointCloud2。
- **真实 PS800-E1：** **保留厂商/RVS 原始输出点云**——深度计算链可能由厂商 SDK 完成，后续无法用简单 pinhole 模型复现，且真实点云噪声本身也是研究证据。

## 11.3 rosbag 策略

```text
所有 episode       → 结构化 CSV/JSON + 关键 RGB/Depth/PointCloud
每个条件           → 保留 1~3 个完整 rosbag 作为审计样本
新 failure code 首次出现 → 保存完整 bag
论文典型案例       → 保存完整 bag
```

即 **5 %~10 % episode 保存完整通信流，其余保存结构化数据和关键帧**。

---

# 12. 仓库边界与文件组织（v2 修正）

## 12.1 与 AGENTS.md 的边界对齐

| v1 建议 | v2 修正 | 原因 |
|---|---|---|
| `<repo>/experiments/configs/*.yaml` | `src/cs625_bringup/config/`（`config/common` 放共享值，`config/sim` / `config/real` 放 profile 值） | AGENTS.md 配置约定 |
| `<repo>/experiments/runs/**` 直接生成 | 保留目录结构，但 **`runs/`、`plots/` 写入 `.gitignore`** | 禁止提交生成物 |

**建议的目录结构（在仓库外或 gitignore 内）：**

```text
experiments/                        ← 加入 .gitignore
├── scene_seeds/
│   ├── target_A_O0.csv
│   ├── target_A_O1.csv
│   └── target_A_O2.csv
├── runs/
│   └── 2026-xx-xx/
│       ├── ep_000001/
│       │   ├── metadata.json
│       │   ├── metrics.json
│       │   ├── rgb/  depth/  cloud/  debug/
│       └── ...
├── summary/
│   ├── episodes.csv      一行一个 episode
│   ├── candidates.csv    一行一个候选视点
│   └── failures.csv      失败模式整理
└── plots/
```

配置本身（策略、权重、阈值）落 `src/cs625_bringup/config/`，`experiments/` 只放 seed 表与结果。

## 12.2 环境入口（不变）

```bash
source scripts/source_dev_env.sh
```

链条固定，不得重排、缩短或混用其他发行版：

```text
/opt/ros/jazzy/setup.bash
  → $HOME/cs625_underlay_jazzy/install/setup.bash
  → <repository>/install/setup.bash
```

## 12.3 依赖缺口

- **`open3d` 未安装**，`sklearn` 未安装。P7.2 的离线估计器若走 Open3D/PCL 配准路径，需先补齐依赖并登记到 `docs/dependencies.md`。
- 现有：numpy 1.26.4、scipy 1.11.4、cv2 4.6.0。

---

# 13. 真机实验与仿真的配合

## 13.1 两趟行程的分工

| | 行程 1：9/28–9/29 | 行程 2：10/9 起 |
|---|---|---|
| **CS625** | R0 只读、R1 planning-only、R2 低速空载一次 | R3 已知位姿抓取、R4 固定视角视觉抓取 |
| **力觉** | **★ F/T 判定（§2.4，硬交付）** | 按判定结果执行预案 |
| **相机** | RVS 无 GUI 采集、内参、点云坐标系、手眼矩阵方向 | 复现并用于 R4 |
| **数据** | 现场小数据集（见下） | 补齐 + 离线验证 |

## 13.2 现场小数据集（行程 1）

至少覆盖：

```text
无遮挡 / 轻遮挡 / 重遮挡
多个观察角度 / 多个距离
每条件 3~5 次
```

目的不是统计，而是确保回家后有真实数据可离线调试。字段：

```text
episode_id / view_id / camera_pose / timestamp
RGB / Depth / PointCloud
RVS_pose / RVS_status
```

## 13.3 10 月中旬的真机数量

```text
固定视角视觉抓取 3~5 次
```

目的：找 TCP 错误、手眼矩阵方向错误、夹爪与工装干涉、安全流程问题。

> **不能据此写"真实机器人抓取成功率达到 XX %"。**

后续正式真机验证围绕关键条件收缩，不复制全部仿真矩阵：

```text
固定视角 vs 主动重观测 × 轻遮挡 / 重遮挡 × 10~20 次配对或匹配试验
```

**仿真负责大样本统计和机制分析；真机负责证明仿真中得到的机制在真实传感噪声、真实手眼误差和真实机器人执行条件下仍然成立。**

---

# 14. 9/19–10 月中旬任务安排

## 14.1 9/19 晚–9/22：新任务最小仿真底座

### 任务 1：冻结机械任务定义（Gate 0）

必须明确：

```text
搬运对象 / 几何尺寸 / 质量 / 坐标原点
允许抓取区域 / 禁碰区 / 夹爪接触位置
初始位置 / 装配或放置目标位置 / 容差
抓取接近方向 / 放置或插接方向
成功判据
```

第一版任务成功：

```text
抓取后抬升 ≥ 0.10 m
保持 ≥ 2 s
搬运无掉落
最终进入装配预备位置容差
无非预期碰撞
```

> 明确写：**搬运至装配预备位置。** 不做精密接触插入。

### 任务 2：模型（CAD 今晚到，建模与链路集成解耦并行）

对象、工位、遮挡物都必须有 Visual / Collision / Inertial、正确单位、合理质量惯量、唯一参考坐标系。**重点检查 CAD/STL 的 mm → m。**

**按仓库既有范式建模**（参考 `p7_ycb_tomato_occlusion_severe_v5.sdf:10`）：

```xml
<visual>   使用 mesh（textured.obj / dae）
<collision> 使用 primitive proxy（cylinder/box），带 <surface><friction>
<inertial> 显式质量 + 惯量张量 + CoM 偏移
<sensor name="..._contact" type="contact"> update_rate=100
```

> **不要直接用凹网格当 collision** —— 物理引擎对非凸网格会静默失败或退化。番茄罐用的是 `ycb_contact_proxy_cylinder`（r=0.0340, l=0.101855, μ=0.8）。

建议坐标：

```text
optical_module_link
optical_module_reference_frame
grasp_frame
assembly_target_frame
pregrasp_frame
preassembly_frame
```

### 任务 3：恢复相机和点云（Gate 1 已 PASS，做增量复验）

```text
Gazebo RGB-D sensor
→ /camera/image → /camera/depth_image → /camera/camera_info → /camera/points
→ cs625_sensor_adapter
→ /sensors/camera/*
```

验收：

```text
RGB / Depth / CameraInfo / PointCloud2 有效
point cloud frame 正确
TF 可到 world/base_link
相机移动后确实产生新帧
遮挡物在图像/点云中真实遮挡目标
RViz 可显示
```

> **已知限制：** 保留网格后点云约 **1.8–2.4 Hz**（配置 10 Hz），软件渲染所致。
> **必须写明 settle 判据**：帧间隔阈值 + 连续 N 帧有效 + 时间戳晚于运动结束，并设超时上限。

### 任务 4：已知位姿机械执行基线（M2）

```text
Gazebo GT → 抓取模板 → pregrasp → grasp → close → lift → transport → preassembly
```

> 只能命名为 **已知位姿机械执行基线**。**不能称为视觉抓取。**

### 任务 5（半天）：45/45 复跑验证

见 §5.1。

### 任务 6：力觉接口修正 + 仿真侧接触后端

见 §2.6。产出 `test/real_ft_probe.sh` 与 `cs625_force_adapter` 骨架。

## 14.2 9/23–9/27：数学建模竞赛优先

科研不设硬验收。只做：日志整理、RVS 文档阅读、TF 链记录、小参数修改、单失败案例检查、精读论文指定章节、整理返校现场问题清单。

**禁止启动：** 大矩阵、真机控制、RVS 插件开发、RViz 自定义 Panel、力控调参、多视点复杂融合。

## 14.3 9/28–9/29：现场行程 1

**硬交付：**

```text
★ F/T 判定结论（§2.4）→ 写入 docs/real_hardware_readiness.md
★ RVS 无 GUI 采集可用性 + 原始数据落盘 + 时间戳 + 点云坐标系
★ 手眼矩阵方向确认（必须写清矩阵方向，禁止只写"使用手眼标定矩阵"）
★ 六维力传感器的实际型号 / 是否支持 / 采购报价（若判定为情形 B）
```

PS800-E1 / RVS 确认项：RGB、Depth、内参、点云、时间戳、点云坐标系、手眼矩阵、连续采集、无 GUI 触发、原始数据保存、RVS ICP/定位结果保存。

CS625 确认项：网络、robot state、joint state、TF、TCP、夹爪安装尺寸、相机安装尺寸、driver、planning-only、RViz 轨迹。

有现场监督时可做：**一次低速空载点到点**。**禁止直接跳到自动实物抓取。**

## 14.4 9/30–10/8：远程推进仿真

**A. M1 活链路化**

```text
把 test/p7_2_estimate_textured_pose.py 等提升为 ROS 节点（cs625_target_perception）
把 P7.3 NBV 提升为 ROS 节点（cs625_view_generation / cs625_view_evaluation）
接口沿用 P7.2/P7.3 已冻结 schema
```

**B. M3 固定视角点云定位基线 + 质量门禁**

```text
PointCloud → ROI → background/plane removal → outlier removal
→ voxel downsample → coarse registration → ICP → quality gate → target pose
```

**C. M4 单次主动重观测**

```text
CAPTURE → LOCALIZE → QUALITY_GATE
   ├─ PASS → GRASP
   └─ RETRY → VIEW_PLAN → MOVE_VIEW → SETTLE → CAPTURE → LOCALIZE → GRASP / ABORT
```

**D. 单 seed 无人值守端到端**（Gate M5 的前提）

**E. 若 9/29 判定为情形 B：完成外置六维力询价与下单（★ 有时限）**

## 14.5 10/9–10 月中旬：现场行程 2

```text
R0 只读连接     joint state / robot mode / TF / 不执行
R1 planning-only  planning scene / 工装 collision / 目标 collision / RViz path / 不执行
R2 低速空载      低速度 / 低加速度 / 安全区域 / TCP / 急停流程
R3 已知位姿抓取  人工或示教对象位姿 → pregrasp → approach → close → lift → hold
R4 固定视角视觉抓取
   PS800-E1 / RVS → object pose → base_link → grasp template
   → planning → manual check → low-speed execution    （3~5 次）
```

**R5 主动重观测：只有 R4 稳定后再做。** 10 月中旬前，**完成 1 次成功单次重观测闭环就已经有价值。**

## 14.6 10 月中旬的目标定位（诚实版）

| | 目标 |
|---|---|
| **硬目标** | Gate 0 / M1 / M2 / M3 通过；行程 2 的 R0–R4 完成；力觉判定与采购决策落地 |
| **力争** | M4 单次主动重观测活链路；单 seed 无人值守端到端 |
| **滚动到 10 月下旬** | Pilot 90 集全量、多 seed 稳定性、S1/S2/S0 对比 |

> **不要把 90 集 Pilot 全量塞进 10 月中旬的硬目标**——它排在 M1–M4 之后，一旦任一环节滑期就会连带滑期。

---

# 15. 坐标系必须正式冻结

至少：

```text
world
base_link
flange / tool0
tool_tcp
camera_link
camera_*_optical_frame
optical_module_reference_frame
grasp_frame
assembly_target_frame
```

必须记录：

| parent | child | translation unit | rotation form | static/dynamic | source | verified |
|---|---|---|---|---|---|---|
| | | | | | | |

核心变换：

```text
B T O = B T F · F T C · C T O
```

其中 B = base，F = flange/tool，C = camera，O = object。

> **绝对禁止只写"使用手眼标定矩阵"。必须写清楚矩阵方向。**

---

# 16. 抓取模板

每个抓取模板包含：

```text
grasp_pose_in_object_frame / pregrasp_offset / gripper_opening
approach_direction / retreat_direction / allowed_contacts / priority
```

**第一阶段：CAD / 人工定义抓取模板。不同时开发抓取检测网络。**

---

# 17. RVS / PS800-E1 与 ROS 边界

推荐逻辑接口：

```text
input:  capture_request / episode_id / view_id / robot_pose
output: target_pose / pose_frame / status_code / quality_fields
        capture_timestamp / raw_data_reference
```

优先原则：

1. **复用 RVS/官方 SDK：** 相机连接、深度计算、标定、ICP/已有定位算子；
2. **ROS 侧负责：** 标准化消息、TF、数据记录、质量门禁、任务状态机、主动视点决策；
3. **只有现有 RVS 无法输出必要字段时**，再考虑插件开发。

近期不做：**在 RVS 内重新实现一套 ICP 或大规模算法框架。**

> **本仓库的已知事实（`docs/migration_from_legacy.md:78-84`）：** 本地 `vision_bridge` 包是 **TCP 目标位姿接收器，不是相机驱动**；当前源码扫描未发现 PS800E1 / Percipio / 图漾 的驱动包或已验证话题映射。因此 `cs625_sensor_adapter` 的真机输入话题必须在行程 1 现场确认，**不得凭空编造 vendor 话题名**。

---

# 18. RViz 近期只做可视化，不做复杂 Panel

应显示：raw point cloud、estimated target pose、grasp frame、assembly target、candidate views、selected view、planned trajectory、current state、failure reason。

优先使用：`Marker` / `PoseArray` / `TF` / `PointCloud2`。**后端稳定后再考虑 RViz Panel。**

> 应用侧 RViz 配置已存在：`src/cs625_bringup/config/cs625_moveit.rviz`（已剔除全部 `elite_*` panel）。

---

# 19. 算法与软件边界

| 内容 | 近期处理 |
|---|---|
| RVS 现有定位算子 | 真实系统基线 |
| Open3D / PCL 配准 | 仿真/离线定位基线（**依赖待补**） |
| MoveIt IK/碰撞/规划 | 直接复用 |
| Gazebo RGB-D | 直接复用 |
| 候选视点生成 | 复用已有代码并适配新对象 |
| 定位质量门禁 | 自己定义与验证 |
| 重观测触发 | 自己定义与验证 |
| 任务约束视点评分 | 自己实现 |
| 停止条件 | 自己实现 |
| 抓取模板 | CAD / 人工定义 |
| **接触状态机** | **自己实现（仿真 Gazebo contact 优先）** |
| RVS 插件 | 缺必要输出时再开发 |
| CS625 示教器插件 | 近期不做 |
| RViz 自定义面板 | 后端稳定后再做 |
| 六维力 / 力控 | **接口核查 + 现场判定；情形 B 下不开发** |
| 外置传感器回灌（`setExternalForceTorque`） | 仅在采购决策为"是"时开发 |

---

# 20. 论文图现在就应该预留的数据

| 图 | 内容 |
|---|---|
| **A** | 不同遮挡下最终任务成功率（x=occlusion, group=strategy, y=SR, errorbar=95 % CI） |
| **B** | 固定视角 vs 主动重观测的位姿误差（before/after translation、rotation error） |
| **C** | 重观测恢复率（first-view failure → recovered by preset → recovered by task-constrained） |
| **D** | 成功率—运动代价权衡（x=extra path length / task time, y=task success） |
| **E** | 失败模式分布 |
| **F** | 候选评分预测 vs 实际收益（predicted score vs realized pose/task improvement） |
| **G（v2 新增）** | 接触状态时间线与接触力/力矩曲线（仿真 contact + 真机 ft_source 标注） |

---

# 21. 开发 Gate（M 系列）

```text
Gate 0  机械任务定义
  通过：对象 / 抓取区域 / 禁碰区 / 抓取方向 / 目标位置 / 成功标准 / 坐标系定义完成

M1  活链路化（P7.2 估计器 + P7.3 NBV 节点化）
  通过：ros2 node list 可见；/perception/target_pose 由估计器而非 GT 发布；
        契约检查通过；单场景复现 P7.2/P7.3 冻结结果

M2  已知位姿执行基线
  通过：GT → grasp → lift → hold → transport → preassembly 成功；无非预期碰撞

M3  固定视角视觉抓取活链路
  通过：PointCloud → pose（不使用 GT）；能评价 GT error；
        estimated pose → grasp → 完整任务

M4  单次主动重观测活链路
  通过：first view reject → candidate view → MoveIt → new capture
        → new localization → grasp

M5  Pilot batch
  通过：单 seed 无人值守端到端；无大面积异常中断；
        所有失败有 failure code；可自动汇总 CSV
  然后才是 90 集全量

F  力觉判定（并列门）
  通过：§2.4 判据表给出明确结论，且写入 docs/real_hardware_readiness.md
```

**10 月中旬前达到 M3；力争 M4。M5 与 90 集全量可滚动到 10 月下旬，不必为日期强行跑。**

---

# 22. Codex / Harness 总提示词（v2 更新）

> 当前项目使用 ELITE CS625、眼在手 PS800-E1、ROS 2 Jazzy、Gazebo Harmonic 和 MoveIt。当前目标不是增加通用算法，而是把已冻结的单场景 P7 链路升级为可配置、可批量的活链路，完成精密光学元件从固定视角定位、抓取、搬运到装配预备位置的可靠闭环，并在此基础上加入按需主动重观测。
>
> **重要前提：** 仓库中 P7.1 感知输入、P7.2 位姿估计、P7.3 NBV、P7.4 MoveIt 已 `PASS/FROZEN`，P7.5 抓取闭环已 `PASS/SINGLE-SCENE`。**不要把这些当成待做任务重做**；缺的是"从 test/ 离线脚本提升为 ROS 节点"的活链路化。历史 P4 的 45 矩阵属已废弃的 `coverage_proxy` 遗留链路，不作为当前失败证据。
>
> RVS 现有 ICP/视觉算子作为真实系统基线；仿真与离线算法通过标准 RGB、Depth、CameraInfo 和 PointCloud2 接口工作。**禁止使用 Gazebo 真值直接生成抓取位姿，真值只用于评价。**
>
> **力觉按工作假设 B 处理：CS625 无腕部六维力传感器，仅有基于关节电流的通用力估计。** 近期不开发力控；只做接口核查、`sensor_name` 一致性修正、以及仿真侧基于 Gazebo contact 传感器的接触状态机。注意 underlay 中 `tcp_fts_sensor` / `cs_ft_sensor` / `ft_sensor` 三个名不一致，`/ft_data` 大概率从未跑通。
>
> 每次任务先检查对象与装配位置的 Visual/Collision/Inertial、TF 链、相机和点云、MoveIt 规划场景、抓取模板与实验记录。开发顺序为：M1 活链路化 → M2 已知位姿执行基线 → M3 固定视角视觉抓取 → M4 单次主动重观测 → 真机 R0–R4。
>
> 不重写相机驱动、ICP、IK、碰撞检测和通用控制器；优先复用已有实现。需要自行实现的是任务相关的定位验收、重观测触发、候选视点评分、停止条件、失败恢复、接触状态机和实验评价。
>
> 批量实验必须区分工程调试、Pilot 和正式论文实验。**三策略必须共享 max_views / max_failed_attempts，候选集与质量门禁完全一致。** 10 个 seed/条件只用于 Pilot；正式主实验优先采用配对 scene seed，每条件至少 30 次，**主论文的成功率确证性比较固定在 O2（重遮挡）并按 50 次/条件规划**，确证性检验不超过 3 个并用 Holm 校正。失败结果不得删除。
>
> 所有结论必须区分"已通过仿真""已通过真实数据离线验证""已通过真机"。**不得把 RViz 显示正常等同于 Gazebo 物理模型正常，不得把单次成功等同于可靠性通过，不得把关节电流估计标注为力传感器测量。**

---

# 23. 执行纪律

以后每次增加实验条件前先问：

1. 这个变量对应论文的哪个研究问题？
2. 它是主因素、鲁棒性因素还是调试变量？
3. 是否已有可测量指标？
4. 是否需要为它增加 scene seed？
5. 是否会让实验量翻倍，但不增加论文信息量？

如果无法明确回答：

> **暂不进入正式实验矩阵。**

**每日最低闭环：**

```text
今天完成：
当前阻塞：
明天第一件事：
```

**阶段评价标准：** 不是今天工作了多少小时，而是是否消除了一个阻塞下一阶段的问题。

---

# 24. 待决问题

| # | 问题 | 时限 | 影响 |
|---|---|---|---|
| 1 | F/T 实际能力（§2.4 判定） | 9/29 | 研究内容三的口径 |
| 2 | 是否采购外置六维力 | 9/30 询价、10/9 下单 | 研究内容三能否在本学期落地 |
| 3 | 新对象的允许抓取区域 / 禁碰区 / 配合间隙 | 9/21 | 决定 `grasp_frame` 与位姿容差阈值 |
| 4 | 夹爪型号、开口、指厚与实物是否一致 | 9/28 | 决定抓取模板与碰撞模型 |
| 5 | PS800-E1 的 ROS 话题映射 | 9/28 | 真机 profile 的 sensor_adapter 配置 |
| 6 | 若判定为情形 B，研究内容三降级是否获导师同意 | 10 月中旬 | 论文目录与研究内容 |

---

# 25. 一句话

> **先把已验证的单场景 P7 链路变成可配置、可批量的活链路，让新光学元件在仿真中被正确看见、正确定位、正确抓取；力觉按情形 B 只做接口核查 + 仿真侧接触状态机，并在 9/29 现场出判定、9/30 完成采购询价、10/9 前下单；用 90 个左右 Pilot episode 把链路和统计格式稳定下来；正式论文再扩展到 540~900 个主实验 episode，确证性比较固定在 O2，并用消融与失败分析解释主动重观测为什么有效。**

---

# 附录 A. 外部方法学参考

以下工作只作为实验设计和指标参考，不要求照搬其算法：

1. Breyer, M. et al., **Closed-Loop Next-Best-View Planning for Target-Driven Grasping**, IROS 2022.
   - 使用 Success Rate / Failure Rate / Aborted Rate / Mean number of views 等任务级指标；
   - 仿真对每种策略运行 400 trials；
   - 抓取成功采用"目标抬升 10 cm"这类明确物理判据。
2. Zhang, X. et al., **Affordance-Driven Next-Best-View Planning for Robotic Grasping**, CoRL 2023.
   - 以抓取任务收益而不是纯几何覆盖选择 NBV。
3. BOP Benchmark.
   - 6D pose 使用 VSD、MSSD、MSPD 等对称感知指标；不建议仅报告普通旋转误差或 ICP RMSE。
4. Ma, H. et al., **Active Perception for Grasp Detection via Neural Graspness Field**, NeurIPS 2024.
   - 强调抓取性能与观测/运动时间之间的权衡。

---

# 附录 B. 工作记录（仓库工作日志要求项）

## 变更文件

```text
新增  docs/worklogs/2026-09-19_PLAN_10_MID_MONTH_TASKS_V2.md
镜像  <vault>/04-课题与项目/课题展开/2026-09-19_CS625_10月中旬任务总览与仿真实验设计_v2.md
镜像  <vault>/04-课题与项目/项目/工作日志/2026-09-19_PLAN_10_MID_MONTH_TASKS_V2.md
```

## 执行命令

```bash
# 45/45 遗留链路证据核对
python3 -c "import json,glob; ..."    # docs/evidence/minimum_showcase_matrix_20260906_r1/episodes/*.json
cat docs/evidence/minimum_showcase_matrix_20260906_r1/summary/failure_codes.csv
cat docs/evidence/minimum_showcase_matrix_20260906_r1/summary/strategy_summary.csv

# 控制器约束修复溯源
git log -S "goal_time" --format='%h %ad %s' --date=short -- src/cs625_bringup/config/sim_controllers.yaml

# 力觉核实
grep -n -i "force\|torque\|wrench" ~/elite-sdk-1.2.0/include/Elite/{DataType,EliteDriver,RtsiIOInterface}.hpp
grep -n -i "force\|torque" ~/cs625_underlay_jazzy/src/eli_cs_robot_driver/src/hardware_interface.cpp

# 统计功效核算
python3 -c "<Wilson / McNemar MDD 计算>"

# 契约与仓库状态
python3 test/contract_checks.py
git status -sb
```

## 通过/失败

```text
45/45 证据核对              PASS（失败码与 phase 时间与推论一致）
控制器约束修复溯源          PASS（d79713b, 2026-09-06，与证据同一 commit）
P7 门禁状态核对             PASS（docs/p7_five_gate_protocol.md）
力觉 API 链路核实           PASS（SDK + driver + launch 链路存在）
力觉传感器物理存在性        NOT DETERMINED（工作假设 B；待 9/28 现场判定）
sensor_name 一致性          FAIL（三处不一致，需修）
python3 test/contract_checks.py  （本次未改代码；见下）
```

## 能力层与关键链

```text
Capability layer
- environment and dependencies   : Jazzy + underlay + overlay；Elite SDK 1.2.0
                                   open3d / sklearn 缺失（待补）
- robot model and simulation       : Gazebo 实体可保留网格；新对象模型 9/19 晚接入
- kinematics, control, planning    : 仿真控制器 active；真机底座 fake-hardware 通过
- vision, hand-eye and TF          : 仿真相机四话题正常（约 1.8–2.4 Hz）；真机相机未接
- active perception / NBV          : P7.3 已 FROZEN（单场景）；活链路化 = M1 待做
- real-hardware integration        : 仅软件底座；两趟行程待执行
- force / contact                  : 工作假设 B；仅接口链路存在，物理能力未判定

Critical-chain status
- URDF -> Gazebo entity           : PASS（网格开关两种模式）
- ros2_control -> joint_states    : PASS（仿真）
- base_link -> camera optical TF  : PASS（仿真）
- RGB-D -> normalized topics      : PASS（仿真，约 2 Hz）
- point cloud -> MoveIt scene     : NOT ACCEPTED（本次未重新验收）
- NBV decision -> robot execution : PASS（P7.3+P7.5 单场景）；活链路 = M4 NOT STARTED
- force -> contact state          : NOT STARTED（仅接口核实）
```

## 仿真/真机对齐

```text
reused common core      : cs625_ap_interfaces / sensor_adapter / view_* / motion_adapter
sim entry point         : sim_base.launch.py → sim_moveit / sim_active_localization
real entry point        : real_base.launch.py（driver + description + MoveIt）
real execution allowed  : execute:=false, require_confirmation:=true（保持）
real motion occurred    : 无
fake hardware used      : 历史冒烟通过，本次未运行
Gazebo used             : 本次未启动（静态分析轮次）
```

## 未做与限制

```text
- 本轮为计划文档轮次，未改任何代码，未启动 Gazebo / MoveIt / 真机。
- 力觉物理能力未判定（工作假设 B），需 9/28 现场按 §2.4 流程确认。
- 45/45 根因是"高置信度工作假设"：证据目录无 config 快照与 git revision，
  无法证明当时使用的是修复前还是修复后的 sim_controllers.yaml。
  验证方法 = §5.1 复跑 3 个 cell。
- vendor underlay 未做任何修改；sensor_name 修正计划由应用侧 launch 提供参数。
```

## 镜像记录

```text
vault 挂载状态 : FAIL（/mnt/g 9p 句柄失效，No such device；Windows 侧 G:\ 正常）
镜像方式       : PowerShell 经 \\wsl.localhost\<distro>\ 转发（见 §附录 C）
校验           : Get-FileHash SHA256 与本地 sha256sum 比对
```

---

# 附录 C. `/mnt/g` 挂载失效与恢复

**现象：**

```text
mount | grep /mnt/g
  G: on /mnt/g type 9p (rw,relatime,aname=drvfs;path=G:;...)
ls -ld /mnt/g
  d?????????  ? ?  ?  ?  ?  /mnt/g
ls /mnt/g
  ls: cannot access '/mnt/g': No such device
```

**根因：** WSL 启动时 Google Drive 的虚拟盘 `G:` 尚未就绪，`/etc/fstab` 里的 `G: /mnt/g drvfs defaults,nofail 0 0` 建立了一个**失效的 9p 句柄**；`nofail` 只是不阻止启动，并不会在 G: 稍后就绪后自动恢复。

**恢复（需要用户在普通终端执行，需 sudo）：**

```bash
sudo umount /mnt/g
sudo mount -t drvfs G: /mnt/g
ls "/mnt/g/<vault 顶层目录>"
```

或从 Windows 侧 `wsl --shutdown` 后，**先确认 Google Drive 已挂载 G:**，再重新进入 WSL。

**本轮绕行方案（无需 sudo）：** 通过 PowerShell 走 `\\wsl.localhost\<distro>\` 读取 WSL 侧文件，直接写入 Windows 侧 `G:\`。

```powershell
# <distro>          = WSL 发行版名（wsl.exe -l -q）
# <workspace>       = 本仓库在 WSL 中的绝对路径
# <vault-root>      = vault 在 G: 上的根目录
Copy-Item -LiteralPath '\\wsl.localhost\<distro>\<workspace>\docs\worklogs\<file>.md' `
          -Destination 'G:\<vault-root>\...\课题展开\<name>.md' -Force
Get-FileHash -Algorithm SHA256 -LiteralPath '<dest>'
```

> 注意：`scripts/sync_worklog.sh` 在挂载失效时会按设计**报错退出**（不静默跳过）。挂载恢复后必须用它重新镜像一次，使仓库工作日志与 vault 工作日志回到同一条受支持路径。
