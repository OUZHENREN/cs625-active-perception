# P7：CS625 主动视觉—抓取闭环仿真协议（Jazzy）

> 本文件保留 P7.5 attachment-assisted 历史机制和 r5--r9 证据口径。
> 自 2026-09-01 起，权威推进顺序改为
> [`P7 五级串行门禁协议`](p7_five_gate_protocol.md)：P7.1 未通过前不得继续
> P7.2--P7.5 集成，本文中的历史抓取链结果不能替代前置感知、位姿或 NBV 门禁。

## 1. 目的与适用边界

本协议把既有 P4「相机视点规划—MoveIt 执行—新鲜 RGB-D 点云」工程闭环延伸为可审计的抓取任务。它只适用于既有 WSL2 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 8.11 叠加环境；不启动真机、不迁移环境，也不修改 Elite 官方或师兄的 underlay 代码。

第一阶段的任务成功只能称为 **attachment-assisted kinematic simulation**：夹爪闭合命令、几何容差、Gazebo 附着状态、抬升高度与保持时间共同构成成功证据。它不是接触/摩擦动力学抓取，更不得把 P3 的 MoveIt `GetStateValidity` 候选拒绝率称为物理碰撞率。该阶段只能用于 P7 工程链路验收；冻结候选协议中的正式抓取主终点还要求非预期碰撞、接触/保持证据和“先碰撞后吸附即失败”的判据，未补齐前必须标为 `NOT_ACCEPTED`。

## 2. 冻结对象和资产

| 对象 | YCBV/BOP ID | 任务角色 | 主要难点 |
| --- | ---: | --- | --- |
| `035_power_drill` | 15 | 非轴对称工具抓取 | 不规则外形、抓取朝向选择与较大尺度。 |
| `005_tomato_soup_can` | 4 | 纹理罐体抓取 | 遮挡下的观测与罐体夹持余量。 |
| `036_wood_block` | 16 | 规则形状泛化对象 | 外形近似规则，检验对纹理/几何先验的依赖。 |

网格版本、米制 AABB、质量和初始惯量近似见 `src/cs625_simulation/assets/ycb/MANIFEST.md`。当前原始网格来自官方 YCB Google 16k 下载；BOP 完整模型 archive 与 `models_info.json` 尚未随代码树版本化，因此报告中不得把本阶段称为 BOP 评测。

## 3. 状态机与接口边界

```text
P4 观察闭环
  → P7-PERCEPTION_GATE
  → E1 预抓取 MoveIt 计划 / 执行
  → E2 接近 MoveIt 计划 / 执行
  → E3 夹爪闭合 + 几何抓取门 + 附着状态
  → E4 抬升 MoveIt 计划 / 执行
  → HOLD(2.0 s) + 目标高度核验
  → TASK_SUCCEEDED | 失败码
```

### 3.1 已实测的 P7 机制边界（更新至 2026-08-31）

- `p7_ycb_tomato_light.sdf` 已在现有 Jazzy/Harmonic 运行时加载 YCB `005_tomato_soup_can` Google-16k visual 网格；AABB 质心已写入 inertial pose。DART 不支持从 SDF 直接建立三角网格碰撞，因此接触仅使用显式 `ycb_contact_proxy_cylinder`，不得以此主张网格级碰撞、摩擦或力闭合。
- 官方 `gz-sim-detachable-joint-system` 已挂接到运行时可寻址的 `wrist_3_link`。`gripper_base_link` 会在 Gazebo 的固定关节折叠中消失，故不能作为插件父 link。
- `cs625_task_orchestrator/p7_attachment_adapter.py` 接收 JSON 复位/附着请求，监听官方 `gz.msgs.StringMsg` 状态。只有 `data: "attached"` 或 `data: "detached"` 与请求一致时才报告成功。实测脱附回执为 `ATTACHMENT_DETACHED`、`attached=false`、`success=true`，并有 Gazebo `Detaching joint` 日志佐证。
- r5 单回合在接近后用 Gazebo 目标物位姿与 `base_link -> p7_grasp_center_link` 实测 TF 计算携物相对位姿，再把与 SDF 一致的圆柱代理作为 MoveIt `AttachedCollisionObject` 加入规划场景。该代理仅用于携物阶段的 MoveIt 几何避障；它不补充接触力、摩擦或力闭合证据。
- 该回执仅证明固定附着状态变更。已知 DART 还报告左指 mimic constraint 不受支持，因此当前夹爪不具备双指接触力、摩擦和持物稳定性的验收条件；这些字段继续为 `NOT_ACCEPTED`。

- `cs625_target_perception`：仅负责输出目标估计、可见性/置信度与其误差审计；不得在该包中驱动夹爪或修改 Gazebo 物体。
- `cs625_motion_adapter`：只执行已验收的臂轨迹与夹爪位置命令，并报告各 action 的结果与时长。
- `cs625_task_orchestrator`：新增的编排边界；执行 E1--E4 转换、重规划和原子 episode 日志，不实现感知算法或低层控制。
- `cs625_simulation`：动态目标、夹爪模型、Gazebo 连接/状态桥接和 ground-truth evaluator，不向策略层泄漏“成功”结论。

### 3.2 P7 臂动作回执接口

`p7_arm_motion_adapter` 以 `/p7/arm_motion_command` 接收一个显式 JSON 位姿命令，允许的 `phase` 仅为 `pregrasp`、`approach`、`lift`。每个命令必须带 `command_id`、`frame_id`、位置和单位四元数；命令描述物理抓取中心 `p7_grasp_center_link`，适配器通过实时 TF 将其转换为 MoveIt 规划末端 `my_end_effector_link` 的目标。

E1 `pregrasp` 把实时六轴起始状态、位置容差盒和姿态容差放入同一个 MoveIt `GetMotionPlan` 位姿目标请求，由规划器在该请求内选择可达且无碰撞的 IK 分支；随后只执行同一响应返回的轨迹。runner 不再先调用独立的 `p7_pregrasp_path_search.py`，也不把另一轮 IK 结果冻结为关节目标。E2 `approach` 与 E4 `lift` 仍先用 collision-aware IK 作可达性门，再计算带碰撞检查的线性 Cartesian 轨迹。三个 phase 执行后均检查实际关节终点和物理抓取中心 FK 误差，并在 `/p7/arm_motion_status` 回传独立的 `ik_time_sec`、`planning_time_sec`、`execution_time_sec` 和 `total_time_sec`。`planning_attempts=5` 是 E1 单个权威 MoveIt 请求内部的采样预算，不等于失败后的外部重试。

上述 E1 一致性修复已通过构建和单元/静态检查。r7 在 E0 前因启动入口未拉起三个 P7 适配器而中止；启动入口随后改为负责 arm、gripper、attachment 三个适配器的启动、就绪门和清理。全新 r9 已通过该统一就绪门、preflight、E0 detach 与完整场景门，但单请求 E1 位姿目标在 `planning_time_sec=4.012036` 后返回 `PLANNING_FAILED`。因此双请求分支漂移已从权威链路中删除，但新的 pose-goal 方案仍未取得运行时成功，不得因接口一致性修复而视为已解决可执行性。

这条接口不计算目标姿态、抓取几何或成功结论；只有编排器将三个 phase 回执和夹爪/附着/Gazebo 高度证据合并到 P7 atomic JSON 后，才可调用本协议的成功判据。仿真参数冻结于 `config/p7_static_grasp_sim.yaml`，其中 `execute=true`、`require_confirmation=false` 仅适用于本协议中的 Jazzy P7 仿真。

## 4. 成功判据与失败码

一个 episode 的字段必须逐步记录，不能由最终结果倒填。

| 指标 | 记为成功的必要条件 | 禁止替代证据 |
| --- | --- | --- |
| 感知成功 `perception_success` | RGB-D 观测新鲜；估计结果带时间戳；相对 Gazebo 真值同时满足平移误差 <= 0.02 m、旋转误差 <= 10 deg；且可见比例 >= 0.15。 | 仅收到 PointCloud2；脚本直接发布目标真值。 |
| 可执行抓取 `executable_grasp` | E1、E2 的 MoveIt 计划与 trajectory action 均成功，闭合命令到达；Gazebo 实测目标中心至夹爪抓取中心误差 <= 0.015 m，且几何门通过。 | 仅有 IK；仅有候选可达率；仅有 MoveIt 末端位姿。 |
| 附着抓取 `attachment_success` | `attachment_state==attached`，夹爪位置到达，且目标与抓取框相对位姿误差 <= 0.015 m。 | 仅发送 attach 请求。 |
| 最终任务成功 `task_success` | `attachment_success`；E4 action 成功；目标质心相对初始高度增加 >= 0.10 m；保持至少 2.0 s 且高度跌落不超过 0.01 m；接触监测器可用并显式报告 `unexpected_collision=false`。 | 单帧目标高度；命令返回成功；缺失碰撞字段时默认“无碰撞”。 |

标准失败码：`PERCEPTION_STALE`、`LOW_VISIBILITY`、`POSE_TRANSLATION_ERROR`、`POSE_ROTATION_ERROR`、`PREGRASP_NO_IK`、`PREGRASP_PLAN_FAILED`、`PREGRASP_EXECUTION_FAILED`、`APPROACH_PLAN_FAILED`、`APPROACH_EXECUTION_FAILED`、`GRIPPER_TIMEOUT`、`GRASP_GEOMETRY_UNOBSERVED`、`GRASP_GEOMETRY_ERROR`、`GRASP_GEOMETRY_REJECTED`、`ATTACHMENT_FAILED`、`ATTACHMENT_POSE_ERROR`、`LIFT_PLAN_FAILED`、`LIFT_EXECUTION_FAILED`、`OBJECT_NOT_LIFTED`、`HOLD_FAILED`、`HOLD_UNSTABLE`、`PHYSICAL_COLLISION_UNOBSERVED`、`UNEXPECTED_COLLISION`、`TASK_TIMEOUT`。正式 `task_success` 要求接触监测器给出显式布尔碰撞观测；监测器缺失或观测为空时，不能默认“无碰撞”。

## 5. 实验设计

### 5.1 因子

- 遮挡：`light`、`medium`、`severe`，每种场景保存遮挡器尺寸、位姿、目标真值可见比例范围与随机数种子。
- 位姿扰动：位置标准差 `0 / 0.01 / 0.02 m`，姿态标准差 `0 / 5 / 10 deg`；每个 episode 记录实际采样扰动，而非只记录等级名。
- 新物体：训练/调参对象为 `035_power_drill` 与 `005_tomato_soup_can`；留出 `036_wood_block` 仅用于锁定策略后的泛化评估。
- 视点策略：`fixed_scan`、`random_reachable`、`coverage_nbv`、`pose_ig_only` 与 `grasp_aware_tcnbv`。其中后两种和真实目标覆盖收益尚未实现；在它们可复算前，当前 run6/P4 只能称为三条工程基线，不能冒充五方法主比较。
- 相机与预算：正式 profile 固定为 `dv89_rgb_sim_v1`、`percipio_ps800e1_depth_sim_v1` 和 `dv89_ps800e1_eye_in_hand_v1`；单回合最多 6 个主动视点，硬墙钟时间 300 s，累计关节 L1 运动预算 60 rad，连续 3 次执行失败或无可行候选即终止。既有 320×240、10 Hz、无显式噪声 P4 仍是 `legacy_gazebo_rgbd_v0` 工程前实验。

### 5.2 最低运行矩阵

正式主比较：3 个对象 × 3 个遮挡等级 × 10 个配对 seed × 5 个方法，共 450 episode。每个 episode 上限为 6 个观测视点、300 s wall-clock 预算，且共享场景、初始关节状态、候选集、噪声随机流、抓取候选、MoveIt 参数和预算。位姿扰动及新物体泛化独立成矩阵，避免与主比较混合后掩盖主效应；`036_wood_block` 同时作为已知 CAD 的正式对象和材质/几何泛化分层对象，训练/调参划分须在执行前锁定。

## 6. 统计与图表

主终点为 `task_success`；同时报告感知成功、可执行抓取、附着抓取、总时长、重规划次数、失败码以及分清口径的候选碰撞拒绝率和物理碰撞率。每张图保留逐 episode 点、均值和 95% CI，并在图注明确样本数与分母。

- 成功率图：分面显示对象 × 遮挡；二项比例及 Wilson 95% CI。
- 时延图：分解感知、E1/E2、闭合/附着、E4、保持阶段；同时给出总时长。
- 鲁棒性图：遮挡、平移扰动、旋转扰动分别成图，禁止用未经运行的单个“综合鲁棒性”数代替。
- 失败码图：以所有启动 episode 为分母，失败阶段堆叠比例与原始计数并列。

## 7. 验收关

在任一报告或论文图中使用抓取指标前，必须同时保留：原子 JSON、控制与 Gazebo 日志、目标高度轨迹、附着状态事件、实际随机参数、环境/资产版本和独立重跑命令。未满足时，相关指标标为 `NOT_ACCEPTED`，而不是填零或按 P4 观测成功率替代。

### 7.1 当前单回合证据与重复性结果

2026-08-31 的隔离 r5 运行按 `test/run_p7_static_episode_capture.sh` 一次执行完成，未在同一证据目录中进行失败后重试。工程链实测：抓取中心几何误差 0.001303 m、抬升 0.119999 m、有效保持 2.226 s、保持高度漂移 0 m，预抓取/接近/抬升的实际终端关节最大误差分别为 0、0.000005 和 0.000001 rad。该回合的 `attachment_assisted_kinematic_chain_success=true`。

独立 r6 使用新的 ROS domain、Gazebo partition、run ID 和证据目录执行同一 runner。其只读预抓取路径门返回 `PATH_OK`，但实际 E1 规划返回 `PLANNING_FAILED`；MoveIt 日志显示实际请求所得路径在 `wrist_2_link` 与 `forearm_link` 之间发生自碰撞并被 `ValidateSolution` 拒绝。r6 按协议立即中止且没有重试。由于主工程结果在独立重复中由成功变为失败，当前重复性判定为 `NOT_REPRODUCIBLE`，稳定 P7-A 验收不得通过。

预登记 r7 使用 domain `82` 和独立 partition；preflight 通过，但第一个 E0 命令前发现 `/p7/attachment_command` 无订阅，现场快照确认三个 P7 适配器均未被启动入口拉起。r7 立即中止，E1 未运行，不能用于判断上述 E1 修复是否有效；预登记 r8 也未在该失败后启动。r7 作为基础设施失败证据永久保留。

修复启动器后的 r9 使用 domain `84` 和独立 partition。三个 P7 适配器实际存在，preflight、E0 与 `scene_full` 均通过；E1 回执明确记录 `goal_constraint_type=pose`、显式初始六轴状态、`0.001 m` 位置容差和 `0.005 rad` 姿态容差，但 OMPL/RRTConnect 在 4.012 s 后报告 `Unable to solve the planning problem`。日志没有给出具体碰撞对，不能把失败擅自归因为自碰撞或单一容差参数。r9 立即中止且未重试，预登记 r10 未启动。

上述运行均没有执行目标感知，且没有物理接触碰撞监测器；r5 的统一合同必须保留 `perception_success=false`、`physical_collision_metric_accepted=false`、`task_success=false` 和主失败码 `PERCEPTION_STALE`。验收报告及证据索引见 [`p7_simulation_temporary_acceptance_2026-08-31.md`](p7_simulation_temporary_acceptance_2026-08-31.md)；不得把 r5 单次工程结果写成“主动视觉抓取成功率”。
