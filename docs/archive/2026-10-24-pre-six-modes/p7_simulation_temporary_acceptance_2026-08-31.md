# P7 附件辅助抓取仿真临时验收（2026-08-31）

## Material Passport

- Origin Skill: `academic-research-suite/experiment-agent`
- Origin Mode: `run + validate`
- Origin Date: 2026-08-31
- Verification Status: `ANALYZED`
- Version Label: `p7_temporary_acceptance_v1`
- Evidence Runs: `p7_jazzy_tomato_attachment_assisted_20260831_r5`, `p7_jazzy_tomato_attachment_assisted_20260831_r6`, `p7_jazzy_tomato_attachment_assisted_20260831_r7`, `p7_jazzy_tomato_attachment_assisted_20260831_r9`

## 1. 验收结论

P7 的附件辅助仿真运动链已经在 r5 **完整成功演示一次**，但 **稳定验收尚未通过**。r5 按预抓取、接近、夹爪闭合、几何门、Gazebo 附着、携物碰撞代理、抬升和保持的顺序一次执行完成；独立等价 r6 的只读路径门虽然通过，权威 E1 执行规划却因自碰撞路径被拒绝而失败。两次运行都遇错即停，均未在原证据目录内重试。

因此当前可接受的结论是“该工程链在现有 CS625/Jazzy 仿真中具备一次可行实现”；不能接受“该工程链已稳定复现”。r6 后已将 E1 改为含显式 `start_state` 的单次权威位姿目标规划，并删除 runner 中的独立预规划。r7 暴露并促成统一启动器修复；修复后的全新 r9 已实际通过统一就绪门、preflight、E0 和完整场景门，但新 E1 在 4.012 s 后仍由 OMPL 返回 `PLANNING_FAILED`。这证明启动入口已运行时接通，也证明仅改为直接 pose-goal 并不足以得到稳定可执行轨迹。r10 按失败后不自动重跑规则未启动。重复性判定仍为 `NOT_REPRODUCIBLE`，P7-A 稳定验收保持 `NOT ACCEPTED`。

即使只看成功的 r5，本结论也不等于完整 P7 通过。该回合没有执行 RGB-D 目标感知，也没有接触/非预期物理碰撞监测器，因此统一合同正确输出：

- `attachment_assisted_kinematic_chain_success=true`；
- `perception_success=false`；
- `physical_collision_metric_accepted=false`；
- `task_success=false`；
- `primary_failure_code=PERCEPTION_STALE`。

因此 r5 可用于证明 CS625 仿真模型、规划末端转换、控制执行和附件辅助携物链曾经完整接通；r6 同时证明当前 E1 规划链仍有随机 IK/规划分支不一致问题。不得写成“主动视觉抓取成功”“稳定抓取成功”“接触抓取成功”或“真实抓取成功”。

## 2. 运行身份与证据

| 项目 | r5 记录 |
| --- | --- |
| 平台 | WSL2 Linux 6.18.33.2，Ubuntu 24.04 用户空间 |
| ROS / Gazebo | ROS 2 Jazzy；Gazebo Sim 8.11.0 |
| ROS domain / Gazebo partition | `80` / `p7_acceptance_20260831_r5` |
| Git HEAD | `5c623e15464059b580f68db36de05baaf1b2d1cb` |
| 工作树 | dirty；本轮 P7 修改尚未提交 |
| 场景 / 对象 | `p7_ycb_tomato_light` / YCB `005_tomato_soup_can` |
| Episode 墙钟时间 | 39.751414 s（同一 boot 的 monotonic markers） |
| 记录 phase 时间和 | 13.637536 s |
| 失败后重规划次数 | 0 |
| 真机运动 | 无；仅 Gazebo 仿真 |

原始汇总为 [`episode.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r5/episode.json)，环境与资产哈希为 [`manifest.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r5/manifest.json)，MoveIt/Gazebo 原始日志为 [`launch.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r5/launch.txt)。目录内 24 个 JSON/TXT 文件均写入 [`checksums.sha256`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r5/checksums.sha256)，本轮复核为 0 个哈希不一致。

r6 的失败回执为 [`e1_pregrasp.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r6/e1_pregrasp.json)，只读路径门输出为 [`pregrasp_search.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r6/pregrasp_search.txt)，完整解释保留在 [`launch.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r6/launch.txt)。r6 的 [`manifest.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r6/manifest.json) 进一步哈希了实际 P7 适配器、runner 与采集脚本；8 个 JSON/TXT 文件均已写入校验和。r6 没有 `episode_end.json` 或 `episode.json`，这是遇错即停的预期证据形态，不是文件丢失。

r7 的 [`preflight.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r7/preflight.json) 通过；[`failure_runtime_snapshot.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r7/failure_runtime_snapshot.txt) 记录了不存在的 attachment 命令/状态主题及缺失的三个适配器节点，[`launch.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r7/launch.txt) 保留原始启动日志。r7 的 6 个 payload/索引文件全部通过 SHA-256 复核，另有 1 个 `checksums.sha256`；没有 E0 回执、E1 或 `episode.json` 是遇错即停的正确形态。

r9 的 [`e1_pregrasp.json`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r9/e1_pregrasp.json) 记录了新的 pose-goal 请求类型、显式起始状态、容差与 `PLANNING_FAILED`；[`failure_runtime_snapshot.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r9/failure_runtime_snapshot.txt) 证明三个适配器均存在并保留 MoveIt 错误片段，[`launch.txt`](evidence/p7_jazzy_tomato_attachment_assisted_20260831_r9/launch.txt) 为完整原始日志。9 个 payload/索引文件均通过 SHA-256 复核，另有 1 个 `checksums.sha256`；E2--E4 与 `episode.json` 按协议不存在。

## 3. 规划末端与物理末端对齐

此前问题来自“命令所描述的抓取中心”与 MoveIt SRDF 中的规划 tip 不是同一个坐标系。当前接口明确区分：

- 物理命令帧：`p7_grasp_center_link`；
- MoveIt 规划 tip：`my_end_effector_link`；
- 转换方式：每条命令读取实时 TF，把物理抓取中心目标转换为规划 tip 目标；
- 执行后验收：同时比较实际关节终点和由实际关节状态计算的物理抓取中心 FK。

r5 的后验误差如下：

| 阶段 | 物理抓取中心位置误差 | 姿态误差 | 实际终端关节最大误差 |
| --- | ---: | ---: | ---: |
| 预抓取 | 0.519 mm | 0.0930 deg | 0 rad |
| 接近 | 0 mm | 0 deg | 0.000005 rad |
| 抬升 | 0 mm | 0 deg | 0.000001 rad |

这证明当前三段运动的规划目标与实际抓取中心已经在所设公差内对齐。它不证明夹爪与物体之间存在接触力或力闭合。

## 4. 逐门结果

| 门 | 判据 | r5 观测 | 结果 |
| --- | --- | ---: | --- |
| Preflight | 3 个控制器 active；安全初始关节误差 <= 0.002 rad；仿真时钟不回退 | 全部 active；最大初始误差 < 4.18e-10 rad；0 次回退 | PASS |
| E0 脱附 | 明确收到 detached 状态 | `ATTACHMENT_DETACHED` | PASS |
| E1 预抓取 | 路径门、规划、执行和后验终点均通过 | `MOTION_SUCCEEDED`；FK 0.519 mm / 0.0930 deg | PASS |
| E2 接近 | Cartesian fraction=1；规划、执行和后验终点均通过 | fraction=1.0；FK 误差 0 | PASS |
| E3 几何门 | 目标中心至抓取中心 <= 15 mm | 1.303327 mm | PASS |
| E3 夹爪 | 双指反馈误差均 <= 2 mm | 最大 1.998203 mm | PASS（余量很小） |
| E3 附着 | 明确收到 attached 状态 | `ATTACHMENT_ATTACHED` | PASS |
| 携物规划场景 | 目标代理以观测相对位姿附着到抓取中心 | 半径 0.034 m、高 0.101855 m 的圆柱代理应用成功 | PASS |
| E4 抬升 | 规划/执行成功；目标中心上升 >= 0.10 m | 0.119999084 m | PASS |
| HOLD | 有效观测 >= 2.0 s；高度漂移 <= 0.01 m | 2.226153 s；0 m | PASS |
| 感知门 | 新鲜 RGB-D、可见比例和位姿误差可审计 | 未执行 | NOT ACCEPTED |
| 接触/碰撞门 | 监测器可用且明确报告无非预期物理碰撞 | 未接入 | NOT ACCEPTED |
| 完整 P7 `task_success` | 所有正式门同时通过 | `false`，`PERCEPTION_STALE` | NOT ACCEPTED |
| 独立重复 r6 | 同一协议再次到达完整工程链 | 只读路径门通过；权威 E1 为 `PLANNING_FAILED` | FAIL |

运动段累计 IK、规划和执行时间分别为 0.094614 s、0.148891 s 和 8.849553 s。时间字段已按阶段只记一次；不得使用修复前存在重复累计的旧回执。

## 5. 关键链状态

| 关键链 | 状态 | 当前证据或阻塞条件 |
| --- | --- | --- |
| CS625 URDF -> Gazebo entity | PASS | r5 启动日志、控制器和关节状态均来自 Gazebo 实体 |
| `ros2_control` -> `/joint_states` | PASS | 3 个控制器 active；初始与三段实际关节终点均有回执 |
| `base_link` -> `p7_grasp_center_link` | PASS | 接近和抬升前均保存实测 TF；三段 FK 后验通过 |
| `base_link` -> camera optical TF | NOT ACCEPTED | 本 P7 fixture 未执行相机链验收；不得用既有 P4 结论替代本回合记录 |
| RGB-D -> normalized sensor topics | NOT ACCEPTED | 本回合未启动 P7 感知门 |
| point cloud -> MoveIt planning scene | BLOCKED | r5 日志显示 `PointCloudOctomapUpdater` 加载失败；只读核查 `moveit_ros_perception` 返回 `Package not found` |
| NBV decision -> robot execution | NOT ACCEPTED | r5 使用冻结 fixture pose，不是 NBV 输出 |
| fixture pose -> MoveIt -> controller -> attachment -> lift | PASS | E1--E4 原子回执、Gazebo 位姿、保持轨迹和附着状态齐全 |

## 6. 仍需保留的异常与限制

1. `moveit_ros_perception` 当前不在已 source 的 Jazzy 环境中，MoveIt 点云 octomap updater 未加载。r5 的避障证据来自显式 ground、occluder、target 和 carried-object primitive，不是实时点云规划场景。
2. DART 日志报告多个三角网格 collision geometry 无法创建；Gazebo 附着只证明固定关节状态和物体随动，不能接受为物理碰撞率、摩擦夹持或力闭合。
3. 夹爪最大终端误差为 1.998203 mm，距离 2 mm 门限仅约 0.001797 mm。当前按冻结门限通过，但正式统计前应检查控制器稳态余量。
4. `ros2 doctor --report` 在 manifest 采集时 20 s 超时；Gazebo 版本、ROS distro、平台、Git 和资产哈希仍已保存，但 doctor 报告不完整。
5. Episode 和校验和写入后，`move_group` 在接收 Ctrl-C 的清理阶段于 `rclcpp::Executor` 析构中退出码为 -11。该异常发生在验收完成之后，没有改变已写入回执，但生命周期清理仍需单独回归。
6. r6 表明只读路径门与实际 adapter 会各自请求一次 IK/规划，可能得到不同关节分支。当前源码已删除 runner 中的独立路径门：E1 在一个含显式 `start_state`、位置约束和姿态约束的 MoveIt 请求中选择 IK 分支，并只执行同一响应的轨迹。该通用修复不冻结 fixture 专用关节角，但仍待新的独立运行验证。
7. r4 是手工诊断链，包含一次规划失败后的显式重试，不能与 r5/r6 合并计算成功率。
8. r7 暴露了复现入口依赖手工启动适配器：旧 `run_p7_tipfix_sim.sh` 只管理 Gazebo/控制器/MoveIt。当前脚本已统一启动 arm、gripper、attachment 三个适配器，检查三个节点全部出现后才报告 `P7_SIM_READY`，退出时按进程组清理；r9 已为该统一启动与清理入口提供运行时证据。
9. r9 已将第 8 项提升为运行时通过，但新的直接 pose-goal E1 未找到解。已有证据只能确定请求使用 RRTConnect、总规划回执约 4.012 s，不能区分“goal constraint sampler 未形成有效 IK 样本”“规划预算/attempt 分配不足”或“可行 IK 分支与起始状态之间无连通路径”。下一步必须先设计可区分这些原因的单次诊断，不能直接放宽容差后反复试到成功。

## 7. 临时验收范围

| 层级 | 结论 | 下一允许步骤 |
| --- | --- | --- |
| P7-A 附件辅助运动链 | 不接受稳定验收；r5 单次可行、r6 E1 失败、r7 未达 E1、r9 新 E1 失败 | 先用可审计的单次诊断区分 pose-goal 采样、规划预算与路径连通性，再冻结修复；不得反复放宽参数刷成功 |
| P7-B RGB-D 感知到抓取 | 不接受 | 先补齐 `moveit_ros_perception` 与真实 P7 目标估计回执 |
| P7-C 接触/摩擦抓取 | 不接受 | 先建立可审计的接触监测和“先碰撞后附着即失败”门 |
| 正式五方法抓取主实验 | 不开始 | P7-B、P7-C 和协议参数冻结后再运行 |
| 真机抓取 | 不允许 | 真机未连接，继续保持 real profile 安全门 |

## 8. 复现入口

仿真、三个适配器和 episode runner 必须在相同的 `ROS_DOMAIN_ID` 与 Gazebo partition 中运行。r5 使用的最终 episode 命令为：

```bash
export ROS_DOMAIN_ID=80
export IGN_PARTITION=p7_acceptance_20260831_r5
export GZ_PARTITION=$IGN_PARTITION
export CS625_P7_SIMULATION_EXECUTION=1
export P7_RUN_ID=p7-20260831-r5
bash test/run_p7_static_episode_capture.sh \
  docs/evidence/p7_jazzy_tomato_attachment_assisted_20260831_r5
```

运行脚本拒绝复用已有证据目录。r6、r7 与 r9 均已按遇错即停规则失败，证据目录必须保留。任何修复后的新运行都必须使用新的 run ID、domain、partition 和输出目录；不得覆盖 r5、r6、r7 或 r9。当前 `run_p7_tipfix_sim.sh` 已把三个适配器纳入同一生命周期和就绪门，r9 已给出该入口的运行时证据。

## 9. 可重复性判定

- 方法：r5 与 r6 使用同一 abort-on-failure runner、同一目标/场景/阈值与相同 P7 配置；r6 使用独立 domain、partition、run ID 和目录。比较不使用墙钟时延作为一致性判据。
- r5：完整工程链成功。
- r6：preflight、E0 和场景门成功；只读路径门成功；权威 E1 规划失败并立即中止。
- 判定：`NOT_REPRODUCIBLE`。
- 理由：主要工程终点由成功变为失败，超过随机实验默认容差；局部前置门一致不足以抵消主要终点不一致。
- r7：preflight 成功；E0 前 attachment 订阅不存在，现场确认三个 P7 适配器均未启动；立即中止，E1 未执行，r8 未启动。
- r9：统一入口、preflight、E0 和场景门成功；权威单请求 pose-goal E1 为 `PLANNING_FAILED`，无具体碰撞对；立即中止，r10 未启动。
- 处置：不自动重试 r6/r7/r9，不删除失败证据；先冻结一个能区分 goal sampling、规划预算与路径连通性的诊断协议，再由用户决定新的独立 run。
