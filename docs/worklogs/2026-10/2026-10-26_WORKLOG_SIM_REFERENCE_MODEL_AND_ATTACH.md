# 2026-10-26 WORKLOG — 仿真参考模型归位 + 附着机制换成 DetachableJoint（未达成 19/19）

> 接 `2026-10-25_WORKLOG_TCP_FRAME_353MM.md` 与所有者 2026-10-25 晚的口头裁定。
> 本轮**没有真机运动**（真机 192.168.1.200 不可达，全轮 ROS_DOMAIN_ID=79 仿真）。

---

## 1. 一句话结果

仿真描述已归位到所有者指定的**参考模型**（厂商 `eli_cs_robot_description/cs625.urdf.xacro`，
工具是含"夹爪 + 两条支撑臂 + 相机安装座 + 相机"的 `tool0_vision.STL`，无方块、无 0920），
"夹住工件"从"位姿写"改成 Gazebo 官方 `DetachableJoint`（attach/detach 已实机验证通过），
仿真判据已改成"工件跟着走 + 进插槽"；**但 `load/efficient` 仍未跑通** —— 卡在
`pre_grasp`/`grasp` 的 `GOAL_TOLERANCE_VIOLATED`（控制跟踪 + 静态工具碰撞两个独立原因，
见 §5、§6）。

## 2. 本轮先纠正的三处错误（上一轮我改坏的，全部已回退）

| 改坏的 | 现象 | 现状 |
|---|---|---|
| 把仿真描述切成仓库包装层 `cs625_ap_description` | Gazebo 里出现 4 个方块（camera_mount 60×60×20、camera_link 30×50×20、两只手指 55×58×146），工具换成 `gripper_0920.stl` | 已回退；`sim_cell.launch.py` 与 `real_moveit.launch.py` 的默认 `description_package` 都改成厂商描述 |
| 用 `gripper_0920.stl` 替换厂商工具网格 | 相机安装座 + 相机的 CAD 一起消失 | 替换路径整段删除，不保留暗开关 |
| 给工件加 `<gravity>false</gravity>` | 工件不落桌（"悬空"） | 已回退；工件恢复重力、19 kg 动态体 |

补充：中途我删过包装层的占位手指但没删 `p7_grasp_center_link`，导致 URDF 非法、机器人实体
没生成（`parent link [gripper_base_link] of joint [gripper_base_to_p7_grasp_center] not found`）。
已整体回退到上一段对话结束时的包装层状态（包装层保留、不再被六模式仿真默认引用）。

## 3. 三个只读审计（证据留在下面的数字里）

### 3.1 参考模型唯一是厂商那份

```
description_package:=eli_cs_robot_description   description_file:=cs625.urdf.xacro
```

依据：`docs/worklogs/2026-10-20_WORKLOG_P4_REALTIME_TCP_POSE.md:440`（真机入口 10-20 起就切到它）、
`2026-10-23_HANDOFF_NEXT_SESSION_START.md:40,44-46`、`2026-10-24_WORKLOG_EXECUTION_ESTOP_ROOT_CAUSE.md:222,226-227`、
`.vscode/tasks.json` 任务 21（base/MoveIt/fake 三支都传这份）。

`tool0_vision.STL` 实测（ASCII 50 MB、183,373 面、189 个连通体）：除夹爪外有一个 **31,222 面的
独立装配体**，中心 (95.6, −0.35, 69.3) mm，正好落在标定的 `camera_mount_xyz=(0.0968, 0.0178, 0.0973)`
附近 —— 即"相机安装座 + 相机"确实在同一个 CAD 网格里（所有者原话："大概多了相机安装座和相机这些"）。

仓库包装层的四个方块全部是本仓库自造：30×50×20 与 90×14×70 来自 2026-08 初代基线
（09-20 提交 `2d156b7` 曾删除）、60×60×20 与 50×50×146 是 10-25 当晚新加的估算体。

### 3.2 TCP / 抓取点 / 预抓取：坐标系没对不上

| 核对项 | 数值 |
|---|---|
| 零件系抓取点 (−0.3235, 0.0829, 0.1793) × 实测工件位姿 | (0.831590, 0.591799, −0.058341) |
| `common.yaml` 的 `grasp` | (0.8316, 0.5918, −0.0583)，**差 0.043 mm** |
| 真机 2026-10-22 实际抓取 TCP | (0.829, 0.588, −0.058)，差 4.6 mm；`workpiece_follows_gripper=true`，抬升 155 mm 后进 1 mm 间隙插槽成功 |
| 三个"工具接触点" | 仿真占位手指 **77 mm** / 两份 CAD 网格物理末端 **207 mm** / 控制器 TCP **353 mm** |
| TCP 在 grasp 时可见网格末端 | 离抓取点 **153 mm**（轴向回退 146 mm + 侧向 46 mm） |
| `grasp` 距工件 bbox 中心 | **221 mm**（设计如此：从底部凹槽、沿 +y 开口面外撑） |

⇒ "抓点与工件实际位姿对不上"**不成立**；"预抓取没到工件中心"是因为抓取点本就在底部凹槽
+ 可见 CAD 末端停在法兰前 207 mm（真机 RViz 也是这个缺口）。**不许用改 TCP 或加盒子去补。**

### 3.3 控制增益：插件级参数才是真正生效的那一处

`gz_ros2_control` 会读 `<plugin>` 元素里的 `<position_proportional_gain>`：

```
runtime/logs/sim_cell_20261026_0052.log:195
[gz_ros2_control]: The position_proportional_gain has been set to: 0.5
```

而把同名参数只写进 `<hardware><param name="position_proportional_gain">` 时**看不到这行日志**，
末端跟踪误差仍有 0.042~0.062 rad。⇒ 插件级注入是必须的（本轮已同时写两处，值一致）。

## 4. 改动清单

| 文件 | 改动 |
|---|---|
| `src/cs625_bringup/launch/sim_cell.launch.py` | `description_package` 默认 `eli_cs_robot_description`、`description_file` 默认 `cs625.urdf.xacro`（注释写明所有者裁定） |
| `src/cs625_bringup/launch/real_moveit.launch.py` | 同上改默认值（真机一直手工覆盖成它，默认不改就是等着踩坑） |
| `src/cs625_bringup/launch/sim_control.launch.py` | prepare 命令接 `--preprocess scripts/sim_robot_description.py`；删死变量 `robot_description_content`；`camera_arguments()` 简化为纯 argv |
| `src/cs625_bringup/scripts/prepare_gazebo_model.py` | 新增可选 `--preprocess`（先注入、后去网格） |
| `scripts/sim_robot_description.py` | ①`position_proportional_gain` 同时注入 `<hardware><param>` 与 `<plugin>`（默认 0.5）；②删除工具网格替换；③新增 `inject_detachable_joint()`（parent=wrist_3_link、child=shielding_module/shielding_module_link、话题 `/cs625/carry/{attach,detach,state}`）；④新增 `inject_tool_collision_proxy()`（闭合态 60×204×214 盒，仅碰撞，视觉仍是 CAD） |
| `scripts/sim_carry_workpiece.py` | 重写：订阅 `/cs625/grasp_state_event`（契约不变），启动先 detach、grasp→attach、release→detach，状态用"先监听再发命令"的握手（状态话题只在变化时发），并向 ROS 广播 `/cs625/carry_state` |
| `scripts/run_pick_and_place.py` | `--world sim` 判据改成工件（`follow`/`travel`/`slot AABB`，真机路径零改动）；`grasp` 后等 `/cs625/carry_state == attached` 再取基线；`sim_object_pose` 带 Gazebo 分区 |
| `config/tasks/world_sim.yaml` | 新增 `sim_workpiece_model: shielding_module` |
| `src/cs625_simulation/assets/cs625_task/shielding_module/model.sdf` | 无净改动（重力已恢复；保留 10-25 的碰撞体抬升 14.8 mm） |
| `test/test_sim_plumbing.py` | 覆盖层三键断言 |
| `src/cs625_bringup/launch/sim_base.launch.py` | 写 `runtime/gz_partition.txt`（脚本层读同一分区） |

## 5. 运行证据

### 5.1 模型与注入（离线 + 运行中）

```
$ python3 src/cs625_bringup/scripts/prepare_gazebo_model.py --output /tmp/gz.urdf --keep-visuals \
    --preprocess scripts/sim_robot_description.py --xacro $(command -v xacro) <厂商 cs625.urdf.xacro> ...
Prepared Gazebo model at /tmp/gz.urdf (kept 8 mesh visual elements for GUI rendering)
硬件事块： <param name="position_proportional_gain">0.5</param> <plugin>gz_ros2_control/GazeboSimSystem</plugin>
插件块  ： <position_proportional_gain>0.5</position_proportional_gain>
工具 link： visual = tool0_vision.STL ；collision = <box size="0.060 0.204 0.214">
gripper_0920 出现 0 次；方块占位 0 个；gz-sim-detachable-joint-system 1 个
```

运行中：`gz topic -l | grep cs625/carry` → `/cs625/carry/{state,attach,detach}` 三条都在；
`gz model -m cs -l` → 有 `wrist_3_link`；`gz model -m shielding_module -l` → `shielding_module_link`。

### 5.2 附着机制手动验证

```
先 gz topic -e -t /cs625/carry/state -n 1 挂监听，再发
gz topic -t /cs625/carry/attach -m gz.msgs.Empty -p "unused: true"
→ data: "attached"          （detach 同理 → data: "detached"）
```

`sim_carry_workpiece.py` 启动日志：`detached 已确认` → `carry_state -> detached`（插件默认吸附，
先松开这一步已跑通）。

### 5.3 失败的运行（2026-10-26 00:30，`runtime/logs/run_load_20261026_0030.log`）

```
[observe:*] 6 段全 ok（自由空间）
pre_grasp  FAIL  chunk executor failed: status=6 error=-5(GOAL_TOLERANCE_VIOLATED)
                 terminal error (rad): wrist_2=0.062 shoulder_lift=0.042 elbow=0.010
grasp      FAIL  wrist_1=0.185 wrist_2=0.069 shoulder_lift=0.058（更差）
工件       被推走：初始 (0.5029,0.6351,0.1272) → (0.6494,1.0258,0.0048)，掉下工件桌
```

把工件搬离 0.7 m 后单独重跑 `pre_grasp`：仍 FAIL，末误差 0.017/0.016 rad > 0.01 容差
⇒ **自由空间也有跟踪滞后**，接触只是把误差放大（0.049→0.185 rad）。
增益 5.0 与 0.5 都试过：接触工况 5.0 更差（力更大、顶得更凶），现已回到 0.5。

## 6. 剩余阻塞（都是可定位的，不是玄学）

1. **跟踪滞后 vs JTC 的 goal 容差**：`sim_controllers.yaml` 的 goal 容差 0.01 rad；实测自由空间
   末端滞差 0.016~0.017 rad。本轮刚发现"插件级 gain 才生效"（§3.3），下一次启动应能显著改善；
   若仍差一点，再评估 time_scale（不改容差 —— 那是验收口径）。
2. **静态工具碰撞与可收拢夹爪的矛盾**：厂商 CAD 是张开态（y 向 240 mm），槽口自由跨距 233 mm；
   真机先收到 204 mm 再进。已加"闭合态碰撞代理"（60×204×214 盒，视觉不变），但从上面的
   数据看 pre_grasp 阶段工件仍被移动了 3 cm —— 说明 pre_grasp 位姿本身已经贴到工件，
   代理盒的尺寸/原点还需要按接触点实测微调（下一步：用 `gz model -p` 逐帧记录工件位移，
   定位到底哪一段接触）。
3. **launch 时序竞态**：本轮最后一次启动出现
   `A controller named 'joint_state_broadcaster' was already loaded`（CM 响应慢 60 s，spawner 重试失败）
   → `joint_state_broadcaster` unconfigured、`/joint_states` 缺失。属于 sim_control 的 12 s 固定
   延时在负载高时不够，需改成"等 CM 服务可用再 spawn"（未在本轮做）。

## Capability layer

```
- environment and dependencies      PASS      ROS Jazzy 链 `--verify` 全绿；Gazebo gz-sim8 8.11.0
- robot model and simulation        PARTIAL   描述已归位到厂商 CAD（含相机安装座/相机）；
                                              pre_grasp/grasp 因跟踪+接触未通过
- kinematics, control and planning  PARTIAL   6 个 observe 阶段 + baseline 全过；pre_grasp 起 ABORT
- vision, hand-eye calibration/TF   PARTIAL   厂商描述的相机是占位（640×480/15 Hz、
                                              位置 TCP+0.05,0,0.05），且缺 camera_*_optical_frame
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED   真机不可达，本轮无真机运动
```

## Acceptance target

- 目标：`load/efficient` 仿真 19/19，且判据是"工件跟着走 + 进插槽"。
- 当前：`pre_grasp`/`grasp` FAIL；附着机制本身可用（手动 attach/detach 已确认）。
- 运行证据：`runtime/logs/run_load_20261026_0030.log`、`runtime/logs/grasp_test_20261025_2358.log`、
  `runtime/logs/sim_cell_20261026_0052.log`（含 `position_proportional_gain has been set to: 0.5`）。
- 用了 Gazebo：是。假硬件：否。**真机运动：否。**

## Critical-chain status

```
URDF -> Gazebo entity              PASS     厂商 CAD 实体生成成功；工具碰撞为闭合态代理
ros2_control -> joint_states        PARTIAL  本轮末次启动 joint_state_broadcaster 因 spawner 竞态未配置
base_link -> camera optical TF      PARTIAL  厂商描述没有 camera_*_optical_frame（已知代价）
RGB-D -> normalized sensor topics   PARTIAL  厂商相机是 640×480/15 Hz 占位，桥接帧名对不上
point cloud -> MoveIt scene         NOT STARTED（本轮未验）
NBV decision -> robot execution     NOT STARTED
```

## Sim/real alignment

```
共用应用核心     是（run_task.py + run_pick_and_place.py）
仿真入口        sim_cell.launch.py（默认 = 厂商描述，与真机 RViz 同源）
真机入口        real_moveit.launch.py（默认已改成厂商描述）
真机执行        未允许（不可达）
门禁状态        环境模型 20261025_153952，gate PASS（本轮刷新过两次）
```

## Intentionally not done

- 未改 `config/tasks/*.yaml` 的位姿/速度（真机 2026-10-22 验收过）。
- 未补厂商描述缺的 `camera_depth_optical_frame` / `camera_color_optical_frame`、未把相机 sensor
  换回实测外参/1280×960（会把"仿真=真机 RViz"这件事再搅浑，需单独立项）。
- 未改工件质量/重力/世界生成物；未改厂商包。
- 未做 `unload` 流程的工件判据（`run_unload()` 不发 `grasp_confirmed`，仍是空跑手臂）。
- 未修 launch 的 spawner 竞态（记录在 §6.3）。

## 下一轮唯一推荐任务

**先只做一件事：让 `pre_grasp` 在仿真里 PASS。** 顺序：①用新的插件级 gain 干净重启（确认
`[gz_ros2_control]: The position_proportional_gain has been set to: 0.5` 在日志里）；
②把工件搬离 0.7 m，单跑 `--phases pre_grasp`，判据 = 不 ABORT 且 `tf2_echo` 的 TCP 与目标差
< 3 mm、工具轴与竖直夹角 < 1°；③PASS 后再把工件放回，用 `gz model -p` 逐帧记录工件位移，
定位闭合态代理盒需要怎样微调（尺寸/原点）。**不要**在 pre_grasp 通过之前去动 insert 或判据。

---

# 续：2026-10-26 白天（同一轮工作，控制卡点已解决，insert 卡在真实接触）

## 1. 一句话结果

`load/efficient` 仿真从"pre_grasp 都过不去"推进到 **15 个阶段里 14 个 PASS**：
工件被 DetachableJoint 真正焊在腕上、`follow 0.0 mm / 0.0°`、跟着走了 0.78 m；
**`insert` 失败**（3 次重规划都被挡），TCP 停在 z=0.316（目标 0.0321）。

## 2. 控制卡点的根因与修复（子智能体 control-fix，已结 task-4）

- 控制律（上游源码 `gz_ros2_control/src/gz_system.cpp::write()`）：
  `v_cmd = position_proportional_gain × (q_cmd − q) × update_rate` ⇒ gain 0.5 × 100 = 有效 50 1/s、τ≈20 ms。
  实测整段轨迹控制器自报误差最大 **3.4e-4 rad**，终点实测关节与计划终点差 **0.00000 rad** ⇒ 跟踪层没缺陷。
- 真卡点：仿真 `camera_link` 的碰撞盒（0.03×0.05×0.02 m，挂在 `wrist_3_link` 局部 (0.004,0,0.403)）
  比 TCP 还低 5 cm，成了工具最低点；pre_grasp 位姿要求它钻进工件桌桌面 **35 mm** → 物理走不到。
  证据：停住时它底面 z=−0.09098 vs 桌面 −0.09097（正压住）；把 elbow 移出折叠姿态后同一条指令立刻到位。
- 修复：`scripts/sim_robot_description.py` 新增 `strip_camera_collision()`（env `CS625_SIM_CAMERA_COLLISION`，
  默认去掉，视觉/传感器/TF/外参不动）。**注意：成功路径不能写 stderr**（`prepare_gazebo_model.py` 把 stderr 当失败）。
- 第二层精度：`plan_joint` 的 ±0.02 rad 目标容差让终点停在"IK 解邻域"。已把它做成配置项
  `goal_joint_tolerance`（真机默认仍是 0.02，行为不变），仿真覆盖层 `config/tasks/world_sim.yaml` 设 **0.002**。
- 结果（`runtime/logs/pregrasp_final_095731.log`）：`pre_grasp ok=True`，TCP 差 **1.9 mm**，工具轴与竖直 **0.83°**，
  6 个 observe 阶段回归 6/6 PASS，工件未被碰动。

## 3. 全流程运行结果（`runtime/logs/run_load_20261026_1010.log`，headless，10:10 起）

```
observe:workpiece_coarse/fine/tables/slot_coarse/slot_fine  ok=True
baseline ok=True
pre_grasp ok=True  grip ok=True  grasp ok=True
carry: attach（grasp_confirmed）→ attached 已确认
lift     ok=True  follow 0.0 mm / 0.00°  travel 0.155 m
transit  ok=True  follow 0.0 mm / 0.00°  travel 0.638 m
staging  ok=True  follow 0.0 mm / 0.00°  travel 0.777 m
pre_insert ok=True follow 0.0 mm / 0.001° travel 0.692 m
insert   FAIL    3 次 "leg failed; replanning from current"，最终 GOAL_TOLERANCE_VIOLATED
                 terminal error: shoulder_lift 0.273 / wrist_3 0.137 / wrist_1 0.091 rad
                 TCP 停在 (1.1345, −0.0103, 0.316) —— 目标 (1.1574, −0.0132, 0.0321)，差 284 mm
```

## 4. insert 为什么挡（带数字）

- 名义夹持关系（工件名义位姿相对 grasp TCP，工具系）：`(+0.1365, +0.3034, −0.1817)` m。
- 实跑记录到的夹持关系：`(+0.129, +0.305, −0.191)` m，**差 (−7.5, +1.6, −9.3) mm**。
- 抓取 TCP 本身的误差只有 (−0.6, +0.8, −1.0) mm ⇒ 那 12 mm 不是机械臂的错，是**工件在仿真里的静止位姿**
  本就偏了：实测静止位姿 (0.5048, 0.6434, 0.1340) vs 名义 (0.5029, 0.6351, 0.1254)，Δ=(+1.9, +8.3, +8.6) mm。
  原因：实测 CAD 位姿让工件包围盒底面比桌面低 14.4 mm，碰撞代理抬了 14.8 mm 之后求解器仍把它顶出 ~8.6 mm。
- 插槽名义间隙：槽宽 457 mm vs 工件长 454.6 mm ⇒ **单边 1.2 mm**。带着 12 mm 关系误差进去必然卡死。
- （旁证：工件 AABB 与夹具 AABB 在卡住时重叠 110/478/183 mm —— 这本身是"工件在夹具里"的正常现象，
  不能据此判碰撞；卡死点是通道 1.2 mm 间隙与 12 mm 误差的矛盾。）

## 5. 下一步两个选项（需所有者拍板，本轮不擅自做）

| 方案 | 做法 | 代价/风险 |
|---|---|---|
| **A（推荐）** | attach 时用**名义夹持关系**（CAD 定义），而不是"实测相对位姿"：真机外撑式夹爪撑住工件后会把它推正到夹爪几何定义的位置，仿真里没有外撑，所以不该把工件那 12 mm 偏移一起带上 | attach 瞬间工件会"跳" ~12 mm（视觉上小跳）；判据 `follow` 仍成立（相对位姿恒定） |
| B | 修工件的**静止位姿**：让它在仿真里就落在名义位姿（调碰撞代理或世界 include 的 z） | 动到世界/资产；而且实测 CAD 位姿本身与桌面矛盾 14 mm，需要重新量 |
| C | 接受 insert 失败，按"仿真不能做插入"记录能力边界 | 六模式仿真只能验收抓取+搬运，不能验收插入 |

## 6. 附带发现（影响复现）

- 每次 `ros2 launch` 被 kill 后都会漏一个 `cs625_sensor_adapter` 孤儿进程吃 100% CPU；攒 10 个后
  controller_manager 会超时（`Failed getting a result from calling /controller_manager/list_controllers`）。
  本轮清掉 10 个才恢复。停仿真请先 SIGINT，再按 PID 清孤儿。
- 期间 gazebo/move_group 各崩过一次（dmesg 有 WSL "Capturing crash"），重启即恢复；跑长流程建议 headless
  （GUI 那一路在软件渲染下占 1 个核以上）。

---

# 续 2：物理外撑夹爪的尝试与"146 mm 支架"几何缺口（2026-10-26 下午）

## 做了什么

- 从厂商 `tool0_vision.STL`（550,119 顶点 / 183,373 面）里按**连通体**拆出两条支撑臂：
  `gripper_arm_left/right.stl`（各 1,382 面，58×34×146 mm，y=±103 mm）+ `tool0_vision_body.stl`
  （其余 180,609 面，含夹爪本体/相机安装座/相机）。**没有任何自建方块当手指**，视觉就是真实 CAD。
- 在 `scripts/sim_robot_description.py` 里新增 `inject_gripper()`：两条臂做成 2 个 prismatic 关节
  （0→21.285 mm/侧，effort 300 N），由已有 `gripper_controller` 驱动（配置补上左臂关节，
  因为 gz_ros2_control 不实现 `<mimic>`）；`_sim_gripper()` 改为发两个手指命令。
- 配套修复（都必要）：
  - SRDF 补 11 条 `disable_collisions`（臂 vs my_end_effector_link/wrist_1-3/camera_link），
    否则 `CheckStartStateCollision` 拒绝**每一次**规划（实测报
    `gripper_left_finger_link - my_end_effector_link`）；
  - `sim_moveit.launch.py` 规划侧描述的 `<ros2_control>` 也改成 Gazebo 硬件块：控制器管理器会
    从 `/robot_description` 抢读描述，读到 MoveIt 那份 mock_components 就失败
    （实测 `class mock_components/GenericSystem ... does not exist`）；
  - `config/tasks/world_sim.yaml` 新增 `sim_gripper_stroke_m: 0.0115`（刚体外撑的"刚好压住"
    行程 ≈1.5 mm 预压；真机 50 mm 开度靠结构弹性，仿真满行程会把干涉量变成推力）。

## 机械上成功了

`/joint_states` 实测两个手指关节都到了 `0.021285`（满行程），`gripper_controller` 变 active，
`grip` 阶段真的驱动了外撑。

## 但暴露一个几何缺口（本轮结论）

- 实测（STL 顶点，eef/TCP 系）：两条支撑臂在 **z −0.203…−0.349 m**；工具本体 −0.146…−0.360 m。
- 厂商描述把工具 identity 挂在 `my_end_effector_link`(TCP=法兰前 353 mm) 上 ⇒ 工具实际在**法兰前
  0…207 mm**，而真机抓取点（TCP）在 **353 mm** ⇒ **中间 146 mm 是没有任何 CAD 描述的支架**
  （2026-09-25 工作日志只写了"未建模的支架"，没有尺寸/形状）。
- 后果：把工具外移去够抓取点，画面就会出现"夹爪没连到法兰上"的悬空间隙（实测 +0.203 m 时
  间隙 196 mm）；不外移，支撑臂就停在抓取点上方 203~349 mm，只能夹空气（实测 lift 时工件
  travel=0）。两种挂法都试过，都不成立。
- 因此本轮**把物理夹爪默认关闭**（`CS625_SIM_GRIPPER=0`），回到厂商原装位（工具与法兰连续、
  视觉正确）+ `DetachableJoint` 附着；`CS625_SIM_CARRY_ATTACH` 默认恢复 1。
  仿真实测回到：控制器 active、carry 起来就 `detached`、工件在桌面原位。

## 下一轮要的第一个输入

**那 146 mm 支架的尺寸**（长度已有 146 mm；需要截面尺寸、以及工具相对法兰的偏心——真机 TCP
还带 −46 mm 的 x 偏置）。有这三种任一种即可：①图纸/照片上的三个数；②师兄 SolidWorks 装配里
那一段的 bbox；③所有者口述"支架大致多粗"。拿到后：建 146 mm 支架（视觉+碰撞）→ 工具挂到
法兰前 146…353 mm → 支撑臂正好落在工件扶手内壁 → 物理外撑夹爪才能成立。
