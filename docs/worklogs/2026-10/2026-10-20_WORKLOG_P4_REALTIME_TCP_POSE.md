---
title: "2026-10-20 WORKLOG：P4 实时 TCP 位姿源（state/tf/topic/static）"
date: 2026-10-20
type: WORKLOG
status: done（真机实时值 PASS；E0/S0/S2 PASS；S3 已回传但几何待核验）
---

# P4：把 rvs_pose 从静态参数改为实时 TCP 位姿

## 一、目标

把重观测命令中的固定位姿 `rvs_pose` 替换为**实时 TCP 位姿**，并保持安全：

- 默认**不向真实机器人/RVS 发任何命令**（dry-run）；
- 拿不到实时位姿时**跳过**，绝不用旧值/瞎值；
- 不启动真机驱动、不调用执行服务、不产生运动。

## 二、位姿来源（`pose_source` 参数）

| 来源 | 数据 | 说明 |
|---|---|---|
| `state`（默认） | `/cs625/raw_state`：`CS625State.actual_tcp_x_mm/y/z`、`actual_rot_x/y/z_deg` | 师兄状态消息本身就是 mm/deg，最贴近 RVS 命令 |
| `tf` | `base_link -> tcp_pose` | 与师兄 `moveit_planner_client_node` 的 TF 约定一致；m/rad → mm/deg |
| `topic` | 任意 `PoseStamped`（默认 `/cs625/current_tcp_pose`） | 给其他状态源留的接口 |
| `static` | `rvs_pose` 字符串 | 仅调试；旧行为保留 |

## 三、改动文件

| 文件 | 变化 |
|---|---|
| `src/cs625_task_orchestrator/cs625_task_orchestrator/p4_pose_source.py` | 新增：四元数→RPY、Pose/TF/CS625State→mm/deg、静态解析、格式校验 |
| `src/cs625_task_orchestrator/tests/test_p4_pose_source.py` | 新增 7 个纯数学单元测试 |
| `src/cs625_task_orchestrator/cs625_task_orchestrator/p4_decision_node.py` | 增加 `pose_source`、`raw_state_topic`、TF 帧、`tcp_pose_topic`、`rvs_dry_run`；重试时取实时位姿，取不到就跳过 |
| `src/cs625_task_orchestrator/package.xml` | 新增 `tf2_ros` 依赖 |

## 四、运行证据（不动机器人）

### 1. 单元测试

```bash
python3 -m pytest -q src/cs625_task_orchestrator/tests
# 42 passed
```

### 2. TF 源（静态 TF，非真机）

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.4 --y 0.1 --z 0.7 --roll 3.14159265 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id tcp_pose
ros2 run cs625_task_orchestrator p4_decision_node --ros-args \
  -p pose_source:=tf -p pose_timeout_sec:=1.0 -p max_retries:=1
```

日志：

```text
等待位姿超时 → VISION_NO_MATCH
实时 TCP 位姿(tf:base_link->tcp_pose): 400.000,100.000,700.000,180.000,0.000,0.000
RVS 未配置，仅发布 /p4/reobserve_request（dry-run）
重试 1/1，已请求重新观测
```

### 3. state 源（合成 CS625State，非真机）

```text
实时 TCP 位姿(state): 419.966,353.329,732.167,-166.891,55.656,-122.934
RVS 未配置，仅发布 /p4/reobserve_request（dry-run）
```

### 4. 安全用例：拿不到实时位姿

```text
等待位姿超时 → VISION_NO_MATCH
无法获取实时 TCP 位姿(state)，跳过 RVS 命令: 尚未收到 /cs625/raw_state
重试 1/1，已请求重新观测
```

→ 没有向 RVS 发任何命令，没有运动。

## 五、真机实时值为什么还是 BLOCKED（历史记录，最新见第九节）

只读排查结果：

- 机器人网络可达（ping 通）；
- `cs625_state_monitor/raw_state_receiver_node` 连接状态端口后，**对方立即关闭连接**
  （`Socket closed by peer`，反复重连）；
- `/cs625/raw_state` 没有数据；ROS 里没有真机驱动、没有 `base_link` TF；
- 怀疑状态端口已被 Windows 侧既有控制程序占用（单一客户端），我们没有抢占，
  也没有启动任何驱动/控制程序。

因此：**代码已支持实时位姿，但当前环境没有可读的真机状态源**。
证据：`~/cs625_legacy_snapshot_20261020/p4_evidence_20261020/state_monitor.log`。

## 六、安全设计

```text
rvs_host        默认空      → 不连接 RVS
rvs_dry_run     默认 true   → 即使配置了 host 也不发送
位姿不可用       记 warning → 跳过该次 RVS 命令
execute         无此路径    → 只调用 /cs625/plan_to_pose_srv
```

## 七、下一步

唯一推荐任务：请你在机器人旁、急停在手的前提下，手动启动**只读状态源**（
`cs625_state_monitor` 或官方驱动），确认 `/cs625/raw_state` 有 `actual_tcp_*` 数据；
若状态端口被现有 Windows 程序占用，需要你决定是否停掉该程序。

状态源就绪后，先用 dry-run 核对位姿：

```bash
ros2 run cs625_task_orchestrator p4_decision_node --ros-args \
  -p pose_source:=state -p rvs_host:=127.0.0.1 -p rvs_dry_run:=true
```

把日志里的 `实时 TCP 位姿(state)` 与示教器/机器人当前 TCP 对照一致后，再考虑
`rvs_dry_run:=false`。在此之前**不发 T3**。

## 八、能力层与关键链

```text
Capability layer
- environment and dependencies : PASS
- robot model and simulation   : PASS（零位桩；未用 Gazebo）
- kinematics, control, planning: PASS（规划服务此前已验证）
- vision, hand-eye, TF         : BLOCKED（无真机状态/无 TF）
- active perception / NBV      : NOT STARTED
- real hardware and safety     : BLOCKED（真机状态只读尝试失败；无运动）

Critical-chain status
- 真机状态 -> /cs625/raw_state  : BLOCKED（端口被关闭；无数据）
- base_link -> tcp_pose TF      : BLOCKED（无真机驱动/TF）
- 实时位姿 -> RVS 命令          : PASS（代码 + 静态 TF/合成状态验证；dry-run）
- 规划 -> 执行                  : NOT STARTED（禁止执行）

Intentionally not done
- 未启动真机驱动/控制程序
- 未抢占机器人状态端口
- 未发任何 T3/运动命令
- 未执行任何轨迹
```


## 九、更新（2026-10-20 19:1x）：远程模式后真机状态已通

- 机器人切到**远程模式**后，`raw_state_receiver_node` 连接保持，TCP
  `bytes_received` 持续增长；
- `/cs625/raw_state` 正常发布：`parser_ok=True`、`has_actual_tool_data=True`；
- 实际 TCP：`(984.565, 623.002, 611.349) mm`，
  RPY `(175.969, 45.879, -134.173)°`；
- P4 节点 `pose_source=state`、`rvs_dry_run=true` 的完整日志：

```text
P4 启动: pose_source=state rvs=127.0.0.1:7000/T3 dry_run=True
等待位姿超时 → VISION_NO_MATCH
实时 TCP 位姿(state): 984.565,623.002,611.349,175.969,45.879,-134.173
rvs_dry_run=true，不向 RVS 发送命令（安全）
重试 1/1，已请求重新观测
等待位姿超时 → VISION_NO_MATCH
P4 结束: ABORT（未执行任何运动）
```

- 结论：实时 TCP 位姿源已接通真机；仍保持 dry-run，未向 RVS 发送任何命令。
- 证据：`p4_real_state3.log/.json`、`state_monitor2.log`、`p4_real_state_summary.txt`。
- 下一步：把日志位姿与示教器/机器人当前 TCP 对照；一致后再由你决定是否
  `rvs_dry_run:=false`。


## 十、真机发送 T3（2026-10-20 19:18，用户授权）

用户确认实时位姿与示教器一致并授权发送 T3。P4 节点参数：
`pose_source=state`、`rvs_dry_run=false`、`max_retries=1`、`pose_timeout_sec=4.0`。

节点日志：

```text
实时 TCP 位姿(state): 984.565,623.002,611.349,175.969,45.879,-134.173
已向 RVS 发送重观测命令: T3,984.565,623.002,611.349,175.969,45.879,-134.173
```

RVS 日志：

```text
19:18:59.562  T3_TCPServerReceive 接收到: T3,984.565,623.002,611.349,175.969,45.879,-134.173
19:19:02.292  emit_pose: 0.984565 0.623002 0.611349 -0.0703541 2.34085 0.799832
19:19:18.270  rvs_icp: 没找到匹配坐标
19:19:18.289  CommonTCPClientNode::Process() > send: NG,POSE,0,0,0,0,0
```

`tcp_server` 收到 `NG,POSE,0,0,0,0,0`（未知类型，按设计不发布 `/target_pose`）。
**没有 S3，没有规划，没有执行，没有机器人运动。**

结论：

- 实时位姿 → RVS T3 通道 **PASS**；RVS 的 `emit_pose` 与实机 TCP 完全一致，
  说明单位和坐标系正确；
- 视觉 **NG** 是因为相机当前不在 box 拍摄位/画面里没有可匹配的 box，
  不是位姿或网络问题；
- 要得到有效 S3，需要人工把机器人/相机摆到 box 拍摄位后再发 T3。

证据：`p4_t3.log`、`rvs_t3_20261020.log`、`tcp_server.log`。


## 十一、真机 E0 成功产出 S0/S2（19:24）

用户确认机器人位于光学工件拍摄位后，先试 `T1`（插槽）与 `T3`（弹仓）均返回
`NG,POSE`（对应模型/视角未匹配），随后试通用采集组 `E0`，成功。

P4 发送：

```text
实时 TCP 位姿(state): 984.565,623.002,611.349,175.969,45.879,-134.173
已向 RVS 发送重观测命令: E0,984.565,623.002,611.349,175.969,45.879,-134.173
```

RVS 日志（19:24:18 收到 E0，19:24:20 发送）：

```text
CommonTCPClientNode::Process() > send: S0,648.004,855.880,70.552,96.156,178.308,89.594
CommonTCPClientNode::Process() > send: S2,717.457,575.399,577.137,13.004,124.455,63.714
```

tcp_server / ROS：

```text
Received 'S0,648.004,855.880,70.552,96.156,178.308,89.594'
  → /target_pose, /slot_precision_view_tcp_pose
Received 'S2,717.457,575.399,577.137,13.004,124.455,63.714'
  → /target_pose, /box_precision_view_tcp_pose
```

`/tmp/rvs_pose_log.jsonl` 也记录了对应 `PoseStamped`（frame=world）。

含义：

- `S0` 是**插槽精拍摄位姿**（相机应移动到的位置），不是插入目标；
- `S2` 是**弹仓精拍摄位姿**，不是抓取目标；
- 下一步应由人工把机器人/相机移到 `S2`（或 `S0`），再分别触发 `T3`（得到 S3）
  或 `T1`（得到 R1）；本轮未做任何机器人运动。

证据：`p4_e0.log`、`rvs_e0_20261020.log`、`tcp_server.log`、`rvs_pose_log.jsonl`、
`state_monitor3.log`。


## 十二、S2 位置发 T3：拿到 S3（几何待核验）

用户手动把机器人移到 S2 位置后，P4 发送：

```text
实时 TCP 位姿(state): 717.456,575.401,577.132,175.969,45.879,-134.173
已向 RVS 发送: T3,717.456,575.401,577.132,175.969,45.879,-134.173
```

RVS 19:29:08 收到；19:29:37 回传：

```text
S3,1007.822,1082.903,144.065,0.673,-0.404,39.438
```

ROS 侧：

```text
/target_pose (world)        = 1.007822, 1.082903, 0.144065
/box_grasp_tcp_pose         = 1.140486, 0.919907,-0.269450
/box_pre_grasp_tcp_pose     = 1.047811, 0.843680,-0.270296
```

**必须核验的风险：**

- 实际姿态与 S2 命令姿态相差 **15.13°**（同为 XYZ 欧拉序；不是顺序问题）；
- S3 目标距基座 **1.486 m**，grasp 距基座 **1.490 m**，pre-grasp **1.372 m**
  （S2 拍摄位只有 1.086 m）；
- grasp z = **-0.269 m**，在基座以下，需确认任务面/坐标系是否合理；
- 因此 **不把该 S3 当作已验证的抓取目标**，未规划、未执行、未运动。

证据：`p4_t3_s2.log`、`rvs_s3_20261020.log`、`p4_s3_summary.txt`、`tcp_server.log`。

下一步建议：用示教器把机器人姿态调整到 S2 的
`(13.004,124.455,63.714)°`（位置保持），再发一次 T3；若 S3 变为合理可达位姿，
再讨论只规划验证。


## 十三、S2 精确姿态后的 T3/S3（19:34）

用户把姿态调整到 S2 后，实际 TCP：
`717.456,575.402,577.134,-166.996,55.545,-116.286`；
与 S2 姿态 `(13.004,124.455,63.714)°` 的旋转差计算为 **0.00°**
（同一旋转的不同欧拉表示）。

T3 回传：

```text
S3,1035.845,1103.227,199.032,0.641,-0.289,39.440
ROS /target_pose            = 1.035845, 1.103227, 0.199032
ROS /box_grasp_tcp_pose     = 1.168020, 0.939530,-0.214362
ROS /box_pre_grasp_tcp_pose = 1.075346, 0.863298,-0.214968
```

相机保存画面 `rvs_latest_color.png` 确认看到的是大型铝制工件/弹仓，不是空匹配。

**仍需核验：**

- 距基座：S3 **1.527 m**、grasp **1.515 m**、pre-grasp **1.394 m**（S2 为 1.086 m）；
- grasp z=-0.214 m 仍在基座以下；
- 这不是 TCP/姿态输入错误（姿态差 0.00°），更像手眼标定/世界坐标系/工件模型或
  “这是否是正确抓取目标”的问题；
- **未规划、未执行、未运动**。

下一步建议：核对 RVS 手眼标定（`DepthToRobotTCP.txt` / `ColorToRobotTCP.txt`）与
世界坐标系定义，或用示教器实测该工件的位置，确认 S3 是否应在 1.5 m 附近、
grasp z 是否应为负；确认前不把 S3 当作可执行目标。


## 十五、根因：RVS 输出几何中心，T_b_tcp 按模型原点

现场测量：

- 工件顶面（以机械臂台面为基准）≈ **0.30 m**；
- 工件几何中心 ≈ **0.199 m**；
- `box.ply` 尺寸 131×430×222 mm，顶面到中心约 111 mm，
  0.30 - 0.111 ≈ 0.19，与 0.199 一致。

因此 **S3 的 z=0.199 是几何中心**，不是 `box.ply` 的坐标原点（原点在顶面/后端，
中心偏移 `(10.5, -215, -105.2) mm`）。T3 组匹配后没有平移，说明
`Percipio3DMatchingNode` 输出的是**中心/质心**，而 `T_b_tcp` 是按**原点**写的。

误差分析：

```text
T_b_tcp（原点 -> TCP） 平移 = (-4, -215, -411) mm
RVS 返回中心（不是原点）
→ 当前 grasp 多平移了中心偏移；world 下约 (0.144, -0.158, -0.108) m
```

按“中心输出”修正后的变换：

```text
T_center_tcp 平移 ≈ (-14.5, 0, -305.8) mm
旋转不变 = diag(-1, 1, -1)
```

修正后的抓取位姿（由本次 S3 中心推算）：

```text
current grasp      = (1.1680, 0.9395, -0.2144)
corrected grasp    = (1.0237, 1.0976, -0.1068)
corrected pre-grasp= (0.9310, 1.0214, -0.1074)
```

修正后 x/y 回到工件中心附近（S3 中心 1.0358, 1.1032），z 抬高约 10.6 cm，
与“工具尖插入深度”一致；当前未修正的 y 偏了约 16 cm。

结论：**不是实时位姿问题，也不是 RVS 算错；是匹配输出约定与 T_b_tcp 原点约定不一致。**
需要修改 `T_b_tcp`（以及同理的 `T_s_tcp`）或在我们的决策层做中心→TCP 修正。
本轮未改代码、未规划、未执行、未运动。


## 十六、根因确认与修复：RVS 用“法兰位姿”，我们错发了“工具 TCP”

师兄 `pose_accuracy_eval.m` 明确记录：

> T1/T3 是法兰中心；TCP 由法兰沿自身坐标系固定平移 `(-46, 0, 353) mm` 得到。

现场 `CS625State.actual_tcp_offset_*` 正好是 `(-46, 0, 353) mm`，完全一致。

### 错误

P4 之前把 `actual_tcp_*`（工具 TCP）当作 RVS 的 T3 输入，导致整差 353 mm：

```text
错误输入 T3 = 717.456,575.402,577.134,...
RVS S3      = 1007.822/1035.845 ... z=144/199
grasp       = (1.168, 0.940, -0.214)  ← 远、低
```

### 正确

用 offset 把 TCP 换算回法兰后：

```text
正确输入 T3 = 509.116,332.963,733.795,-166.996,55.545,-116.286
RVS S3      = 601.488,766.593,356.355,0.645,-0.515,57.668
grasp       = (0.779, 0.654, -0.057)
pre-grasp   = (0.715, 0.552, -0.058)
```

S3 z≈356 mm，与历史成功抓取记录 z≈355 mm 一致；grasp 距基座约 1.0 m，合理。
因此 **师兄代码、`T_b_tcp`、ACSb 约定都没有问题**；问题在我们的实时位姿源用了
工具 TCP 而不是法兰。

### 修复

- `p4_pose_source.py` 新增 `state_to_flange_values()`；
- `p4_decision_node` 新增 `report_frame` 参数，默认 `flange`；
- 单元测试 43 passed；
- **未改师兄代码**；RVS T3 只做视觉计算；未规划、未执行、未运动。

证据：`p4_t3_flange.log`、`p4_t3_flange2.log`、`rvs_t3_flange_20261020.log`、
`p4_t3_flange_summary.txt`。


## 十七、真机状态下的只规划验证（PASS）

新增只读状态桥 `test/state_joint_bridge.py`：订阅 `/cs625/raw_state`，把
`actual_joint_deg` 转成 `/joint_states`（只发布状态，不向机器人发指令）。

启动师兄的 `elite_cs625_moveit_config cs625_moveit_with_client.launch.py`，
`/cs625/plan_to_pose_srv` 就绪，`/joint_states` 为真机关节角
（例如 `[49.85, -77.94, -127.49, -8.03, 88.51, 82.21] deg`）。

用修正后的 S3 派生位姿做只规划：

```text
pre-grasp (0.7149, 0.5525, -0.0581)
  success=True, planning_time=0.0228 s
grasp     (0.7791, 0.6538, -0.0571)
  success=True, planning_time=0.0216 s
```

- 未调用 `/cs625/execute_last_plan`；
- 机器人无运动；
- 规划场景只含机器人模型（未加载真实环境点云），因此这只是“可达/可规划”验证，
  不是碰撞安全验收。

证据：`p4_plan_only_20261020.txt`、`p4_plan_only_summary.txt`、
`state_joint_bridge.log`、`legacy_moveit_real.log`。

下一步：若要做执行验证，必须先加载真实环境碰撞场景、确认轨迹、保持
`execute=false` 先审查；本轮未执行。


## 十八、真实驱动启动与执行尝试（未运动）

为执行做的工作：

1. 修复驱动 launch 的 `robot_description` 参数：`ParameterValue(..., value_type=str)`，
   否则 launch 把 URDF 当 YAML 解析，启动直接报错。
2. 发现 `real_base.launch.py` 默认把仓库应用描述（带 Gazebo 夹爪插件）传给真实驱动，
   改为 `description_package=eli_cs_robot_description, description_file=cs625.urdf.xacro`
   后驱动正常启动。
3. 驱动提供 `arm_controller`，MoveIt 配置默认却指向 `joint_trajectory_controller`；
   已将 `cs625_moveit_controllers.yaml` 改为 `arm_controller` 并激活控制器。
4. 启动真实驱动 + 师兄 MoveIt 规划客户端，规划 pre-grasp 成功（51 点）。

执行尝试：

- 调用 `/cs625/execute_last_plan` 返回 `success=False`；
- MoveIt 日志显示 goal accepted、`waitForExecution timed out`、controller 状态 RUNNING
  后 finished；
- **执行前后 `/joint_states` 六个关节值完全不变，机器人没有实际运动。**

判断：不是规划/服务错误，而是驱动/机器人未真正进入外部控制或使能状态（驱动日志显示
`Power off to power on`，但没有任何实际关节变化）。本轮就此停止，未继续发送运动指令。

补丁记录：`~/cs625_legacy_snapshot_20261020/patches/0001-elite-control-robot-description.patch`、
`.../0002-moveit-controller-arm.patch`。

证据：`real_driver_only.log`、`senior_moveit_real.log`、`p4_exec_attempt_20261020.txt`。


## 十九、headless 模式执行尝试（仍未运动）

驱动改用 `headless_mode:=true` 后，`resend_external_script` 返回成功，驱动日志：

```text
EliteCSPositionHardwareInterface: Send external control: success
io_and_status_controller: Successfully resent robot program
```

随后：

- `arm_controller` active，action server=1；
- MoveIt 规划 pre-grasp 成功（51 点）；
- 执行请求被 controller 接受，但 MoveIt 报 `PREEMPTED` / `waitForExecution timed out`；
- 18 秒 `/joint_states` 采样只有 **1 个唯一状态**，机器人没有运动；
- `robot_task_running` 未观察到有效数据。

判断：驱动已把外部控制脚本发到机器人，但机器人侧任务状态仍未进入可执行
（可能示教器仍在本/手动模式、外部控制程序未真正 PLAYING，或安全状态限制）。
本轮未继续发送运动指令。

证据：`real_driver_headless.log`、`senior_moveit_real2.log`、
`joint_samples2.log`、`p4_exec_headless_attempt_20261020.txt`。


## 二十、真实 pre-grasp 执行成功（2026-10-20 21:11）

端口修正后（`elite_control.launch.py` 默认 50001/50002/50003/50004），机器人到驱动的
50001/50003/50004 连接保持 `ESTAB`，`arm_controller` active。

执行 pre-grasp：

```text
规划：success=True
执行：/cs625/execute_last_plan 返回 success=False（MoveIt waitForExecution 超时）
驱动：arm_controller "Goal reached, success!"
最终 TCP（tf2_echo）= 0.715, 0.552, -0.058
目标 pre-grasp     = 0.714906, 0.552456, -0.058137
```

机器人真实运动并到达 pre-grasp；`execute_last_plan` 的 false 只是 MoveIt 等待时间
（6 s）短于控制器实际执行时间（约 12 s），不是运动失败。未执行 grasp/插入。

证据：`joint_samples3.log`、`real_driver_ports.log`、`senior_moveit_real3.log`、
`p4_exec_success_20261020.txt`。


## 二十一、pre-grasp 执行中的超调观察

现象：机械臂在执行 pre-grasp 时明显越过目标点又回伸；最终 TCP 仍精确等于目标
`(0.715, 0.552, -0.058)`。

已确认：

- `plan_to_pose_srv` 使用 MoveIt/OMPL 做**关节空间**规划，只约束终点位姿，不约束
  中间 TCP 路径；中间关节路径可能让 TCP 先越过目标再回来。
- `execute_last_plan` 在 6.4 s 报 `waitForExecution timed out`，但 controller 状态仍是
  `RUNNING`，没有 preempt；controller 12.9 s 后 `Goal reached, success!`。因此超调
  不是“超时取消后停顿再回伸”造成的。
- 关节采样只覆盖执行前 10 s，最后约 3 s（含超调段）没有记录；最终关节值等于目标。

初步结论：超调更可能来自**规划出的关节空间轨迹本身的 TCP 路径**，而不是控制器
跟踪或 MoveIt 取消。下一次需要完整记录整段 `/joint_states` 和规划轨迹，并用
Cartesian/Pilz 直线路径或路径约束来约束接近段。

未执行 grasp/插入；机器人停在 pre-grasp。


## 二十二、日冻结与交接

2026-10-20 21:25 冻结：机器人停在 pre-grasp（TCP `0.715,0.552,-0.058`），
WSL 驱动/MoveIt/桥全部停止，无待执行轨迹，未再发运动指令。

冻结交接与下一阶段决策见 `2026-10-20_HANDOFF_DAY_FREEZE_AND_NEXT.md`；
实验数据实时记录协议见 `docs/experiment_data_protocol.md`；
首个真机 run 记录在 `~/cs625_legacy_snapshot_20261020/runs/20261020_real_pregrasp/`。
