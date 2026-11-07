# 2026-10-25 WORKLOG — 仿真末端坐标系差 353 mm 的定位与修复（load/efficient 13/19 → 18/19）

> 接 `2026-10-25_HANDOFF_SIM_REAL_ALIGNMENT.md`。那份交接把 `load/efficient` 跑不完
> 归因为"IK 换了分支"；本轮用实测把它纠正为**坐标系口径错误**，并修好。
>
> 本轮**没有真机运动**。真机 192.168.1.200 仍不可达（`ping` 100% 丢包）。

---

## 1. 一句话结果

`load/efficient` 仿真从 **13/19 → 18/19**；根因不是"IK 换分支"，而是
**仿真和真机用的不是同一份机器人描述，`my_end_effector_link` 差了 353 mm**
（真机 = 工具尖，仿真 = 法兰）。剩余唯一失败段 `straighten` 属于**物理卡滞**
（槽内旋转，控制器报 `goal_time_tolerance exceeding by 88.0 s`），是另一层问题。

## 2. 根因链（每条都有出处）

```
① 真机启动 MoveIt 时覆盖了机器人描述
   docs/worklogs/2026-10-24_WORKLOG_EXECUTION_ESTOP_ROOT_CAUSE.md:226
   ros2 launch cs625_bringup real_moveit.launch.py description_package:=eli_cs_robot_description

② 厂商描述里末端 link 在工具尖
   ~/cs625_underlay_jazzy/install/eli_cs_robot_description/share/.../urdf/cs625.urdf.xacro:119-123
   <joint name="flange_to_eef_joint"><parent="flange"/><child="my_end_effector_link"/>
     <origin xyz="-0.046 0 0.353"/>

③ 仓库描述里末端 link 就在法兰
   src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro
   <origin xyz="0 0 0"/>            ← 2026-09-20 改的

④ 而任务位姿是"夹爪尖"口径
   config/tasks/common.yaml: "坐标系：全部为世界系（base_link）下的 TCP 位置"
   ⇒ 仿真把法兰摆到了本该是夹爪尖的位置 → 系统性低 353 mm
```

**两条独立数值证据（都可在本机复现）**

```
A. 用真机录制的 pre_grasp 终点关节做正解
   FK(关节) 法兰            = [0.7561, 0.4346, 0.2993]
   再加 (-0.046,0,0.353)    = [0.7839, 0.4794, -0.0528]
   真机 meta 的 actual      = [0.784,  0.479,  -0.053 ]     ← 三位小数吻合
   若用 0920 网格长度 0.207 = [0.7578, 0.4382, 0.0923]       ← 差 161 mm

B. 用 FALLBACK_SEEDS 里注明的 coarse 观测位关节做正解
   法兰                     = [0.3447, 0.0933, 0.7138]
   加 353 mm                = [0.5915, 0.3497, 0.7038]
   common.yaml 同名观测位    = [0.5886, 0.3482, 0.7075]     ← 差 4.9 mm
```

**"207 mm" 与 "353 mm" 的关系（本轮定案）**

两份描述建模的是同一把物理工具，只是坐标系原点放的地方不同：

```
厂商 tool0_vision.STL（ASCII，scale=0.001）在 eef 系下 z ∈ [-0.360, -0.146]
eef 在法兰 +0.353 处  ⇒  换算到法兰系 = [-0.007, +0.207]
仓库 gripper_0920.stl bbox 实测        = [-0.007, +0.207]      ← 完全一致
```

所以 207 mm 是**工具网格本身的长度**，353 mm 是**真机控制器认为的 TCP 位置**，
差值 146 mm 是尚未建模的支架。`scripts/verify_fixture_clearance.py:70` 里那句
"工具网格从法兰沿 +Z 伸出 0.207 m" 说的是网格，不是 TCP。

## 3. 改了什么（3 个文件，+60 / -28）

### 3.1 `src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro`

把两个 fixed 关节的父子关系对调，做到"物理位置零变化 + TCP 归位"：

```
改前：flange --(eef,   0 mm)--> my_end_effector_link --(eef_to_tool0, 0mm)--> tool0 --{夹爪,相机}
改后：flange --(flange_to_tool0, 0 mm)--------> tool0 --{夹爪,相机}
      flange --(flange_to_eef_joint, -0.046,0,0.353)--> my_end_effector_link（纯 TCP，无几何）
```

同时：
- `visual / collision / inertial`（工具网格与 4.580 kg 质量）从 `my_end_effector_link` 移到 `tool0`；
- `<gazebo reference="my_end_effector_link"><gravity>false</gravity>` 改为 `tool0`（质量在哪就落在哪）；
- `eef_to_tool0` 改名为 `flange_to_tool0`（无其它引用，已 grep 确认）。

**坐标系含义（必须理解）**

```
parent_frame → child_frame                         物理含义
flange        → tool0                              工具本体的安装面（网格按法兰坐标导出，
                                                   夹爪与相机都挂这里，位置保持不变）
flange        → my_end_effector_link               真机控制器的 TCP = 夹爪尖，
                                                   沿工具轴 353 mm、沿 -X 46 mm
```

`config/handeye/tcp_to_camera.json` 里 `T_flange_tcp` 与 `T_flange_cam` 都是**法兰系**，
所以法兰是唯一锚点，工具与相机不能再被 TCP 带着走。

**为什么相机不跟着动**：厂商的 `camera_joint` 是占位值（`0.05 0 0.05`, rpy `0 0.785`，
注释自认"5 cm in front of the EEF"）；仓库的 `tool0_to_camera_mount`
（`0.096754 0.017803 0.097328` + 标定 rpy）才是真标定值。改完实测 `camera_link`
位置**未变**（`-1.2769, -0.3959, +0.1209`）——这正是我们要的。

### 3.2 `src/cs625_bringup/config/cs625_active_perception.srdf`

工具几何换了宿主 link，`disable_collisions` 必须跟着换，否则
`CheckStartStateCollision` 会拒绝**每一次**规划：

```
第一次只改 URDF 时实测报错：
  PlanningRequestAdapter 'CheckStartStateCollision' failed, because
  '3 contact(s) detected : gripper_left_finger_link - tool0,
   gripper_right_finger_link - tool0, tool0 - wrist_3_link'
  → plan-only 5/19（连观测段都 PLANNED_FAILED）
```

把 6 条豁免的 `my_end_effector_link` 改为 `tool0` 后恢复正常。**这是同一次改动的
必要部分，不是额外功能。**

### 3.3 `.vscode/tasks.json`（另一件事，一并修）

99 号"DSH Web 快速启动"任务报 `[99.2] 找不到 dsh`。根因：任务用 `bash -lc`（login shell），
只读 `~/.profile`，**不读 `~/.bashrc`**，而 nvm 只写在 `~/.bashrc:119-121`；
且 `dsh` 位于 npx 缓存 `~/.npm/_npx/<hash>/node_modules/.bin/`，本来就不在任何全局 PATH。

修法：脚本内加 `ensure_node()`（显式加载 nvm / 直接取 nvm 版本目录）+
`dsh_cmd()`（PATH → npx 缓存 → 全局 npm 入口三级兜底），端口探测加 `ss` 兜底。

## 4. 验证（全部为本机可复现命令）

```bash
source scripts/source_dev_env.sh --full

# 离线
python3 -m pytest test/ -q                       # 221 passed
python3 test/contract_checks.py                  # Phase 0–6 PASS

# 两份描述的结构化对比（只有 flange_to_eef_joint 不同）
python3 /tmp/cs625_urdf_diff.py                  # 改后：同名关节无差异
                                                 # my_end_effector_link 差值 356.0 → 0.0 mm

# 仿真（headless，真机未参与）
ROS_DOMAIN_ID=79 ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
ROS_DOMAIN_ID=79 python3 scripts/run_task.py --scope load --mode efficient --world sim --plan-only
ROS_DOMAIN_ID=79 python3 scripts/run_task.py --scope load --mode efficient --world sim \
    --confirm --allow-release --no-record
```

| 项目 | 修前 | 修后 |
|---|---|---|
| plan-only / 19 段 | 19/19（但落点错，法兰当 TCP） | 19/19（落点正确） |
| `--confirm` 执行 | **13/19** | **18/19** |
| `pre_grasp` | FAIL（813 点 / 81.2 s，实测 z=0.1831） | **PASS**（142 点 / 14.0 s） |
| `grasp` | FAIL（GOAL_TOLERANCE） | **PASS** |
| `staging` | PLAN_FAILED | **PASS** |
| `pre_insert` | FAIL | **PASS** |
| `insert` | 超时中断 | **PASS** |
| `straighten` | 未跑到 | **FAIL**（新暴露：物理卡滞） |

`pre_grasp` 的 IK 解与真机录制几乎重合，是分支归位的最直接证据：

```
仿真解 [0.736, -2.028, -2.005, -0.672, 1.585, 1.298]
真机解 [0.718, -2.025, -1.999, -0.671, 1.578, 1.290]
```

## 5. 剩余问题（诚实记录）

`straighten`（槽内 9.3° 旋转，0.005 m/s）失败，原因是**物理卡滞**，非坐标问题：

```
[ERROR] [tolerances]: State tolerances failed for joint 1 / 3 / 5
[WARN] [joint_trajectory_controller]: Aborted due to goal_time_tolerance exceeding by 88.007424 seconds
```

即轨迹实际执行比计划多跑了 88 秒仍未到位。可能的成因（下一轮查，本轮不下结论）：

1. 工件（`body` + `rail` 两个 box）被夹住后在插槽内旋转，与夹具发生几何干涉；
2. 仿真夹爪是"刚性 tool + 21 mm 手指行程"，没有真实外撑机构，夹持力/接触行为失真；
3. 槽内单边间隙约 1 mm，而工具/夹具在 Gazebo 里是 box 近似（dartsim 不支持 mesh 碰撞）。

## Capability layer

```
- environment and dependencies      PASS      ROS Jazzy 链、DSH Web 启动任务已修
- robot model and simulation        PARTIAL   TCP 口径已对齐真机；straighten 仍卡；
                                              臂/夹具/桌子在 Gazebo 里仍无 mesh 碰撞体
- kinematics, control and planning  PASS      六模式 plan-only 全过；执行 18/19
- vision, hand-eye calibration/TF   PARTIAL   TCP 口径已定案（353 mm）；
                                              仿真 YOLO 仍 no mask（无贴图渲染，已知限制）
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED   真机不可达，本轮无真机运动
```

## Acceptance target

- 目标：`load/efficient` 仿真执行 19/19。
- 当前：18/19，唯一失败 `straighten`。
- 运行证据：`runtime/phase_log_load.json`（本轮执行）、
  `runtime/logs/phase_log_load_20261025_1517_backup.json`（修前 15:17 那次）、
  `runtime/logs/phase_log_load_planonly_after_fix.json`。
- 是否用了 Gazebo：**是**。是否用假硬件：否。**是否真机运动：否**。

## Critical-chain status

```
URDF -> Gazebo entity              PASS     末端 link 已与真机一致，TF 实测吻合离线预测
ros2_control -> joint_states        PASS     三个控制器 active，/joint_states ≈100 Hz
base_link -> camera optical TF      PASS     相机位置改动前后完全一致（标定未被破坏）
RGB-D -> normalized sensor topics   PARTIAL  点云可出，但渲染无贴图 → YOLO no mask
point cloud -> MoveIt scene         PASS     环境模型 gate PASS，夹具/桌面碰撞体已发布
NBV decision -> robot execution     NOT STARTED
```

## Sim/real alignment

```
共用应用核心     是（scripts/run_task.py + run_pick_and_place.py）
仿真入口        sim_cell.launch.py（cs625_ap_description + 本次修正）
真机入口        real_moveit.launch.py（description_package:=eli_cs_robot_description）
真机执行        未允许（机器人不可达）
门禁状态        环境模型 20261025_070113，gate PASS
```

**附带隐患（已解除）**：`real_moveit.launch.py` 的 `description_package` 默认值仍是
`cs625_ap_description`。真机此前每次都手动覆盖成厂商描述才没出事；改完本轮的
description 之后，即使忘记覆盖也不会再出现 353 mm 口径错误。

## Intentionally not done

- **未publish/改** `p7_grasp_center_link`（在法兰 +0.077 m，与真机抓取点 +0.353 m 不一致）。
  它只被 P7 时代的 `p7_static_grasp_sim.yaml` 与测试引用，六模式流程不用；需单开一轮决定重锚或删除。
- **未补** 臂/夹具/桌子的 mesh 近似碰撞体（所有者已原则批准，但要求单开一轮，
  回归判据 = 真机轨迹回放零误差）。
- **未动** 相机链（真标定值保持原样）。
- **未改** vendor 包（`eli_cs_robot_description` 只读参考）。
- `straighten` 的卡滞原因未定论，只记录现象与三条候选成因。

## 下一轮唯一推荐任务

查 `straighten` 为什么在槽内旋转时多跑 88 秒：把该段的规划轨迹逐点与夹具/工件的
几何关系打印出来，确认是"工件旋转时与夹具干涉"还是"夹爪刚性化导致接触力失真"。
判据：`load/efficient` 仿真 19/19。**不要**在没有定位之前去改速度或时间参数。
