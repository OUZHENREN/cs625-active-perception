# 2026-10-21 WORKLOG — real plan-only（OMPL + Cartesian）

## 冻结状态与目标

- 机器人已由人工移回观测光学工件的位置；
- 本日志只记录 **plan-only**：未激活执行控制器、未调用执行服务、机器人未运动；
- 输入目标沿用 2026-10-20 的 pre-grasp / grasp 位姿（RVS 输出与工件未变）；
- 当前真实 TCP：`0.737, 0.350, 0.648` m；
- 当前关节（按名字）：`shoulder_pan 0.31296, shoulder_lift -1.36087, elbow -1.88340, wrist_1 -0.93553, wrist_2 2.14259, wrist_3 1.12455` rad。

## 重要发现：`/joint_states` 的顺序不是标准顺序

`/joint_states` 的 `name` 顺序为：

```text
elbow_joint, shoulder_lift_joint, shoulder_pan_joint,
wrist_1_joint, wrist_2_joint, wrist_3_joint
```

**不能按位置数组下标直接当 standard 关节顺序**；所有分析和数据记录必须按 `name` 映射。
昨天数据里“目标关节值看起来对不上”就是这个原因，不是控制错误。

## Plan-only 结果

### 1. OMPL pre-grasp

```text
plan_to_pose_srv: success=True, planning_time=0.0217 s, 51 点
planned TCP path:
  path length        0.7518 m
  straight distance  0.7355 m
  max deviation      0.0669 m（相对直线）
  max projection past target 0.0001 m
  min distance to target    0.0001 m（最后一点）
```

结论：**这次 OMPL 规划的 TCP 路径单调接近，没有越过目标再回伸**。

### 2. Cartesian pre-grasp（只规划，不执行）

使用 MoveIt `/compute_cartesian_path`（`moveit_msgs/GetCartesianPath`）：

```text
fraction = 1.0
error_code = 1 (SUCCESS)
points = 61
duration = 5.936 s
start q = 当前观测位（按名字映射）
end q   = [0.83270, -2.01763, -2.02075, -0.66710, 1.58343, 1.39700]
```

### 3. Cartesian pre-grasp → grasp（120 mm，只规划）

```text
fraction = 1.0
error_code = 1 (SUCCESS)
points = 21
duration = 1.946 s
end q = [0.85437, -2.12402, -1.81884, -0.76234, 1.58328, 1.41867]
```

## 超调问题的新判断

- 今天的 **OMPL planned TCP path 本身没有超调**，Cartesian 两段也都 `fraction=1.0`；
- 昨天观察到的“越过后回伸”没有在这两条规划路径中复现；
- 因此昨天的问题更可能来自 **执行跟踪/控制器**，或昨天那次随机规划路径与今天不同；
- 下一次若执行 Cartesian 路径，需要整段记录 joint/TCP；如果仍超调，优先查 controller
  跟踪、`servoj_gain/lookahead` 和速度缩放，而不是继续改规划器。

## 安全与未做事项

- 未调用 `/cs625/execute_last_plan`；
- `arm_controller` 保持 **inactive**；
- 未加载环境碰撞体；Cartesian 规划只基于当前 MoveIt 场景；
- 未执行 grasp/插入，未产生运动。

## 文件

- run 记录：`~/cs625_legacy_snapshot_20261020/runs/20261021_planonly_cartesian/`
  - `meta.json`
  - `plans/planned_tcp.json`、`plans/planned_joint.json`
  - `plans/cartesian_plan.json`、`plans/cartesian_grasp_plan.json`
  - `logs/`
- 当前真机栈：driver + MoveIt（仅规划），`arm_controller` inactive。

## Capability layer / critical chain

- 真实驱动 → `/joint_states`：PASS（仅状态，未执行）；
- MoveIt 规划（OMPL / Cartesian）：PASS（plan-only）；
- `arm_controller` 执行：NOT ACTIVATED；
- 真实运动：NOT PERFORMED；
- 碰撞环境：NOT LOADED；
- 下一步必须先检查规划路径并得到用户确认，再执行。

## 下一步

1. 用户检查 Cartesian pre-grasp / grasp 的路径和速度缩放；
2. 若同意，先执行 **pre-grasp**，全程记录 joint/TCP；
3. 若 pre-grasp 无超调，再执行 grasp（120 mm Cartesian）；
4. 执行前加载环境碰撞体。


## 碰撞场景加载尝试（17:15）

为执行前加载环境碰撞体：

- 将保存的环境点云发布到 MoveIt 的 `/sensors/camera/points`；
- 使用仓库 `real_moveit.launch.py` + `moveit_sensors_3d.yaml` 启动 MoveIt；
- octomap 已生成：resolution 0.02，data length 7680。

但碰撞检查结果：

```text
/check_state_validity: valid=False
contact: <octomap> <-> shoulder_link
带碰撞的 Cartesian 规划: fraction=0.0
```

原因：环境点云包含机器人自身（肩部）点，MoveIt 把机器人当前状态判为碰撞。
因此**本轮没有执行运动**。需要先过滤机器人自身点云，或改用只包含桌面/料盒/插槽的
碰撞体，再重新规划。


## 实际执行 pre-grasp 后的超调（17:31）

- 执行 Cartesian 61 点轨迹，动作返回 `Goal successfully reached!`；
- controller reference（目标）：
  `[0.8327, -2.0176, -2.0207, -0.6671, 1.5834, 1.3970]`
- controller feedback（实际）：
  `[0.9378, -2.2352, -2.0095, -0.6280, 1.4814, 1.4650]`
- 实际 TCP：`0.665, 0.552, -0.235`，比 pre-grasp 目标 `0.715, 0.552, -0.058`
  低约 17.7 cm，机械臂接近桌面。
- 关节误差约 0.2 rad；action 仍报告 success，说明当前 goal 未配置/未执行严格的
  goal tolerance，不能把 action success 当成“到位”。

结论：这不是“速度设小”导致的规划错误；controller 参考已经是目标，实际反馈越过了
参考，属于执行/跟踪层问题。已停止全部 WSL 控制节点，未再发送运动指令，等待排查
servoj 参数、速度缩放、goal tolerance 和反馈映射。

证据：`p4_evidence_20261020/OVERSHOOT_20261021.txt`、
`p4_evidence_20261020/exec_joint_samples.csv`。


## 分段 Cartesian plan-only（18:09）

按“接近段用 Cartesian 直线”的方案，从安全位置重新规划：

```text
当前 → approach（pre-grasp 上方 5 cm）:
  fraction=1.0, 35 点, 3.339 s, max_step=0.01, jump_threshold=0.0
approach → pre-grasp:
  fraction=1.0, 13 点, 1.138 s, max_step=0.005
pre-grasp → grasp（120 mm）:
  fraction=1.0, 21 点, 1.946 s, max_step=0.005
```

参数：`avoid_collisions=true`，速度/加速度缩放 0.1；`jump_threshold=0.0` 表示关闭
关节跳变检查，不是零误差。

未执行：`arm_controller` inactive，机器人无运动。此分段规划解决了“接近段路径形状”，
但不解决上次观察到的执行层超调；下一步必须做短距离低速收敛测试。


## 分段执行收敛测试（18:12–18:16）

在安全位置依次执行：

```text
1 cm 向上:   SUCCESS, feedback=reference, error=0
10 cm 向下:  SUCCESS, TCB z=0.349, error=0
20 cm 向下:  SUCCESS, TCP z=0.149, error=0
approach:    SUCCESS, TCP 0.715,0.552,-0.008, error=0
pre-grasp:   SUCCESS, 到达旧目标
```

每段都设置了 `goal_tolerance=0.02 rad`、`path_tolerance=0.05 rad`、
`goal_time_tolerance=1 s`，速度/加速度缩放 0.05。

重要变化：pre-grasp 完成后，用户告知**工件位置已经变化**，旧目标失效。因此：
- 未执行 grasp/插入；
- 当前机器人停在旧 pre-grasp，需手动移回观测位；
- 下一轮必须重新做 RVS 视觉采集，不能用旧目标继续。

数据：`~/cs625_legacy_snapshot_20261020/runs/20261021_staged_cartesian_convergence/`


## 新 S3 的 plan-only（19:07）

新 S3：
```text
pre-grasp TCP = 0.79104, 0.68366, -0.05750
grasp TCP     = 0.87842, 0.76590, -0.05704
```

分段 Cartesian plan-only（未执行）：
```text
S2 → approach（pre-grasp 上方 5 cm）: fraction=1.0, 126 点, 12.436 s
approach → pre-grasp:                fraction=1.0, 17 点, 1.524 s
pre-grasp → grasp（120 mm）:         fraction=1.0, 39 点, 3.792 s
```

机器人仍在 S2，未抓取、未插入。


## approach 执行超调（19:09）

执行 S2 → approach（126 点，12.436 s），动作报告 success：

- approach 目标 TCP: `0.79104, 0.68366, -0.00750`
- 实际 TCP: `0.804, 0.679, -0.113`（z 低约 10.5 cm）
- controller reference 与 feedback 误差约 0.14 rad
- 未执行 pre-grasp/grasp；已停止控制栈

结论：短段（1/10/20 cm）能收敛，但 12.4 s 长轨迹仍出现执行层超调，且 action success
不能代表 feedback 收敛。下一步必须把长 approach 拆成多个短段逐段验证，并监控 TCP 偏差；
在解决长轨迹跟踪前不执行抓取/插入。


## 分 4 段执行到 approach 成功（19:17）

第一次分段执行因外部控制未建立（50001/50003/50004 无 ESTAB），controller 只接受轨迹但机器人不动，
goal_time 超时中止。重新 `resend_external_script` 后连接建立。

随后把 7.36 s 的路径拆成 4 段（每段约 1.9 s），逐段执行并检查：

```text
chunk1 → TCP 0.875,0.765,0.293
chunk2 → TCP 0.848,0.738,0.194
chunk3 → TCP 0.815,0.707,0.078
chunk4 → TCP 0.791,0.684,-0.007  (approach 目标)
```

四段均 `Goal successfully reached`，controller feedback 误差为 0。未执行 pre-grasp/grasp。


## pre-grasp 执行成功（19:19）

approach 后执行 approach → pre-grasp（17 点，1.524 s）：

```text
目标 TCP: 0.791, 0.684, -0.057
实际 TCP: 0.791, 0.684, -0.057
controller feedback 误差: 0
```

动作 `Goal successfully reached`，无超调。未执行 grasp/插入。


## grasp 位执行成功（19:20）

pre-grasp → grasp（120 mm）拆成 2 段，每段约 1.9 s：

```text
chunk1 → TCP 0.836,0.726,-0.057
chunk2 → TCP 0.878,0.766,-0.057  (grasp 目标)
```

两段均成功，feedback 误差 0。未闭合夹爪、未抬起、未插入。


## 从 grasp 位反向退出 8 cm（19:57）

从 grasp `0.8784,0.7659,-0.0570` 沿 pre-grasp 反方向退出 8 cm，分 2 段执行：

```text
chunk1 → TCP 0.849,0.739,-0.057
chunk2 → TCP 0.820,0.711,-0.057  (目标 0.8202,0.7111,-0.0573)
```

两段均成功，feedback 误差 0；夹爪闭合在 0 mm，未持工件；未执行找视角。


## 追加反向退出 12 cm（20:00）

从 TCP `0.8202,0.7111,-0.0573` 再沿同一反方向退 12 cm，分 3 段执行：

```text
chunk1 → TCP 0.795,0.687,-0.057
chunk2 → TCP 0.764,0.658,-0.058（约）
chunk3 → TCP 0.733,0.629,-0.058  (目标 0.7328,0.6288,-0.0578)
```

全部成功，feedback 误差 0；累计距 grasp 约 20 cm；夹爪闭合 0，未持工件；未找视角。


## 找视角策略修正（20:08）

所有者声明：工件位姿基本在历史观测位姿朝向附近；插槽位姿固定；基础环境拍摄在这两者之间。

因此不再做全周 J1 盲扫，改为以历史 S2 观测位为基准做小范围校正。

实测：
- 抬高到 z=0.55 后，J1 +10°、-10° 均返回 `NG,POSE`；
- 回到历史 S2 位姿（TCP `0.6163,0.5924,0.5772`，朝向历史值）后，T2 成功返回 S2；
- 说明朝向比单纯高度更关键，J1 单转不能替代 S2 朝向。

后续：S2 附近微调即可；插槽用固定观测位；环境位取 S2 与插槽观测位之间。


## 找视角策略补充：插槽位不固化（20:10）

所有者要求：插槽观测位**不能固化/写死**，只能作为基准拍摄位。
每次拍摄后必须结合实际的图像、RVS 返回和位姿质量做微调。

修正后的流程：

```text
基准观测位 -> 拍摄 -> 检查 S2/S3/图像质量
  -> 有偏差则小步调整 J1/J4/Z
  -> 再拍摄；直到当前次可用
```

工件观测仍以历史 S2 朝向附近为基准；插槽基准位作为起始点，不作为永久固定位。


## S2 位 T3 成功（20:12）

在 S2 观测位触发 T3，约 29 s 后返回：

```text
S3: 0.73676,0.921292,0.356568 (z≈356.6 mm)
grasp: 0.880,0.767,-0.057
pre-grasp: 0.792,0.684,-0.057
```

S3 z 落在历史成功范围；未抓取。插槽拍摄位仍未固化。


## 环境扫描方向修正与数据（20:29）

所有者纠正：J1 正向不是插槽侧；正向只做到 +15° 就够，负向才是插槽侧。
本轮已采集：

- 初始安全位 E1：`environment_point_cloud/20261021202302017`
- J1 +15° E1：`environment_point_cloud/20261021202621834`
- J1 -15° E1：`environment_point_cloud/20261021202824183`
- J1 -30° E1：`environment_point_cloud/20261021202911677`（可能过度，先不采用）

已知额外采集 +30°/-30° 位置偏离，暂不纳入合并；机器人已退回初始安全位。
尚未合并点云或生成 MoveIt 碰撞场景。


## 环境点云合并与覆盖检查（20:32）

合并 `initial + J1+15 + J1-15` 三组 E1 点云：

- 2 cm 体素：5096 个；
- ROI（x 0.15–1.30, y 0.30–1.35, z -0.25–0.85）俯视 4 cm 网格覆盖：50.9%；
- 覆盖图：`/tmp/env_coverage_topdown.png`；
- 合并点云：`/tmp/environment_merged_initial_p15_m15.pcd`。

注意：ROI 是临时定义，包含地面/机器人自点；50.9% 不能作为最终验收，只说明仍有明显空洞。


## 碰撞场景输出（20:49）

用 STL 解析 + 当前 latched 目标位姿，向 MoveIt 应用 diff planning scene：

```text
box_collision
slot_collision
slot_table (草稿包围盒)
workpiece_table (草稿包围盒)
```

`applied=true`，`/get_planning_scene` 返回 4 个对象；未执行机械臂运动。
桌子包围盒仍是草稿，尚未做最终边界确认。
