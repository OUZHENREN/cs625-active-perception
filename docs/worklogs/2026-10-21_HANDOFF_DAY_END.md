# 2026-10-21 HANDOFF — 真机视觉抓取、夹爪、环境扫描与碰撞场景

## 今日结论

今天把真机链路从“能看、能规划”推进到：

```text
RVS S2/S3 视觉链路
→ 分段真实执行
→ pre-grasp/grasp 位姿
→ 反向退出
→ EFG80 夹爪自动控制 + ROS 服务
→ PyQt5 可视化面板
→ J1/J4/其他关节环境扫描
→ 环境点云合并
→ 工件/插槽/桌面碰撞场景
```

但**还没有完成真实抓取、抬起或插入**。

## 1. 视觉链路

- RVS 命令端口 `127.0.0.1:7000`，位姿接收由 `tcp_server:9999` 完成。
- T2 在历史 S2 位姿成功：
  - S2: `0.616272, 0.592403, 0.577179`
- T3 在 S2 成功：
  - S3: `0.73676, 0.921292, 0.356568`（z≈356.6 mm）
  - grasp: `0.880, 0.767, -0.057`
  - pre-grasp: `0.792, 0.684, -0.057`
- J1 正向只做到 +15°；正侧不是插槽侧；负向才是插槽侧。
- 插槽 T1 在你手动调整相机位置后成功：
  - R1 / slot_target_pose: `1.241629, 0.181709, 0.443245`
  - slot_insert_tcp_pose: `1.157, -0.013, 0.032`
  - slot_pre_insert_tcp_pose: `1.100, -0.012, 0.377`
- 当前插槽拍摄基准位：
  - TCP `1.044, -0.011, 0.726`
  - 四元数 `[-0.059, 0.938, 0.060, 0.336]`
  - 仅作为基准，未固化。

## 2. 真实执行

已完成并记录：

- S2 → approach：4 段，TCP 到 `0.791,0.684,-0.007`。
- approach → pre-grasp：TCP 到 `0.791,0.684,-0.057`。
- pre-grasp → grasp：2 段，TCP 到 `0.878,0.766,-0.057`。
- grasp → 反向退出 8 cm：TCP 到 `0.820,0.711,-0.057`。
- 再反向退出 12 cm：累计约 20 cm，TCP 到 `0.733,0.629,-0.058`。
- 各段 feedback 误差为 0，未发生执行超调。

关键事实：

- 夹爪闭合到 0 mm 时没有在两指之间停住，说明**工件当时不在夹爪两指之间**，没有真正夹住。
- 急停发生在靠近插槽/工件、且 MoveIt 场景为空时；已确认“空场景下的无碰撞”不可信。

## 3. 夹爪

设备：HITBOT EFG80，COM9，115200，从机 1。

已验证：

- Modbus 读写：位置/速度/电流寄存器 2/4/6；状态 64/65/66。
- 张开 50 mm、闭合 0 mm、状态读取成功。
- Windows TCP 桥：`127.0.0.1:7500`，命令 `status/open/close/pos`。
- ROS 服务已封装在 `cs625_motion_adapter`：
  - `/cs625/gripper/status`
  - `/cs625/gripper/open`
  - `/cs625/gripper/close`
  - 状态 topic：`/cs625/gripper/status`
- `HITBOT Studio` 不能与桥同时占用 COM9。

## 4. 可视化

- 新文件：`scripts/cs625_ui.py`（PyQt5）。
- VS Code 任务：
  - `13. 启动 CS625 Qt 可视化面板`
  - `14. 打开 RViz（前台窗口）`
  - `15. 启动视觉 mesh + RViz（前台）`
- 新文件：`scripts/visual_mesh_publisher.py`
  - 发布 `box.glb` / `slot_pointcloud.glb` 到 `/visualization_marker_array`
  - 用于在 RViz 显示扶手等完整外观
- 说明：绿色是碰撞体；视觉 GLB 才有完整扶手。

## 5. 环境扫描

RVS E1 生成：

```text
runtime/Model/environment_point_cloud/<时间戳>/environment_point_cloud.pcd
```

已采集的有效视角：

```text
20261021202302017  初始
20261021202621834  J1 +15°
20261021202824183  J1 -15°
20261021202911677  J1 -30°（插槽桌侧）
20261021203637550  J1 -30° + J4 +5°
20261021203717817  J2/J3/J5/J6 微调
20261021203951177  补充插槽桌视角
```

- 7 视角合并：`/tmp/environment_merged7.pcd`，29,484 点（1 cm 降采样）。
- 覆盖图：`/tmp/env_merged7_topdown.png`。
- 工件桌和插槽桌是两张独立桌面；当前桌面 bbox 仍是草稿。
- 点云中包含机器人自身点，尚未做正式自点过滤。

## 6. 碰撞场景

已应用 4 个对象：

```text
box_collision
slot_collision
slot_table（草稿 bbox）
workpiece_table（草稿 bbox）
```

草稿桌面：

```text
workpiece_table: center 0.60, 0.925, -0.15; size 0.90, 0.75, 0.10
slot_table:      center 1.175, 0.225, -0.11; size 0.55, 0.45, 0.10
```

数据文件：

```text
~/cs625_legacy_snapshot_20261020/runs/20261021_collision_scene/
```

## 7. 未解决问题

1. **slot pre-insert 规划失败**：
   - IK 成功（`ik_error=1`）；
   - joint-space plan-only 返回 `99999`；
   - 移除 `box_collision` 后仍失败；
   - 清空碰撞场景后仍失败。
   - 结论：**不是“必须先装上工件”导致的**，也不是当前碰撞体单独阻挡；更像目标姿态/规划器/起点约束问题，仍需后续排查。
2. **桌面 bbox 是草稿**，插槽/工件碰撞 mesh 可能缺少真实扶手等结构。
3. **环境扫描未做正式自点过滤**，也未最终验收覆盖率。
4. **没有真实抓取/抬起/插入**；夹爪闭合到 0 时工件不在两指之间。
5. RViz 的视觉 GLB 是否完整显示仍需你目视确认。

## 8. 当前状态

```text
机器人：插槽拍摄基准位（TCP 1.044,-0.011,0.726）
夹爪：0 mm 闭合
碰撞场景：已恢复 4 个对象
视觉 mesh：运行中
RViz：已启动
机器人未运动
```

## 9. 明天建议顺序

1. 先确认/修正两张桌子的碰撞 bbox。
2. 排查 slot pre-insert 的 `99999`：目标姿态、规划器、起点、容差。
3. 给工件/插槽碰撞体补扶手等真实结构（如果它们是刚性障碍）。
4. 完整环境扫描 + 自点过滤 + 覆盖验收。
5. 再进入真实抓取/插入。

## 10. 数据入口

- 今日 worklog：
  `docs/worklogs/2026-10-21_WORKLOG_PLANONLY_CARTESIAN.md`
- 找视角计划：
  `docs/worklogs/2026-10-21_PLAN_AUTO_VIEW_SEARCH_J1_J4.md`
- 运行数据：
  `~/cs625_legacy_snapshot_20261020/runs/`
- 插槽成功记录：
  `runs/20261021_slot_t1_success/`
- 碰撞场景：
  `runs/20261021_collision_scene/`


## 11. 追加：pre-insert 规划已定位（21:07）

- 按 `tcp_server` 精确公式重算 pre-insert：
  - 位置 `1.099593, -0.012473, 0.377270`
  - 朝向是插入姿态绕局部 y 轴 -10°，再沿新 z 轴退 350 mm，**不是直接用 slot 朝向**。
- 精确 pre-insert 的 IK 成功：
  - `ik_error=1`
  - IK 关节解可作为关节空间目标。
- 用该 IK 解做 joint-space plan-only：**成功，52 点**。
- 但同一精确定位的 pose-goal 规划仍返回 `99999`。

结论：

```text
目标可达
路径存在
不是 collision scene 导致
不是“必须先夹住工件”导致
问题在 pose-goal 约束/规划器，而不是目标本身
```

后续可用“先求 IK，再以关节空间目标规划”的方式绕过 pose-goal 失败；尚未执行。


## 12. 可借鉴的师兄规划入口（21:10）

师兄的 `cs625_trajectory_tools` 中：

- `/cs625/plan_to_pose`：本质仍是 MoveGroupInterface `setPoseTarget` + plan，和我们失败的 pose-goal 相同；
- `/cs625/plan_to_target`：支持 `use_joint_target=true`，走
  `planAndPublishToJointTarget`，即强制用关节目标规划并校验关节端点；
- 任务链的 E5/E6 是 pre-insert / pre-insert-rotated，E7 是插入，E9 是释放。

因此应该借鉴的是：

```text
tcp_server 计算 slot pre-insert
→ IK 求关节解
→ /cs625/plan_to_target (use_joint_target=true)
→ 或本地复用同样的 joint-space 规划
```

而不是继续直接对 slot pre-insert 做 pose-goal 规划。


## 13. pre-insert plan-only 适配器验证成功（21:12）

运行 `scripts/preinsert_plan_only.py`，由 T1 发布最新 slot target：

```text
pre-insert: 1.099605, -0.012507, 0.376953
IK: success
joint plan: success
points: 55
```

轨迹写入 `/tmp/preinsert_plan.json`，未发送 controller goal、未执行机械臂。


## 14. 阶段收敛（21:16，按所有者纠正）

- pre-insert 规划问题在 plan-only 层面基本解决：
  - 精确 pre-insert 位姿；
  - IK 成功；
  - 关节空间规划成功，55 点；
  - RViz 检查无穿模；
  - **尚未执行**，所以不等于插入完成。
- 明天首要任务已纠正为：
  **工件基准位附近主动微调，寻找最佳工件拍摄位。**
  计划文件：`docs/worklogs/2026-10-22_PLAN_SLOT_BASELINE_FINE_SEARCH.md`
  （文件名保留初始名，内容已改为工件优先）。
- 后续顺序：
  工件最佳拍摄位 → 主动移动视角完成环境建模 → 回到工件附近抓取/中转/预插入。
- 插槽固定在桌面，位姿不变；R1、slot_insert、slot_pre_insert 和碰撞数据直接复用，不重新搜索插槽拍摄位。
