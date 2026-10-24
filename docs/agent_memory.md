# Agent Memory

长期备忘。只记会反复用的规则、常量、入口。
工作过程写 `docs/worklogs/`，这里不写流水账。

更新规则：
- 追加新条目时写日期；不删历史结论。
- 稳定规则和会变的数值分开放。
- 机器人移动后，更新“当前状态”。
- 一屏能看完最好；超过就压缩旧内容。

最后更新：2026-10-21

## 1. 铁律

- 先 plan-only，确认后再执行。
- 不跑 100% 速度；低速、分段、每段看 feedback/TCP。
- 急停后人工确认安全，再恢复；不自动续跑。
- 不用未文档化的 raw TCP 运动；能用官方/师兄接口就用。
- 保留有意的 120 mm pre-grasp 偏移。
- 装配动作（抓取、插入、取出）：time_scale ≥ 5.0，规划速度 ≤0.025，段间停顿 ≥1.5 s；插入更慢。
- 装配按小段执行，每段停、读 feedback/TCP，再走下一段。
- 真实和仿真 profile 分开，不混用。

## 2. 每天开始任务前

- 先做一次完整环境扫描（RVS E1）。
- 生成/更新碰撞场景后，才允许接近工件/插槽。
- 空 MoveIt 场景下的“无碰撞”不可信。
- 扫描点云要过滤机器人自身点；E1 原始点云含自点。

## 3. 工件与插槽

- 工件位姿基本在历史 S2 观测朝向附近，不做全周 J1 盲扫。
- 工件拍摄位：历史 S2 附近，局部微调 J4/J5/J6，再 J2/J3、J1 小角度。
- 工件桌、插槽桌是两张不同的桌子。
- 环境只定义：工件桌、插槽桌、工件、插槽；其余算杂物，暂不规划。
- 插槽固定在桌面，物体位姿不变。
- 插槽的 R1、insert、pre-insert、碰撞 mesh 可直接复用。
- 插槽拍摄位不固化，只作为基准；每次按 T1 结果微调。
- J1 正向不是插槽侧；当前工位插槽在负向。正向只到 +15°。

## 4. 视觉命令

- RVS 命令端：`127.0.0.1:7000`。
- 位姿回传：`tcp_server` 的 9999。
- T2：粗采样，返回 S2 观测位。
- T3：在 S2 触发，返回 S3 抓取位。
- T1：插槽采样；位姿对时返回 R1，位姿不对时 NG。
- E1：环境扫描，写入：
  `runtime/Model/environment_point_cloud/<时间戳>/environment_point_cloud.pcd`
- T1/T3 的命令输入是法兰；S2 原值直接作 TCP 观测位，不要再转法兰。
- RVS_AI 必须在线；T3 可能耗时数十秒，不要过早判失败。

## 5. 夹爪

- 设备：HITBOT EFG80。
- 串口：COM9，115200，从机 1。
- 控制寄存器：2=位置，4=速度，6=电流。
- 状态寄存器：64=初始化，65=夹持状态，66=位置。
- Windows 桥：`127.0.0.1:7500`，命令 `status/open/close/pos`。
- ROS 服务：`/cs625/gripper/status`、`/open`、`/close`。
- HITBOT Studio 与桥互斥，不能同时占 COM9。
- 夹爪闭合到 0 不等于夹住工件；要看状态和两指间是否有工件。

## 6. 环境与碰撞

- 工件桌：高约 z=-0.10。
- 插槽桌：高约 z=-0.05。
- 当前碰撞对象：`box_collision`、`slot_collision`、`workpiece_table`、`slot_table`。
- 两张桌子的 bbox 还是草稿，没最终校准。
- `box_collision.STL` / `slot_collision.STL` 是简化碰撞体，可能没有扶手。
- 视觉 GLB 有完整外观，用 `scripts/visual_mesh_publisher.py` 发到 `/visualization_marker_array`。
- 视觉显示不等于碰撞完整；真实扶手若要碰撞，必须另加几何。

## 7. 规划

- pose-goal 规划在 slot pre-insert 上会返回 `99999`。
- 精确 pre-insert 姿态：`tcp_server` 计算，不是直接用 slot 朝向。
- 同一目标：IK 成功；用 IK 关节解做 joint-space 规划成功。
- 结论：pre-insert 用“IK + 关节目标规划”，不要死磕 pose-goal。
- 本地 plan-only 适配器：`scripts/preinsert_plan_only.py`。
- 已验证：pre-insert plan-only 55 点；未执行。
- 师兄可借鉴入口：`cs625_trajectory_tools` 的 `PlanToTarget(use_joint_target=true)`，以及 E5/E6/E7/E9 步骤。

## 8. 数据记录

- 运行记录放 `~/cs625_legacy_snapshot_20261020/runs/`。
- 工作日志放 `docs/worklogs/`，写入后同步 Obsidian。
- 任务关键节点截图 1–3 张，不连续录屏。
- 截图存 `runs/<日期>_<任务>/images/`，路径写进 `meta.json`。
- 文档归档/移动/取代必须所有者批准。

## 9. 工具入口

```text
UI:            scripts/cs625_ui.py
视觉 mesh:     scripts/visual_mesh_publisher.py
pre-insert:    scripts/preinsert_plan_only.py
夹爪桥:        C:\Users\Public\hitbot_bridge.ps1
插槽成功数据:  runs/20261021_slot_t1_success/
碰撞场景:      runs/20261021_collision_scene/
日终 handoff:  docs/worklogs/2026-10-21_HANDOFF_DAY_END.md
```

VS Code 任务：

```text
13. 启动 CS625 Qt 可视化面板
14. 打开 RViz（前台窗口）
15. 启动视觉 mesh + RViz（前台）
```

## 10. 当前状态（2026-10-21 收工）

```text
机器人：插槽拍摄基准位
TCP: 1.044, -0.011, 0.726
q:   [-0.059, 0.938, 0.060, 0.336]
夹爪：0 mm 闭合
碰撞场景：4 个对象已应用
RViz / 视觉 mesh：运行中
机械臂：未运动
pre-insert：55 点轨迹已生成，未执行
```

## 11. 固定数据（2026-10-21）

```text
S2:          0.616272, 0.592403, 0.577179
S2 q:        [-0.414438, 0.776655, 0.159180, 0.446889]
S3:          0.73676, 0.921292, 0.356568
grasp:       0.880, 0.767, -0.057
pre-grasp:   0.792, 0.684, -0.057
slot R1:     1.241629, 0.181709, 0.443245
slot_insert: 1.157, -0.013, 0.032
slot_pre:    1.100, -0.012, 0.377
```

这些是当前工位数据，布局变了要重新确认。

## 12. 下一步

1. 工件基准位附近主动微调，找最佳工件拍摄位。
2. 主动移动视角，完成环境建模。
3. 回工件附近：抓取、中转、预插入。
4. 插槽数据复用，不重搜、不固化拍摄位。


## 13. 2026-10-22 更新

- 机械臂重连成功；arm_controller active。
- 今日环境扫描 5 视角完成，合并 `/tmp/environment_20261022.pcd`（23,608 点）。
- 当前只应用两张桌子的碰撞体；工件/插槽 mesh 尚未加入。
- 工件 S2 候选位：
  - TCP `0.613300, 0.353323, 0.573928`
  - 重复 T2 结果 `0.615353, 0.354540, 0.574506`，波动约 2 mm。
- 机器人已在 S2 候选位；T3 成功：
  - S3 `0.652989, 0.703460, 0.355241`
  - grasp `0.831, 0.590, -0.058`
  - pre-grasp `0.766, 0.489, -0.059`
- 未抓取；仍需在 S2 附近微调比较，确认最佳拍摄位。


## 14. 2026-10-22 S2 微调结论

- 基准 S2：`0.615353,0.354540,0.574506`；基准 S3：`0.652989,0.703460,0.355241`。
- J4 +2° / -2° 候选 S3 差异 <0.1 mm，属于噪声。
- 结论：不再盲目加大微调角度；基准 S2 可作为当前最佳拍摄位。
- 当前机器人停在 J4-2 候选；未抓取；碰撞场景仍只有两张桌子。


## 15. 2026-10-22 pre-grasp plan-only

- 最佳 S2：`0.615353,0.354540,0.574506`。
- S3：`0.652989,0.703460,0.355241`。
- 精确 pre-grasp（box target × T_b_tcp × +120 mm x）：
  `0.7663,0.4893,-0.0593`，q `[-0.4815,0.8764,0.0071,0.0016]`。
- IK 成功；joint plan 成功，116 点。
- 场景已有 box/slot/tables 四个碰撞对象；未执行抓取。


## 16. 2026-10-22 夹爪桥与 pre-grasp

- UI 夹爪离线原因：Windows 桥 `127.0.0.1:7500` 没启动。
- 启动 `C:\Users\Public\hitbot_bridge.ps1` 后，EFG80 状态正常；夹爪张开到 50 mm。
- S2 候选位执行 pre-grasp：5 段全部 status=4。
- 目标 `0.7663,0.4893,-0.0593`；实际 `0.784,0.479,-0.053`。
- 未闭合夹爪、未抓取。


## 17. 2026-10-22 pre-grasp 精确校正

- 校正前偏差约 1.8 cm；Cartesian 9 点校正成功。
- 实际到位：`0.766,0.489,-0.059`，目标 `0.7663,0.4893,-0.0593`。
- 夹爪保持 50 mm 张开；未闭爪、未抓取。


## 18. 装配速度（2026-10-22 所有者要求）

- 抓取、插入、取出都属于装配段。
- 速度上限：10%。
- 默认：5%。
- 插入尤其要慢，优先 2–5%，并分段执行。
- 接近/观察段也不默认全速；按当前低速策略执行。


## 19. 2026-10-22 pre-grasp → grasp plan-only

- grasp 目标：`0.8307,0.5906,-0.0582`
- 朝向：`[-0.4815,0.8764,0.0071,0.0016]`
- IK 成功；关节规划成功，31 点；规划速度 5%。
- 未执行 120 mm 进给，夹爪仍张开 50 mm。


## 20. 2026-10-22 grasp 进给受阻

- pre-grasp → grasp Cartesian 仅 fraction=0.72。
- 4 段以 5% 执行后 TCP `0.813,0.562,-0.059`，未到目标 `0.8307,0.5906,-0.0582`。
- 夹爪保持 50 mm 张开，未闭爪。
- 剩余 28% 被碰撞检查截断；下一步要查是 box_collision/夹爪碰撞还是目标本身在物体内。


## 21. 2026-10-22 夹爪初始化与装配速度修正

- 夹爪必须先初始化；未初始化时闭合可能不到位。
- UI 已增加“初始化夹爪”按钮和初始化状态；初始化未完成时禁用张开/闭合。
- 桥新增 `init` 命令（Modbus 写寄存器 0）。
- 之前 5% 只是规划参数，未验证实际速度；装配执行改用：
  `scripts/execute_assembly_chunks.py --time-scale 2.0 --dwell 1.0`
- 段间必须停顿至少 1 s，检查 feedback/TCP 后再走下一段。
- 抓取/插入/取出速度 ≤10%，默认 5%，插入优先 2–5%。


## 22. 2026-10-22 回撤与重规划

- 回撤到 pre-grasp 成功：`0.766,0.489,-0.059`。
- 使用慢速执行：time_scale=2.0、dwell=1.0。
- 重新规划 pre-grasp → grasp 仍 fraction=0.72。
- 没有重放完整进给；夹爪保持 50 mm 张开。


## 23. 2026-10-22 box 碰撞移除测试

- 临时移除 box_collision 后，pre-grasp → grasp 仍为 fraction=0.72。
- 说明剩余 28% 不只是 box 碰撞导致；可能涉及桌面、夹爪或 planner/path。
- 已恢复 box_collision；未执行；夹爪保持 50 mm 张开。


## 24. 2026-10-22 桌面碰撞隔离

- 移除两张桌子碰撞体后，pre-grasp → grasp fraction=1.0（34 点）。
- 恢复桌子后 fraction=0.72。
- 结论：阻挡来自草稿桌面 bbox，不是 box_collision。
- 桌面 bbox 需要校准后再抓取；未执行进给，夹爪仍 50 mm 张开。


## 25. 2026-10-22 关节路径抓取接近

- 用 IK + joint-space 规划避开桌面，33 点。
- 4 段慢速执行；前 3 段成功，第 4 段 status=6（goal_time -5）。
- 实际 TCP `0.818,0.543,-0.078`，目标 `0.8307,0.5906,-0.0582`。
- 夹爪仍 50 mm 张开，未闭爪。


## 26. 2026-10-22 急停后的恢复流程（重要）

- 每次急停后，必须先重新连接/激活 arm_controller，否则：
  - action goal 可能被接受但机器人不动；
  - 或 action goal 响应超时/REJECTED。
- 推荐顺序：
  1. 释放急停并确认安全；
  2. `resend_external_script`；
  3. `switch_controllers --deactivate arm_controller`；
  4. `switch_controllers --activate arm_controller`；
  5. 确认 `arm_controller active`、50001/50003/50004 ESTAB；
  6. 再发运动命令。
- 本次回撤到 pre-grasp 已通过上述步骤成功。


## 27. 2026-10-22 夹爪进入抓取位与回零

- 夹爪不先收拢，开口太大，进不到工件抓取位。
- 进入抓取位前必须先收拢/闭合夹爪两指；到抓取位后再按抓取流程动作。
- 回零/复位必须使用初始化命令，不能直接 close 到 0。
- 直接闭合到 0 会多出一段冗余，仅用于收拢/抓取，不作为回零。
- UI 已改名：`收拢/闭合（进给前）`、`初始化 / 回零`，并加提示。


## 28. 2026-10-22 夹爪真正零位

- 直接 `close` 到 0 不是真正的机械零位，会留下略长于工件两扶手间距的间隙。
- 进抓取位前必须先执行初始化，让夹爪回到真正 0 位，再收拢/进给。
- 本次初始化：init 1 → 5，位置 -2.4 → 0，state 0。
- 只有初始化完成后，闭合/进给才安全。


## 29. 2026-10-22 真正零位后的抓取进给

- 初始化后真零位，joint plan 32 点。
- 慢速分段：前 3 段成功，第 4 段 status=6（goal_time -5）。
- 实际 TCP `0.804,0.539,-0.080`，目标 `0.8307,0.5906,-0.0582`。
- 未继续硬走、未抓取；最后一段可能存在接触/阻力。


## 30. 2026-10-22 急停后重新观测

- 急停释放后按流程重连控制器。
- 重新 T2：S2 `0.617682,0.353789,0.574123`。
- 重新 T3：S3 `0.653648,0.703244,0.355295`。
- 新 grasp `0.832,0.592,-0.058`；pre-grasp `0.768,0.490,-0.060`。
- 只观测，未移动机械臂；夹爪真零位。


## 31. 2026-10-22 新模型源后继续

- 更换 RVS 模型源后：T2 → S2 `0.617876,0.353517,0.573330`；T3 → S3 `0.653422,0.703592,0.355183`。
- pre-grasp 精确目标 `0.7677,0.4903,-0.0595`；6 段慢速到位后偏差约 2.5 cm，Cartesian 9 点校正到 `0.768,0.490,-0.059`。
- 夹爪保持真零位（init=5，0 mm 收回）用于进入两扶手之间。
- 未执行 grasp 进给。


## 32. 2026-10-22 抓取前必须初始化夹爪（所有者要求）

- 每次执行抓取之前，在**预抓取位**就要提前对夹爪做初始化。
- 目的：保证夹爪处于真正的零位/正确开度，不会因残留间隙或开度过大造成碰撞。
- 顺序：到 pre-grasp → 夹爪初始化（确认 init=5）→ 再进给抓取。
- 与既有规则一致：回零必须用初始化，不要用 close 到 0 代替。

## 33. 2026-10-22 抓取进给被截断的真实原因

- `check_state_validity` 在 grasp IK 状态返回接触对：
  - `box_collision <-> my_end_effector_link` depth 0.001（正常，抓取接触）
  - `workpiece_table <-> camera_link` depth 0.0265（真正阻挡）
- 结论：不是夹爪，也不是整张桌子挡路，而是**相机 link 与工件桌碰撞框相交**。
- 需按相机实际几何/位置修正碰撞框，或校正相机 link 的碰撞体；不要靠删桌子绕过。


## 34. 2026-10-22 体素碰撞场景替代草稿桌面框

- 草稿桌面 bbox 会覆盖没有桌面的自由空间，导致抓取进给被误截断（fraction 0.708）。
- 改用点云体素化生成碰撞体：`scripts/build_environment_collision.py`
  - z 带 -0.13..-0.07，5 cm 体素，202 个体素框，命名 `env_voxel_*`；
  - 移除 `workpiece_table` / `slot_table` 草稿框；
  - 保留 `box_collision` / `slot_collision`。
- 效果：抓取进给 fraction 0.708 → 0.958；grasp 处仅剩 box 的 1 mm 预期接触。
- 抓取到位：目标 `0.8316,0.5918,-0.0583`，实际 `0.829,0.588,-0.058`。
- pre-grasp 处先做夹爪初始化（init=5，0 mm 收回），再执行进给。

## 35. 2026-10-22 环境碰撞使用规则

- 不要用手画大 bbox 表示桌子；会挡掉真实自由空间。
- 用合并点云 + z 带 + 体素生成碰撞体，或按实际桌面范围分段建框。
- 每次重建后，用 `check_state_validity` 看接触对，确认阻挡是预期接触还是误挡。


## 36. 2026-10-22 速度再次下调（所有者反馈）

- 上次 grasp 进给 `time_scale=2.5` 仍偏快；要求至少再慢一半。
- 新标准：
  - 装配段 `time_scale ≥ 5.0`
  - 规划 `max_velocity_scaling_factor ≤ 0.025`
  - 段间 `dwell ≥ 1.5 s`
  - 每段读 feedback/TCP，超差即停
- 成功流程复盘文档：
  `docs/worklogs/2026-10-22_WORKLOG_VOXEL_GRASP_FLOW.md`


## 37. 2026-10-22 师兄任务流（装卸）结构备忘

装载 E0–E9（cs625_task_manager）：

```text
E0 reset IO
E1 pre-grasp   → box_pre_grasp_tcp
E2 enter grasp → box_grasp_tcp
E3 grasp       → 夹爪动作 + confirm_grasp
E4 transit     → 中转点（代码里无固定定义，需人工/外部给定）
E5 pre-insert  → slot_pre_insert_tcp（沿插入轴退 350 mm）
E6 pre-insert rotated → slot_pre_insert_rotated_tcp（原地旋转 -10°）
E7 final insert → slot_insert_tcp（沿 z 直插）
E8 release     → confirm_release
E9 retreat     → home
```

卸载 D0–D9 类似，区别是 D4 pre-remove rotated、D5 remove、D6 transit、D7 place、D8 release、D9 retreat。

## 38. 2026-10-22 抓取/释放时的碰撞对象管理

- `grasp_confirmed`：先从 world 移除 `box_collision`，再 attach 到 `my_end_effector_link`。
- `release_confirmed`：移除 attached，再按 slot 反推位姿放回 world：
  `T_world_box = T_world_slot @ T_s_tcp @ inv(T_b_tcp)`。
- 事件话题：`/cs625/grasp_state_event`（String）。

## 39. 2026-10-22 插入速度监督（cs625_compliant_placement）

- 通过 `/io_and_status_controller/set_speed_slider` 改速度，不直接发运动。
- 阶段与速度：SOFT_APPROACH 0.10 → APPROACH 0.30 → CONTACT_HOLD 0.05。
- 接触：tau_metric ≥ 0.8；安全阈值 2.0。
- 卡滞：progress_window 0.30 s、epsilon 0.5 mm、jam_confirm_cycles 10。
- 目标容差 3 mm。


## 40. 2026-10-22 中转点选定

- 中转点：`0.95, 0.45, 0.35`，朝向沿用 grab 朝向 `[-0.4835,0.8753,0.0075,0.0018]`。
- 位置依据：两桌之间的空隙带，z 高于桌面约 0.40 m，IK/无碰撞/可继续到 slot_pre_insert 全部通过。
- 备选：`1.0, 0.45, 0.35`。
- slot_pre_insert（由固定插槽位姿算）：`1.0996, -0.0125, 0.3773`。
- 中转到 pre-insert：168 点 / 16.7 s。

## 41. 2026-10-22 OMPL 规划不具确定性

- 同一 start/goal，两次规划分别得到 301 点/30 s 与 763 点/76 s。
- 不能把单次规划时长当作固定指标；执行前要重新规划并检查段数与时长。
- 中转这类大位移，优先考虑“退出 → 抬升 → 平移”的分段 Cartesian，而不是一次关节空间长路径。

## 42. 2026-10-22 抓取后退出顺序（建议）

```text
夹紧
→ 沿插入轴反向外退 120 mm（回到 pre-grasp 位置）
→ 垂直抬升 10–15 cm
→ 再前往中转点
```
理由：夹爪在两扶手之间，必须先退出才能抬升/平移。


## 43. 2026-10-22 抓取后退出顺序：规划实测

把 `box_collision` attach 到 `my_end_effector_link` 后，从 grasp 位做 Cartesian 测试：

```text
抬升 10 mm : fraction 1.000, 11 点
抬升 30 mm : fraction 1.000, 19 点
抬升 50 mm : fraction 1.000, 26 点
后退 60 mm : fraction 1.000, 31 点
后退 120 mm: fraction 1.000, 54 点
```

结论：规划层面两种顺序都无碰撞，但碰撞检查不反映“手指沿扶手内壁刮擦”这类接触风险。

实测几何（工件 mesh + S3 位姿 + 点云桌面高度）：

```text
工件底面 world z = -0.1085
工件桌桌面 z    = -0.0972   → 底面低于桌面 1.1 cm（平退会撞桌）
插槽桌桌面 z    = -0.0455   → 底面低于插槽桌面 6.3 cm
工件底面相对 TCP 低约 5.0 cm
```

推荐顺序（物理优先，含桌面高度差）：

```text
1. 夹紧（低速低电流）
2. 抬升 20–30 mm（清掉工件桌，余量 1–2 cm），停 1 s 确认带起
3. 沿入刀方向后退 120 mm
4. 再抬升到搬运高度（TCP ≈0.35，工件底面约 0.30，远高于两张桌）
5. 前往中转点
```

插入位校核：slot_insert TCP z=0.032 → 工件底面约 -0.018，高于插槽桌面 2.7 cm，可插入。

原因：抓取是在两扶手间隙内张开夹紧；大幅垂直抬升会让手指沿扶手内壁上滑，而水平后退是已验证入刀路径的反向。

注意：真实夹住后必须发 `grasp_confirmed`，让 box 从 world 转为 attached，否则规划仍把它当障碍。


## 44. 2026-10-22 夹爪夹持与电机噪声（所有者反馈）

- 轻微夹紧（目标略大于接触宽度）会让电机持续憋劲、一直叫。
- 正确做法：夹到接触宽度后把目标回设到接触值（state=0 到位），停止持续施力。
- 本次接触宽度：扶手内间距约 52.2 mm；目标 55/60 推不动，停在 52.19。
- 目标回设 52 后 position=52、state=0，电机安静。

## 45. 2026-10-22 夹爪控制通道改为一次性命令

- 常驻 TCP 桥（127.0.0.1:7500）被杀后无法再启动，UI 显示夹爪离线。
- 改为一次性脚本 `C:\Users\Public\hitbot_cmd.ps1`：
  `-Cmd status|init|open|close|pos|grip -Value <mm>`
- UI 已改为 subprocess 调用该脚本，不再依赖 7500 端口。
- UI 新增“夹紧（低速低电流）”按钮 + 目标 mm 数值框（默认 52）。
- 常驻桥不可用时，一次性命令仍可工作。

## 46. 2026-10-22 场景状态（当前）

- box_collision 已 attach 到 `my_end_effector_link`；world 中已移除（203 个对象）。
- 夹爪 52 mm、state=0，处于接触但未持续施力。
- 尚未抬升、未搬运。


## 47. 2026-10-22 抓取成功（真机）

- 夹爪接触宽度约 52.2 mm；目标回设 52 mm 保持，state=0，电机安静。
- 所有者目视确认：抬升时**工件跟随**，说明夹持有效。
- 抬升分 4 段（5/50/50/50 mm），累计 **15.5 cm**，TCP z 由 -0.058 到 0.097。
- 后退 120 mm（两段 60 mm），TCP x 由 0.829 到 0.709，z 保持 0.097。
- 工件底面估计 z≈0.047：高于工件桌 14.4 cm、高于插槽桌 9.3 cm。
- 执行参数：planning velocity 0.02、time_scale 5.0、dwell 1.5 s。

## 48. 2026-10-22 抬升量修正（所有者要求）

- 只抬 20–30 mm 不够；搬运前至少抬 **15 cm**，避免工件蹭到两张不同高度的桌面。
- 15 cm 抬升后工件底面才稳定高于插槽桌面。


## 49. 2026-10-22 险情：搬运接近插槽时几乎碰撞（急停）

- 现象：从转运行进到 slot_pre_insert 过程中，末端/工件几乎撞到插槽框，所有者急停并手动抬高末端。
- 直接原因候选：
  1. 附着的碰撞体用的是 `box_collision.STL`（简化、无扶手），真实工件外廓更大；
  2. slot_pre_insert 由固定插槽位姿计算，但搬运路径经过插槽框附近，安全余量不足；
  3. 第 1 段执行 TIMEOUT，说明命令未按预期完成，位置与规划不一致。
- 教训：
  - 附着工件做碰撞检查时，**必须用完整外廓或加 padding**，不能只用简化 collision mesh；
  - 接近插槽前应先在插槽外侧设立“安全接近点”，再低速直线接近；
  - 任何 TIMEOUT 都必须先停下查原因，不能继续下一段。
- 处置：急停已触发，机械臂未继续运动；待释放急停后按流程重连控制器再评估。


## 50. 2026-10-22 到达插槽预插入点

- 附着工件碰撞体放大 1.10 倍（补足简化 mesh 缺少扶手的问题）。
- 新增插槽外侧安全接近点 `1.0748,-0.0122,0.5252`（预插入点沿插入轴再退 150 mm）。
- 执行：安全接近点（183 点）→ 直线低速进入预插入点（158 点，速度 0.01，time_scale 3）。
- 实际到位 `1.100,-0.012,0.377`，与目标一致；夹持仍 52 mm。
- 插入未执行。

## 51. 2026-10-22 E6/E7 语义待确认

按 tcp_server 公式：
```text
insert   = slot × T_s_tcp               位置 1.1574,-0.0132,0.0321
rotated  = insert × Ry(-10°)            **位置与 insert 相同**，朝向偏 -10°
pre_insert = rotated × (−350 mm z)      位置 1.0996,-0.0125,0.3773
```
因此 E6 `pre_insert_rotated` 的目标其实在插槽入口处（350 mm 内），
与“先在远处旋转对齐、再直插”的直觉不一致。

进插槽前必须与所有者确认 E6/E7 的真实动作序列，不能凭推断执行。


## 52. 2026-10-22 附着碰撞体位姿被忽略（重要）

- 现象：RViz 里工件碰撞体显示在夹爪原点附近，位置和真实工件不符。
- 原因：通过 `/apply_planning_scene` 下发的 AttachedCollisionObject，
  MoveIt 把 `mesh_poses` 存成**单位变换**（0,0,0 + identity），
  无论用 link 帧还是 world 帧下发，查询回来都是 identity。
- 后果：附着碰撞体位置错误 → 碰撞检查低估工件外廓 → 上次接近插槽险些碰撞。
- 解决：不再依赖 attach，改为 **世界系 box 跟随 TCP**：
  `scripts/update_grasped_box.py`
  `T_world_box = T_world_tcp @ T_link_box`，每次规划/执行前更新。
- `T_link_box` 由抓取时刻的 TCP 与 box target 位姿算出，约 (-0.009, 0.215, -0.411)。

## 53. 2026-10-22 工作体位姿语义

- `/box_target_pose` 是工件原点（ACSb 语义），mesh 本地 z 从 -0.459 到 +0.006。
- 即原点接近工件顶部，mesh 向下延伸约 46 cm。
- 抓取 TCP 在工件中部偏下（z≈-0.058），所以 box 原点比 TCP 高约 0.41 m。


## 54. 2026-10-22 插入被挡的真实原因（自碰撞）

- 插入 Cartesian 只到 fraction 0.17–0.26 就停。
- `check_state_validity` 接触对显示：
  `box_collision ↔ wrist_2_link / wrist_3_link / my_end_effector_link`
  即**工件碰撞盒与自身腕部相交**，不是撞插槽。
- 原因：`box_collision.STL` 是实心块，把夹爪进入的开口区域也包住了；
  师兄用 attached 的 `touch_links` 豁免这类自碰撞。
- 本环境 `/apply_planning_scene` 会把 attached 的 `mesh_poses` 存成 identity
  （link 帧/world 帧都试过），所以 attach 方案不可用。
- 手工下发 ACM 会抹掉 SRDF 默认的自碰撞豁免，导致相邻 link 反而报碰撞，不可用。
- 结论：需要**把工件碰撞模型拆成避开夹爪区域的两块**（或只保留下半部），
  才能既贴合真实外廓又不与自身碰撞。


## 55. 2026-10-22 工件碰撞建模的结论（重要，勿再走弯路）

尝试过的做法与结果：

```text
1. attach + touch_links（师兄做法）
   → 本环境 /apply_planning_scene 丢弃 attached 的 mesh_poses，位姿永远是 identity，不可用
2. 手工下发 ACM 豁免
   → 覆盖 SRDF 默认豁免，相邻 link 报假碰撞，更糟；只能靠重启 move_group 恢复
3. world 系实心 mesh 跟随 TCP
   → 位姿正确，但实心块包含夹爪开口 → 与腕部自碰撞，插入 fraction 0.17–0.26
4. 点云体素模型（忠实）
   → 位姿正确、几何真实，但仍与腕部/手指接触，插入被挡
5. 点云体素 + 挖空夹爪通道
   → 规划可过，但删掉了 132/261 体素，模型失真，不能作为安全碰撞模型
```

结论：

- **不要用“挖空几何”来绕过自碰撞**，那会让碰撞检查失效。
- 正解有两条：
  - A. 让 attach 真正生效（例如运行师兄的 `collision_object_publisher` 节点，
    它用 `touch_links` 原生豁免自碰撞）；
  - B. 插入阶段改用师兄的 `cs625_compliant_placement` 力/接触监督，
    配合插槽区域扣除，而不是纯靠碰撞检查。
- 当前状态：已恢复忠实体素模型（261 个体素，无挖空）；机器人在预插入点静止。


## 56. 2026-10-22 方案 A 验证通过（师兄 attach 可用）

- 启动师兄 `collision_object_publisher`（tcp_bridge），参数指向我们的 mesh：
  `slot_mesh_path` / `box_mesh_path`，`attached_link=my_end_effector_link`。
- 依赖 `trimesh`，本机未装 → 写了最小 shim：
  `scripts/shims/trimesh.py`（仅支持 STL 的 load/vertices/faces/is_empty）。
- 发 `grasp_confirmed` 后：
  - 日志显示 attached box pose 正确 `(-0.004, 0.215, -0.411)`；
  - 但查询场景时 `mesh_poses` 仍显示 identity（MoveIt 存储层如此），
    **实际碰撞模型位置正确**，因为插入位姿检查已无自碰撞。
- 效果：
  - 插入 entrance / final 两个位姿 `check_state_validity` 均 valid=True，无接触；
  - 350 mm 插入 Cartesian 由 fraction 0.17 → **0.597**。
- 剩余 ~40%（约 140 mm）仍被挡，属于工件进入插槽的实际接触段。

结论：
- 搬运期工件碰撞建模用师兄的 attach 节点（方案 A）成立。
- 最后接触段必须靠师兄的 `cs625_compliant_placement` 力/接触监督（方案 B），
  不能期待纯碰撞检查通过。


## 57. 2026-10-22 插入执行（部分完成）

- 顺序：pre_insert（-10° 倾角）→ 直进 350 mm → 入口 → 转正。
- 碰撞处理：工件用师兄 attach（touch_links 生效），插入段移除 slot_collision（等同师兄的插槽扣除）。
- 执行：7 段，每段 time_scale 2.0–3.0、dwell 1.0，全部 status=4，除最后一段。
- 实际：TCP `1.153,-0.013,0.058`；目标 `1.1574,-0.0132,0.0321`，**剩约 26 mm**。
- 最后一段 `Aborted due to goal_time_tolerance exceeding by 38 s`，说明近接触段无法按轨迹收敛。
- 未强行继续压入；夹爪仍 52 mm、state=0。
- compliant_placement 会在 done/aborted 后自复位为 IDLE，不适合当连续守门；
  需要每次执行前重新 start。


## 58. 2026-10-22 插入到位（已坐入插槽）

正确顺序（所有者纠正）：
```text
E5 pre_insert（-10° 倾角，350 mm 外）
→ E6 保持倾角直线进到底（不先转正）
→ E7 到位后原地转正 10°
```

本次结果：
```text
insert 目标 1.1574,-0.0132,0.0321
insert 实际 1.157, -0.013, 0.032
转正前 ry≈9.5°，转正后 ry≈-0.49°
```

卡滞与恢复（重要经验）：
- 卡在距目标 26 mm 时，前后都动不了；不是纯机械卡死。
- 真因：多次 goal_time 中止后控制器进入不可执行状态；另外 compliant 节点把速度滑块压到 10%。
- 恢复：`resend_external_script` + deactivate/activate arm_controller + 恢复 speed slider=1.0。
- 恢复后以 10 mm 步长（速度 0.5%、容差 0.002 rad）补完剩余段。

注意：小步移动时关节容差必须 ≤0.002 rad，否则 0.02 rad 会直接判定到位而实际不动。


## 59. 2026-10-22 险情：释放后在插槽内直接运动（急停级）

- 现象：夹爪尚未完全退出、机械臂也未到安全距离时就开始旋转/移动，几乎发生碰撞。
- 根因：我把“退出”和“回基准位”混在一次规划里，规划器返回的轨迹包含旋转，
  而当时末端仍在插槽内。
- 强制规则（释放后的退出顺序，必须遵守）：
```text
1. 先收拢手指并初始化（init 到 5、position=0）
2. 纯平移沿插入轴退出，不许旋转，逐步到离开插槽 ≥150 mm
3. 确认 TCP 与插槽有足够距离
4. 才允许旋转/移动到插槽拍摄基准位
```
- 任何一步失败（TIMEOUT/status≠4）都必须停下，不允许带着不确定状态继续下一段。


## 60. 2026-10-22 效率规则（所有者要求）

- 遇到 TIMEOUT / status≠4 时，**不要反复重试或等待**，立即执行控制器恢复：
  `bash scripts/recover_control.sh`
  （resend_external_script → deactivate → activate → speed slider 1.0）
- 恢复后直接重试同一条轨迹，不再做多余探测。
- 经验：多数“命令不动”都是这个原因，恢复后一次即可通过。


## 61. 2026-10-22 端到端任务跑通（真机）

```text
S2/S3 观测 → pre_grasp → 夹爪初始化 → 夹紧(52.2mm) → grasp 进给
→ 抬升 15.5cm → 中转 0.95,0.45,0.35
→ 安全接近点 → pre_insert 1.0996,-0.0125,0.3773（-10°）
→ 带倾角直线插入到 1.157,-0.013,0.032
→ 原地转正（ry 9.5° → -0.49°）
→ 释放（收拢手指 51.4→30→20→init 0）
→ 纯平移退出 40+40+50+50mm
→ 回插槽拍摄基准位 1.039,-0.017,0.740
```

关键事实：
- 工件确实被夹住并搬运（所有者目视确认跟随）。
- 插入全程未发生碰撞，两次急停都是流程/间距问题，不是模型错误。
- 退出必须“先初始化手指 → 纯平移离开 → 再旋转/平移到基准位”。
- 控制器在 goal_time 中止后会失效，用 `scripts/recover_control.sh` 一次恢复即可。


## 62. 2026-10-22 编排脚本（A 步骤）

- 新增 `scripts/run_pick_and_place.py`：13 个阶段的参数化编排
  （pre_grasp → gripper_init → grip → grasp → lift → transit → staging
   → pre_insert → insert → straighten → release → withdraw → baseline）。
- 新增 `config/pick_place_real.yaml`：本次验证过的全部位姿与速度参数。
- 内置规则：
  - 每阶段分段执行 + 段间 dwell；
  - 任一分段失败 → 自动 `scripts/recover_control.sh` → 重试一次；
  - `release` 之后 `withdraw` 只沿插入轴平移；
  - 默认 dry-run，必须 `--confirm` 才执行运动；
  - 每阶段记录 TCP 到 `--log` JSON。
- 已 dry-run 通过（13 阶段正确枚举），尚未跑真实全流程。


## 63. 2026-10-22 师兄代码与本仓库的合并映射（重构依据）

实测结论：能复用的直接用，不能用的写清原因，避免再试一遍。

| 能力 | 师兄实现 | 我们的现状 | 结论 |
|---|---|---|---|
| 位姿链（S2/S3/R1/insert/pre_insert） | `tcp_server` | 直接运行其二进制 | **复用**，不再自写公式 |
| 工件/插槽碰撞体 | `collision_object_publisher`（attach + touch_links） | 已用其二进制 + `scripts/shims/trimesh.py` | **复用**，不要自己写 ACM |
| 插槽区域扣除 | `environment_collision_generator` | 暂无，插入段手工移除 slot | 待接入（重构项） |
| 插入接触/卡滞监督 | `cs625_compliant_placement` | 已能启动，服务可用 | **复用**，需 raw_state 输入 |
| 机器人原始状态（含关节力矩） | `cs625_state_monitor/raw_state_receiver_node` | 已跑通，9.2 Hz，占用 30001 第三连接 | **复用**，注意与驱动共存 |
| 任务状态机 E0–E9 | `cs625_task_manager` | 未接入，我们用自己的编排脚本 | 待比对（重构项） |
| 力补偿笛卡尔 | `cs625_trajectory_tools` cartesian_force_compensated | 未用 | 后续插入可选 |
| 插槽/工件视觉模型 | `static_model_publisher` | 我们自写 `visual_mesh_publisher` | 可合并（重构项） |

重构建议（不急着动代码）：
```text
1. 位姿与碰撞：全部走师兄节点，我们只做“调用 + 校验”，删掉重复实现
2. 编排：把我们的 run_pick_and_place.py 作为上层，底层调用师兄的
   tcp_server / collision_object_publisher / compliant_placement
3. 状态机：若要接 cs625_task_manager，先比对 E0–E9 与我们的 13 阶段差异
4. 视觉显示：把 visual_mesh_publisher 合进 static_model_publisher 的用法
```

## 64. 2026-10-22 已写入代码的安全约束

`scripts/run_pick_and_place.py` 现在会拒绝不合规配置：
```text
velocity_scale        ≤ 0.05
assembly_velocity_scale ≤ 0.02
safe_clear_mm         ≥ 100
goal_tolerance        ≤ 0.005 rad
insert 行程 < 100 mm  视为配置错误
```

文档入口（不要新建重复文档）：
```text
长期规则与经验：docs/agent_memory.md（本文）
单次流程复盘：docs/worklogs/2026-10-22_WORKLOG_VOXEL_GRASP_FLOW.md
日终交接：    docs/worklogs/2026-10-21_HANDOFF_DAY_END.md
```


## 65. 2026-10-22 大幅 J4 旋转找视野（所有者想法）

- 现状：相机基本朝下拍桌面，视角差时只能微调，收效有限。
- 新策略：**大幅旋转 J4（wrist_1）**，把相机从“朝下”改成“朝前/侧前”，
  以获得更广视野，用于自动寻找最佳拍摄位。
- 这是**大幅调整**，不是原来的几度微调。
- 执行要求（按今天经验）：
  - 先 plan-only，确认大角度旋转不与料盒/插槽/桌面干涉；
  - 分段执行（大角度一次走完容易超时或撞），每段后读 TCP/姿态；
  - 旋转前保证末端已离开障碍 ≥10 cm；
  - 相机朝向变化后，S2/S3 的视觉语义会变，需要重新 T2/T3 验证。
- 搜索顺序建议：J1 粗定位 → **J4 大幅换视角** → J2/J3 微调高度 → 小步精修。

## 66. 2026-10-22 六维力传感器（后续）

- 当前只有关节力矩（`/cs625/raw_state` 的 joint_torques），无可用的 TCP 六维力。
- 后续会加装六维力传感器，届时：
  - `cs625_compliant_placement` 的 `use_force_contact` 可置 true，用真实接触判定；
  - 插入/装配可改为力控主导，而不是靠位置+卡滞推断；
  - 需要同步更新 raw_state 的 `actual_tcp_wrench` 字段来源。

## 67. 2026-10-22 响应速度问题：RVS 与重连

- RVS 响应慢：T3 约 20–40 s，T1 对位不当会直接 NG；急停后重新开始也慢。
- 我方的“重连慢”已解决：
  `scripts/control_health.py` 秒级检查（socket / 控制器 / joint_states），
  `--fix` 时调用 `scripts/recover_control.sh` 自动恢复；
  编排脚本在**每个阶段前**做一次健康门，不健康就先恢复，不再干等。
- RVS 侧建议（待实施）：
  - 常驻 RVS_AI，避免每次重启推理；
  - 触发后按超时轮询结果，超时只重发一次，不连续重试；
  - 缓存最近一次有效位姿并带时间戳，陈旧数据直接拒绝；
  - 把“等待 RVS”与“机械臂动作”解耦，等待期间不做运动。


## 68. 2026-10-22 固定位姿思想（所有者结论，卸载的前提）

- 插槽机械固定 → 装入后的工件位姿也是**永久固定**的。
- 因此卸载不需要重新做 T1/S3 定位，位姿可直接由固定链推出：
```text
insert TCP        = slot_origin × T_s_tcp          = 1.1574, -0.0132, 0.0321
box_in_slot       = insert TCP × inv(T_b_tcp)      = 1.1635, 0.2032, 0.4423
pre_insert (350)  = 1.0996, -0.0125, 0.3773
```
- 视觉的作用降级为**印证**：拍一张插槽照片，确认工件在、没有明显歪斜即可，
  不需要精确重定位。
- 配置已写入 `config/unload_real.yaml`。

注意：若上一次插入本身有偏差（今天我们最后是转了 10° 就位），
照片印证能抓到“明显没坐正”的情况，但抓不到 1–2 mm 级偏差。


## 69. 2026-10-22 卸载流程已实现（脚本层）

- `scripts/run_pick_and_place.py --flow unload --config config/unload_real.yaml`
- 阶段：d1_staging → d2a_entrance_tilted → d2b_straighten → d3a_init →
  d3b_grip → d3c_pull_test → d3d_pull_check → d3e_return → d4_tilt_for_removal →
  d5_extract_* → d6a_lift → d6b_transit → d7a_place_pre_grasp → d7b_place_grasp →
  d8_release → d9a_withdraw → d9b_lift → d9c_baseline
- 复用：健康门、分段执行、失败自动重连、release 后先平移再旋转。
- 新增 `--until <phase>`：可只跑到指定阶段，便于逐段验收。
- 已 dry-run 通过（18 个阶段枚举正常），未执行运动。


## 70. 2026-10-22 卸载流程修正 + 拍照印证要求

- 修正：卸载进入段必须拆成三段
  `d2a_pre_insert（staging→pre_insert）→ d2b_entrance_tilted（350 mm 进到入口）
   → d2c_straighten（原地转正）`
  之前把 staging→入口 合成一段（约 500 mm），1% 速度下控制器 goal_time 超时。
- 要求：**进入抓取前必须先拍照印证**，即使是固定位姿也要比对。
  本次照片 `ImageSave_slot_near/20261022202400848/color.png` 显示工件在槽内、基本坐正。
- RViz 看不到实体是因为为规划把 box/slot 从场景移除；验证后必须恢复，
  否则可视化与实际不一致。


## 71. 2026-10-22 危险动作根因：失败后重放旧轨迹

- 现象：卸载进入过程中出现一次突然的大幅上升，很危险。
- 根因：分段执行失败后，我的恢复逻辑把**同一段轨迹重新播一次**；
  此时机械臂已经在该段中途，重播会让它先跳回该段起点 → 大幅突跳。
- 修复（已写入 `scripts/run_pick_and_place.py`）：
  - 分段失败**不再重放**，直接返回，由上层**从当前实际状态重新规划**；
  - 加保护：若某段首点与当前关节差 > 0.35 rad，拒绝执行该段；
  - 每个阶段最多重新规划 3 次，仍失败则停止。
- 规则：**任何“重试”都必须是“重新规划”，不能是“重放旧轨迹”**。

## 72. 2026-10-22 夹爪初始化时机（卸载）

- 进入插槽**之前**必须初始化夹爪（残留间隙会撞）。
- 放置完成、机械臂**退出之前**必须再初始化一次。
- 已加入卸载流程：`d2_init_before_slot`、`d8b_init_before_retreat`。


## 73. 2026-10-22 可视化面板“夹爪离线”修复（编码问题）

- 现象：UI 日志刷 `'utf-8' codec can't decode byte 0xd3`，夹爪显示离线。
- 根因：PowerShell 用中文控制台代码页（GBK）输出错误/提示，Python 按 utf-8 解码失败。
- 修复：
  1. `scripts/cs625_ui.py`：subprocess 取 bytes，按 utf-8 → gbk → cp936 依次尝试，
     最后 errors="replace" 兜底；
  2. `C:\Users\Public\hitbot_cmd.ps1`：`[Console]::OutputEncoding = UTF8`
     强制统一输出编码。
- 注意：PowerShell 脚本里 `param()` 必须是第一条语句，
  编码设置只能放在 `param()` 之后，否则脚本直接报参数错误。
- 编排脚本里的同类问题也已按同样方式修好。


## 74. 2026-10-22 夹爪持续响声的成因（重要）

- 现象：夹紧后电机一直响；把目标回撤到 50 mm 就不再响。
- 成因：这类“向外撑开”的电爪靠**持续保持电流**维持夹力。
  目标设为略大于接触宽度（如 52 mm，接触约 52.2 mm）时会一直施力 → 持续响。
- 实践规则：
  - 需要真夹持时接受一定持续电流，但要把电流限值调低（0.1–0.15 A）；
  - 仅需定位/进入时，把目标收到明显小于接触宽度（如 20–30 mm），不施力；
  - 听到异响先降电流或回撤目标，不要长期让它顶住。
- 助手脚本 `hitbot_cmd.ps1` 已加串口重试（COM9 被 UI 轮询抢占时不再直接失败），
  UI 轮询从 2 s 放慢到 4 s。


## 75. 2026-10-22 夹爪行程与响声的定量结论

实测（0.1 A 最小电流）：

```text
目标 52.0 → 到 52.0, state=0   不施力（没有真正夹紧）
目标 52.5 → 卡在 52.231, state=1  轻夹，有持续保持电流（会响）
目标 53.0 → 卡在 52.231, state=1  夹更紧，仍受 0.1 A 限流
目标 54.0 → 卡在 51.77,  state=1
```

结论：
- 实际扶手内间距约 **52.2 mm**；
- 这类电爪**没有自锁**，保持夹力必须持续通电 → **夹持时一定会有电流声**；
- “行程设得比间隙长”确实会导致一直顶住，52 mm 是“刚好不顶但也几乎不夹”，
  52.5 mm 才是“轻夹且有限流保护”的合理值；
- 已把两套配置改为 `grip_width_mm: 52.5` + `grip_current: 0.1`，
  编排脚本支持传 `-Current`。
- 策略：只在必须夹持时保持通电，尽量缩短夹持时长；听到异响先降电流。


## 76. 2026-10-22 夹爪夹持宽度定标（修正）

- 所有者经验：**50 mm 已完全顶住扶手**；52 mm 之后电源一直显示约 1 A（持续顶住）。
- 归零后有偏移误差，**显示位置与实际开度不完全一致**，因此不能用显示值当物理真值。
- Modbus 寄存器 70 的电流反馈在本设备上读数不可信（50 mm 报 2.02 A、52 mm 报 0）。
- 因此定标为：
  - 夹持宽度 **50 mm**（两套配置已改）；
  - 电流限值 0.1 A；
  - 以**电源显示电流**作为实际受力的直观依据：持续 >0.5 A 说明在硬顶，应降目标。
- 修正此前“接触 ≈52.2 mm”的说法：受归零偏移影响，实际接触约 50 mm。


## 77. 2026-10-22 严重错误：init 未完成就发 grip（必须记住）

- 现象：卸载抓取后夹爪停在 `init=1, position=-2.4`，既没张开也没夹住，
  同时机械臂位置偏了约 6 mm，后续对不上。
- 根因：
  1. `init` 需要约 10 s；我在发出 init 后约 1 s 就发了 grip，
     grip 被忽略/覆盖，夹爪留在初始化中途；
  2. 轻拉测试把末端拉离了 6 mm，而“返回”这一步在测试后没有执行，
     所以位置没回到插入位。
- 修复（已写进 `scripts/run_pick_and_place.py`）：
  - `wait_gripper_init()`：轮询直到 `init==5` 才继续；
  - `grip_and_wait()`：发出 grip 后轮询直到 `state==0` 且位置与目标差 ≤1 mm；
  - 卸载流程 d3 改为“init → 等待完成 → grip → 等待到位”。
- 通用规则：
  **任何夹爪命令之后都要确认状态到位，才能让机械臂继续动。**


## 78. 2026-10-22 初始化过于频繁（已修正）

- 现象：每次跑流程都执行 init，约 10 s 校准动作，反复触发很烦。
- 原因：流程把 init 写成无条件步骤；重跑/分段执行时会重复。
- 修复：`ensure_gripper_initialized()` —— 先读状态，
  `init==5` 就跳过并记录 "already initialised；skipping init"，
  只有未初始化（急停/掉电/异常后）才真正执行 init。
- 用于：进入插槽前、抓取前、释放退出前。


## 79. 2026-10-22 抽出速度过快（所有者纠正）

- 现象：d5 抽出比装入时明显快，不符合装配速度要求。
- 原因：卸载配置沿用了装入的 `time_scale: 2.0` 与 `extract_steps_mm: 40`，
  实际等效速度大约快一倍以上。
- 修正（`config/unload_real.yaml`）：
  - `extract_steps_mm: 40 → 10`（每段 10 mm）
  - `time_scale: 2.0 → 5.0`
  - `dwell_sec: 1.0 → 1.5`
- 原则：**抽出比装入更保守**，因为是在已装配状态下受力拔出。


## 80. 2026-10-22 来回抽搐的根因（危险，必须杜绝）

- 现象：抽出时“出去 → 瞬间回撤 → 出去 → 瞬间回撤”地反复。
- 根因：失败后**重放了同一段旧轨迹**。
  旧轨迹的首点是“上一次规划时”的位置，比当前实际位置更靠里；
  控制器执行时先让机械臂回到该首点 → 表现为瞬间回撤，然后再次向外。
  每次失败重试就重复一次 → 来回抽搐。
- 修复（三层）：
  1. 编排脚本：分段失败不再重放，改为“重连 + 从当前状态重新规划”；
  2. 抽取脚本：同样改为重新规划，并比较首点与当前关节（>0.05 rad 就重规划）；
  3. 执行器 `execute_assembly_chunks.py`：新增 `--max-start-offset`（默认 0.10 rad），
     首点与实测关节偏差超限直接拒绝执行并返回 FIRST_POINT_FAR。
- 原则：**轨迹只能从当前真实状态开始生成，绝不复用旧轨迹**。


## 81. 2026-10-22 抽搐真正根因：规划起点使用陈旧关节状态

- 现象：修掉“重放”后仍在原地来回（抽出→回撤→抽出）。
- 真因：规划请求里我显式填了 `start_state = 本节点最后收到的 /joint_states`。
  执行时该节点不在 spin，状态比实际落后约一步（~10 mm）；
  下一段规划就以“落后 10 mm”的位置为起点 → 控制器先把机械臂拉回该起点，
  再向外走 → 静止看就是来回抽动。
- 另一个放大器：执行器首点保护阈值 0.10 rad 太松（10 mm 仅约 0.02 rad），
  拦不住这种偏移。
- 修复：
  1. **规划请求不再填 start_state**，让 MoveIt 用它自己监测的实时状态（125 Hz）；
  2. 执行器 `--max-start-offset` 默认收紧到 **0.03 rad**；
  3. 抽取循环增加**进展检查**：单步位移不足请求量 50% 立即停止。
- 通用原则：**规划起点必须来自 MoveIt 的实时状态或新鲜订阅，绝不能用本进程缓存的旧值。**


## 82. 2026-10-22 日终状态与入口

- 装入全流程真机跑通；卸载推进到 d5 抽出约 130 mm 后**工件卡在插槽内**。
- 物理状态：
```text
安全模式 1（正常）
TCP 1.125, -0.015, 0.158，-10° 倾斜
夹爪 50 mm 夹持中
工件卡阻，不要直接继续拉
插槽/工件碰撞体当前已从场景移除
```
- 明天先脱困（2–3 mm 反向微动，两次无效即停），再继续卸载。
- 完整交接：`docs/worklogs/2026-10-22_HANDOFF_EVENING_UNLOAD.md`
- 单次流程复盘：`docs/worklogs/2026-10-22_WORKLOG_VOXEL_GRASP_FLOW.md`


## 83. 2026-10-22 日终复核：最后一次“不动”是链路掉线

- 复核 `ss -tn`：`50001/50003/50004` 全部断开，只剩 `30001/29999`。
- 因此最后一次“goal 接受但不动、31 s 后 abort”是**反向控制链路掉线**，
  不是机械卡死（急停后未重新 resend 就会这样）。
- 明天顺序：
```text
1. bash scripts/recover_control.sh        恢复 50001/50003/50004
2. 再判断工件是否真的卡在插槽（目视 + 力/位移）
3. 若真卡：2–3 mm 反向微动 → 或减小倾角 → 或人工
```
- 今天所有脚本已通过 `python3 test/contract_checks.py`（去掉了硬编码路径/IP）。


## 84. 2026-10-23 严重违规：未经同意松开夹爪（最高优先级铁律）

- 事实：在末端可能悬空、工件可能附着的情况下，我直接执行了 `init`（收拢手指），
  等同于松开夹爪，工件可能从空中坠落。所有者现场制止。
- **铁律（不可违反）**：
```text
任何会让夹爪张开/收拢/释放的动作（init、open、grip 到更小值、close）
都必须在以下条件之一成立时才可以执行：
  A. 已确认夹爪上没有工件；或
  B. 已确认工件处于安全承载状态（放在桌面、完整插入插槽、已被人手托住）；
并且必须得到所有者明确同意。
```
- 默认行为改为：**除非所有者当次明确同意，绝不主动释放夹爪**。
- 判断“有没有工件”不能靠推测，要靠：
  1. 所有者目视确认；或
  2. 明确的搬运状态记录（何时夹起、何时放下），且中途没有异常。
- 我今天错在：把“工件在插槽里”推断成“夹爪是空的”，而实际手指仍插在扶手间隙里，
  并且末端悬空。**推断不等于确认。**


## 85. 2026-10-23 事件结果与后续

- 结果：工件未损坏，已完整回到插槽内部（所有者目视确认）。
- 现场状态：
```text
夹爪 position=0（全闭合，空夹）
TCP 1.127, -0.013, 0.158（-10° 倾斜，比插入位外约 130 mm）
工件：完整在插槽内
```
- 场景已恢复：`box_collision`（插槽内位姿）+ `slot_collision` 重新加入。
- 卸载流程状态：d5 抽出的成果为零（工件仍在槽内），需要重新执行 d2/d3。
- 重新开始前必须先做：拍照确认 + 所有者同意，再进入插槽。


## 86. 2026-10-23 分阶段速度策略（所有者确认满意）

```text
手上有工件（尤其从插槽取出）: 极慢
  planning 0.01, time_scale 5.0, 每段 40–60 mm, 段间停 1.5 s
  本次纯平移退出 240 mm 分 4 段实测“速度控制得不错”，作为基准
空手（单纯进入取件、空载移动）: 可以快一些
  planning 0.02, time_scale 2.0, 每段 60–100 mm
```
判断依据永远是：**是否夹持工件**，而不是阶段名称。


## 87. 2026-10-23 装配段速度铁律（所有者强调）

- 工件与插槽的径向间隙**左右各约 1 mm**，属于装配环节。
- 这类阶段（插入、转正、抽出、倾斜准备）：
```text
默认必须“全程均匀 + 安全低速”
只有在真机验证过之后，才允许采用“前快后慢”的不均匀策略
```
- 代码已加约束：`TIGHT_CLEARANCE_PHASES` + `--allow-uneven-insert`
  未加该参数时，即使配置里写了 slow_tail 也会被强制改成均匀低速。
- 涉及阶段：`insert`、`straighten`、`d2c_straighten`、`d4_tilt_for_removal`、`d5_extract`
- 配置已改为 uniform：insert/straighten 0.005 + ts3.0；d2c/d4/d5 0.005 + ts3.0
- 非装配阶段（空载移动、抬升、中转、放置接近前段）仍可用不均匀速度。


## 88. 2026-10-23 插槽位姿可能已轻微移动（重要排查方向）

- 所有者观察：昨天装配卡住时，**插槽桌可能发生了轻微移动**，
  不排除这是昨天装入/取出失败的原因之一。
- 因此：**插槽位姿不是绝对恒定**，固定链假设需要定期校验。
- 若本次取出再次失败，排查方向加一条：先用 T1 重新测量插槽位姿，
  与基准值比较平移/旋转偏差。
```text
基准: 1.241629, 0.181709, 0.443245
      q = (-0.0016288, 0.0042531, 0.0033840, 0.9999839)
```
- 环境建模要求：**必须认真比对插槽位姿是否变化，不能只拍照不验证**。
  建议每次任务开始前做一次 T1 + 位姿比对，偏差超阈值（如 1 mm / 0.5°）就报警。
- 抽取分段改为 **20 mm**（所有者选择）：速度参数不变、仍匀速，减少停顿次数。


## 89. 2026-10-23 效率优先流程（所有者规划，后续做）

- 先完成“正确优先”的装入/取出全自动封装；
- 之后**单独再做一套效率优先流程**：同样动作、时间上加速，
  例如转正 10° 这一步至少可以快一倍。
- 两套流程共用同一组位姿与安全闸门，只有速度参数不同；
  命名建议 `--profile safe` / `--profile efficient`，避免混用。
- 效率流程的前提：安全流程已多次验证通过（重复性数据）。

## 90. 2026-10-23 进展检查误判（已修）

- 现象：抽出第一步执行成功（TCP 前移 20 mm），但脚本报“no progress”并停止。
- 原因：执行后立刻读取 TCP，TF 仍是执行前的旧值。
- 修复：执行后先等 1.5 s，再连续采样 5 次取最大位移，再判断是否卡住。


## 91. 2026-10-23 带工件时的路径规划必须经过高位 staging

- 现象：带工件从中途点（工件桌与插槽之间，y≈0.16）直接规划到 pre_insert，
  三次尝试全部 `code 99999`（无解），即使换成位姿目标也不行。
- 有效做法：**先规划到 staging（1.0748,-0.0122,0.5252，高位开阔），再规划到 pre_insert**，
  两段都 `code 1` 且实际执行成功。
- 原因：附着工件体积大，中途点到插槽的通道不足（插槽、桌面体素都在场景里时）。
- 规则：**带工件进出插槽，一律走 staging 高位**；空手时可以尝试直接路径。
- 另记录两个低级失误（已修）：
  - 分段脚本用错循环变量导致分段文件没生成 → 执行器报 NO_CHUNKS；
  - `pkill` 模式匹配到自身命令行，把整条命令杀掉 → 改用脚本文件执行清理。


## 92. 2026-10-23 重大疏忽：MoveIt 位姿目标容差过松（必须记住）

- 现象：末端姿态偏 3.79°，用位姿目标规划“修正”，规划器返回 success，
  但实测仍偏 3.3°——因为位姿目标的默认容差是 **0.05 rad ≈ 2.9°**，
  偏差在容差内就算“达到目标”。
- 后果：以为修好了，实际工件仍然歪，属于典型的“假成功”。
- 修复：
  1. 姿态目标容差收紧到 **0.008 rad（0.46°）**；
  2. 新增 `scripts/verify_pose.py`：执行后用**实测位姿**对比目标，
     位置 >3 mm 或姿态 >0.01 rad 判 FAIL；
  3. 规则：**任何“对准”类动作，必须用实测值验收，不能只看规划器返回 success。**
- 通用教训：`status=4`、`code=1` 只代表“控制器/规划器认为完成”，
  不代表物理对准。装配相关动作一律加实测验收。


## 93. 2026-10-23 绝对精度提升方向（与六维力并列的后续规划）

所有者要求：作为后续减小误差的方向之一，与加装六维力传感器同等地位。

```text
第一档（最划算，优先）：机械基准
  插槽/工件桌加定位销 + 衬套（或球-锥运动学联接）
  把"位姿"变成"机械重复定位"，重复精度可达微米级
  这是"插槽固定"能真正成立的前提

第二档：测量与标定（外协更划算）
  激光跟踪仪 Leica AT960 / FARO Vantage：绝对精度 0.02–0.05 mm
  摄影测量系统：0.05–0.1 mm
  便携测量臂 FARO ScanArm：0.02–0.05 mm
  用途：机器人 DH 标定、基座与工件坐标系标定

第三档：末端与视觉
  高精度 3D 相机（如 Zivid 2）：0.1 mm 级
  末端激光位移传感器：单维 μm 级
  六维力传感器：不提升绝对精度，但可用柔顺搜索补偿残余误差
```

当前判断：0.47 mm 残余误差中**大部分来自基准不确定**（插槽位姿靠 RVS/flange 链推导），
不是机器人本体不准（重复定位 0.02 mm）。因此先做机械基准收益最大。


## 94. 2026-10-23 带工件时运输点必须抬高

- 原运输点 `0.95, 0.45, 0.35`：带工件时规划 `code 99999`（无解）。
- 抬高到 `0.95, 0.45, 0.55`：规划成功（212 点），实际执行 4 段全部成功。
- 与“必须走 staging 高位”是同一规律：**工件占用体积大，低通道不足**。
- 两套配置的 transit 已改为 z=0.55。


## 95. 2026-10-23 关键澄清：碰撞定位姿"清零"是读取问题，不是场景失效

实验结论（在新启的 move_group 上复现）：

```text
1. 通过 /apply_planning_scene 或 /planning_scene 添加碰撞体
   → GetPlanningScene 读回来的 primitive_poses / mesh_poses 全是 (0,0,0)
   → 与 is_diff、frame_id、full/diff 都无关

2. 但在末端当前位置放一个 0.12 m 盒子后调用 check_state_validity：
   → blocker_at_tcp <-> camera_link 0.0274  （碰撞被正确检出）
   → my_end_effector_link <-> full_box 0.044（先前那个"读作原点"的盒子其实位置正确）
```

推论：
- **MoveIt 内部几何是正确的，碰撞检查有效**；
- 只是 `GetPlanningScene` 的序列化把位姿丢了（诊断用途不可信）；
- 我写的 `check_carried_clearance.py` 因为读场景位姿，得出"环境空/碰撞"的错误结论，需改为自己维护几何。

真正的碰撞原因因此回到最初判断：
**手上工件没有有效的碰撞表示**（attached 的 mesh 位姿在此环境同样失效），
所以规划器不知道工件在哪 → 撞了插槽桌。

修复方向（下一步）：
```text
A. 工件改用"世界系盒体跟随 TCP"表示（T_world_box = T_world_tcp × T_link_box 内部有效）
B. 触碰对（工件与手指重叠）用"只建模外廓、避开夹爪区域"的方式处理
C. 每次规划/执行前后用 check_state_validity 做实测校验，而不是读场景
```

另注意：本环境 check_state_validity 会报出 `shoulder_link<->base_link_inertia`
这类本应被 SRDF 豁免的自碰撞，说明默认 ACM 可能未生效，需要单独确认。


## 96. 2026-10-23 工件碰撞方案 B（URDF payload link）实施记录

已完成的改动：
```text
src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro
  新增 xacro 参数 payload_enabled / payload_size_* / payload_center_*
  新增 link payload_box（带最小惯量）与 fixed joint eef_to_payload_box
  origin xyz=(-0.009,0.215,-0.411) rpy=(0,pi,0)
  collision box 114 x 441 x 465 mm，中心 (0.019,-0.215,-0.2265)
  关闭时（payload_enabled=false）碰撞盒缩成 1 mm，不影响规划

src/cs625_bringup/config/cs625_active_perception.srdf
  新增 payload_box 与 eef/tool0/gripper_*/camera_*/wrist_* 共 10 对豁免

src/cs625_bringup/launch/real_moveit.launch.py
  新增 payload_enabled 启动参数并映射进 robot_mappings
```

注意事项（踩过的坑）：
```text
1. 必须用 description_package:=cs625_ap_description +
   description_file:=cs625_active_perception.urdf.xacro 启动，
   否则加载的是供应商 URDF，payload_box 根本不存在
2. xacro:property 不能直接承接 $(arg) 用于条件判断，
   要直接写 <xacro:if value="$(arg payload_enabled)">
3. 按 group 检查碰撞时只算组内连杆，payload 不在组链里；
   验证要用 group_name=""（整机）
```

未完成的验证：
```text
payload_box 目前仍未在 check_state_validity 中产生接触
（在 payload 内放探针只报 my_end_effector_link，不报 payload_box）
下一步需要确认 move_group 的碰撞模型是否包含该 link，
可能方向：SRDF 是否需把 payload 放进一个 group、或 collision matrix 初始化问题
```


## 97. 2026-10-23 碰撞机制的最终正确理解（推翻之前的错误结论）

```text
1. GetPlanningScene 读回的 primitive_poses / mesh_poses 一律为 0
   → 这是"读取接口不可信"，不代表场景失效
2. 功能测试证明 MoveIt 内部使用的是正确位姿：
   - 在末端处放探针盒子 → 立刻报 my_end_effector_link 接触
   - 在工件实际位置放探针 → 报 box_collision（师兄 attach）接触 0.02
3. MoveIt 不检查"世界物体 ↔ 世界物体"，
   所以"世界盒子表示工件 vs 桌面体素"的校验永远无碰撞（我之前的方法错误）
4. 因此师兄的 attach + touch_links 方案本来就是对的，
   不需要自造 payload link（已全部回退）

正确做法：
  工件碰撞 = collision_object_publisher 的 attach
  环境碰撞 = env_voxel 体素 + slot_collision
  校验 = check_state_validity（内部有效），不要读场景位姿做几何判断
```

## 98. 2026-10-23 效率铁律（所有者要求）

```text
单次工具调用超时上限 60 s（100 s 也不接受）
动作发出后 60 s 内没有位移 → 立刻排查原因，不要继续等
```

## 99. 2026-10-23 位姿读取的坑（导致反复"原地打转"）

```text
用缓存的最后一条 PoseStamped 当时值 → 读到旧位姿
→ 规划目标永远基于旧位置 → 重复同一小步 → 看起来"不动"
修复：每次读取必须等到新的一帧（计数增加）再返回
/tcp_pose_broadcaster/pose 实测 125 Hz，可用
```


## 100. 2026-10-23 日终：六类模式架构与下一轮入口

所有者确立的目标结构（下次落地）：

```text
单一入口 python3 scripts/run_task.py --scope {load|unload|full} --mode {safe|efficient}
  scope: load=抓取+装入  unload=取出+放置  full=四段全流程
  mode:  safe=匀速低速/多分段/逐段验收    efficient=巡航快+末端慢，装配段仍匀速
配置六份互相独立：
  config/tasks/common.yaml（世界常量+可复用位姿）
  config/tasks/{load,unload,full}_{safe,efficient}.yaml
```

强制环境建模（下次第一步实现）：
```text
scripts/model_environment.py → runtime/environment/<ts>/model.json
插槽偏差 >1 mm 或 >0.5° 拒绝；工件偏差 >3 mm 或 >1° 拒绝
```

三轮调试：
```text
1. 可用性：6 类模式 dry-run + 真机各 1 次，经验写进代码
2. 效率  ：固化可复用位姿，非装配段提速，目标 ≤ 安全模式 60% 时间
3. 验收  ：6 类模式各 ≥3 次，采集数据
```

当日结束现场：
```text
TCP 1.0309, 0.6861, 0.4398
夹爪 0 mm 空夹 init=5，安全模式 1
工件已由所有者手动放回工件桌
```
详细交接：`docs/worklogs/2026-10-23_HANDOFF_MODES_ARCHITECTURE.md`


## 101. 2026-10-23 环境建模的「常量 / 变量」区分（所有者强调，设计依据）

```text
恒定量（每次任务都要验证没变）：
  插槽绝对位姿（T1 实测）  阈值：偏差 >1 mm 或 >0.5° 拒绝
  工件桌 / 插槽桌 碰撞体（点云体素）
  作用：确保插槽没有移动、桌面模型可用

每任务变量（每次必须重新测量）：
  工件初位姿（T3/S3 实测）
  —— 每次抓取面对的工件位姿都不同，禁止沿用旧值或写死常数
  —— 测不到/置信度不足就拒绝开始任务
```

放置位的推导（取出任务的正确做法）：

```text
place_TCP = workpiece_pose_ref × T_b_tcp
workpiece_pose_ref 来源优先级：
  1) 本次全流程任务开始时测得的工件初位姿
  2) 上一次「抓取并装入」任务记录里的工件初位姿
  3) 都没有 → 拒绝执行取出任务，绝不退化成固定坐标
```

因此需要任务记录链：`runtime/tasks/<task_id>.json`
```text
task_id / scope / 开始时工件初位姿 / 插槽基准 / 建模时间戳 / 关联的上一条任务
```

scope 对建模数据的依赖：
```text
load   ：恒定量 + 本次工件初位姿
unload ：恒定量 + 参考工件位姿（推导放置点）
full   ：恒定量 + 本次工件初位姿（首尾共用同一份）
```


## 102. 2026-10-23 观测基准位与视角搜索策略（所有者确立）

基准位（历史实测，作为搜索起点，不是终点）：
```text
S0 插槽观测位（T1 之前）   = 插槽拍摄基准位 1.044, -0.011, 0.726
S2 工件观测位（T2 返回）   基准 0.615353, 0.354540, 0.574506
S3 工件抓取位（T3 返回）   基准 0.652989, 0.703460, 0.355241
R1 插槽目标位              1.241629, 0.181709, 0.443245
```

视角搜索的调整原则（所有者明确）：
```text
大幅调整：J1 / J2 / J3   —— 改变整体站位与俯仰，用于粗定位
小幅调整：J4 / J5 / J6   —— 末端的精细姿态修正
目标函数：提高 RVS 返回的分数 + 提高「插槽 / 工件在视野中的完整度」
```

搜索流程（建议实现顺序）：
```text
1. J1 粗扫（按历史 S2/S0 方位为起点，逐步小角度扩展）
2. J2/J3 调整高度与距离，使目标完整入画
3. J4/J5/J6 微调姿态，改善遮挡与入射角
4. 每次采集都记录：关节角、RVS 分数、可见度/完整度、时间戳
5. 以「分数 + 完整度」联合排序，选出本次任务的最佳观测位
6. 选出的位姿写回任务记录，作为本次任务的观测基准
```

注意：S2/S3 是"起点"不是"固定值"；工件位姿每次变化，观测位需要跟着重新搜索。


## 103. 2026-10-23 RVS 结构勘察（可重构的确切位置）

```text
工程文件      runtime/*.rvs           —— 纯 JSON（约 1.3 MB）
             顶层键 Dashboard / edge_list / group_ports / ParaSettings / widget_list
             即算子图、连线、组端口都在 JSON 里，可程序化分析（改前必须备份）

Group 复用    支持导出/导入 *.group.xml；runtime/group_xml/ 为模板目录

AI 推理       RobotVisionSuite_AI/RVS_AI_Center.exe 是独立服务
             AILibrary/AIConfig.json:
               TCPServerPort = 2024
               NetConfig[]: mode(MaskRCNN/YOLO/KeyPoint), device(cpu/cuda),
                            modelFilePath(result.mask), scoreThreshold
             AITrainConfig.json: 图像尺寸 1280x960、batch、epochs、训练目录
             训练样本: RobotVisionSuite_AI/box/<timestamp>/rgb.png + rgb.json（标注）

二次开发      rvs_sdk（C++ 插件算子示例）、python 目录
```

重构可行的三条路径（按投入产出排序）：
```text
A. 工程层（不碰 AI）：
   精简算子链、加 ROI 裁剪、降采样、模板缓存、两段式粗定位+精定位、
   去掉重复触发 —— 直接降低每次 T2/T3 的耗时

B. AI 推理层（提升帧率与算子掌控）：
   RVS 模型是私有格式（result.mask），但训练数据+标注可导出
   → 用同一批样本自训 YOLO/ONNX，在我们自己的 ROS 2 节点用 GPU 推理
   → 帧率、阈值、后处理全部可控；RVS 保留为回退

C. 观测层：
   若能拿到相机流，自建高频观测；否则接受 RVS 低帧率 + 多次采集融合
```

注意：所有改动都在副本上进行，`RobotVisionSuite` / `RobotVisionSuite_AI` 安装目录保持原状。


## 104. 2026-10-23 RVS 文件保护规则（所有者要求：不许破坏原件）

```text
1. RVS 安装目录（RobotVisionSuite / RobotVisionSuite_AI）视为只读
   —— 我方只允许读，不做写入/改名/删除
2. 任何工程改动必须另存为新文件名（如 ..._opt_v1.rvs），原件保持不动
3. 改动前先归档：~/cs625_rvs_archive/<日期>/，并记录 sha256
4. AI 训练数据与标注只读取，不改写
5. 验证通过后由所有者决定是否切换启用哪份工程
```

已归档（2026-10-23）：
```text
~/cs625_rvs_archive/20261023/
  CS625_SlotBox_VisualGuidance_RVS.rvs   sha256 0c225ed301bccdc245bf48b21c57486a6b3692bb7a4e483744a94c71da8cc5aa
  AIConfig.json
  AITrainConfig.json
```

核对结论：今日被修改的 RVS 文件均为 RVS 自身运行/标定产生，
我方操作全部为只读，未做任何写入。


## 105. 2026-10-23 RVS 与 ROS 的分工模式（所有者确立）

首选（若可并行）：
```text
RVS  ：负责「详细位姿」处理（模板匹配、位姿解算、精度优先）
ROS  ：负责「实时观测 + 推理」（高频、低延迟，用于搜索与监视）
两边结果按时间戳融合，RVS 结果为主，ROS 结果为辅/监视
```

回退（若相机不能共用）——**分阶段交替**，同样可落地：
```text
阶段 A（RVS 主导）：环境建模 + 详细位姿确立
  工件桌 / 插槽桌碰撞体、插槽绝对位姿、工件初位姿
阶段 B（ROS 主导）：后续所有流程
  实时观测、目标确认、抓取/插入/取出过程的在线监视
  需要时再切回 RVS 复测（例如每轮任务开始前）
```

实现要求：
```text
1. 阶段切换要有明确的「交接文件」：model.json + task 记录（谁在何时测得什么）
2. 两套结果都带时间戳与来源标记（rvs / ros），上层按新鲜度与置信度选用
3. 阶段切换时相机句柄必须完全释放/重新获取，避免独占冲突
4. 若确认相机可并存（GigE 且允许共享），再升级为并行模式
```

待验证（决定走哪条）：
```text
· 相机是 GigE 还是 USB3
· 是否允许多进程同时打开（多数工业相机默认独占）
· ROS 2 侧是否有对应驱动（图漾/Percipio 有社区 ROS 2 驱动）
```


## 106. 2026-10-23 ROS 2 直连 PS800-E1 相机（攻关结果）

已完成：
```text
1. RVS 已停止（命令口 7000 / AI 口 2024 关闭），相机（GigE，地址见现场配置）在线
2. 官方驱动 percipioxyz/camport_ros2 克隆到 ~/cs625_camport_ws（自带 camport4 SDK 4.2.33）
3. 编译成功；依赖 ros-jazzy-camera-info-manager / image-publisher 已装
4. 两处最小改动（仅克隆仓库内，原件备份 /tmp/percipio_*.bak）：
   · percipio_device.cpp：device id 形如 IP 时走 TYOpenDeviceWithIP，
     且**不能提前 return**（alive=true 与事件注册在函数尾部）
   · percipio_camera_node_driver.cpp：广播枚举失败时遍历以太网接口按 IP 打开
5. 结果：设备识别 PS800-E1，序列号 207000164956，
   发布 /camera/{color,depth}/{image_raw,camera_info} 等 8 个话题
```

当前瓶颈：
```text
彩色与深度均为约 0.75 Hz（1.33 s/帧），
且 1280x960 与 640x480 帧率几乎相同 → 不是带宽/分辨率问题
frame_rate_control:=true frame_rate:=15 也无效
怀疑：相机侧曝光/帧率等参数被 RVS 之前的设置固化（用户集/默认参数），
      或 SDK 取帧 timeout 200 ms × 重试 10 次导致低效重试
```

下一步（下次继续）：
```text
1. 停节点后用 SDK 小工具读/写关键参数：
   TY_INT_FRAME_RATE、RGB 曝光（TY_INT_EXPOSURE_TIME / TY_FLOAT_EXPOSURE_TIME_US）
2. 若曝光很大（如 >200 ms）→ 降到 10–30 ms，再测帧率
3. 仍慢则查 MTU（巨型帧 9000）与 WSL 转发开销
4. 稳定后把启动脚本固化进仓库 scripts/（现在临时在 /tmp/run_cam*.sh）
```

结论：**ROS 2 直连相机这条路已打通**，剩下的是参数调优，
不再需要 RVS 参与实时观测，符合"RVS 做详细位姿、ROS 做实时观测"的分工。


## 107. 2026-10-23 ROS 端能力边界与「复刻 RVS 算子」计划

ROS 端可独立完成（已验证或标准能力）：
```text
图像采集：彩色 23.5 fps（640x480）、深度约 0.85 fps、点云（camport_ros2 + IP 打开）
2D 算法：OpenCV（模板匹配、轮廓、特征点、分割后处理）
3D 算法：PCL / Open3D（降采样、平面分割、法向、ICP、FPFH+RANSAC 粗配准）
AI 推理：自训 YOLO/ONNX → TensorRT/GPU，帧率与阈值完全可控
标定数据：直接读取现有手眼标定文件（DepthToRobotTCP / ColorToRobotTCP）
运动与编排：MoveIt + 我们的任务编排、日志、数据采集
实时性：多线程/GPU，无 GUI 依赖
```

RVS 独有（当前无法在 ROS 直接复现）：
```text
· 图漾专用 3D 模板匹配算子（Percipio3DMatchingNode）：针对自家相机噪声调优，
  模型为厂商 .ply + 参数（modelRelSamplingDistance、estimate_normal 等）
· RVS_AI_Center 的标注/训练 GUI 流程
· 手眼标定向导（结果文件可复用）
· 若干专用滤波/掩膜算子
· 厂商验证与技术支持
```

实测结论（支持「搬到 ROS」）：
```text
· 相机可被 ROS 独占访问（RVS 关闭时），RGB 23.5 fps 可用
· RVS 无头/自动运行不可靠：程序能启动，但加载为空工程时 7000/2024 端口不监听，
  TCP 服务依赖工程与运行状态，GUI 绑定强
```

复刻路线（建议分三期）：
```text
一期：纯 ROS 实时观测（RGB 23.5 fps）
     目标检测 + 2D 粗定位，作为主动重观测的前端
二期：深度静态采图 + AI 分割 + 点云配准（复刻 ProjectMask2Cloud + 3D 匹配）
     用 Open3D/PCL 实现，模型由 CAD/点云模板生成
三期：与 RVS 同场景对比（分数、位姿差、耗时），达标后 ROS 为默认，RVS 降为回退
```


## 108. 2026-10-23 二期（深度位姿）首跑结果与待办

链路已跑通（ROS 侧，无 RVS）：
```text
彩色 + 对齐深度 + 内参 → YOLO-seg 掩膜 → 反投影点云 → 质心 + PCA → 发布 6D 位姿
scripts/depth_pose_once.py 一次输出：
  pose xyz/quat + 点数 + xyz 尺寸 + rms
```

首跑暴露的问题：
```text
extent = 2.93 x 2.39 x 1.98 m（明显过大），rms = 0.0 mm
→ 说明掩膜覆盖区域过大（模型在当前视距下过分割），
  或深度尺度/对齐有问题
→ 必须肉眼检查掩膜叠加图，并确认深度编码（16UC1，单位 mm）
```

结论：**位姿解算的"最后一公里"取决于两件事**
```text
1. 拍摄视角要让目标占据合适画面比例（正是所有者提出的"用机械臂调整拍摄方位"）
2. 掩膜质量要验证（叠加显示），必要时补样本重训
```

下一步：
```text
1. 给 depth_pose_once.py 加 --dump 输出掩膜叠加图与点云 PLY，肉眼确认
2. 用机械臂把相机移到 S2/S0 类视角（带安全门：健康检查、碰撞场景、低速分段）
3. 再用厂商 .ply 做 ICP 精配准，输出最终 6D
```


## 109. 2026-10-23 二期位姿失真的定位结果

实测数据：
```text
/camera/depth/image_raw  encoding=16UC1, 640x480, 单位 mm
  depth[240,320] = 1526 mm
  depth[120,160] = 1196 mm
  depth[240,480] = 3573 mm
  depth 内参 K: fx≈465.7, fy≈465.8, cx≈324.3, cy≈224.7
掩膜：最大连通域过滤后已正确贴合工件
```

结论：
```text
1. 深度本身是正常的（mm 量级、数值合理）
2. 但 depth 内参与 color 内参不同（cx 324 vs 320 附近，fx 465 vs 彩色各自值）
   → **掩膜像素直接查 depth 存在配准偏差**，反投影出的点云被拉大（2.9 m）
   → rms=0 是"大面积近似平面"的副产品，不代表精度
```

修复方向（下次直接做）：
```text
A. 让 depth_registration_enable 真正生效（可能需同时开 point_cloud_enable），
   使 depth 与 color 同内参同分辨率，再按掩膜查深度
B. 或者把掩膜按内参与外参投影到 depth 相机坐标系后再取深度
C. 校验方法：取掩膜中心像素的深度值，应与实测距离（约 1.5 m）一致，
   且反投影后尺寸应在工件真实尺寸（~0.5 m）量级
```


## 110. 2026-10-23 主动重观测的调整策略（算法与调试基础，所有者确认）

### 位姿与视角的三条铁律（今天踩过的坑）
```text
1. 位置与姿态必须联动
   只抬高 z 不改姿态 → 相机变成俯视桌面，工件直接不在视野（实测 fitness 0.41→0.163）
2. 观测位 ≠ 抓取位
   抓取位在扶手间隙、更近更低；观测位要能看到工件整体
   两者混用会把相机贴到工件上，视野被占满
3. 靠近必须沿"相机光轴"
   TCP_new = TCP + d × 光轴单位向量（由当前姿态决定），
   不是简单改 x/y/z 单轴
```

### 实时闭环调整法（当前唯一有效方式）
```text
每一步：小角度/小位移调整 → 立刻拍一帧 → 用画面判断 → 决定继续或回退
判据（按优先级）：
  a) 工件是否完整在画面内（不在 → 先调姿态）
  b) 工件占画面比例是否 60–80%（RVS 训练分布；过小 → 沿光轴靠近）
  c) 掩膜是否只覆盖工件（是否误纳背景）
  d) ICP fitness > 0.5 且 inlier RMSE < 5 mm
```

### 调整自由度分工（所有者此前确立，本节沿用）
```text
J1 / J2 / J3 → 大幅调整（站位、高度、距离）
J4 / J5 / J6 → 小幅调整（末端姿态、遮挡改善）
优化目标：RVS/我们自己的分数 ↑，目标在视野中的完整度 ↑
```

### 已知的基准位（起点，不是固定值）
```text
S2 工件观测位：0.615353, 0.354540, 0.574506
              q = (-0.414438, 0.776655, 0.159180, 0.446889)
              实测：工件完整可见，占画面约 45%，ICP fitness 0.41
下一目标：沿光轴靠近约 0.3 m → 占比 60–70% → fitness 期望 >0.5
```

### 当前环境约束（调试时必须遵守）
```text
· 位姿解算用 640×480（1280×960 深度有效点≈0，实测）
· 深度范围门限按实际距离设置（S2 处约 4.5 m，不要用 2.2 m）
· 掩膜用 640 版模型；背景亮区会被误判，需范围门限兜底
· 每次移动前查三项：safety_mode / 反向端口 / 控制器健康
```


## 111. 2026-10-24 拍摄基准位是「粗采样」位（所有者纠正）

```text
此前记录的 S0 / S2 / S3 / R1 等，全部属于【粗采样基准位】
—— 它们是"能在较大视野下稳定看到目标"的位姿，不是精采样位。

RVS 的完整机制是三层两级：
  视点位两级：粗观测位（precision_view 系列） → 精观测位（由粗结果推出）
  命令两级  ：T2（任意位置→S2 粗） → T3（在 S2 触发→S3 精）；插槽同理 T1
  算法两级  ：工件 modelRelSamplingDistance 30 mm 粗 / 插槽 15 mm 精
  estimate_normal = 1（两级都需要法向）

含义：只到粗采样位并不能保证"完全识别"，
必须再进入精采样位（更近、更好视角）做第二次配准。
```

实测支持：即使在粗观测位（工件占画面 75–80%），
我们自训的 YOLO 掩膜仍会把底部桌面粘连进来（尺寸 1.75 m vs 真实 0.45 m），
说明"粗位 + 单次配准"不足以支撑位姿，必须补精采样阶段。

后续设计（待实现）：
```text
粗阶段：任意视角 → 检测 + 粗位姿（容错，几十度也可）
精阶段：移到粗位姿推出的精观测位 → 掩膜 + ICP 精配准（毫米级）
判据  ：fitness > 0.5 且 inlier RMSE < 5 mm 才接受
```


## 112. 2026-10-23 晚：1.4 m 横向偏差的两个根因（已修，真机 0 运动验证）

偏差不是"相机姿态×远距离放大"，而是两处独立的量级错误，都会把目标抛到米级：

```text
A. 外参没有用 RVS 手眼，用了手搓常量
   真机链读 config/handeye/tcp_to_camera.json，里面是"相机几乎在 TCP 上"
   + 45° 倾角的手写值；与 RVS 手眼标定相差 135° / 0.34 m。
   修法：scripts/sync_real_handeye.py 从 RVS 的三个标定文件推导
        T_tcp_cam = inv(T_flange_tcp) @ T_flange_cam_color_optical
        （T_flange_tcp 就是 URDF 固定关节 flange→my_end_effector_link
          = -0.046 0 0.353，也正是 /tcp_pose_broadcaster/pose 报告的帧）
        并带 --check；测试 test/test_real_handeye_chain.py 锁住组合关系

B. 深度单位错了 4 倍
   设备自报 DepthScaleUnit = 0.25（mm/计数），驱动把原始计数直接当 mm 发布。
   修法：camport_ros2 的 publishDepthFrame 用 depth_scale_ 换算成毫米
        （docs/dependencies.md 已登记这是该 fork 的第 3 处改动）
```

零运动「已知靶点」验证（scripts/known_target_calibration.py，一帧 RGB-D）：
```text
工件 PCA 三轴  0.493 / 0.406 / 0.352 m   已知 0.114 / 0.441 / 0.465  → PASS
桌面法向(base) (0.007, -0.012, 1.000)    倾角 0.8°                  → PASS
桌面高度       -0.103 m                  历史实测带 -0.13..-0.07     → PASS
工件质心(base) (0.935, 0.736, 0.218)
```
地面真值来源：工件自身尺寸（与位姿无关）+ 桌面水平面（与旋转无关）+ 桌面高度
（与平移无关），三者互相独立，所以一次就能把"深度标尺 / 旋转 / 平移"分开定位。

修后 coarse 预演：目标点 (0.851, 0.616, 0.212)、距离 0.48 m（修前是
(-0.376, -1.054, 0.265)、2.02 m）。

顺带确认：
```text
· 深度已配准到彩色（两者 K 完全相同、边缘与剪影对齐），
  所以掩膜像素查深度是对的——§108 说的"配准偏差"其实是深度标尺
· 正式结论改写：不是"配准偏差"，是"深度单位"；两者表现相似但根因不同
· 掩膜粘连桌面仍在（§8.8），但可以先用"桌面平面剔除"兜底量尺寸
```


## 113. 2026-10-23 深夜：把「精观测位」算对（所有者当场纠正）

所有者原话：**"正常来说，刚刚的速度是正常速度"** —— 即 move_group 速度缩放
0.05 就是这台 CS625 在真机上的**正常工作速度**，不是"为了安全才放慢"。
所以：
```text
· 观测/中转移动用 0.05，20–50 s 的轨迹时长是常态，不要当异常也不要再"提速"
· 装配段（插入/抽出/转正）才降到 0.005–0.01
· 规划时长 20–50 s 也属正常，不要因为"慢"去怀疑链路
```

精观测位曾经算错，三个错误叠加，记下来防止再犯：
```text
错法（coarse_to_fine_pose.py 旧版）：
   fine_TCP = target − unit(target − camera_pos) × standoff(默认 1.0 m)
   → standoff 被当成 TCP 到目标的距离
   → 相机在 TCP 后方 296 mm，实际相机距离 = 1.0 + 0.255 ≈ 1.25 m
   → 相机从 0.52 m 被推远到 1.25 m，工件占画面 52% → 7.9%（比 RVS 的粗观测位还远）
   → 姿态是脚本里写死的 S2 四元数，不是算出来的

正确算法：
   T_base_cam 由当前 TCP × T_tcp_cam 得到（这才是相机的真实位置）
   standoff 定义为【相机到目标】的距离
     desired_cam.position = target − unit(target − cam_pos) × standoff
     保持当前相机姿态（光轴不变）
     desired_TCP = desired_cam @ inv(T_tcp_cam)
   护栏：standoff ≥ 当前相机距离时拒绝执行（除非 --allow-back-off），
         并打印预计占画面比例（面积 ∝ 1/d²）
   参考量级（RVS 自己的两级）：S2 粗观测 = 相机到工件 0.65 m；S3 抓取 = 0.30 m
   精观测取 0.40–0.45 m → 占画面 60–80%

实测（2026-10-23 23:1x）：
   相机到目标 0.444 m，占画面 69%，TCP 到位误差 4.4 mm / 1.5°
   两次不同臂位姿下的已知靶点标定：桌面高度 -0.103 / -0.101 m、
   桌面倾角 0.8° / 0.7°、工件质心差 ~12 mm → 外参链跨位姿自洽
```

另外两条经验：
```text
· 规划目标别用装配级公差：0.5 mm 球 + 0.001 rad 会让普通观测位规划失败（code 99999），
  观测位用 5 mm + 0.02 rad 足够（下一级会重新解算目标）
· 精阶段 ICP 不能走 TF：原来 lookup base_link ← 相机帧 落到错误的树上，
  结果 (-0.37,-0.92,0.38) fitness 0.36，与"修前坏目标点"几乎一样；
  改成 TCP × T_tcp_cam 后位置才回到工件附近
```




## 114. 2026-10-24 凌晨：定位基准点 = 必须观测点（所有者明确规则）

所有者原话：「记住你的位姿定位点，这是我们定位位姿的关节，所以不管怎么样，
都需要将这个点计入必须观测点」。

含义与约束：

```text
· 工件上有一个定位基准点（当前是【左上角】那个切角/边角，之前缺失、已补齐）
· 这个点是位姿解算的锚，不是可有可无的特征
· 由此：任何视角生成 / NBV / 观测位搜索，都必须把「该点在视野内且未被遮挡」
  作为【硬约束】，而不是软评分项
```

工程落地建议：

```text
1. 在模型坐标系里把该角点固定下来（写成常量，随模板一起版本化）
2. 候选视角评分先过硬门：锚点投影在图像内 + 深度有效性 + 不被自身/桌面遮挡
3. 精阶段配准完成后，额外校验：锚点位置误差（用模型角点 vs 实测角点）
4. 抓取位姿由位姿 × T_b_tcp 推得，锚点的可见性同时也保证抓取朝向可靠
```


## 115. 2026-10-24 凌晨：启动 RVS 的正确方式（所有者指定）

```text
· 启动入口：C:\Users\Public\Desktop\RobotVisionSuite.lnk
  （不要用 D:\...\RobotVisionSuite.exe 直接起，所有者指出那是错的入口）
· 必须加载的工程：
  D:\Program Files (x86)\RobotVisionSuite\runtime\CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs
· 加载后必须【启动运行】，流程组才会响应 T2/T3 消息
```

PowerShell 启动方式：

```powershell
Start-Process 'C:\Users\Public\Desktop\RobotVisionSuite.lnk'
# 若需带工程文件：
Start-Process 'C:\Users\Public\Desktop\RobotVisionSuite.lnk' -ArgumentList '"D:\Program Files (x86)\RobotVisionSuite\runtime\CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs"'
```

判断是否真的起来了（两处证据，缺一不可）：

```text
1. runtime/log/rvs_all/rvs_all_<今天时间>.log 出现：
     获得 N 设备列表 / tycam 启动 / TyCameraResource > device_online: true
2. 发一次 T3 后日志出现：
     T3_TCPServerReceive 接收到: ...
     [rvs_icp] pose fitness [0] : 0.9xx     ← 真的匹配成功
     send: S3,<mm 六个数>
```

注意：RVS 与我们的 ROS 相机互斥，启动 RVS 前必须停掉 percipio_camera。


### 115.1 更正（03:0x 实测）：命令行带工程路径**无效**

把工程路径作为参数传给 .lnk，RVS 起来的是空工程（窗口标题变成 `new.rvs`），
不会加载目标工程。正确做法只有：

```text
① Start-Process 'C:\Users\Public\Desktop\RobotVisionSuite.lnk'   ← 只把程序拉起来
② 在 RVS 界面：文件 → 打开 →
   D:\Program Files (x86)\RobotVisionSuite\runtime\CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs
③ 点【启动 / 运行】，节点图开始执行
```

验证工程真的加载并运行（三条证据）：
```text
· 窗口标题显示 CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs（不是 new.rvs）
· 日志出现：获得 N 设备列表 / tycam 启动 / TyCameraResource > device_online: true
· 发 T3 后日志出现 T3_TCPServerReceive 接收到 + [rvs_icp] pose fitness 0.9xx + send: S3,...
```

另：启动 RVS 前必须停掉 ROS 相机（互斥），并确认没有残留的 percipio_camera 进程。


## 116. 2026-10-24 05:0x：运动与采集的硬规则（所有者明确）

### 116.1 速度限制（两次警告后定稿）

```text
· 速度缩放速度 velocity_scale  ≤ 0.05（这是本机"正常速度"，不要更高）
· 轨迹 time_scale ≥ 2.0（1.0/0.4 会被控制器判为超速）
· 后三个关节"可以适当加速"= 在 0.05 / 2.0 框架内，不要单独调高 velocity
· 决不允许为了快而调整缩放参数——宁可慢，不可超限
```

### 116.2 关节使用规则

```text
· J1/J2/J3（肩/肘）保持基本角度不动（约 44 / -94 / -130 deg）
· 调节相机视角只允许用 J4/J5/J6（腕部三轴）
· 任何单次关节变化 ≤ 30°，且要有碰撞检查
· 禁止用"TCP 位姿目标"做大范围重构（规划器会选绕 J1 转 160-200° 的路径，
  已两次造成擦碰与一次 J1 大摆）
```

### 116.3 事件记录

```text
2026-10-24 04:2x  绕 J1 的大轨迹导致小臂擦近插槽桌 → 所有者急停
2026-10-24 04:2x  J1 继续旋转撞到物体 → safety_mode=3（保护性停止）
2026-10-24 04:5x  调整速度过快（time_scale 0.4 / velocity 0.15）→ 所有者警告超限
恢复：单关节 ≤30° 步进 + recover_control.sh + 精确关节目标规划，最终回到基准位
```


## 117. 2026-10-24 05:4x：采集"拍歪"问题已用几何门禁根治

### 反复出现的错误（所有者第 N 次指出）

```text
现象：采集完才发现工件不在画面中心（贴边/跑出画面），整批作废重拍
原因：① 用 stale 位姿算相机目标；② 用模型原点（角点）当工件中心；
      ③ 拍完不做任何校验，直接入库
```

### 根治方式（已写进 scripts/capture_dataset.py）

```python
projected_target_ok(tcp, T_tcp_cam, intrinsics, part_xyz, part_quat)
  · 用已知工件位姿 + CAD 包围盒（半尺寸 0.2275/0.0655/0.2325，质心 -0.208,0.066,-0.011）
  · 投影 8 个角点到图像，取凸包
  · 判据：投影面积 ≥ 10%；投影中心必须在画面内；中心偏移 ≤ 0.35
  · 不合格 → 该帧直接判废，不入库
```

注意两个坑：

```text
1. 不能用 YOLO 判"有没有拍到"——它会对着幕布"检测"出 76% 的掩膜
2. 近距帧允许超过画面（overflow 是正常的），只要求中心在画面内
```

### 工件位姿与几何中心的关系（易错）

```text
ICP 输出的是【模型原点】= 工件的一个角点，不是几何中心。
要用几何中心取景时必须换算：
  center = part_pose_t + R(part_quat) @ [-0.208, 0.066, -0.011]
新位置实测：pose (1.0382, 0.8691, 0.1219) → 中心 (1.187, 0.709, 0.132)
```


## 118. 2026-10-24 06:0x：粗观测基准位（所有者手动设定，必须记住）

### 位姿

```text
TCP   (0.5886, 0.3482, 0.7075) m
姿态  quat (0.2954, -0.7023, -0.2539, -0.5958)   [x,y,z,w]
```

### 关节（度）

```text
J1 shoulder_pan    43.1
J2 shoulder_lift  -57.4
J3 elbow         -146.3
J4 wrist_1         14.4
J5 wrist_2         92.8
J6 wrist_3         89.8
```

### 用途与关键性质

```text
· 这是一切【粗观测】的起点位姿：从这一位姿能观察到工作台上几乎所有工件摆放位置
· 视野实测：工件占画面下部约 35%，同时能看到幕布、墙面、木桌与工作区（见 coarse_view.png）
· 粗定位流程：先回到此位姿 → 拍一帧 → ICP 求工件位姿 → 再据此算精观测位
· 与之前"基准拍摄位" (0.813, 0.606, 0.436) 的区别：
    - 基准拍摄位是【精观测】用（工件占画面大、距离 0.45 m）
    - 本粗观测位是【搜索/发现】用（看得广，距离约 0.8-0.9 m）
```


## 119. 2026-10-24 07:3x：插槽观测的粗观测基准点（所有者指定）

### 位姿

```text
TCP   (0.7802, -0.1458, 0.7986) m
姿态  quat (0.1834, -0.8502, -0.1133, -0.4803)   [x,y,z,w]
```

### 关节（度）

```text
J1 shoulder_pan   -21.1
J2 shoulder_lift  -72.5
J3 elbow         -111.0
J4 wrist_1        -37.5
J5 wrist_2        128.5
J6 wrist_3         62.7
```

### 用途

```text
· 插槽一切粗观测的起点（与工件的粗观测位 §118 并列，两个不同的基准）
· 从该位姿可完整看到插槽：导向架 + 槽口平台 + 安装支架 + 底板
· 插槽精观测基准（2026-10-24 07:4x 更新为更优点）：
    TCP (0.788, 0.072, 0.759)   相机距插槽 0.95 m   俯角 20°   方位 +15°
    插槽投影面积 52.9%（完整落入画面 100%，中心偏移仅 0.072）
    对比旧值 TCP (0.718, 0.029, 0.801)：面积 36.6%、偏移 0.183 → 已弃用
    选择依据：近距 0.60 m 帧虽大但溢出（仅 38–50% 在画面内），不可用于位姿评估
· 插槽本体固定位姿（师兄 RVS 标定）：
    (1.2416, 0.1817, 0.4432)   quat (-0.0016, 0.0043, 0.0034, 0.99998)
```

### 插槽可用模型（标注/取景用）

```text
Model/slot_pointcloud.STL            96,760 面（毫米制）
Model/slot_pointcloud20260127.ply    300,000 点（米制）
src/cs625_simulation/assets/cs625_task/slot_fixture/meshes/slot_fixture.stl
                                      0.232 × 0.745 × 0.506 m（仿真夹具）
```


## 120. 2026-10-24 09:3x：本夜成果与下一段对话起点

```text
已完成
  · 数据采集 181 帧（22 批次），工件 5 个摆放位置 + 槽内场景 + 插槽 + 桌面 + 环境
  · 两类自动标注 93 帧（工件 CAD 投影 74 / 插槽点云投影 31），目视验证通过
  · YOLO 重训 40 轮，mask mAP50 0.861（工件类 0.927），旧模型仅 0.492
  · 采集脚本内置五道守卫（工作盒/观测包络/夹爪间距/取景门禁/规划器预检）+ --dry-run

未完成（下一段对话的真实起点）
  · 粗观测→精观测→取位姿 的运动链路：规划成功但执行反复 TIMEOUT
    卡点：scripts/execute_assembly_chunks.py 第116行 duration+20 超时太短
    建议：改为 max(duration+20, 60)
  · 桌面两类标注（平面判据分不开地板/幕布/墙）→ 已降级为两类
  · 任务分段试跑 A/B/C/D 与任务阶段数据
  · NBV 接入与评估

详细交接：docs/worklogs/2026-10-24_HANDOFF_NEXT_SESSION_YOLO_AND_PIPELINE.md
```


## 121. 2026-10-24 10:0x：粗观测→精观测跑通 + 精观测位位姿收敛到 1 mm

### 关键结论

```text
· 昨晚的"执行 TIMEOUT"不是超时太短：2.8 s 的轨迹等了 22.8 s，六个关节读数一点没变
· 直接原因：驱动栈半死 —— arm_controller inactive、reverse-control 端口未建立，
  而 /io_and_status_controller 仍在发布【陈旧】的 safety_mode=7 / robot_mode=3
· 教训：话题上有数据 ≠ 数据是当前的；判定"能不能动"必须 三样一起看
    ① safety_mode=1 / robot_mode=7   ② arm_controller active   ③ control_health.py sockets=True
· 重启驱动栈 + 激活 arm_controller + 重启 MoveIt 后，状态立刻变 1/7，sockets True，臂可动
```

### 精观测位（本轮实测，已执行）

```text
粗观测位  TCP (0.580, 0.350, 0.704)
精观测位  TCP (0.5084, 0.2969, 0.5626)   相机距工件 0.68 m  俯角 20°  方位 +10°  look-at
          由 scripts/capture_dataset.py::camera_target() 计算（库里现成算法，勿另写）
          15 个候选里只有 4 个过门禁（工作盒 0.35..1.10/0.25..1.05/0.25..0.80 + 间距≥0.38 + 终点构型在观测包络内）
          az+10_el+20 / az+20_el+20 / az+10_el+30 / az+20_el+30 可用
```

### 工件位姿（精观测位 3/3 帧一致）

```text
xyz   (0.5029, 0.6351, 0.1254)    quat (0.1910, 0.9816, 0.0019, -0.0068) [x,y,z,w]
几何中心 (0.7206, 0.6182, 0.1335) = xyz + R(quat)·[-0.208, 0.066, -0.011]
位置离散 1.00 mm  转角离散 0.285°  覆盖率 84%  median 3.4 mm   STABILITY: PASS
与 08:xx 粗定位 (0.499,0.641,0.127) 相差 6 mm 以内，跨会话一致
```

### 粗观测位 79 mm 的用法

```text
粗观测位下工件被画面下边缘裁掉（掩膜贴到第 960 行），可见表面不足 → 四重对称翻支，3 帧离散 79 mm
精观测位居中后立刻收敛到 1 mm
→ 这就是"精观测位"的物理价值，不要试图靠收紧覆盖率门禁去救粗观测位的翻支
```

### 本轮改的文件

```text
scripts/execute_assembly_chunks.py
  + --no-motion-window（默认 10 s，要求 >0.02 rad 运动却不动 → NO_MOTION + 机器人状态，返回 8）
  等待窗口 duration+20 → duration*2+40（控制器自己的放弃点是 2*duration+25，客户端必须晚于它）
  失败时打印 safety_mode/robot_mode 与轨迹错误码名字；NO_MOTION 时先 cancel goal
（曾给 coarse_to_fine_pose.py 加间距门禁，已撤回：capture_dataset.py 早已有 MIN_TCP_CLEARANCE=0.38，
  同一套规则不要在第二个脚本里重写）

详细工作日志：docs/worklogs/2026-10-24_WORKLOG_EXECUTION_ESTOP_ROOT_CAUSE.md
```


## 122. 2026-10-24 11:3x：六类模式落地 + 强制环境建模 + 全流程只规划验证

### 一句话

六类模式（load/unload/full × safe/efficient）全部落地并**只规划跑通**（51 段/全流程），
机械臂全程未动；机器人网络不可达，真机执行段暂缓。

### 关键数字（plan-only，未执行）

```text
load/safe      13/13 段   规划 77.9 s
load/efficient 13/13 段   规划 49.1 s   ← 安全模式的 63%（效率来自取消分段 + 取消 dwell + 巡航 0.12）
unload/safe    38/38 段   规划 288.8 s
unload/efficient 38/38 段 规划 285.6 s  ← 几乎无提速：时间都花在槽内 0.005 m/s（物理约束）
full/safe      51/51 段   365.6 s
full/efficient 51/51 段   334.6 s
```

### 新增入口与配置

```text
scripts/run_task.py --scope {load|unload|full} --mode {safe|efficient}
    [--confirm | --plan-only | --dry-run] [--start-joints q1..q6] [--handover confirm|auto]
    → 合并 config/tasks/common.yaml + <scope>_<mode>.yaml 成 runtime/effective_*.yaml
    → 加载 runtime/environment/latest/model.json 过门禁
    → 调 scripts/run_pick_and_place.py --flow pick_place|unload
scripts/model_environment.py   离线建模 + evaluate_gate() 纯函数门禁（四种拒绝）
```

### 关键设计决定

```text
· --start-joints：plan-only 不需要机器人。每段以上一段规划终点为 start_state 串起来，
  所以机器人不可达时也能验证整条流程（本轮就是这么做的）
· 槽内紧间隙（insert/straighten/d2c/d4/d5_extract）速度 ≤0.005 m/s 且禁止快巡航+慢尾段，
  两种模式都不放宽；d2b_entrance_tilted 不算紧间隙（真机以 0.03 跑过）
· 速度上限按模式且逐阶段校验：safe 巡航 0.08 / 装配 0.03；efficient 巡航 0.12 / 装配 0.03
  上限取"真机证明过的最大值"，校验器不得比实践更严（否则会判死已验证配置）
· full = load → 默认 handover 人工确认 → unload
· 环境门禁：插槽 >1mm/>0.5°、工件 >3mm/>1°、点数 <20k、模型 >2h → 拒绝
```

### 流畅性排查结论（未动机械臂）

```text
· MoveIt 已经是 TOTG（ompl.response_adapters 含 AddTimeOptimalParameterization）→ 不需要"换掉 IPTP"
· TOTG 不约束 jerk；没有 jerk 限制
· 无实时内核（WSL2 PREEMPT_DYNAMIC），FIFO 实时调度被拒（Operation not permitted）
· cyclictest 未安装
· URDF 末端负载是占位值（相机 0.01 kg / 机身 0.05 kg / 手指 0.001 kg，质心全 0）→ 重力补偿会偏
· 笛卡尔步长 0.003 m（比建议 5~10 mm 更细）
→ 最可疑：无 RT 内核 + FIFO 被拒（125 Hz 环路抖动）、末端负载/质心占位值
```

### 阻塞

```text
机器人地址 No route to host（相机地址正常）→ 需要所有者确认上电/网线/IP；地址本身写在 ~/.cs625_local.env（CS625_ROBOT_IP / CS625_CAMERA_IP），不要写进仓库
```

详细工作日志：docs/worklogs/2026-10-24_WORKLOG_SIX_MODES_AND_ENV_MODEL.md
源码权威性分级：docs/source_authority.md


## 123. 2026-10-24 21:3x：六模式只规划全过 + 真机夹具几何 + 双 UI + 实验归档 + 工作区重整

### 一句话

六类模式在**真机夹具碰撞体**存在下全部只规划通过（19/19、38/38、57/57 段）；
仿真世界由实测生成、夹具换成真机 RVS 几何；两套 UI、实验自动归档、工作区重整完成。
真机仍不可达，无真机运动。

### 关键决定

```text
1. 槽内紧间隙：速度规则与碰撞规则分开
   TIGHT_CLEARANCE_PHASES    insert/straighten/d2c/d4/d5_extract → ≤0.005 m/s
   NO_COLLISION_CHECK_PHASES 上面这些 + d2b_entrance_tilted      → 规划不做碰撞检查
   原因：单边间隙 ~1 mm < 网格误差，照常检查会把真机每天能做的动作判成碰撞；
   报告里必须写明"仿真插入可行 = 几何导向下可行"
2. 夹具几何用真机 RVS 模型（assets/cs625_real_cell/slot_fixture，60k 面）
   放置语义：模型原点 = slot_origin（不是 bbox 中心），只取 yaw
   旧仿真夹具（cs625_task/slot_fixture）槽宽 447.5 vs 真机 341.6 mm，只有 2/9 层贯通插入路径 → 弃用但保留
3. 撤出/进入只在扶手区慢：slow_band_mm，按"离座入位的轴向回退距离"定义，
   暂定 [0, 80] mm，待所有者按真机扶手高度确认
4. 相机：modeling 1280x960 彩色+深度 0.79 fps / fast 640x480 仅彩色 23.1 fps
   1280x960 BGR8 ~3.7 MiB > Fast DDS 默认 SHM 段(~512 KiB) → SENSOR_DATA(best-effort) 丢 60-80% 图；
   改用 RELIABLE 后 image_raw 计数才等于真实帧率
5. 实验数据：run_task --confirm 自动 start→stop→archive 到 ~/cs625_experiment_archive
```

### 六模式只规划结果（夹具碰撞体已发布）

```text
load/safe 19/19 · load/efficient 19/19 · unload/safe 38/38 · unload/efficient 38/38
full/safe 57/57 · full/efficient 57/57          单测 185 passed
```

详细工作日志：docs/worklogs/2026-10-24_WORKLOG_SIX_MODES_ACCEPTANCE.md
