# 当前架构（2026-11-07）

> 取代已归档的 `architecture.md`（P3/P4 时代的分层描述）。
> 本文只写**现在真实存在的东西**：包边界、脚本分层、示教链路的 0–16 步、两种模式。

---

## 1. 三层结构

```text
┌──────────────────────────────────────────────────────────────┐
│ 任务层（scripts/，不是包）                                    │
│   安全模式  scripts/resume_teaching.sh                        │
│   效率模式  scripts/teaching/teaching_debug.py                │
│   UI       scripts/cs625_panel.py（两模式面板）               │
├──────────────────────────────────────────────────────────────┤
│ 应用核心（scripts/teaching/）                                 │
│   teaching_sequence.py   示教链路主体（0–16 步）              │
│   plan_joint_locked.py   规划（IK + 分支锁 + 速度契约）        │
│   execute_assembly_chunks.py  分段执行与监控                  │
│   compliant_insert.py    插入段 effort 监督                   │
│   assembly_search.py     装配搜索（绕 TCP Y 边转边插）        │
│   straighten_in_slot.py  槽内分步转平 + 自由转动空间测量      │
│   rvs_pose_client.py / rvs_fitness.py / pose_library.py       │
│   pose_math.py / adjust_tcp.py / verify_pose.py 等            │
├──────────────────────────────────────────────────────────────┤
│ ROS 2 包（src/，12 个）                                       │
│   interfaces / description / sensor_adapter / target_perception│
│   view_generation / view_evaluation / motion_adapter /         │
│   task_orchestrator / experiment_tools / simulation /          │
│   bringup / rviz_panel                                        │
└──────────────────────────────────────────────────────────────┘
```

**边界规则**：任务层只拼装，不写运动逻辑；应用核心不直接依赖包内部实现；
包之间只通过消息/服务/动作交互。

---

## 2. 两种模式（唯一入口）

| 模式 | 入口 | 特征 | 用途 |
|---|---|---|---|
| **安全模式** | `scripts/resume_teaching.sh` | 门检全开、静置 15 s、fitness 门控、位姿库拦截、失败自动重启驱动栈并 `--resume` 续跑 | 演示、取数据、给导师看 |
| **效率模式** | `scripts/teaching/teaching_debug.py` | 可分阶段、可改速度、可跳静置与门检、每次运行自动归档 `runtime/debug_runs/` | 十一月视觉算法开发、十二月力控实验、MVP 方案 A/B/C 对比 |

**两者跑的是同一条链路**（`teaching_sequence.py`），效率模式只是它的调试外壳。
六模式（load/unload/full × safe/efficient）的源码按要求**冻结保留**但不作为入口：
`scripts/run_task.py`、`scripts/run_pick_and_place.py`、`scripts/cs625_ui.py`。

---

## 3. 示教链路 0–16 步

| 步 | 名称 | 关键动作与守卫 |
|---|---|---|
| 0 | 环境模型 | `model_environment.py` 生成 `runtime/environment/latest/model.json` |
| 1 | 插槽精观测 | 到教学 S0 → 静置 15 s → T1（fitness ≥0.9，帧间 30 mm/3°）→ 位姿库比对 |
| 3 | 工件粗观测 | 基线 A→B→C 依次尝试，T2 只决定相机站位（fitness ≥0.8，帧间 60 mm/8°，60 s 等待预算） |
| 4 | 工件精观测 | 到 S2 视角 → 静置 → T3（fitness ≥0.9）→ 成功后把该视角提升为基线 B |
| 5 | 预抓取 | 抓取位上方 150 mm；**5b 门** 1.5 mm / 0.4° |
| 6 | 进入抓取 | 竖直下降（姿态强制竖直，`GRASP_SNAP_VERTICAL`）；**6b 门** |
| 7 | 张爪夹持 | 夹爪 init + 闭合到位；**7b** 挂载工件到规划场景 |
| 8 | 抬升 | 0.15 m，持件速度上限 0.013 m/s |
| 9 | 抬到安全高度 | z = 0.53 |
| 10 | transit | 持件转运（环境场景已清空） |
| 11 | staging | 槽前分级位 |
| 12 | 旋转 | 绕工具 RPY 转到插入入口姿态（−10° 倾斜） |
| 13 | 下降到预插入 | **13b 位姿门** 1.0 mm / 0.4° + 目标正确性检查 |
| 14 | 插入 | `compliant_insert.py`：0.002 m/s、6 段、effort 监督 |
| 15 | 转平 | `straighten_in_slot.py`：绕 TCP 自身 Y 轴 0.5°/步，遇阻回退，报告自由转动空间 |
| 16 | 释放 | 需 `--allow-release`，默认拒绝 |

**贯穿性守卫**（任何模式下都不跳过）：

```text
· 保护停止后不自动重试；连续两次 NO_MOTION → 自动重启驱动栈 → 仍失败则整链中止
· 工件附近（低于安全高度）禁止横向移动，必须先竖直抬升（LIFT_FIRST_LATERAL_M = 20 mm）
· 速度契约：计划的 max|dq|/dt 不得超过请求速度的 1.5 倍，否则拒绝出计划
· 持件阶段巡航速度硬上限 CARRY_SPEED_CAP = 0.013 m/s
```

---

## 4. 插入权威与位姿固化

```text
优先级：runtime/slot_reference_verified.json
      → config/tasks/common.yaml 的 teaching_slot_reference（备份）
      → 当天 R1（仅当两者都不存在）

原因：当天 R1 与已验证位姿实测差 33 mm，用它直接算插入目标会撞槽（2026-11-05）
R1 仍然采集，但只打印距离，不参与计算
```

---

## 5. 数据与运行产物

```text
runtime/environment/latest/model.json   环境模型（门禁要求 <2 h）
runtime/rvs_teaching/*.json             RVS 采集（T1/T2/T3）
runtime/slot_reference_verified.json    已验证槽位姿（插入权威）
runtime/pose_library.json               工件/插槽位姿库（fitness ≥0.9 才入库）
runtime/teaching_run_state.json         最后成功阶段（供 --resume）
runtime/debug_runs/<stamp>/             效率模式每次运行的完整归档
```

---

## 6. 尚未完成（诚实记录）

```text
✗ 插入段未在真机完成（372 mm 不是物理底部，三面接触）
✗ 槽内转 10° 未验证（脚本已就绪，方向与师兄源码一致，但未在真机跑通）
✗ 六维力传感器未安装（十二月），B 类指标全部"未采集"
✗ 自研视觉管线未接入（十一月主线）
! 保护停止后的自动恢复已实现，但仍需人工确认现场
```
