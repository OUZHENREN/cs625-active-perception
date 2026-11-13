# 归档：脚本层遗留（2026-11-13）

> 归档 = **只读保留，不是删除**。这里的一切都不再被当前流程使用，只用于溯源。
> **不要照着这里的脚本执行命令。**

## 归档原因

2026-11-13 的工作区盘点（所有者要求"梳理出明显脱节、不再使用的文件并归档"）中，
确认以下脚本在**活动范围**（`scripts/ src/ test/ config/ docs/reference|manuals|decisions|plans`、
根 `README.md`/`AGENTS.md`、`.vscode/tasks.json`）内 **0 引用**，且都有明确的现行替代者
或被更新的决策取代。全部 `git mv` 移动，历史完整。

## humble/ —— Humble 时代的构建/迁移工具

| 归档文件 | 原路径 | 为什么归档 | 现行替代 |
|---|---|---|---|
| `bootstrap_humble.sh` | `scripts/bootstrap_humble.sh` | 要求 `ROS_DISTRO=humble`，Jazzy 下直接 exit 2 | `scripts/source_dev_env.sh`（唯一环境入口） |
| `doctor_humble.sh` | `scripts/doctor_humble.sh` | 同上 | 同上 + `scripts/control_health.py` |
| `wsl_local_build.sh` | `scripts/wsl_local_build.sh` | 文件头自述 "HISTORICAL MIGRATION TOOL — NOT PART OF THE CURRENT JAZZY WORKFLOW" | `scripts/build.sh` |

## six-mode/ —— 六模式（已按 ADR-005 冻结）时代的执行件

| 归档文件 | 原路径 | 为什么归档 | 现行替代 |
|---|---|---|---|
| `plan_fine_view.py` | `scripts/plan_fine_view.py` | 11-12 的精观测位规划执行器；11-13 方案 A 落地后 `nbv:workpiece` 阶段改调 `nbv_from_reference.py` | `scripts/nbv_from_reference.py` |
| `nbv_move.py` | `scripts/nbv_move.py` | 11-12 闭环换位实验，当天日志明写"尚未接进流程"；最终从未进入任何配置/代码路径 | `scripts/nbv_from_reference.py` |
| `preinsert_plan_only.py` | `scripts/preinsert_plan_only.py` | 把轨迹写到 `/tmp/preinsert_plan.json`，全仓库无读取者 | `scripts/teaching/pose_math.py` + `plan_joint_locked.py` |
| `update_grasped_workpiece.py` | `scripts/update_grasped_workpiece.py` | 默认输入 `runs/20261022_workpiece_model/workpiece_voxels.json` 已不存在；10-22 交接即标"备选方案" | `scripts/update_grasped_box.py` + `attach_workpiece.py` |
| `auto_label_frames.py` | `scripts/auto_label_frames.py` | 单类几何标注，README 已标"被 four_class 取代" | `scripts/auto_label_four_class.py` |

## teaching/ —— 示教链路上的手工/一次性工具

| 归档文件 | 原路径 | 为什么归档 | 现行替代 |
|---|---|---|---|
| `measured_slot.json` | `scripts/teaching/measured_slot.json` | 2026-10-28 一次性人工对中数据，0 消费者；其 six_mode 值已固化进 `config/tasks/common.yaml` | 配置常量 |
| `flatten_in_place.py` | `scripts/teaching/flatten_in_place.py` | 11-04 卡滞后的手工整平工具 | `scripts/teaching/straighten_in_slot.py`（0.5°/步、遇阻回退） |
| `insert_probe.py` | `scripts/teaching/insert_probe.py` | 11-04 一次性下探力矩曲线（CSV 停在 11-04） | `scripts/compliant_insert.py` effort 监督 + `assembly_search.py` |
| `probe_lateral.py` | `scripts/teaching/probe_lateral.py` | 11-04 触碰式找槽横向中心 | `scripts/teaching/assembly_search.py` C 阶段 |

## 同步改动（同一提交）

- `test/contract_checks.py`：`required_files` 移除已归档的 Humble 两个脚本（保留 `build.sh`/`test.sh`）；
  `LEGACY_ENV_ALLOWED_FILES` 清空（活动树中已无允许提及 Humble 的文件）。
- `scripts/README.md`：索引按当前状态重写（两种模式主线、六模式冻结、去掉已归档项）。

## 可逆性

全部为 git 跟踪文件，历史完整。恢复单个文件：

```bash
git mv docs/archive/2026-11-13-scripts-legacy/<group>/<file> <原路径>
```

并撤销上表的同步改动。

## 仍然保留、但属于"疑似过时"的脚本（未归档，等所有者决定）

盘点中另有 25 个脚本属于"0 代码引用但可能是手动工具/未完成计划"，**没有**归档，
清单见 `docs/worklogs/2026-11/2026-11-14_WORKLOG_WORKSPACE_AUDIT.md`。
其中两个**明确不能动**：

- `scripts/sync_task_world.py`：README 标"旧"，但 `src/cs625_bringup/scripts/apply_task_scene.py`
  声明与它保持同步，且被 2 个 pytest 加载、`contract_checks` 要求存在；
- `scripts/calibrate_workpiece_frame.py`：11-11 日志里列为"工件摆回示教位姿后再跑"的
  **未完成标定计划**。
