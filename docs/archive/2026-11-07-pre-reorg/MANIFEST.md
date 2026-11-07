# 2026-11-07 文档重整：旧路径 → 新路径

本次重整把过时文档留存在此目录（只读，不再维护），并把仍有价值的内容合并/更新到正式位置。
**没有任何内容被删除**：这里保留的是重整前的原件。

## 一、直接归档（内容已过时，原件仅作历史）

| 原件 | 归档原因 |
|---|---|
| `architecture.md` | P3/P4 时代的分层描述，与现状不符；已由 `reference/architecture_current.md` 取代 |
| `experiment_data_protocol.md` | 旧实验记录协议；已由 `reference/experiment_metrics.md`（六类 18 项）取代 |
| `migration_from_legacy.md` | 迁移已完成，属历史 |
| `real_hardware_readiness.md` | 真机前的就绪门，已并入 `reference/real_cell.md` 的开工检查 |
| `source_authority.md` | 其中"以 2026-10-22 标定为准"已被 2026-11-05 裁决否定（见 ADR-005） |

## 二、合并（原件在此，内容已并入新文档）

| 原件 | 合并去向 |
|---|---|
| `simulation.md` | `reference/simulation.md`（第 1 节 定位与原则） |
| `sim_scene_from_real_runs.md` | `reference/simulation.md`（第 2 节 场景生成流程） |
| `sim_fidelity.md` | `reference/simulation.md`（第 3 节 保真度实测） |
| `real_run_reference.md` | `reference/real_cell.md`（第 2 节 真机轨迹索引） |

## 三、更新后移动（内容改写为现状，原件在此）

| 原件 | 新位置 | 更新要点 |
|---|---|---|
| `interfaces.md` | `reference/interfaces.md` | 六模式契约 → 位姿契约 + 观测采集中间产物 |
| `frames_and_topics.md` | `reference/frames_and_topics.md` | 补 tcp_pose_broadcaster / teaching 采集话题 |
| `project_layout.md` | `reference/project_layout.md` | 补 teaching/ 与 scripts 分层现状 |
| `dependencies.md` | `reference/dependencies.md` | 补相机↔RVS 互斥、WSL 镜像网络、Windows 防火墙 |
| `file_viewer.md` | `reference/file_viewer.md` | 原样移动（工具仍可用） |

## 四、工作日志

`docs/worklogs/*.md` 全部按月份归位到 `docs/worklogs/2026-09/`、`2026-10/`、`2026-11/`，
**文件名不变**；`docs/worklogs/README.md` 与两篇 2026-08 的历史日志保留在 worklogs 根目录。

## 五、VS Code 任务

`vscode_tasks_six_mode.json` = 重整前的 17 条任务（六模式入口）。
新任务表只有 5 条，见 `.vscode/tasks.json` 与 `docs/README.md`。

## 六、可逆性

所有移动均通过 `git mv` 完成，历史完整保留。若需恢复某一篇：
`git log --follow -- <新路径>` 可追溯，或直接从本目录复制回原位。
