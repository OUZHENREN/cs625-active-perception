# 2026-11-13 工作日志：全工作区盘点、归档与长期文档刷新

> 所有者指令（2026-11-13 23:5x；实际执行跨过午夜到 11-14 00:2x，按所有者要求
> **整批计入 2026-11-13**）："总览完整工作区，梳理出明显脱节、不再使用的文件并归档；
> 对当前工作区做一次全面复盘和整理；尤其是 README 和真机调试经验这类长期会查看的文档，
> 请重点核对和更新。" 同轮另定两条最高规则（AGENTS.md 第 10/11 条：自动分派子智能体、
> 收尾换角度复盘）。
> **本轮没有任何真机运动，也没有启动 Gazebo/相机**；全部为离线盘点、归档与文档修正。

## 0. 盘点方法（可复核）

```text
· 规模：git 跟踪文件 —— scripts/ 135、src/ 209、config/ 19、test/ 46、docs/ 1590（多为记录）
· 引用图：把活动范围的文本文件合成语料，逐个脚本/文档统计引用（排除 docs/worklogs|evidence|archive）
· 两个只读审计子智能体并行（scripts-audit / docs-audit），主负责人复核关键证据后才动手
· 复核口径：每条"当前状态"结论回到 ADR / 代码常量 / 配置 / git 历史核对
```

## 1. 归档（git mv，历史完整、可逆）

### 1.1 脚本层 12 个 → `docs/archive/2026-11-13-scripts-legacy/`

| 分组 | 文件 | 依据 |
|---|---|---|
| `humble/` | `bootstrap_humble.sh`、`doctor_humble.sh`、`wsl_local_build.sh` | 要求 Humble/自述"NOT PART OF THE CURRENT JAZZY WORKFLOW"；现入口 `source_dev_env.sh` |
| `six-mode/` | `plan_fine_view.py`、`nbv_move.py`、`preinsert_plan_only.py`、`update_grasped_workpiece.py`、`auto_label_frames.py` | 活动范围 0 引用；均被现行件取代（nbv_from_reference / pose_math+plan_joint_locked / update_grasped_box / auto_label_four_class） |
| `teaching/` | `measured_slot.json`、`flatten_in_place.py`、`insert_probe.py`、`probe_lateral.py` | 0 引用；数据已固化进配置、工具被 straighten_in_slot / compliant_insert / assembly_search 取代 |

### 1.2 仿真资产 69 MB → `docs/archive/2026-11-13-p7-nbv-legacy-code/src_cs625_simulation/assets/ycb/`

YCB 三个对象（tomato soup can / power drill / wood block，含 `raw/*.tgz`）**活动范围 0 引用**：
唯一使用者是已归档的 `p7_ycb_tomato_*.sdf`（9 个世界）。活动 `assets/` 现只剩
`cs625_real_cell`、`cs625_task`。同时把 `sim_base.launch.py` 里误导性的变量名
`ycb_resource_dir` 改为 `assets_resource_dir`（它指向的是整个 assets 目录）。

### 1.3 死测试 1 个 → 同 P7 归档 `test/legacy_compliant_smoke.py`

导入 `cs625_compliant_placement.msg/srv`，该包**不在本仓库、不在活动 underlay**，
只存在于冻结的 `~/cs625_legacy_underlay_jazzy`（按 ADR-004 只读、不参与运行）⇒ 该测试无法运行。

### 1.4 同步改动

- `test/contract_checks.py`：`required_files` 移除已归档的两个 Humble 脚本；
  `LEGACY_ENV_ALLOWED_FILES` 清空（活动树已无允许提及 Humble 的文件）。
- `scripts/check_docs.py`：归档引用解析从"一层目录"改为**递归**（新归档按 group 分层，
  如 `docs/archive/<stamp>/six-mode/<file>`），否则 `agent_memory` 里对旧脚本的引用会被判失效。
- `scripts/README.md`：索引重写——两种模式为主线、六模式标"冻结"、去掉已归档条目。

## 2. 长期文档刷新（所有者点名项）

### 2.1 根 README.md（上一轮已改，本轮补细节）
观测前缀一行注明"面板效率页走 run_task，不是 ADR-005 的 teaching_debug 效率模式"；
插入权威括号改为"R1 只作对比、不能用来算目标"。

### 2.2 docs/README.md（文档唯一入口）
更新日期 11-07→11-13；一句话状态改为两模式主线 + 视觉测量链已达标 + 两个阻塞
（规划场景无环境、方案 A 未真机验收）；"下次开工先看"指向 11-13 交接与 11-13 工作日志；
VS Code 任务 7→8；目录树补 `2026-10-24-pre-six-modes/`、`2026-11-13-p7-nbv-legacy-code/`；
evidence 说明改为"P7 的 129 项已随 P7 归档"；reference 树由 4 份补到常用 8 份 + 其余。

### 2.3 真机调试经验手册（29 条）
按"只追加、不改历史"的自身规则，就地追加更正，不改写原结论：
- **#8**：5b/6b 容差原写 0.10°，更正为 **1.5 mm / 0.40°**（13b 才是 1.0 mm / 0.10°，代码 :1605/:1617/:1772）；
- **#20**：保护停止 z 原写 0.0806，与"371 mm"算术不符，更正为 **0.0721**
  （槽顶 0.443245 − 0.371 = 0.0721，与 `teaching_insert_end.tcp` 一致）；
- **#14**：①"基准 B 待开发"已取消（粗观测位固定，agent_memory §138/§139）；
  ②`teaching_observation_baselines` 实为粗观测站位 A/B/C，不是朝向基准；
  ③记录的 J6=+171.6° 违反 §133 禁令，不得直接复用；
- **#23**：注明 `assembly_search.py` 未接入主链，主链是 compliant_insert + straighten_in_slot；
- 索引后加**作用域**说明：RVS 条目（#1/#2/#13/#16）只适用示教链路；
- #29 标注"截至 11-13 仍未真机验收"；开工清单第 6 条注明 RVS 仅示教链路需要。

### 2.4 docs/agent_memory.md（只追加）
- 头部"最后更新 2026-10-21" → **2026-11-13**；
- §3/§4 的"只认当次新鲜 R1"、§4 的"持件 ≤0.020 m/s"就地加指针；§10/§11/§12 标题标〔历史〕；
- 新增 **§139 口径更正汇总**：插入权威（verified JSON 优先，R1 仅兜底）、持件速度
  （`CARRY_SPEED_CAP=0.013`、SPEED 表实际值、插入 0.002）、手眼已验明（§131）、
  当前两模式主线、粗观测基准点、禁止构型补充。

### 2.5 reference / ADR / plans
- 手眼状态：`real_cell.md`（2 处）、`simulation.md`（2 处）、`task_pose_derivation.md`
  由"未验明/差 0.16 m"改为"**2026-11-10 已验明**（`T_base_tcp @ T_tcp_cam`，≤3 mm）"；
- 门检容差：`interfaces.md` 改为 5b/6b 1.5 mm/0.40°、13b 1.0 mm/0.10°；`architecture_current.md` 13b 同步；
- 示教链路步数 0–16 → **0–18**（编号 2 已删除）：architecture、project_layout、docs/README 共 6 处；
- TF 链：`frames_and_topics.md`、`simulation.md` 改正为 `flange → tool0`（identity）与
  `flange → my_end_effector_link`（TCP，−0.046, 0, 0.353）是**兄弟节点**，不是串联；
- `architecture_current.md` 的"自研视觉未接入"改为"已接入（0.14–0.5 mm）"；
- `plans/2026-11.md`：1280×960"有效点≈0"与同文件 76.1% 自相矛盾 → 更正；数据流改 modeling 1280×960；
  手眼一行注明 11-10 已验明；
- `ADR-001` 状态标 Superseded by ADR-004；`ADR-005` 的 VS Code 5→8、RVS 措辞澄清；
- `file_viewer.md` 删除不存在的"VS Code 任务 50"；`manuals/debug/camera.md` 偏振 A/B 标"11-09 已结案"；
- `AGENTS.md`：config 路径改为真实布局（`config/tasks/`、`config/handeye/`、
  `src/cs625_bringup/config/`、`config/real/`）；"Two layers"→"Four layers"；
  rviz_panel 标冻结；验证阶梯补 `teaching_debug.py --dry-run`。

## 3. 明确**没有**归档的（需所有者决定）

25 个"0 代码引用但可能是手动工具/未完成计划"的脚本**一律保留**，其中两个不能动：

- `scripts/sync_task_world.py`：README 标"旧"，但 `src/cs625_bringup/scripts/apply_task_scene.py`
  声明与它保持同步，且被 2 个 pytest 加载、`contract_checks` 要求存在；
- `scripts/calibrate_workpiece_frame.py`：11-11 日志里是"工件摆回示教位姿后再跑"的**未完成标定**。

其余 25 个候选（0 代码引用，但可能仍要手动用）——**未归档，等所有者拍板**：

```text
record_frames.py            旧 1 Hz 录像（现用 experiment_recorder）
depth_pose_once.py          单帧深度位姿（ICP 之外的老路径）
coarse_to_fine_pose.py      粗→精早期流水线
build_environment_collision.py  PCD→碰撞体（被 model_environment 覆盖）
auto_label_four_class.py    四类几何标注（重做训练集可能用）
capture_environment.py      环境负样本采集
visual_mesh_publisher.py    RViz 可视网格发布
ui_plots_demo.py            曲线控件演示
live_status_page.py         渲染 phase_log 的状态页
record_sim_screen.sh        全桌面录屏（P7 证据用）
ros_rgb_observer.py         早期 RGB 观察器
accuracy_log.py             RVS 精度表（RVS 仅示教链路）
fine_adjust.py              手动微调（[13b] 已用 adjust_tcp）
measure_effort_baseline.py  effort 基线采样（结论已固化）
validate_carried_pose.py    持件位姿校验（与 check_carried_clearance 重叠）
known_target_calibration.py 已知目标标定
insertion_configuration_sweep.py  插入构型可行性离线分析（论文复算可能用）
run_teaching_chain.py       分步驱动（被 teaching_sequence 取代）
shims/trimesh.py            师兄 tcp_bridge 的兼容垫片
run_minimum_showcase_matrix.sh    P4 矩阵（contract_checks 要求存在）
sync_task_world.py          ★ 被 src 契约 + 2 个 pytest 依赖，不能动
calibrate_workpiece_frame.py ★ 未完成标定计划，不能动
bootstrap/doctor/wsl_local_build   已归档（见 §1.1）
```

## 4. 验证（全部离线）

```bash
source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q          # 275 passed
python3 test/contract_checks.py     # Phase 0–6 static contract checks: PASS
python3 scripts/check_docs.py       # ALL DOC CHECKS PASSED（29 节手册、59 路径、0 失效）
```

换角度复盘（AGENTS.md 第 11 条）的结果：
- 反证 1：12 个已归档脚本在活动范围**0 悬空引用**；
- 反证 2：`assets/` 仍有 `cs625_real_cell`、`cs625_task`，`sim_base.launch.py` 资源路径成立；
- 反证 3：`0–16` 已清零；`372 mm` 统一为 `371–372 mm`；手眼"未验明"已清零；
- 反证 4：reference 里所有"六模式"表述都带冻结/历史标注，无"六模式是入口"残留；
- 反证 5：`scripts/README.md` 不再出现已归档脚本名。

## 5. 能力层与状态（诚实）

```text
Capability layer: 无变化（本轮是工作区整理，不动运动/感知能力）
  环境与依赖      PASS（离线：pytest/contract/docs 三项全绿）
  真机            NOT RUN —— 本轮没有真机运动、没有启动 Gazebo/相机
Intentionally not done（需所有者决定）:
  · 25 个"疑似过时"脚本未归档（清单见 §3）；
  · agent_memory §3/§4/§10–§12 的旧条目按"只追加"规则保留原文，仅加指针与 §139 汇总；
  · docs/worklogs、docs/evidence、docs/archive 内的历史记录未做任何改写。
```

## 6. 复现命令

```bash
source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q && python3 test/contract_checks.py && python3 scripts/check_docs.py
git log --oneline -1 --stat        # 看本轮改了哪些文件
```
