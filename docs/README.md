# CS625 视触融合精密装配 — 文档导航

> 更新：**2026-11-13**。本页是**唯一入口**，其余文档按"现在有效 / 历史归档"分区。
> 旧文档一律保留在 `archive/`，只读，不再维护。

---

## 一句话状态

```text
硬件：Elite CS625 真机 + 图漾 PS800-E1 相机（RVS 仅用于示教链路每次的 T1/T2/T3 观测；插入定位已不依赖它）
主线：效率模式（`teaching_debug.py`，开发中）+ 示教模式（`resume_teaching.sh`，已冻结）
已完成：观测 → 抓取 → 抬升 → 转运 → 分级 → 旋转 → 预插入（示教链路真机跑通）；
        视觉测量链真机 0.14–0.5 mm（粗观测带先验 → 精观测带先验）
未完成：插入到底（三面接触，371–372 mm 不是物理底部，重复性 1.1 mm）、槽内转 10°、六维力（十二月）、
        规划场景里没有桌子/夹具（自动运动的前置阻塞）、方案 A 真机验收
```

---

## 下次开工先看

| 文档 | 用途 |
|---|---|
| [`worklogs/2026-11/2026-11-13_HANDOFF_VISION_MILESTONE.md`](worklogs/2026-11/2026-11-13_HANDOFF_VISION_MILESTONE.md) | **最新交接**：视觉里程碑现状、可用能力表、安全红线、未完成项 |
| [`plans/2026-11.md`](plans/2026-11.md) | **本月做什么**：三件交付、每周产物、验收线、明确不做 |
| [`worklogs/2026-11/2026-11-13_WORKLOG_P7_ARCHIVE_AND_PLAN_A.md`](worklogs/2026-11/2026-11-13_WORKLOG_P7_ARCHIVE_AND_PLAN_A.md) | 11-13 全天：P7 归档、方案 A 选站、文档纠错 |

## 先看这几份（按顺序）

| 顺序 | 文档 | 什么时候看 |
|---|---|---|
| 1 | [`agent_memory.md`](agent_memory.md) | **每次开工先扫一遍**：硬规则、常量、禁止事项 |
| 2 | [`reference/architecture_current.md`](reference/architecture_current.md) | 想知道代码怎么分层、示教链路 0–18 步是什么 |
| 3 | [`manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md`](manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md) | **上真机前**：29 条实机经验（含装配工艺、断联排障、安全） |
| 4 | [`reference/experiment_metrics.md`](reference/experiment_metrics.md) | 要做实验、要出数据之前 |
| 5 | [`reference/real_cell.md`](reference/real_cell.md) | 换场地、网络异常、开工就绪检查 |
| 6 | [`manuals/debug/README.md`](manuals/debug/README.md) | **看不到数据时**：相机/ROS 图的常用调试命令，正常输出与失败含义 |
| 7 | [`reference/pose_conventions.md`](reference/pose_conventions.md) | **动位姿之前**：相机系/对象系/内参分辨率/模板单位与原点，全部带实测证据 |
| 8 | [`reference/robot_status_codes.md`](reference/robot_status_codes.md) | **机器人不动时**：safety_mode / robot_mode 全部取值、常见组合与处置顺序 |

---

## 目录结构

```text
docs/
├── README.md               ← 本页
├── agent_memory.md         ← 长期规则（活文件，只追加）
├── reference/              ← 现在有效的技术参考（13 份，常用 8 份列在这里）
│   ├── architecture_current.md      架构 + 链路 0–18 步 + 两模式
│   ├── experiment_metrics.md        六类 18 项指标体系
│   ├── real_cell.md                 真机工位 + 就绪检查 + 网络排障
│   ├── simulation.md                仿真环境（总述/场景生成/保真度）
│   ├── pose_conventions.md          相机系/对象系/内参/模板单位（§4.1 手眼已验明）
│   ├── robot_status_codes.md        safety_mode / robot_mode 取值与处置
│   ├── frames_and_topics.md         TF 链与话题契约
│   ├── task_pose_derivation.md      任务位姿推导（插槽/插入链）
│   └── 其余：interfaces / dependencies / project_layout / file_viewer
├── manuals/                ← 实机经验手册（29 条）+ debug/ 调试命令分册
├── plans/                  ← 月度计划（YYYY-MM.md，一个月一份；旧月份不重写）
├── decisions/              ← ADR（含 ADR-005 两模式与插入权威）
├── worklogs/               ← 工作日志，按月份分目录
│   ├── 2026-09/  2026-10/  2026-11/
├── evidence/               ← 实验证据（P7 的 129 项已于 11-13 随 P7 归档；
│                              现留 P4 矩阵 + 视频出处图，不整理）
└── archive/                ← 历史归档（只读）
    ├── 2026-11-07-pre-reorg/   11-07 文档重整前的原件 + MANIFEST
    ├── 2026-10-24-pre-six-modes/  六模式之前的阶段性文档
    └── 2026-11-13-p7-nbv-legacy-code/  P7 旧仿真流水线（代码 + 运行证据 + YCB 资产）
```

---

## 两种模式怎么用

### 安全模式（演示 / 取数据）

```bash
source scripts/source_dev_env.sh --full
bash scripts/resume_teaching.sh
```

门检全开、静置 15 s、fitness 门控、位姿库拦截；失败会自动重启驱动栈，
并从"最后成功阶段"续跑（`runtime/teaching_run_state.json`）。

### 效率模式（十一月视觉开发 / 十二月力控）

```bash
python3 scripts/teaching/teaching_debug.py --stage 5            # 只跑预抓取
python3 scripts/teaching/teaching_debug.py --from 12 --to 13    # 只跑旋转+下降
python3 scripts/teaching/teaching_debug.py --stage 14 --repeat 3 --no-settle
```

每次运行归档到 `runtime/debug_runs/<时间戳>/`（params / log / captures / effort / result）。
`result.json` 已按六类 18 项指标建好字段，无传感器的项写 `未采集`。

**两种模式跑的是同一条链路**（`scripts/teaching/teaching_sequence.py`）。
六模式源码（`run_task.py` / `run_pick_and_place.py` / `cs625_ui.py`）**冻结保留**，不再作为入口。

---

## VS Code 任务（8 个）

| # | 任务 | 说明 |
|---|---|---|
| ① | 开工 · 真机 | 驱动栈 + MoveIt + 四项门 |
| ② | 安全模式 · 示教 | `resume_teaching.sh` |
| ③ | 效率模式 · 调试 | `teaching_debug.py --interactive` |
| ④ | 面板 · 两模式（4K） | `cs625_panel.py` |
| ⑤ | 自检 · 离线 | pytest + 文档一致性 + 环境 |
| ⑥ | DSH · 启动/打开界面 | 复用已在跑的 dsh web 实例 |
| ⑦ | 相机 · 查看/换档 | `camera_view.py`（只碰相机，不碰机械臂/夹爪） |
| ⑧ | 收工 · 关闭所有驱动 | `stop_all_drivers.sh` |

---

## 常用命令

```bash
# 环境（任何 ROS 命令前必须先 source）
source scripts/source_dev_env.sh --full

# 动臂前的四项门
python3 scripts/control_health.py
python3 scripts/preflight_motion.py

# 离线测试与文档一致性
python3 -m pytest test/ -q
python3 scripts/check_docs.py

# 工作日志同步到 Obsidian（写完后必须做）
scripts/sync_worklog.sh docs/worklogs/2026-11/<file>.md
```

---

## 文档规则

1. **只追加、不覆盖**：`agent_memory.md` 与手册只增条目，不改历史条目。
2. **归档要批准**：把文档移进 `archive/` 属于所有者决策，需明确同意；本次重整已获批准。
3. **计划逐月一份**：`plans/` 下文件名就是月份（`YYYY-MM.md`），当月方案只写当月文件；
   旧月份文件不重写、不覆盖（2026-11-08 所有者决定，见 `docs/plans/2026-10.md` 头部）。
4. **新日期取代旧日期**：新版另起日期文件名，不覆盖旧版。
5. **实验数据不动**：`evidence/` 与 `runtime/` 下的实验产物一律不整理、不删除。
6. **文档也要自检**：`scripts/check_docs.py` 会检查章节编号、索引一致性、路径与提交哈希。
