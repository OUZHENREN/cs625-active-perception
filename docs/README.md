# CS625 视触融合精密装配 — 文档导航

> 更新：2026-11-07。本页是**唯一入口**，其余文档按"现在有效 / 历史归档"分区。
> 旧文档一律保留在 `archive/`，只读，不再维护。

---

## 一句话状态

```text
硬件：Elite CS625 真机 + 图漾 PS800-E1 相机（RVS 离线采集位姿）
已完成：观测 → 抓取 → 抬升 → 转运 → 分级 → 旋转 → 预插入（整链真机跑通）
未完成：插入到底（三面接触，372 mm 不是物理底部）、槽内转 10°、六维力（十二月）
十一月重心：自研视觉算法（替代 RVS 做位姿采集）+ 仿真平台搭建
```

---

## 下次开工先看

| 文档 | 用途 |
|---|---|
| [`worklogs/2026-11/2026-11-09_HANDOFF_NEXT_SESSION_VISION_ALGORITHM.md`](worklogs/2026-11/2026-11-09_HANDOFF_NEXT_SESSION_VISION_ALGORITHM.md) | **视觉算法研究交接**：现状、起点、第一步、陷阱清单 |
| [`worklogs/2026-11/2026-11-08_WORKLOG_PANEL_RVIZ_AND_CAMERA.md`](worklogs/2026-11/2026-11-08_WORKLOG_PANEL_RVIZ_AND_CAMERA.md) | 面板/RViz/相机这一轮的完整记录 |

## 先看这几份（按顺序）

| 顺序 | 文档 | 什么时候看 |
|---|---|---|
| 1 | [`agent_memory.md`](agent_memory.md) | **每次开工先扫一遍**：硬规则、常量、禁止事项 |
| 2 | [`reference/architecture_current.md`](reference/architecture_current.md) | 想知道代码怎么分层、示教链路 0–16 步是什么 |
| 3 | [`manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md`](manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md) | **上真机前**：29 条实机经验（含装配工艺、断联排障、安全） |
| 4 | [`reference/experiment_metrics.md`](reference/experiment_metrics.md) | 要做实验、要出数据之前 |
| 5 | [`reference/real_cell.md`](reference/real_cell.md) | 换场地、网络异常、开工就绪检查 |

---

## 目录结构

```text
docs/
├── README.md               ← 本页
├── agent_memory.md         ← 长期规则（活文件，只追加）
├── reference/              ← 现在有效的技术参考
│   ├── architecture_current.md      架构 + 链路 0–16 步 + 两模式
│   ├── experiment_metrics.md        六类 18 项指标体系
│   ├── real_cell.md                 真机工位 + 就绪检查 + 网络排障
│   └── simulation.md                仿真环境（总述/场景生成/保真度）
├── manuals/                ← 实机经验手册（29 条）
├── plans/                  ← 阶段计划
├── decisions/              ← ADR（含 ADR-005 两模式与插入权威）
├── worklogs/               ← 工作日志，按月份分目录
│   ├── 2026-09/  2026-10/  2026-11/
├── evidence/               ← 实验证据（133 项，不整理）
└── archive/                ← 历史归档（只读）
    └── 2026-11-07-pre-reorg/   本次重整前的原件 + MANIFEST
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

## VS Code 任务（只有 5 个）

| # | 任务 | 说明 |
|---|---|---|
| ① | 开工 · 真机 | 驱动栈 + MoveIt + 四项门 |
| ② | 安全模式 · 示教 | `resume_teaching.sh` |
| ③ | 效率模式 · 调试 | `teaching_debug.py --interactive` |
| ④ | 面板 · 两模式（4K） | `cs625_panel.py` |
| ⑤ | 自检 · 离线 | pytest + 文档一致性 + 环境 |

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
3. **新日期取代旧日期**：新版另起日期文件名，不覆盖旧版。
4. **实验数据不动**：`evidence/` 与 `runtime/` 下的实验产物一律不整理、不删除。
5. **文档也要自检**：`scripts/check_docs.py` 会检查章节编号、索引一致性、路径与提交哈希。
