# 文档索引

> 这里只放**现在有效**的文档。旧文档（2026-10 之前）在 [`archive/`](archive/)，只读保留。

## 先看这几份

| 文档 | 什么时候看 |
|---|---|
| [`../README.md`](../README.md) | 第一次进入项目：现在能做什么、三分钟上手 |
| [`../AGENTS.md`](../AGENTS.md) | 动手改代码之前：工作方式与硬规则 |
| [`source_authority.md`](source_authority.md) | 同一个量有多个数值时：以哪份为准（A/B/C 分级） |
| [`project_layout.md`](project_layout.md) | 想知道文件该放哪、包边界是什么 |
| [`interfaces.md`](interfaces.md) | 话题/服务/动作契约，任务层入口数据流 |
| [`frames_and_topics.md`](frames_and_topics.md) | TF 树与坐标系含义 |
| [`simulation.md`](simulation.md) | 仿真怎么起、与真机怎么 1:1 对齐 |
| [`dependencies.md`](dependencies.md) | 第三方/师兄代码边界与版本 |
| [`real_hardware_readiness.md`](real_hardware_readiness.md) | 上真机前的检查清单 |
| [`experiment_data_protocol.md`](experiment_data_protocol.md) | 实验数据字段与归档约定 |

## 长期记忆

| 文档 | 内容 |
|---|---|
| [`agent_memory.md`](agent_memory.md) | 位姿基准、踩过的坑、关键结论（按 § 编号增长，只追加不删） |

## 工作日志

[`worklogs/`](worklogs/) 按日期命名，是"每天到底做了什么"的唯一记录：

```text
YYYY-MM-DD_WORKLOG_<主题>.md     当天的工作与证据
YYYY-MM-DD_HANDOFF_<主题>.md     交接给下一次会话
YYYY-MM-DD_PLAN_<主题>.md        计划
YYYY-MM-DD_SPEC_<主题>.md        设计/规格
```

每次写完都必须镜像到 Obsidian：`scripts/sync_worklog.sh docs/worklogs/<file>.md`。

## 证据与数据

| 位置 | 内容 |
|---|---|
| [`evidence/`](evidence/) | 已验收的证据文件（图、日志、摘要） |
| [`evidence/experiment_index.md`](evidence/experiment_index.md) | 实验归档索引（正式实验的长期记录） |
| `~/cs625_experiment_archive/` | 实验原始数据的裁剪归档（不在仓库里，太大） |

## 决策记录

[`decisions/`](decisions/) 保存 ADR（架构决策记录），例如
[ADR-004](decisions/ADR-004-legacy-senior-underlay.md)：师兄代码以只读 underlay 形式保留。

## 归档

[`archive/`](archive/) 存 2026-10-24 之前的历史文档（Phase 0/1 交接、P5/P6/P7 协议与验收、
旧仿真说明、旧的 diagnostics）。它们**不再维护**，只在复盘时查阅。归档规则见
[`archive/README.md`](archive/README.md)。
