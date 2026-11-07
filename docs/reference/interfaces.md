# 应用层接口契约（2026-11-07）

> 取代归档的 `interfaces.md`（那份描述六模式时代的 scope/mode 契约）。
> 六模式源码冻结保留，但**不再是入口**；当前入口是两种模式。

## 1. 两种模式（唯一入口）

| 模式 | 入口 | 契约 |
|---|---|---|
| 安全模式 | `scripts/resume_teaching.sh` | 门检全开（5b/6b/13b，1.5/1.0 mm，0.4°）、静置 15 s、fitness 门控（T1/T3 ≥0.9、T2 ≥0.8）、位姿库拦截、失败自动重启驱动栈并 `--resume` |
| 效率模式 | `scripts/teaching/teaching_debug.py` | `--stage/--from/--to/--no-gates/--no-settle/--speed-override/--repeat`；每次运行归档到 `runtime/debug_runs/` |

**两种模式都不跳过**：保护停止规则、工件附近先抬升守卫、释放许可。

## 2. 采集中间产物（RVS → 磁盘 → 链路）

```text
runtime/rvs_teaching/T1_<ts>_aNN.json   插槽位姿；fitness、请求位姿、返回位姿
                                     T2_...json  工件粗观测视角（只要 z 合法）
                                     T3_...json  工件位姿（抓取依据）
字段：command / position / orientation / fitness / fitness_window / request_*
校验：工作空间盒、四元数范数、逐命令 z 窗口、≤6 h 新鲜度、帧间离散度
```

## 3. 插入权威（2026-11-05 裁决）

```text
优先级：runtime/slot_reference_verified.json
      → config/tasks/common.yaml 的 teaching_slot_reference（备份）
      → 当天 R1（兜底，且打印警告）
```

## 4. 运行状态与续跑

```text
runtime/teaching_run_state.json  {last_phase, resume_start_from, stamp}
写入时机：每个 move 阶段成功后
消费方：--resume（面板「续跑」按钮 / resume_teaching.sh）
```

## 5. 实验产物

```text
runtime/debug_runs/<stamp>/params.json   本次参数 + 常量快照
                           /log.txt
                           /captures/    本次 T1/T2/T3
                           /effort.csv   力矩曲线
                           /result.json  六类 18 项指标（无通道写"未采集"）
```

## 6. 文档与检查

```text
scripts/check_docs.py   手册章节连续性、索引一致性、活文档路径与提交哈希
规则：历史文档（worklogs/archive）不参与路径检查；活文档必须全部可解析
```

> 来源：由归档原件 `interfaces.md` 更新，从"六模式契约"改写为"两模式 + 采集/权威/续跑契约"。
