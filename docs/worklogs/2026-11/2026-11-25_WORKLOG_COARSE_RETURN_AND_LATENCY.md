# 2026-11-25 工作日志：一键回粗观测位打通 + 两处真机故障 + 延迟规矩

> 目标（所有者）：真机已全部启动，继续 11-24 的任务。
> 结果：一键回粗观测位**真机执行成功**（8.2 s 到位，TCP 与目标差 0.2 mm）；
> 期间修掉两个真实缺陷，并新增两条硬规矩。

## 0. 时间线

| 时间 | 做了什么 | 结果 |
|---|---|---|
| 17:06 | 开工四项门 + 夹爪串口 | 全通过；**夹爪串口恢复**（init=5, position_mm=0，11-24 的阻断解除） |
| 17:10 | 核对停车位 | 最大偏差 0.75°，TCP 与记录差 4 mm → 已在停车位，第 2–3 步无需再动 |
| 17:12 | `goto_workpiece_coarse.py` 只规划 | **IK_FAILED：wrist_1 需转 94° > 90° 翻转阈值** |
| 17:14 | 加 `--allow-flip` 再规划 | 通过：同分支、单轴 94°、其余五轴 <1.5°、轨迹抽样无接触 |
| 17:20 | 独立 FK 复核扫掠 | TCP z 0.057→0.508 单调上升，距基座轴线最近 0.162 m，无越界 |
| 17:25 | 用 353 mm TCP 做工件间隙检查 | **所有者纠正第三次**：未抓取前 TCP 是虚拟点，实体是 207 mm/±147.3 mm（手册 56） |
| 17:30 | 改用 `check_tool_envelope.py`（正确口径） | 最坏间隙 **240 mm**，安全；此前 73.7 mm 结论作废 |
| 17:36 | 所有者点任务 ⑬ 失败（无 `--allow-flip`） | 修：任务内置 `--allow-flip` + 放行时打印六关节表（手册 57） |
| 17:41 | 首次 `--execute` | **失败 rc=1，机械臂一步没动**：下发前区间检查里 `dict(SAFE_JOINT_LIMITS)` 在 3 元组上抛 ValueError |
| 17:44 | 修 `dict()` bug，抽成 `trajectory_within_safe_limits()` 并写真实调用测试 | 393 passed |
| 17:45 | 再次 `--execute` | **成功**：governor 8.21 s/0.200 rad/s 单段，status=4 SUCCESSFUL |
| 17:46 | 复核 TCP | (0.6238, 0.3738, 0.5076) = config 目标，误差 0.2 mm |

## 1. 关键教训

```text
① 未抓取前 353 mm 的 TCP 是虚拟抓取基准点（所有者第三次纠正）：
   实体只有夹爪手指 + 电机保护座 = flange 前 207 mm、横向 ±147.3 mm。
   用它做间隙检查会得到完全错误的"过近"结论（73.7 mm vs 实际 240 mm）。
   → 手册第 56 条；agent_memory 新增硬行。

② 停车位 ↔ 粗观测位 天生差 94°（停车位就是"只把 J4 转到 −80.5°"设计的）：
   一键任务必须内置 --allow-flip 并打印六关节表，否则永远失败。
   → 手册第 57 条。

③ 只 grep 源码的测试挡不住运行期错误：
   昨晚 test 里修正了 dict(SAFE_JOINT_LIMITS) 的写法，但**源码没改**，
   测试只断言"字符串存在"，于是缺陷一路带到真机。
   现在改成真正调用 trajectory_within_safe_limits() 的测试。
```

## 2. 新增规矩（所有者 2026-11-25）

```text
· 所有与真机机械臂有关的动作：超过 1 分钟没有反馈，不允许继续等待，必须停下来报告。
  （11-25 实况：一次 --execute 失败后我让它空等了约两分钟，所有者明确制止。）
· 我自己的推理延迟（每轮 1–2 分钟）不属于"机械臂慢"，必须如实区分，
  并且用更少的调用次数、更短的回复、后台任务 + 立即汇报来降低等待感。
```

## 3. 变更文件

```text
scripts/execute_assembly_chunks.py   新增 safe_joint_limit_map() / trajectory_within_safe_limits()，修 dict() 崩溃
scripts/goto_workpiece_coarse.py     --allow-flip 时打印六关节表；docstring 写清停车位路径
.vscode/tasks.json                   任务 ⑬ 内置 --allow-flip
docs/manuals/…EXPERIENCE.md          新增第 56、57 条
docs/agent_memory.md                 新增两条硬行（末端实体口径、停车位↔粗观测位）
test/test_audit_regressions.py       把"grep 源码"改成真实调用
```

## 4. 验证

```bash
python3 -m pytest test/ -q            # 393 passed
python3 scripts/check_docs.py         # ALL DOC CHECKS PASSED
python3 scripts/goto_workpiece_coarse.py --execute --allow-flip   # 真机成功，8.2 s
```
