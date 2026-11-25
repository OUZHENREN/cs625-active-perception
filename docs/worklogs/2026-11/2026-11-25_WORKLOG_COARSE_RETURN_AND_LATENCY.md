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

## 3. 18:00–18:30 效率模式逐步验证（所有者：工件今天已换位置）

工件今天被移动了：11-24 的 138 mm 外的新位置。开始前把失效先验归档
（`workpiece_prior.json` → `workpiece_prior_20261124_stale.json`），拍一帧确认工件在视野内。

### 18:00 第 1 步：粗观测（起点的 mode 起点的位姿）+ 粗测

```text
命令：run_task.py --scope load --mode efficient --world real       --phases observe:workpiece_coarse_efficient,measure:workpiece_coarse --confirm
结果：EXIT=0，measure 阶段 40 s
  覆盖率 0.98（11-24 为 0.871）｜yaw −25.1°（11-24 为 +9.2°）
  中心 (885.8, 686.6, 127.4) mm（比 11-24 移动 138 mm）
  重复性 0.66 mm / 0.101°｜verdict OK｜2 组×2 帧实拍（先验未顶替）
```

### 18:05 第 2 步：选站换位（−29@20261120）+ 精测

```text
离线选站：yaw −25.3° → −29@20261120（tier1，差 3.3°，cov 0.976）
         摆放检查 15.9 mm / 3.33°（限 50 mm / 5.0°）→ 通过
nbv 执行：ok，到位复核无接触 ✅
精测第 1 次：**REFUSE**
  reasons: 低覆盖 0.926 < 0.95；中心重复性 1.1 mm > 1.0；姿态重复性 2.22° > 0.5
  逐组：g00 fit 0.926 / g01 fit 0.965（两者互差 0.03°）/ g02 fit 0.772（离群，差 2.22°）
  → 3 组里 2 组几乎完美，单组瞬时异常把整体判据拉爆
精测第 2 次（原地重测，不动臂）：**OK**
  覆盖率 0.978｜重复性 0.42 mm / 0.078°｜yaw −25.8°
```

### 18:15 相机面板"无图像"：查出两个真问题

```text
现象：效率页相机区一直显示「无图像（相机未启动或 RVS 占用）」，
      但同一时刻 measure_object 用同一台相机正常出图。

独立复现（BEST_EFFORT + 四个候选话题，8 秒）：
  /camera/color/image_raw          4 帧 ✅
  /camera/depth/image_raw          6 帧 ✅
  /sensors/camera/color/image      0 帧（驱动侧未发布）
  /sensors/camera/depth/image      0 帧
→ 相机与话题都正常，问题在面板侧。

真因（代码缺陷）：
  ① `COLOR_TOPIC` 本身就是 "/camera/color/image_raw"，而 panel_camera 里
     又写了一遍同名话题 → **同一个话题被订阅两次**（无意义且浪费）；
  ② 面板对"收不到图像"没有任何诊断输出，只能看到一句占位文字，
     无法判断是"没订阅上"还是"没发布"。
修：话题候选去重（`dict.fromkeys`）并把归一化话题排前面；
    15 s 无帧时打印一次诊断：每条候选话题的**发布者数量**。
```

### 18:20 示教器切换本地/远程 → 立即重挂 + 2° 探针

```text
所有者：刚在示教器上切了一次本地控制 / 远程控制。

按手册第 31/32 条：切换后反控通道常常"端口还在 ESTAB、指令根本不执行"，
所以状态话题（robot_mode=7 / safety_mode=1 / 3 条 ESTAB）**不算证据**。

处置：
  python3 scripts/ensure_robot_ready.py --force  → 已重挂为 RUNNING
  2° 微动探针（wrist_3 +2.000°）                 → 实测 +2.00° ✅ 跟指令走
```

### 18:25 新规矩写进代码：测量失败自动重测

```text
所有者：测量第一次没过要自动重测，不要直接失败等人。

实现（scripts/run_pick_and_place.py）：
  MEASURE_ATTEMPTS = 3
  run_observation_check(command, pose, attempts) 对含 measure_object.py 的命令
  自动重试 3 次，每次失败立刻打印原因并重测；model_environment 等检查不重试，
  避免掩盖真问题。measure / observe 两个调用点都已传 attempts=MEASURE_ATTEMPTS。
本次 18:07 那次 REFUSE 正好就是需要这个机制的场景（原地重测即通过）。
```

## 4. 验证

```bash
python3 -m pytest test/ -q     # 393 passed
python3 scripts/check_docs.py  # ALL DOC CHECKS PASSED
真机：粗测 OK（0.98）→ 选站 −29 → 精测重测 OK（0.978 / 0.42 mm / 0.078°）
      2° 探针命中 +2.00°；相机面板已修但**未在重启后的面板上复核**
```

### 18:50–19:10 相机面板"无图像"：找到真因（两个 worker 互相饿死）并修复

```text
所有者重启面板后仍然没有相机数据。逐层实验（全部可复现）：
  单线程单节点订阅        → 4 帧/8 s ✅
  面板 CameraView 单视图  → 13–17 帧/15–20 s ✅
  两个 CameraView 同时跑  → **两边各 0 帧** ❌（真实面板结构）
  去掉共享 SPIN_LOCK      → 仍 0 帧（与锁无关）
  纯 rclpy 两节点两线程   → 7+5 帧 ✅（不带 Qt 时正常）
  纯 rclpy 两节点一执行器 → 0 帧
→ 结论：一个进程同时跑两个相机订阅 worker，本机 DDS/rclpy 组合两边都收不到。

修（panel_camera.py）：
  shared_worker() 单例：全面板只跑一个订阅线程，一次订阅彩色+深度，两路都 emit；
  两个 CameraView 按自己的 _source 过滤（start 里 lambda 过滤连接）；
  stop() 只断开；aboutToQuit → stop_shared_worker() 收尾。
  实测两视图各 17 帧/15 s，状态栏显示 color 1280×960。

同时修掉：
  · CameraWorker 误用 self._node_suffix（CameraView 的属性）→ 15 s 诊断时抛
    AttributeError 直接退出；
  · COLOR_TOPIC 与硬编码话题相同 → 同一话题订阅两次，已去重；
  · 15 s 无帧诊断改为打印每条候选话题的发布者数量。

→ 手册第 61 条。**需要重启面板（任务 ④）后由所有者肉眼确认出图。**
```
