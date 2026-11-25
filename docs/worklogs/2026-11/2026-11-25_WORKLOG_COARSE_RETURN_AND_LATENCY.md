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

### 19:30–19:45 桌面点云能否使用：修通"测量数据 → 环境模型"这条链

```text
所有者问：当前拍摄的桌面点云是否能够正常使用？这是悬而未决的事情。

实测结论：**不能直接用**，缺两样东西
  ① 缺位姿：measure_object 写 depth.npy + meta.json，meta.json 里没有机械臂位姿；
     model_environment 需要每组 metadata.jsonl（{"file","tcp_matrix"}）+ <stem>_depth.npy，
     而 tcp_matrix 必须是 T_base_tcp。
  ② 单位不同：measure_object 存的是米(float32)，model_environment 按 uint16 毫米处理
     （阈值 150–2000），直接复用会把每帧清零 → "fewer than 1000 valid points"。
  另外 runtime/environment/latest 还停在 2026-11-05（20 天前），dry-run 一直报 MISSING。

处置（所有者选 A：不重拍）：
  measure_object 采集后调用 write_env_metadata()：
    T_base_tcp = T_base_cam @ inv(T_tcp_cam)（T_base_cam 本来就有），
    每组写 <组名>_depth.npy（米→毫米 uint16）+ metadata.jsonl；无位姿则不写。
  当天三份历史数据集用同一函数补齐，无需重拍。

结果（用 19:18 slot_coarse 的 5 组数据重建）：
  frames 5｜valid points 28108｜collision voxels 545
  workpiece_table z=-0.1102 m（与实测桌面 −0.111 一致）x[0.840,1.320] y[0.400,0.880]
  slot_table z=-0.0497 m x[1.200,1.680] y[-0.260,0.200]
  首次 build gate REJECT：工件偏差 142.77 mm / 10.26°（正是今天换的位置）
  → --accept-workpiece-move 重建后 build gate PASS，verify-only 全 ok（age 0.0 h）

必须记住的边界：
  · 模型里的 workpiece 字段来自 config 历史值，不是本次实测，不能当抓取依据；
  · slot 字段与插入参考位姿差约 39 mm（坐标系约定差），插入仍用 slot_reference_verified.json；
  · 真正可用的是桌子几何与 545 个碰撞体素。

→ 手册第 62 条；新增 test/test_measure_env_metadata.py（3 条，锁住单位换算与 tcp_matrix 口径）。
```

### 20:00–20:10 插槽观测改粗/精两级 + 环境模型改到 slot_fine 之后

```text
所有者："A，但是难道不应该放在插槽精观测之后吗？这样出来的插槽位姿才更准"

查配置后发现比顺序更严重的问题：**插槽的精配准写在粗观测位阶段**
  observe:slot_coarse 的检查 = measure_object --preset slot --groups 5（5 组精配准）
  observe:slot_fine   的检查 = 只比基线，完全不测量
→ 与工件流程（coarse 粗测 → fine 精测）相反。

改（所有者批准 A）：
  · slot_coarse → --stage coarse --groups 2 --frames 2 → slot_coarse.json
  · slot_fine   → --stage fine --groups 5 --prior slot_coarse.json → slot_latest.json
                  && check_slot_reference（测量与比对是一个整体）
  · 新增 model:slot 阶段（run_pick_and_place 的 model 分支）：读 slot_latest 的 dataset，
    跑 model_environment --accept-workpiece-move，放在 slot_fine 之后；
  · 删掉 observe:tables（检查是 none，纯空跑 11.6 s）。

插槽位姿可用性（所有者问"最新位姿与历史基准差多大、可用吗"）：
  今天 19:18 实测 [1.2562, 0.1419, 0.4428] vs 11-24 基线 [1.2558, 0.1422, 0.4428]
    → 0.5 mm / 0.07°     插槽没动
  今天实测 vs 插入参考 [1.2194, 0.1389, 0.4444] → 37.0 mm / 0.88°
  11-24 基线 vs 插入参考                       → 36.5 mm / 0.84°
    → 恒定口径差（两次独立测量只差 0.5 mm）；插入仍用 slot_reference_verified.json
  结论：**插槽可用**。

配准环节确认：measure_object --preset slot 用夹具 CAD
（slot_fixture_collision.stl）三平面 RANSAC + ICP 精配准；工件用弹仓 STL + 扶手判据。
→ 手册第 63 条。
```

## 5. 20:40–21:05 收工：环境模型阶段验证通过，真机已关机

### 20:22 model:slot 阶段首次失败（我的两处 bug）

```text
① NameError: name 'pathlib' is not defined
   → run_pick_and_place.py 只 from pathlib import Path，我的新分支写了 pathlib.Path。补 import。
② 第二次仍失败：ok=False、detail 只有"模型重建｜"、**耗时 3.9 s**（真建模要几分钟）。
   查真因：slot_latest.json 里
     stamp   = 20261125_191808        ← 测量结果时间戳
     dataset = .../measure_20261125_191925  ← 实际采集目录（与 stamp 不同！）
   我之前按 stamp 补 metadata，补到了另一次采集的目录，被引用的那个目录仍然没有
   metadata.jsonl → model_environment 直接跳过全部 5 组。
   按 dataset 字段逐份重补后：rc=0、build gate PASS、5 帧、435 碰撞体素。
   → 手册第 64 条；agent_memory 增加"测量结果与数据集"一行。
```

### 本次真实收获（所有者点名）

```text
所有者："现在的一键回到停机位极其干净利落，很好用，棒。"
对应 11-24 的改动：goto_park_pose 走安全换位白名单（0.20 rad/s、单段执行、
时间戳地板 0.05 s），不再分三段、不再被 0.25 s/点的地板拉长。
→ 这是本轮"分段只用于接触段"的设计在真机上的正面验证。
```

### 今日真机验证汇总（11-25）

```text
✅ 一键回工件粗观测位（wrist_1 94° 单轴、单段 0.20 rad/s、8.21 s、到位误差 0.2 mm）
✅ 一键回停车位（所有者："极其干净利落"）
✅ 工件粗测（工件已换位置）：覆盖率 0.98、yaw −25.1°、重复性 0.66 mm/0.101°、40 s
✅ 选站 −29@20261120（差 3.3°、摆放 15.9 mm/3.33°）+ 换位到位无接触
✅ 工件精测：首次 REFUSE（3 组中 1 组离群）→ 原地重测 0.978 / 0.42 mm / 0.078°
✅ 插槽观测链：tables→slot_coarse→slot_fine 全 ok，插槽基线比对 PASS（0.5 mm / 0.07°）
✅ 示教器切本地/远程后：ensure_robot_ready --force + 2° 探针实测 +2.00° 命中
✅ 相机面板：定位真因为"两个 CameraView 各起一个订阅线程会两边都收不到帧"，
   改为单共享线程后两视图各 17 帧/15 s（所有者已确认出图）
✅ 环境模型：测量数据集补 metadata 后可直接建模，build gate PASS
⏸ 未验证（真机已关机）：pre_grasp / grasp / grip 三个接触段（工件已换位置，需重跑）
⏸ 未验证：model:slot 阶段在 run_task 里的完整串联（底层命令已 rc=0 验证）
```

### 下次开工清单

```text
1. 任务 ①（四项门 + control_health），若示教器动过则加 ensure_robot_ready --force + 2° 探针
2. bash scripts/gripper_link.sh --status（夹爪 init=5、position_mm=0 才算就绪）
3. 任务 ④ 面板，确认效率页相机出图（今天已修，重启后应正常）
4. python3 scripts/goto_park_pose.py            # 只规划，看单段 0.20 打印
5. python3 scripts/goto_workpiece_coarse.py --execute --allow-flip   # 一键回粗观测位
6. 按所有者确认的阶段清单跑效率模式；本轮建议直接到 grip，验证接触三段
7. 每阶段看 secs 字段，把固定开销的新数值补进手册第 54 条
```

## 6. 今日提交

| 提交 | 内容 |
|---|---|
| `acbeae2` | 一键回粗观测位任务内置 --allow-flip + 六关节表；末端实体口径（353 虚拟点）手册 56/57 |
| `a03cd08` | 下发前区间检查 dict() 崩溃修复（真机 rc=1 但机械臂未动）+ 收工日志 |
| `df20096` | 测量失败自动重测 + 面板话题去重 + 示教器切换规程（手册 58–60） |
| `2dd404f` | 面板相机真因：一个进程一个订阅线程（手册 61） |
| `39f2a71` | 测量数据集可用于环境建模（metadata + 单位换算，手册 62） |
| `本次`   | 插槽粗/精两级 + model:slot 阶段 + 删 observe:tables（手册 63/64） |
