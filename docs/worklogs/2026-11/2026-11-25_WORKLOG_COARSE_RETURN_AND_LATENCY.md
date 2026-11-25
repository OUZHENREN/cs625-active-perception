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

## 7. 21:50 插槽三套口径的追问（所有者）与插槽精观测降到 3 组

```text
所有者：slot_fine 做 3 组精配准（已改）；并问"你对比的基准位姿是师兄以前的示教位，
还是我十一月手动摆的那个？如果是后者，以前用 RVS 测量对比也差 30 多毫米，
是不是说明我们的配准一直有偏差？"

查证结果（四组数字全部溯源）：
  ① 师兄 RVS 标定 slot_origin（config/unload_real.yaml，model.json 里 slot 的来源）
        (1.2416, 0.1817, 0.4432)
  ② 我们的 CAD ICP 测量（今天 19:18）  (1.2562, 0.1419, 0.4428)
     11-24 同管线测量               (1.2558, 0.1422, 0.4428)   ← 自比 0.5 mm
  ③ 所有者 2026-11-04 手动验证参考    (1.2194, 0.1389, 0.4444)
  两两差：②vs① 42.0 mm｜②vs③ 37.0 mm｜①vs③ 48.2 mm

结论：不是"配准漂移"，而是**三个来源各自的原点定义不同**（RVS 标定 / CAD 模板原点 /
物理验证过的插入 TCP）。差值是常数（②vs③ 今天 37.0、11-24 36.5，只差 0.5 mm）。
所以：判断"插槽有没有动"用 ② 自比可靠；"插入往哪走"只能用 ③。
2026-11-08 / 11-09 的日志早就记过"当天 R1 差 33 mm，不能用"，是同一件事。

**新发现的风险（已记入手册第 65 条）**：
  效率模式插入用的是 config 常量 insert (1.1574, −0.0132, 0.0321)，
  与 11-04 验证过的参考 insert (1.1362, −0.0547, 0.0324) 差 **46.6 mm**。
  示教链用参考值、效率模式用旧常量 → 跑效率模式插入会偏 46 mm。
  必须在跑插入前统一并请所有者确认。

改动：slot_fine 的测量组数 5 → **3**（load/full_efficient 两处）。
→ 手册第 65 条。
```

## 8. 22:15–22:40 插入链统一到 11-04 参考（所有者指令）

```text
所有者："以 11-04 手动验证的插入参考当做我所有的示教位姿，不管效率模式还是示教模式。"

先回答了"三套位姿口径不同"的追问（手册第 65 条）：
  我们的测量同时输出两个点 —— CAD 模板原点(translation) 与 包围盒中心(centre_mm)，
  两者相差 320 mm；模板原点本身也不在角点、不在中心（距 bbox 中心 77/193/254 mm）。
  配对结果：
    我们的 CAD 原点 vs 参考 slot   → 37.0 mm，其中 **36.8 mm 全在 x**（Δy 仅 3.1）
    我们的 CAD 原点 vs 师兄 RVS    → 42.4 mm，主要是 **Δy 39.8**
  参考文件自述 "chain shifted 45.5 mm toward the robot in x"（11-04 日志 8+5+27.5+5 累计平移）
  → 30–40 mm 是三种口径（CAD 原点 / RVS 模型原点 / 经验调过的插入 TCP）叠加，不是配准漂移。

改动：
  · run_pick_and_place 新增 reference_insert_chain()：从
    runtime/slot_reference_verified.json 经 pose_math.insert_from_slot 推导
    pre_insert / insert(倾斜 −10°) / straighten(平放)，各沿工具 x 回退 10 mm；
    四个插入阶段全部改用；参考缺失则回落 config 并打印警告。
  · config 的 insert / pre_insert / insert_q / pre_insert_q 同步为推导值，
    并注明是"推导结果、非手调"，供 advance_axis 与慢速带几何使用。
  · 新增 test/test_insert_chain_authority.py（3 条）。

实测偏差（改前 → 改后）：
  pre_insert  110.3 mm（Δz 101.5，缺 +100 mm 抬高）→ 0
  insert       43.2 mm（Δy −41.6）              → 0
  straighten   43.1 mm                          → 0
  另外发现 config 旧 insert_q 是**平放**姿态（w≈−0.004），
  而进槽必须 −10° 倾斜（2026-11-04：无倾斜下降撞槽口）→ 现已改为倾斜姿态。
→ 手册第 66 条。真机未验证（已关机），下次跑插入前先用 --plan-only 复核。

## 9. 23:00 纠正：旧插入常量没有"偏"，是两个插槽权威（所有者追问）

```text
所有者追问两点：
  ① "你如何确定位姿存在偏差？师兄在 RVS 里是角点定位，你现在是拿 CAD 中心点比？"
  ② "效率模式的实际偏差你确定原因了吗？我目前没那么理解你说的这些偏差"

先承认：我上一轮把"两条链差 43/110 mm"说成"效率模式偏了"，**表述错误**。
用 git 取出改动前的旧值，与"从师兄 RVS slot_origin 推导"的结果逐位比较：

  旧 config pre_insert [1.0996, -0.0125, 0.3773]
  推导（RVS slot, -10°, 不加抬高, 不回退） [1.0996, -0.0125, 0.3773]   Δ = 0.0 mm
  旧 config insert     [1.1574, -0.0132, 0.0321]
  推导（同上）                        [1.1574, -0.0132, 0.0321]   Δ = 0.0 mm

→ 旧常量是**正确推导**出来的（由师兄 RVS 插槽位姿），本身没有任何错误。
真因是**两个模式用了不同的插槽权威**：
  示教链 = 11-04 手动验证参考（+100 mm 抬高、工具 x 回退 10 mm）
  效率模式 = 由师兄 RVS slot_origin 推导
两个权威本身差 48 mm（Δy 42.8、Δx 22.2）→ 两条链相差 43–110 mm。
不是"配准偏差"，也不是"常量写错"，是"同一件事有两个真相"。

关于"用角点还是中心"（所有者指出师兄是角点定位）：
  · 我们的测量输出两个字段：`translation` = **CAD 模板原点**（这是位姿，用于推导）；
    `centre_mm` = **包围盒中心**（只用于碰撞体/范围判断，owner 说得对）。
  · 但 CAD 模板原点本身既不在角点也不在中心（距 bbox 中心 77/193/254 mm），
    所以它**不能直接和师兄的角点位姿相减**——之前 37/42/48 mm 的对比就是跨口径相减。
  · 结论：跨来源比较绝对坐标没有意义；判断"动没动"只能用同一管线自比（0.5 mm）。

所有者问"要不要像抓取那样重新标定插入位姿"：
  建议要，而且这是唯一能彻底终结口径争论的办法 → 手册第 67 条给出方案：
  手动摆出能干净插入的位姿 → 记录 TCP + 六关节 + 同时刻插槽测量 →
  算 insert_in_slot_frame = inv(T_slot) @ T_tcp（与 grasp_in_workpiece_frame 同构）→
  之后插入目标 = T_slot(实测) @ insert_in_slot_frame，两模式共用；
  验收：往返 0.000 mm、IK 同分支、先 --plan-only。

文档纠正：手册第 66 条已改写（保留"我最初判断错了"的留档），新增第 67 条标定方案。
```

## 10. 23:15 位姿计算的"标准"（所有者要求，附 66 mm 决定性实验）

```text
所有者给出客观事实并追问：
  · RVS 位姿曾支撑插入（9 月虚拟机 + 师兄程序、10 月底复现）；
   但示教模式改用 RVS 最新一次后在 **X 方向大幅偏置**、插不进去（有日志）；
  · 11 月手动摆的位姿是最近最准的，但已过半个月不再可靠；
  · "我们在计算位姿时应该也有一套标准，不能像 RVS 一样开盲盒。"

决定性实验（同一片点云、同一 CAD 模板，只换配准路径）：
  先验引导 + ICP（现在用的）→ (1.2562, 0.1419, 0.4428)
  三平面角点（师兄的做法）  → (1.2308, 0.1991, 0.4656)    相差 **66 mm**
  三条组角点路径重复性：x 4 mm / y 11 mm / z **19 mm**

真因：插槽夹具以大平面为主，**ICP 在面内无约束**（平面沿自身平移不改变拟合度），
可以沿面滑移几十毫米 → 表现为 x 方向漂。这正是历史上 RVS 位姿 X 偏置的同类问题。
工件的三平面 + 扶手特征强约束，所以工件测量稳定（0.42 mm / 0.078°）。

结论：插槽的"绝对位姿"不是良定义的量，争 37/42/48 mm 无意义；
必须 ① 角点锚定测量（三平面交，或角点初值 + 禁止面内滑移的受约束 ICP）
     ② 手动标定一次 insert_in_slot_frame（与 grasp_in_workpiece_frame 同构）
     ③ 插入目标 = T_base_slot(实测) @ insert_in_slot_frame

文档：docs/reference/task_pose_derivation.md 新增 §6「我们自己的位姿标准」，
把抓取链（已成功，作为范例）与插槽缺口、以及共同验收线写清楚；手册第 68 条记实验。
```

## 11. 23:30 为明天准备插入位姿标定工具（所有者方案）

```text
所有者问："我明天直接让你知道工件插到底之后位于哪个位姿，然后让夹爪带着它
手动取出和旋转，让你记住这些位姿行不行？"
答复：可行，而且比推导可靠 —— 插槽是面内弱约束（第 68 条），手动摆出的位姿是真的能插进去的。

新增 scripts/record_insert_poses.py：
  · --name insert|straighten|pre_insert|withdraw 记录（TCP + 六关节 + 时间 + 备注）；
  · **静止确认**：连续 1.2 s 关节速度 ≈0 且位置稳定在 0.2° 内才允许记录（手拖松手会漂）；
  · 安全：任一行程超出绝对区间（肩 −130/10、肘 −160/−60、腕 ±150）拒录；
  · --show 查看；--derive 推导 insert_in_slot_frame（往返校验应为 0）；--promote 写入 config。
  · 只读状态、只写本地文件，不发任何运动指令。

离线已验：推导数学往返 0.0 mm；新增 test/test_record_insert_poses.py（3 条）。

跨口径交叉核对（相对变换的真正价值）：
  师兄常数 T_s_tcp = diag(-1,1,-1) + (−82, −193, −412.5) mm
  由 11-04 参考换成的相对变换 ≈ (−112, −198.4, −411.8) mm
  → x 差约 30 mm、y 5 mm、z 0.7 mm —— 这是两条链在相对意义下的真实差别。
→ 手册第 69 条。

## 12. 23:40 六维力传感器接入（今晚完成，离线可验）

```text
所有者给了硬件信息：坤维 KWR75、RS485、COM3、延时已改 1 ms，并给出三本手册。

读手册（KWR75用户手册V21.6.75.3.3LB，非定制版长数据格式 28 字节）得到协议：
  460800 8N1；命令 0x48AA0D0A（1 kHz 流）/0x49AA0D0A（单次，需发两次）/0x43AA0D0A（停）
  28 字节帧 = [0x48|0x49] 0xAA + 6×float32(小端) + 0x0D0A
  分量 Fx Fy Fz Mx My Mz，**力单位 kgf、力矩 kgf·m**（不是 N），故发布前乘 9.80665。

新增：
  config/ft_sensor/kwr75.yaml      串口/帧格式/单位换算/命令帧/话题 + 与数采软件互斥说明
  scripts/ft_sensor_node.py        发布 /ft_sensor/wrench（N/N·m）+ /ft_sensor/diagnostics，
                                   流/轮询两种模式，抗丢字节重同步，--self-test 用手册样例帧自检
  experiment_recorder              候选话题加入 /ft_sensor/wrench
  test/test_ft_sensor_protocol.py  手册样例逐位校验、kg→N、坏帧拒收、乱码重同步、半帧拼接
  手册第 70 条                     完整接线步骤（usbipd / dialout / latency_timer=1 / 关 SenserLinker）

离线验证：--self-test 通过（Fx −9.77873 vs 手册 −9.7787，六分量全对；容错切帧 2/2）。
真机未验（传感器尚未安装到法兰，法兰未加工）。
→ 407 测试通过；提交 1acc5e1。

## 13. 23:50 数据地图（所有者："我对数据在哪完全没有掌控感"）

```text
所有者的核心诉求：不只是"面板看不到数据"，而是**不知道自己能拿到什么数据、放在哪里**。
力矩数据（驱动器电流反推）不知道存哪、相机点云和各类指标也不知道。

查清的事实（写进 docs/reference/data_map.md）：
  ① 关节力矩**一直在采**：归档 timeseries.csv 的 joint_effort_1..6（20 Hz），
     没有单独 effort.csv —— 这就是之前"找不到力矩"的原因。
  ② 六维力的列**早已预留**：fx fy fz tx ty tz，当前 100% 为空；
     接上 KWR75 后由 experiment_recorder 自动填充，表结构不用改。
  ③ 点云原始数据在 runtime/vision_dataset/measure_<stamp>/g*/（cloud.npy / depth.npy /
     color.png / meta.json / metadata.jsonl），可以重新跑算法、重新出图。
  ④ 面板"视觉实验数据"卡片**只读两个文件**（workpiece_latest.json / slot_latest.json），
     不读 timeseries、不读点云、不读环境模型 —— 所以"面板看不到数据"是设计缺口，
     不是数据没采到。

新增：
  docs/reference/data_map.md     完整数据清单：位置/格式/频率/怎么读 + 常见误解 + 出图入口
  scripts/data_report.py         一条命令列出全部数据产品（存在？多新？多大？关键指标？）
                                 --ros 额外检查 /joint_states、TCP 位姿、/ft_sensor/wrench
  docs/README.md                 导航表第 0 项登记 data_map（开工先看）
  docs/agent_memory.md           新增"数据在哪"一行，指向 data_map + data_report

实测输出（今晚）：测量结果 4.5 小时前 OK；归档实验 3 个，力矩列全部有值；
环境模型 2.7 小时前（已超 2 小时门限，提示会被环境门拒）；插入标定尚未生成。
```

## 14. 00:10（已过零点，按 2026-11-25 计）待办体系 + 日期归属规则

```text
所有者两条新要求：
  ① 需要一个**独立于工作日志**的周待办 md，每周更新、上周归档；
     理由是"比 Obsidian 那边幻觉更低，因为那边不是真实的工作环境"。
  ② 日期归属：从前一天晚上连续做到凌晨的工作，**全部算前一天**。

落地：
  docs/TODO.md                 当前周唯一待办入口（七个小节：真机窗口 / 六维力 / 数据通道 /
                               视觉计划更新 / 对比实验升级 / 论文出图 / 本周已完成）
  docs/todos/                  归档目录（每周一份 <年>-W<周>_<起>_<止>.md）
  scripts/todo_week.py         --show 统计；--rollover 归档并重开新周（只留未完成/阻塞，
                               归档只新增不覆盖，测试覆盖这两个不变量）
  AGENTS.md 第 13 条           日期归属规则（00:00–06:00 新建日志默认沿用前一天）
  AGENTS.md 第 14 条           待办体系规则（日志记"发生了什么"、TODO 记"要做什么"）
  docs/agent_memory.md         两行硬规则指向上面两条
  docs/README.md               导航第 0a 项登记 TODO.md

把"三个漏项"正式登记进 TODO（所有者要求记住、暂不推进）：
  漏项一 数据通道待补 5 项；漏项二 视觉实验计划需更新 4 项；漏项三 对比实验升级 M1–M5。
另：docs/literature/2026-11-26_EXPERIMENT_DESIGN_FROM_LITERATURE.md ——
  从 Zotero（G:\我的云端硬盘\Zotero，3476 条 / 199 篇学位论文）反推的实验设计，
  把"三臂视觉对比"升级为误差链建模/杠杆臂敏感度/R&R/接触状态/可靠度五模块。
  文件名按"开始日期"也无法回避跨零点：内容产生于 11-25 夜间，故命名为 11-26 与
  TODO 里的引用保持一致，此处注明以免将来混淆。

测试：411 passed（新增 test/test_todo_week.py 4 条）。

## 15. 00:20–00:50（按规则记 11-25）大论文纲要 v2 + 遗漏任务盘点

```text
所有者反馈三件事：
  ① 大论文总纲要需要按现状更新（10-30 版"有很多地方有所出路了"）；
  ② 待办文件"在仓库里看不到"；
  ③ 要求把"提出需求就立刻登记待办"变成固定动作。

②的真因：`docs/TODO.md` 与 `scripts/todo_week.py` **确实存在且已提交**
   （git ls-files 可查），但 `docs/todos/` 是**空目录**，git 不跟踪空目录 →
   在远端/仓库视图里看不到归档目录；另外 TODO 没有镜像到 Obsidian，而所有者习惯在
   Obsidian 里找文档。处置：补 `docs/todos/README.md` 让目录进入 git；
   并确认 **TODO 不同步 Obsidian**（所有者决定：那边只做工作日志的每日记录）。

①的产出：`docs/plans/2026-11-26_PLAN_THESIS_OUTLINE_V2.md`
   · 标题/定位：保留 10-30 版的三模块框架，但**写作顺序按"哪里有数据"重排**
     （第 2 章视觉与选站、第 3 章误差链先成稿；第 4 章力控后置）
   · 列出 10-30 版已脱节的 8 处（NBV→站位表、GICP→三平面+CAD ICP、C++→Python、
     MODBUS→ros2_control、跳过DL→主测量不依赖DL、六维力已到货但未装机、偏振待确认、
     模块占比与实际进度不符）
   · 明确"横向偏差 = 深度 × 姿态误差"是几何算术并给出实际数值
     （371 mm × tan(0.078°) = 0.51 mm；1° = 6.5 mm），同时标注为**待 M2 实测验证**，
     不当作已测结论 —— 这是对"杠杆是不是幻觉"质疑的正面回答

③的落地：`AGENTS.md` 第 14 条补充"需求即时登记"；`docs/TODO.md` 新增登记规则。

文档命名口径（避免以后混）：计划/参考类按**创建时刻**命名；工作日志按**工作开始日**归属。

## 16. 00:45 大论文纲要 v3（所有者指 v2 单薄）

```text
所有者三点反馈：①②已确认；③交叉偏振**已停用**（章节须按"已停用+原因"写）；
并质疑 v2 章节单薄、"你真的通读我的文献库了吗"。

先把库真正通读一遍（不再只看 Top-10）：
  全库 3574 条：journalArticle 594 / thesis 199 / conferencePaper 163 / patent 16
  学位论文来源 Top：哈工大 46、重大 17、浙大 10、哈工程 7、华科 7、中科大 6
  所有者批注归属：**[2078] An overview of hand-eye calibration（24 条）**、
                  [2447] 6D pose 抓取（5 条）、[683] 大型光机模块柔性装校（1 条）
→ 关键发现：他自己读得最深的是**手眼标定**，而 v2 把它压成一句话；平台章也已完成 80% 却被压缩。

v3（`docs/plans/2026-11-26_PLAN_THESIS_OUTLINE_V3.md`）：
  8 章：绪论 / 平台 / **手眼标定与测量精度** / 观测位选择 / **误差链建模** / 六维力柔顺 / 集成与仿真 / 总结
  前五章合计 65% 工作量，新增真机实验只有约 4 h（3.4 R&R 1.2 h + E1 2 h + M2 1 h）
  力控章给出**降级预案**（法兰长期不到位则降为"系统设计与驱动实现"）
  交叉偏振按"已停用"写入平台章

待所有者确认三项：标定方法对比做到几种解法、降级预案触发条件（建议 >6 周）、ANSYS 是否补瞬态。

## 17. 01:00 大论文纲要 v4（所有者：不要照现状写、NBV 还能做）

```text
所有者两点关键纠正：
  ① "一板一眼用我当前已经做的来进行会不会不太好" → 现状应是**基线/对照组**，论文主体是研究增量；
  ② "NBV 也还可以做吧，比如在选站基础上进行适当的腕部重构，以达到更好的观测效果"
     → 我在 v2/v3 里把 NBV 判成"真机不可行、只能查表"，这个判断过窄；应升级为
       **站位先验 + 受约束局部视角优化**。

v4（`docs/plans/2026-11-26_PLAN_THESIS_OUTLINE_V4.md`）：
  · 题目建议 C：面向遮挡目标定位的可达性观测规划与视触融合精密装配方法研究
    （B 作方法主线、A 作应用验证，对应所有者已确认的"保留 A 题目 + 倒序写作"的折中）
  · 三个研究问题 Q1 可达性判定 / Q2 受约束局部观测优化 / Q3 不确定度驱动决策
  · 8 章每章三段式：基线（现状）→ 本文方法（增量）→ 验证
  · NBV 可行性分析：候选={roll±10/20°, pitch±5/10°, 侧移±20/40 mm}；
    评分用**离线光线投射**（CAD + 环境点云预测可见性），真机只执行少数候选；
    物理约束已列出（相机在 TCP 后 0.3 m、手腕电缆限位 ±45° 与规划器 ±150° 冲突待实测、J4 抱闸）
  · 迭代路线 I1（现状）→ I2 局部优化 → I3 不确定度闭环 → I4 力控 → I5 集成
  · 增量工作量单列：前五章新增真机约 6.5 h（3 章 1.5 h + 4 章 4 h + 5 章 1 h）

TODO 新增："局部视角优化（I2）"研究增量任务（离线评分器先行）+ 三项待确认。

## 18. 01:05 大论文纲要：先读同类硕博论文再定章法（V2 现行，旧三版归档）

```text
所有者三点纠正：
  ① "分个 8 章也挺扯的，多看几篇文献尤其是硕士和博士论文再给综合答案"
  ② 题目暂用 A；现用题目仍是《基于主动视觉的精密光学模块机器人可靠抓取方法研究》，之后要改
  ③ 手腕限位 **±150°**（不是笔记里的 ±45°）、**允许侧移**；旧版本 V2/V4 归档，
     之后版本重新命名 **V2**

做法（不再自己拍脑袋定章法）：
  从 Zotero 库（storage 里 198 篇学位论文 PDF 全在）按关键词挑 6 篇，用 pypdf 的
  **书签（outline）**抽一级标题：
    柔顺拆装机械臂(硕士) 5 章｜轴孔装配柔顺控制(硕士) 6 章｜大口径反射镜抓取工装(硕士) 4 章
    侧装装校机构运动学与误差(硕士) 6 章｜主动柔顺侧送装校机构(硕士) 6 章｜深度强化学习抓取(硕士) 5 章
  → **结论：5–6 章是这条线的常态**，模板为"绪论 → 平台/总体方案 → 方法一 → 方法二 → [实验] → 总结"，
    而且多数是**每个方法章自带实验小节**。

产出：
  · 归档：`docs/plans/archive/2026-11-26_thesis_outline_drafts/`（原 V2/V3/V4 + 归档说明）
  · 现行：`docs/plans/2026-11-26_THESIS_OUTLINE_V2.md` —— **6 章**
      1 绪论
      2 实验平台与总体方案
      3 遮挡目标位姿测量与可达性观测规划（标定 + 粗精测量 + 可达性 + **局部视角优化**）
      4 误差链建模与装配可行性
      5 六维力感知与柔顺装配控制（法兰阻塞 + 降级预案）
      6 结论与展望
    每章：内容 / 研究问题 / 增量 / 产出；前四章新增真机约 6 h，其余离线可做。
  · 依据写进文档顶部（6 篇样本的章数与结构表），不再是无依据的章数。

按所有者确认更新口径：题目 A、腕限 ±150°、允许侧移、章数 6。
