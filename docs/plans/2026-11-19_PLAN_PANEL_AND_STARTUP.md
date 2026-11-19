# PLAN：面板与开工/收工修复 + 恢复能力补齐（2026-11-19）

> 状态：**计划，未执行**。所有者要求先出计划再动手。
> 输入：所有者 2026-11-19 的 5 条代码需求 + 2 个问题；三项只读审计（面板布局、
> 启停与恢复能力、观测失败阈值）正在并行进行，返回后补全数字。
> 原则：一轮只解决一个明确问题；先修"会让人反复踩坑"的（崩溃、相机、日志），再改观感。

---

## 0. 优先级总览

| 级别 | 任务 | 为什么先做 | 预计改动 |
|---|---|---|---|
| **P0** | B1 面板每 100ms 崩溃（`KeyError: '界面'`） | 终端刷屏、日志被淹没，无法判断其他问题 | `cs625_panel.py` 1 处 |
| **P0** | A 相机接入开工①/收工⑧ | 昨天"没收到相机信号"浪费半场；根因是开工脚本不管相机 | `start_real_stack.sh`、`stop_all_drivers.sh` |
| **P0** | B2 左栏加宽 3 倍 + 日志自动换行 | 所有者明确要求：不再靠横向拖动看日志 | `cs625_panel.py` 布局 |
| **P1** | B3 相机页错乱、B4 视觉流程/实验数据、B5 3D 状态/运行记录 | 观感与信息密度；依赖 B2 的新宽度 | `cs625_panel.py` 布局 |
| **P1** | C 效率模式补"失败自动重挂 + 续跑" | 昨天我卡住的直接原因之一（效率入口没有恢复能力） | `run_task.py`、面板效率页 |
| **P2** | D 观测失败的阈值/位姿策略 | 需要先定方向（放宽阈值 vs 补偿位姿） | 待定 |

---

## A. 开工① / 收工⑧ 接入相机

### A.1 现状（已核实）

```text
start_real_stack.sh     只起：驱动栈 + MoveIt + 四项门；**不启动相机**
stop_all_drivers.sh     已会停：percipio_camera launch（pgrep）+ camera_view.py
camera_mode.py          相机的唯一启动入口：python3 scripts/camera_mode.py modeling
                        （自己 source ~/cs625_camport_ws，写 runtime/camera.pid，带 fps 自检）
```

后果：昨天开工后 `/camera/color/image_raw` 有 3 个订阅者、**0 个发布者**，
观测检查报"没收到深度或内参"，而界面上看不出"相机没起"。

### A.2 改法

**`scripts/start_real_stack.sh`**：在四项门之后追加"相机"一段：

```text
1. 若 runtime/camera.pid 存在且进程活着 → 复用，打印 "相机已在运行（pid/分辨率/fps）"
2. 否则执行 python3 scripts/camera_mode.py modeling（它会 source 相机工作区并自检 fps）
3. 验证：/camera/color/image_raw 的 Publisher count ≥ 1，且帧率在 0.5–1.2 fps
   （这台相机开深度就是 0.79–1.0 fps，是正常值，不是卡住）
4. 失败时打印三条可照做的排查（按顺序）：
   · RVS 正占用相机（两者互斥）→ 关掉 RVS 再试
   · 相机没上电/网线松 → 检查供电与网口
   · ~/cs625_camport_ws 没 build → colcon build 后重试
5. 结尾打印一行汇总：机械臂 ✓ / MoveIt ✓ / 相机 ✓（或 ✗ 及原因）
```

**`scripts/stop_all_drivers.sh`**：补两处
- 停相机后清理 `runtime/camera.pid`（现在只 pgrep，可能留下死 pid 文件）
- 若相机停不掉，打印"仍有 percipio_camera 进程（PID xxx），请手动确认"

### A.3 验收

```bash
bash scripts/start_real_stack.sh     # 一条命令后：机械臂+MoveIt+相机都就绪，且打印汇总
ros2 topic info /camera/color/image_raw   # Publisher count: 1
bash scripts/stop_all_drivers.sh     # 三者全停，无残留进程、无残留 pid 文件
```

---

## B. 4K 面板效率模式页

### B.1 崩溃（P0，1 行级修复）

**根因**（已核实）：`key_labels` 只注册了 10 个键
（`T1 fitness / T3 fitness / 最近采集 / 上次阶段 / 反控 / 当前阶段 / 相机 / RViz / 分支 / 提交`），
而 `_refresh_keys()` 里写了 `self.key_labels["界面"]` → 每 100 ms 抛 `KeyError: '界面'`。

**改法（二选一，建议 a）**：
- a. 把 `"界面"` 加入注册元组，并把 `key_box.setFixedHeight(268)` 提到约 296（多一行）；
- b. 删掉该行（窗口尺寸信息价值低），并在 `_refresh_keys` 里对所有键用 `.get()` 兜底。

无论选哪个，**都要给 `_refresh_keys` 的写入加保护**，避免下次再加键时又崩。

### B.2 左栏加宽 + 日志换行（P0）

```text
现状：left.setFixedWidth(400)；日志用 LogView(QPlainTextEdit)，长行需横向拖动
目标：左栏约 1200 px（4K 下 3 倍）；日志自动换行
改法：
  · left.setFixedWidth(400) → 用 QSplitter(Horizontal) 包住左/右栏，
    初始 setSizes([1200, 2640])，并设 setStretchFactor(0,0)/(1,1)；
    或最小改动：setFixedWidth(1200) + setMinimumWidth(900)
  · LogView 里 setLineWrapMode(QPlainTextEdit.WidgetWidth)（自动换行）
  · 阶段/速度/参数三个 GroupBox 已在 left_layout 内，宽度自动跟随
  · 日志保持 stretch=1（占左栏剩余高度）
```

### B.3 相机页错乱（P1）

```text
现象：有图像时嵌入不好/错乱
原因（待审计确认）：QLabel 直接 setPixmap 且 setScaledContents(True) 会拉伸变形；
      或容器宽度变化时未按纵横比缩放
改法：保持 4:3（1280×960），按控件尺寸做 QPixmap.scaled(size, KeepAspectRatio,
      SmoothTransformation)；或自写 AspectRatioWidget 固定比例；
      容器给 setSizePolicy(Expanding, Preferred) 并把最小宽度设为 640
```

### B.4 视觉流程表 + 视觉实验数据（P1）

```text
问题：字体被行高遮挡；左右/上下留白过稀；实验数据拥挤、左上角两个标题错乱
改法：
  · 行高：按字体 metrics 设 verticalHeader().setDefaultSectionSize(30)，
    并对多行文本用 setWordWrap(True) + setRowHeight(计算值)
  · 列宽：setSectionResizeMode(ResizeToContents)（短列）+ Stretch（说明列）
  · 布局：把"视觉实验数据"移到"视觉流程"下方；两者用 QSplitter(Vertical)，
    实验数据区改为可拖动比例（默认 60:40）
  · 实验数据左上角两个标题：合并为一个标题栏（或去掉重复标题），只保留一处
```

### B.5 3D 状态 / 运行记录（P1）

```text
问题：排布不均、行距不足导致字体被遮挡、信息密度低
改法：与 B.4 同一套行高/列宽规则；三块（实验数据、3D 状态、运行记录）横向并排，
      stretch 1:1:1；每块最小宽度 380；表格不设固定行高，统一 30 px
```

### B.6 目标布局比例表（4K 3840×2160）

```text
根：QSplitter(H)  左 1200 : 右 2640
左：阶段选择(固定高) → 速度覆盖(固定高) → 运行参数(固定高) → 按钮(34) → 日志(拉伸)
右：QSplitter(V) 上 60% / 下 40%
    上：相机 1280×960（4:3） + 视觉流程表(拉伸)
    下：视觉实验数据 : 3D 状态 : 运行记录 = 1 : 1 : 1
```

---

## C. 效率模式补"失败自动重挂 + 续跑"

### C.1 现状（已核实的能力矩阵）

| 入口 | 失败自动重启驱动栈 | 断点续跑 | 重挂外部控制脚本 |
|---|---|---|---|
| `resume_teaching.sh`（示教/安全） | ✅ `recover_driver_stack.sh --timeout 120` | ✅ `teaching_sequence.py --resume`（读 `runtime/teaching_run_state.json`） | ✅ 脚本内 |
| 面板安全页 | ✅（"续跑"按钮） | ✅ | ✅ |
| `run_task.py`（效率/六模式） | ❌ 只调 `ensure_robot_ready.py` 后直接跑 | ❌ 失败即退出 | ⚠️ 昨晚才给 `ensure_robot_ready` 加 `--force` |

**结论：能力没被删，只是效率入口从来没接这套东西**（git 验证：整理仓库时未删除任何恢复脚本）。

### C.2 改法

```text
1. run_task.py 失败路径：
   a. 先 python3 scripts/ensure_robot_ready.py --force（重挂外部控制脚本）
   b. 仍失败 → scripts/recover_driver_stack.sh --timeout 120
   c. 新增 --resume：读 runtime/phase_log_<step>.json，取最后成功阶段，
      从下一阶段继续（与 teaching_sequence --resume 同思路）
2. 面板效率页：加"续跑"按钮（与安全页一致），失败后一键续跑
3. 把"中止后必须 --force 重挂"从文档规矩变成代码路径（自动执行）
```

### C.3 验收

```text
· 故意让某一阶段失败（例如 --until 一个会失败的测量阶段）
· 再跑 run_task.py --resume → 从失败阶段的下一阶段继续，而不是从头再来
```

---

## D. 昨天为什么拍不了（回答问题 4）

### D.1 结论：**不是阈值太严，是"只看到 2 个面"**

```text
事实：3 组测量彼此一致（0.637/0.688/0.121，重复性很好），但每组只识别到 2 个平面，
      扶手分 0.17 → 质量门判"可信组 0/3"。
原因：工件相对示教位置平移了约 (−24, +112, −10) mm；站位是按示教位置教的，
      相机看过去第三个面已经出视野。
```

**关键判断：不能靠放宽"3 个平面"来解决**。两个平面无法约束绕法向的旋转（偏航），
放宽会得到"看起来成功但偏航错了"的位姿——这正是之前反复踩的坑（对称翻转解）。

### D.2 三个方向与建议

| 方向 | 效果 | 代价/风险 | 建议 |
|---|---|---|---|
| ① 放宽阈值（平面数/覆盖率/扶手分） | 能通过质量门 | **偏航不可观测 → 错误位姿**，下游抓取/插入会偏 | ❌ 不推荐 |
| ② 按工件实际位置补偿末端位姿 | 视野跟回工件 | 昨天实测需 47.6° 腕部变化，执行中止（大改构型） | 🟡 作为研究项，不阻塞主链 |
| ③ 摆放约束 + 增加站位 | 基础链路立刻可用 | 需要人工把工件摆回示教位置附近；或再教几个站位 | ✅ **先做** |

**建议的组合**：
```text
短期（本周）：测量前要求工件在示教位置 ±X mm 内（X 待审计给出，初步估计 30–50 mm）；
             超出时流程直接提示"请把工件摆回示教位置附近"，而不是测到 2 个面才失败。
中期：按"位置分档"增加站位（同一偏转角、不同工件位置各教一个），
      或让选站同时考虑平移（需要解决大改构型问题）。
```

**待补数字**：X（工件平移多少仍能看到 3 个面）——等 `capture-failure-analysis` 审计给出，
依据是相机视场角、站位到工件距离、工件 bbox。

---

## E. 回答问题 3：为什么感觉"变笨了"

### E.1 先给结论

```text
· 恢复/续跑功能**没有被删除**（git 记录：整理仓库时未删任何 recover/resume 脚本）。
· 但这套能力只接在"示教/安全模式"上；我昨晚用的是"效率模式"入口（run_task.py），
  它从来没有自动重启 + 续跑。
· 我自己的失误有三条，属于"没按已有规矩做"，不是"代码没了"：
  ① 驱动挂死后没有按手册 #28 立刻重启驱动栈，而是先反复读话题、反复超时；
  ② 轨迹中止后没有重挂外部控制脚本就继续发指令（当时还没有 --force，昨晚才补）；
  ③ 大量用 `ros2 topic echo` 子进程读状态，DDS 发现慢时反复超时，浪费了时间。
```

### E.2 本轮要做的机制性改进（防止再犯）

```text
· 效率入口接上恢复链（见 C）——这是"能力退化感"的根因
· 所有状态读取改用 rclpy + 重试（不再用 echo 子进程）；本轮已把临时工具验证可行
· 手册 #31 + 开工清单第 9 条已固化"中止后 --force + 2° 探针"
```

---

## F. 执行计划（分两批，每批独立可验收）

### 第 1 批（P0，当天可完成）

```text
1. B1 修 KeyError（1 处 + 兜底）
2. B2 左栏 1200 + 日志自动换行
3. A 相机接入开工/收工 + 一条命令自检
验收：面板不再刷错误；日志不用拖动；start_real_stack 后相机有发布者
```

### 第 2 批（P1）

```text
4. B3/B4/B5 布局重构（相机比例、表格行高列宽、实验数据下移、三表并排）
5. C 效率入口补恢复 + 续跑（run_task --resume + 面板按钮）
验收：4K 下所有表格字体不被遮挡；失败后能一键续跑
```

### 待定（P2，等审计数字）

```text
6. D 的 X 值确定 + 摆放约束提示 + 站位扩充方案
```

### 分工（按写作用域互不重叠）

```text
· 面板改动（cs625_panel.py）→ 一个子智能体，我复核
· 启停脚本（start_real_stack.sh / stop_all_drivers.sh）→ 另一个子智能体
· 恢复/续跑（run_task.py + 面板按钮）→ 我做（跨文件，需要判断）
· D 的策略 → 等审计，再单独一轮
```

---

## G. 风险与回滚

```text
· 面板布局改动大：保留原布局代码注释不删；改完先在窗口化模式跑一遍再上 4K
· start_real_stack 加相机：相机启动约 10–20 s，会让开工变慢；用"已在运行则复用"规避
· run_task 加自动恢复：**不能自动重试有风险的运动**——恢复只做"重挂 + 续跑到下一阶段"，
  不做"重放失败的那一步"
· 所有改动走 main 分支、提交前跑 scripts/pre_commit_check.sh
```

---

## H. 审计补充与计划修正（2026-11-19，startup-recovery-audit 返回）

> 这一节**修正**前面的 A.2、C.2，并补充几项之前没发现的 P0 缺口。执行时以本节为准。

### H.1 开工脚本本身在"假装成功"（新增 P0）

```text
实测：驱动栈在跑，但 /camera/color/image_raw 发布者=0、depth 话题是 Unknown；
      start_real_stack.sh --check-only 仍返回 0，只说"控制连接正常"，完全不提相机。

三个掩盖失败的缺陷（必须一并修，否则加了相机也照样"假成功"）：
  ① 第 100/101 行 control_health.py --activate 与 preflight_motion.py 的退出码**没检查**；
     最后一行 `gripper_link.sh --ensure || true` 让脚本**恒返回 0**。
     ⇒ 控制器没激活/四项门失败时，开工仍显示"完成"。
  ② "已在运行"判据 sockets_live()（第 35 行）只 grep `'sockets': True`，
     不看 controller / joint_states ⇒ 驱动半死（11-18 那种）会被判健康并跳过重启。
     ⇒ 改用 control_health.py 的整体退出码。
  ③ 顺序应为 activate → ensure_robot_ready → preflight，且每步 `|| exit`（现在是 activate→preflight→ensure）。
```

### H.2 相机不能无条件启动（修正 A.2）

```text
· RVS 与 ROS 相机互斥：RVS_AI_Center 在跑时必须**跳过**并大声提示，
  否则示教/RVS 流程当天就坏。
· 建议参数：--camera auto|on|off，auto = 检测到 RVS 就跳过。
· 相机失败必须 exit≠0，并打印三条可照做命令：
  关 RVS → camera_mode.py modeling / 查网线供电 / build ~/cs625_camport_ws。
· camera_mode.py status **不能**当快速健康检查：相机没起时实测耗时 37 s，
  depth 缺失仍返回 0，还会打印旧 camera.log 的 driver_fps。
  ⇒ 新增 `health` 子命令（rclpy 图 API：pid + color/depth/info 发布者数 +
    读一帧 CameraInfo 确认 1280×960，rc 0/1/2）；`start` 用 `health || modeling` 实现幂等，
    否则 modeling 每次都会重启相机。
```

### H.3 收工与任务包装（补充）

```text
· stop_all_drivers.sh：不读 runtime/camera.pid、不清 pid；多个 camera launch 时只 head -1；
  LEFT_LINK 里硬编码了 IP。→ 全部改成按 pid 文件 + 进程组处理。
· .vscode/tasks.json 第 67 行 `; echo 完成` 会在 stop 返回 1 时照样打印"已全部关闭"。
  → 改成 `&& echo 完成 || echo 失败`。
```

### H.4 相机"卡在 fast 档"的恢复盲点（新增 P0，很可能就是昨天反复报没深度的原因之一）

```text
run_pick_and_place.py 第 2728 行 grasp 阶段会把相机切到 fast；
恢复 modeling 只写在 atexit（仅正常退出时执行）。
进程被 SIGKILL/SIGTERM 杀掉后，相机留在 fast 档（无/低深度），
下一次观测就报"没收到深度"——会被误判成"相机没起"。
⇒ 修法：开工时用 camera_mode.py health 检测**当前档位**，不是 modeling 就切回；
   run_task 失败路径（finally）里也恢复 modeling。
```

### H.5 续跑语义修正（修正 C.2，重要）

```text
① "从最后成功阶段的下一阶段继续"是错的：phase_log 最后一条通常就是失败阶段且 ok=false。
   ⇒ 应从**失败阶段本身**重试，并且按 **index** 而不是名字
     （load_efficient 里 observe:slot_coarse 出现两次，按名字会错位）。
   ⇒ 重试到 grasp 之后必须先确认夹爪确实夹着件，否则拒绝续跑。
② 失败路径自动重启驱动栈与 agent_memory 的"急停后人工确认安全再恢复、不自动续跑"冲突。
   ⇒ 自动只做 `ensure_robot_ready.py --force`（不动机器人）；
     驱动栈重启 + 续跑做成**人工一键** scripts/resume_task.sh（计划新增，尚未存在）（对称 resume_teaching.sh），
     run_task 失败时把这条命令打印出来，由所有者决定是否执行。
③ run_task 每步调 ensure_robot_ready 时**没带 --force**（11-18 "接受但不执行"识别不了）；
   resume_teaching.sh 只 `control_health --activate`，同样没 --force。
④ teaching_run_state 只有 move 名映射，观察阶段（slot_fine / workpiece_*）映射为 null，
   `--resume` 会直接拒绝；且现有状态文件是 11-07 的 insert/14，缺年龄校验。
```

### H.6 另外两个 P0 缺口（新增）

```text
① run_task 失败提前 return 会让 experiment_recorder **一直跑且不归档**。
   ⇒ 在 finally 里 stop/archive。
② 面板的两个 _abort() 只 kill 父 pid，run_task 的子 runner 可能**继续动臂**！
   （RViz 面板才是按进程组 kill。）⇒ 统一改成 kill 进程组，并加"确认已停"的复核。
```

### H.7 修正后的优先级

```text
P0（当天）：B1 崩溃 · B2 左栏/日志 · A+H.1 开工脚本三缺陷+相机接入 · H.4 相机档位恢复 · H.6② 中止进程组
P1（次日）：B3/B4/B5 面板布局 · H.5 续跑（resume_task.sh + --force + 按 index 重试失败阶段）· H.6① recorder finally
P2：D 观测策略（等 capture 分析）+ 站位扩充
```

---

## I. 面板审计结果（2026-11-19，panel-audit 返回；**修正 B 节**）

### I.1 崩溃根因修正（B.1 作废，以此为准）

```text
KeyError 发生在 **安全页 SafeTab**，不是效率页。
· key_labels 注册在 788–797 行（10 个键）；_refresh_keys（1054–1089）写 11 个，
  唯一未注册的是 1088 行 key_labels["界面"]。
· 该行由 commit 9bc0b0c（2026-11-08）加入，但没往 789 行元组补键 ⇒ 每次 tick 必抛。
· 频率是 **500 ms/条**（617 行 setInterval(500)），不是 100 ms。
· 后果比"刷屏"更严重：异常从 _tick 抛出后，_refresh_captures 与状态行都不执行，
  1089 行的 _refresh_session 永远跑不到 ⇒ "分支/提交"永远是"—"、最近采集表永远空。
· 修法（推荐①）：① 删除 1088 行；② 若要保留"界面"，必须同时把键加进 789 行元组，
  并把 799 行 key_box.setFixedHeight(268) 提到 ≥300（11 行实测需约 276）。
```

### I.2 效率页真实布局（B.2–B.6 的数据依据）

```text
· 效率页**没有 QSplitter**（QSplitter 只在安全页 679–688，比例 24/30/46），全是 HBox/VBox/Grid。
· 左栏 setFixedWidth(400)（1329）；相机列 fixedWidth(1300)（1425）；
  流程列 minWidth 760（1430）；数据列 minWidth 700（1437）
  ⇒ 效率页 minHint 3180×1330；4K 下实测左 400 / 右 3174。
· 右栏竖向：相机行 1036 / 力矩 338 / 六维力 338 / 底行 206。
```

### I.3 具体改法与实测目标值

```text
① 日志（对应 B.2）
   1329 行 400 → 1200（实测日志 viewport 388 → 1188，3.06×）；
   526 行 NoWrap → WidgetWidth + WrapAtWordBoundaryOrAnywhere，关横向滚动条；
   上方三框重排：1353 行阶段 2→3 列、1369-70 行速度 2→4 组/行、1383-87 行参数 3 行→1 行
   （实测三框高度 287/132/99 → 191/78/51，日志高度 1368 → 1566）。
   注意：当前 stage 框 minHint 531 > 实际 400，勾选框文字**已经被截断**。

② 相机（对应 B.3）
   panel_camera.py：1280 图时 CameraView minWidth 1304 > 相机组 fixedWidth 1300 → 右溢 9 px；
   640 图在 1300 宽框里大片黑边；2560 图把窗口最小尺寸顶到 3560×2140（4K 装不下）；
   _toggle_fit 只设 widgetResizable，_render 263 行始终不缩放 ⇒ "铺满"无效。
   修：实现 KeepAspectRatio 等比缩放（效率页默认）；1:1 模式滚动条 AsNeeded；
   cs625_panel 1425 行去掉 fixedWidth(1300)，改 min 660 / max 1314 + 伸缩；
   1614–1621 的 _ensure_camera_room 在 fit 模式跳过或按屏幕封顶。

③ 视觉流程 + 实验数据（对应 B.4）
   **双重标题**：1248 行 VisionDataBox 本身是 QGroupBox，1436 行 make_group 又包一层同名
   QGroupBox ⇒ 两行标题重叠（截图确认）。
   列宽被覆盖：每 2 s 的 refresh 在 1303 行 resizeColumnsToContents，
   覆盖 1264–66 的 190/250/250，实测变成 130/85/688（表头居中、值左对齐 ⇒ 错位）。
   空白：流程表行高写死 78（1147）、数据行高 34（1263），容器高 1036，
   流程内容仅 417 ⇒ 570 px 空白；数据内容 265 ⇒ 699 px 空白。
   修：去掉 1303 行；列改 Fixed 190 / Stretch / Stretch；行高按 QFontMetrics 或 ResizeToContents；
   表高=内容高；数据框移到流程列下方（camera_row 改 相机 + tail[pipeline 上 / data 下 / stretch]）。

④ 流程表列（"字体被遮挡"的机制）
   现在 stretch 落在"耗时"列（1154–56），实测 90/60/140/360/271 ⇒ "输出"固定 360、"耗时"271。
   且行高写死 78，字体放大（Noto CJK/150%）时 3 行即 81 > 78，竖向滚动条又 AlwaysOff（1149）
   ⇒ 直接被裁。
   修：列 0:90 / 1:70 / 2:150 / 3:Stretch(输出) / 4:90(耗时)；行高按字体算。

⑤ 底行（3D 状态 + 运行记录）
   1476/1479 两个 fixedHeight(206) → 底行 320；status 固定宽 640、results 拉伸；
   1453 行行高 24 按字算（lineSpacing+8）；1456 行时间列 96 太窄（截图 "20261107…"），
   1560 行 len(name)>15 判断有误 ⇒ 改 HHMMSS + tooltip。
   实测 results viewport 仅 157 px（10 行只露 5.2 行）；status 内容 191 > 186 已裁 5 px。
```

### I.4 目标比例（3600×2000 实测原型）

```text
左 1200（33.5%）/ 右 2374
右竖：相机行 1036 / 力矩 281 / 六维力 281 / 底行 320（+4 spacing）
相机行：相机 1314（55%）+ tail 1056（45%）
tail 内：流程框 292 + 数据框 285 + stretch 451（= 所有者要的动态区）
底行：3D 640 + 运行记录 1730
列宽：流程 90/70/150/Stretch/90；数据 190/Stretch/Stretch；运行记录 120/60/70/60/80/Stretch
行高：line = QFontMetrics.lineSpacing()（本机 18，Noto 约 20–21）
      流程 2*line+12；数据 line+12；记录 line+8；3D max(32, line+14)
```

### I.5 面板审计额外发现（**新 bug，需一并修**）

```text
① 速度覆盖 / 运行参数**根本没接上**：speed_edits、repeat_edit、check_nogates、
   check_nosettle 在 _build_command 里从未被使用 ⇒ 面板上改速度、改重复次数、
   勾"跳过位姿门/跳过静置"都不生效。（这是功能性 bug，不只是观感）
② workpiece_latest 缺字段被当 0.00 mm 显示，违反"未采集≠0"的规则。
③ 修好崩溃后 _refresh_captures 会每 500 ms 扫 189 个 json ⇒ 需降频或按 mtime 缓存。
```

### I.6 面板改动的风险（必须遵守）

```text
· LogView 被安全页 713/813 复用 ⇒ 改换行会同时改变安全页（可接受，但要一起验证）
· panel_camera 被安全页 827 复用（830 行 fixedHeight(660) 本身就在裁 960 高图像）
· 移动 vision_data 时必须保留属性名（1317–1318 的定时器引用它）
· stage_checks 的 key 是序号，1491–1492 依赖顺序 ⇒ 重排网格不要改 key
· 左栏 1200 后效率页 minHint ≈ 3290：4K 仍可，1080p 放不下 ⇒ 相机 fit 模式要把
  最小高从 1036 降到约 360，否则小屏不可用
```

---

## J. 观测失败分析（2026-11-19，capture 审计返回；**修正 D 节**）

### J.1 两条修正（我之前的说法不准确）

```text
① "工件平移 112 mm" 不准确：112 mm 是相对 workpiece_zero 的 CAD 原点差。
   −16 站位真正的标定基准是 11-13 的 −16° 复测（workpiece_minus15：yaw −15.96°、
   coverage 0.967、扶手 0.901、3 组 3 平面）。相对它，昨天只平移 **47 mm（几乎全 +y）**、
   yaw 差 **−3.26°**。
   47 mm + 3.26° 对第三面可见性的损害 ≈ 纯平移 112 mm（yaw 匹配）——所以"112 mm"作为
   等效劣化量成立，但物理分解是 47 mm + 3.3°。

② "只看到 2 个面"是物理事实，但**质量门给出的理由（扶手分低）是软件假阴性**：
   同一份点云、同一个最终位姿，在未剔点的原始云上扶手匹配 = 0.877 / 0.890。
```

### J.2 质量门真正卡住的那一条 + 一个稳定复现的软件缺陷

```text
卡住的判定项只有一个：#5 对称性 valid（扶手分 0.134/0.18/0.19 < 0.5）→ 可信组 0。
其余项按数据都会过（覆盖率 0.988、中心重复性 0.86 mm、姿态 0.10°）。

为什么扶手分会假低（已离线复现，与 JSON 逐位一致）：
  1. g01/g02 只有 2 面 → 走两平面回退；
  2. 回退的初值偏 180°（正方形截面的镜像分支，扶手 0.0）；
  3. 回退用这个错初值 reject_with_pose 剔点 → 删掉 17,549 点，其中 84% 落在扶手 15 mm 内
     → 扶手 CAD 匹配 627/627 → 176/627；
  4. 随后多起点求解在被删过的云上算扶手 → 最大 0.18 → valid=False。
  ⇒ 它其实已经把姿态翻回正解（与最终 pose_base 差 0.07 mm），但**证据被自己删掉了**。
  ⇒ 这个缺陷在任何"2 平面测量"上都会稳定发生，不是偶发。
```

**顺带发现两个死门 / 隐患**：

```text
· choose_symmetry 全仓库从未被调用，而它才是唯一会写 symmetry_check["ambiguous"] 的地方
  ⇒ measure_object 的"翻转歧义"门永远读到 0（死门）。
· workpiece_focus 球裁 r=0.30 m，而工件 8 个 CAD 角点距球心 0.326–0.337 m
  ⇒ 四个角长期被裁；建议提到 0.36–0.38。
```

### J.3 物理缺口：第三面是"掠射"，不是"出视野"

```text
第三面（131 mm 宽窄侧面）中心仍在画面内 (520,409)；问题在入射角：
  验证时 77.9° → 昨天 82.4°，|cos| 0.210 → 0.133（−37%），距离 +7%（投影再 −14%）
  ⇒ 投影面积 −44%：模型 3761 → 2091 px，与实测 2091/2326 吻合；点云占比 0.8%。
  掠射后该面投影宽度只剩约 22 px，卡在"能拟合"和"分不出平面"之间（3 帧里 1 帧分不出）。
所以工作日志里"第三个面已不在视野内"应改为"**在视野内但掠射到不可靠**"。
```

### J.4 三个方向的定量结论（修正 D.2）

| 方向 | 结论 |
|---|---|
| ① 放宽阈值 | **不做**。扶手 ≥0.5 降到 0.18 等于放进 180° 错解（错解实测 0.04–0.10）；第三面锚定 5000 不是门但 <5000 时角点抖 10+ mm（g00 用 2091 点算出 z 差 130 mm）；平面 ≥800 也无关（昨天第三面本就有 ~2100 点）。**没有任何一条放宽有收益。** |
| ② 补偿位姿 | 可行但不作主线：推导位姿能把第三面恢复到 ~4234 px，但需要整臂 2.51 rad / 腕部 47.6°，昨天已实测中止并留下越界姿态；J4 未修。 |
| ③ 其他（推荐） | 见 J.5 |

### J.5 推荐顺序（替代原 D.2）

```text
1. 【今天就能做，0 风险】摆放约束：工件平移 ≤30 mm 且 yaw 误差 ≤3.5°。
   依据：第三面预测像素表（yaw 差 3.3° 时，平移 30 mm → 2437 px，比昨天失败点 2091 高 15%）；
        若 yaw 能压到 ≤1°，平移可放宽到 ≈85 mm。
   实现：开工前用粗测位姿与站位参考做数值检查（离线算，不动臂），超出就直接提示"请把工件
        摆回站位参考位置附近"，而不是测到 2 个面才失败。

2. 【纯代码，离线验收】修两平面回退缺陷 + 缺轴显式化：
   · 改 coarse_pose.py:726–742：不要在对称性判别之前用未收敛初值剔点
     （或用原始/仅去背景的云做对称性判别）；
   · 把 unconstrained_axis_obj 变成精观测的**显式拒绝理由**（"缺一轴，需换视角"），
     替代现在"扶手低"的误报；
   · 验收：用 11-18 三组数据离线跑，预期 g00 仍被 prior 门剔除、g01/g02 报"缺一轴"
     而不是"扶手低"。

3. 【代码 + 一次真机】选站加"平移一致性"：
   昨天 yaw −19.46° 时 −16（差 3.46°）以 0.09° 击败 −23（差 3.54°）被选中，
   但两者的"derived vs stored 平移差"差 7 倍：
       −16：80.2 mm，第三面模型 2059 px（实测 2091）
       −23：10.9 mm，第三面模型 4603 px（2.2 倍余量），且 −23 也是已验证站位
   ⇒ 选站排序改为"yaw 容差内 + 平移一致性/第三面预测像素"，而不是纯 yaw 最近。
   ⇒ 风险：−23 在当前臂位下的 IK/构型未验证（−16 当时 J3=−151.9° 已接近 −160° 下限），
     必须先 plan-only 确认再上真机。

4. 【J4 修好后】补偿位姿作受控实验：保持工具姿态的纯平移（move_tcp_linear），
   不用 plan_pose_ijk 大幅改构型；或 OMPL + 拆 ≤15° 小段 + 每段复核。

5. 【不做】原地精测兜底：粗观测站位对昨天位姿的第三面只有 ~284 px（比 −16 的 2090 还差 7 倍），
   在这台工件/这个视角下测不出精位姿。
```

### J.6 判据表（可直接写进论文/提示文案）

| 判据 | X（yaw 匹配） | X（yaw 差 3.3°） |
|---|---|---|
| 单帧能 RANSAC 分出（≥800 px） | ≈255 mm | ≈172 mm |
| 能出 3 平面但无余量（≥2000 px） | ≈113 mm | ≈58 mm |
| **稳定出 3 平面（≥2400 px）** | **≈85 mm** | **≈30 mm** |
| 达到验证时余量（≥3000 px） | ≈45 mm | ≈0 mm |

**工程判据：平移 ≤30 mm 且 yaw ≤3.5°；yaw 压到 ≤1° 时可放宽到 ≈85 mm。**
（模型为一阶近似、用 2091 px 标定，误差 ±20–30%，只作量级判断。）

