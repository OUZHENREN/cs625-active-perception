# 2026-11-09 WORKLOG 相机 ROS 端查看器 + 换档按钮（只碰相机）

> 相关：`docs/plans/2026-11.md` D0（数据入口普查）
> 真机使用：相机（真机，驱动已由所有者启动）。**未发机械臂指令、未启动夹爪、未动 MoveIt。**
> 仿真使用：无。

## 1. 本轮要解决的问题

所有者今天只做相机，不做夹持：需要一个能直接看 ROS 端相机的窗口，并且能在窗口里换分辨率 /
fast 模式（现在只能命令行跑 `camera_mode.py`，看起来不方便）。

现有资产的边界：
- `scripts/camera_mode.py`：换档逻辑（重启相机 launch、只动 `runtime/camera.pid`、实测 fps），**已成型，不重写**；
- `scripts/cs625_panel.py`：完整操作面板，但相机只是其中一栏，且订阅 `/sensors/camera/*`（需要 sensor_adapter 在跑）；
- `scripts/panel_camera.py`：1:1 显示控件，订阅 `/sensors/camera/*`。

→ 结论：把"看"和"换档"拼成一个独立窗口，复用 `camera_mode.py` 的子进程接口，不改它的逻辑。

## 2. 新增 / 修改的文件

```text
A  scripts/camera_view.py       新增：ROS 端相机查看器（彩色/深度/红外 + 换档按钮 + 截图）
M  .vscode/tasks.json           新增任务「⑦ 相机 · 查看/换档（只碰相机）」
M  docs/README.md               VS Code 任务表 5 → 7（补 ⑥ DSH、⑦ 相机）
M  docs/plans/2026-11.md        D0 普查注明"看数用 camera_view.py"
A  docs/worklogs/2026-11/2026-11-09_WORKLOG_CAMERA_VIEW.md   本文件
```

## 3. 相机查看器做了什么

```text
订阅：/camera/color/image_raw、/camera/depth/image_raw、/camera/left_ir/image_raw（可选）
      同时订阅 /sensors/camera/* 作为后备（sensor_adapter 在跑时优先驱动话题）
显示：彩色 / 深度 / 红外 三选一；默认铺满，可切 1:1 原始像素（不缩放）
状态栏：分辨率、编码、真实到达帧率、深度有效点比例、中位深度、来源话题
换档：[fast 640×480] [modeling 1280×960] [high 2560×1440] [停止] [状态]
      —— 后台子进程跑 camera_mode.py <action>，输出进窗口下方日志，界面不卡
截图：runtime/camera_snapshots/<时间戳>_<类型>.png
安全：只操作相机 launch 进程（runtime/camera.pid）；不发机械臂指令、不启动夹爪、不碰 MoveIt
```

深度显示是**按 0.2–2.0 m 映射成灰度**，无效点（0）显示为黑；状态栏同时给出有效点比例，
这正好是 D0 普查要填的数。

## 4. 执行的命令与结果

```bash
python3 -m py_compile scripts/camera_view.py                              # PASS
source scripts/source_dev_env.sh --full
QT_QPA_PLATFORM=offscreen python3 -c "... 构造窗口 + 三种编码转换 + 退出 ..."  # PASS
python3 scripts/check_docs.py                                             # PASS
python3 -m pytest test/ -q                                                # 293 passed
```

离屏冒烟测试里第一次退出时报 `terminate called without an active exception`（exit 134）：
原因是窗口关闭时 ROS 订阅线程还在跑。已加 `shutdown()` 并接到 `closeEvent` 与 `aboutToQuit`，
复测 exit code 0。

## 5. 兼容性说明

- 相机订阅用 **BEST_EFFORT**：驱动以 reliable 发布时 best-effort 订阅端仍能收（反过来不行），
  所以两种发布方式都能看。
- 换档时相机会被重启，图像会短暂中断，状态栏会显示"等待图像"，这是正常的。
- 红外话题默认不在：需要 launch 时 `left_ir_enable:=true`；窗口只是不会切到该页签。

## 6. 剩余限制

```text
· 深度图按固定 0.2–2.0 m 映射灰度；更近/更远会截断（状态栏的中位深度是真实值）。
· 未做 PreSetMode（Default/Quality/Fast）切换：datasheet 有这档，但 camport 驱动没有把它
  暴露成 ROS 参数（TY_CONFIG_MODE_PRESET0/1/2 只在 SDK 里）。要试只能在 RVS 或 SDK 侧改。
· 窗口不写数据集；采数据仍走 D1 的 vision_capture.py。
```

## 7. 能力层与关键链（本轮）

```text
Capability layer：vision 的"可观测性"改善（能看见分辨率/帧率/有效点比例），算法层未推进。
Critical-chain status（未变）：
  RGB-D -> normalized topics   PARTIAL（彩色可用，深度 0.79 fps 是 Default 档）
  base_link -> camera optical  NOT ACCEPTED（按所有者决定，本月不做手眼）
Sim/real alignment：本轮只用真机相机；real motion 未发生。
Intentionally not done：夹爪、机械臂运动、MoveIt、数据集落盘、PreSetMode。
```

## 8. 补充（2026-11-09 18:0x）：所有者看到窗口后反馈的三类问题

### 8.1 窗口标题乱码

现象：标题栏里中文变成方块。原因：WSLg 的窗口装饰由 Weston 绘制，字体没有中文字形，
Qt 里的中文字体管不到标题栏。处置：标题改成 ASCII `CS625 Camera - ROS view / mode switch`，
界面内的中文保持（按钮/状态栏/日志都正常，截图已验证）。

### 8.2 按钮文字对比度不足

现象：按钮左浅右灰、文字发白。原因：上一版只设了 `QWidget { color }`，按钮底仍是系统浅色，
浅色字落在浅色底上。处置：写完整 QSS——按钮底 `#18232e`、边框、悬停、按下、选中、**禁用**
五种状态全部显式配色；并显式挑中文字体（本机可用 `WenQuanYi Micro Hei`）。

### 8.3 "彩色：等待图像"——这才是真正的问题

本机诊断结果（18:0x，`ROS_DOMAIN_ID=0`）：

```text
节点 0 个；话题只有 /rosout 与 /parameter_events
percipio_camera 进程：无
```

也就是说：**这个 WSL 会话里现在没有任何 ROS 节点在跑**，相机没有发布者。
窗口自己的订阅会让 `/camera/*` 出现在话题列表里（但发布者 0），所以"话题存在"不等于"相机在发"。

处置：
1. 窗口新增 **诊断** 按钮：一次打印 ROS_DOMAIN_ID/RMW、节点表、相机话题及其发布者数、
   percipio_camera 进程；没有节点/没有发布者时直接给结论。
2. 10 秒无图像时在日志里自动提示一次（相机没起 / 另一个终端 ROS_DOMAIN_ID 不同 / RVS 占着）。
3. 想直接看就点窗口里的 **启动 · modeling 1280×960**（等价 `python3 scripts/camera_mode.py modeling`），
   它会用 `~/.cs625_local.env` 里的相机 IP 起相机 launch。

## 9. 18:0x–18:15 相机后端拉起 + 第一组真实数据

所有者贴的终端显示 `ros2 node list` 为空、没有 `/camera/*`，问"是不是要你搭好后端"。
处置：直接在本会话拉起相机并验证。

```bash
$ ping -c 2 192.168.1.120                     # 相机可达，1.9 ms
$ python3 scripts/camera_mode.py modeling     # 用 ~/.cs625_local.env 的相机 IP
[camera] stopping pid=143121                   # 清理 11-07 留下的旧 pid
[camera] starting: color=1280x960 depth=1280x960 point_cloud=on
[camera] pid=143360 log=runtime/camera.log
mode=modeling fps=0.796 res=1280x960 depth=0.796fps@1280x960
```

验证：

```text
ros2 topic list | grep camera → color/depth image_raw + camera_info + depth_registered/points …
publisher count = 1（/camera/color/image_raw、/camera/depth/image_raw）
ros2 node list → /camera/camera, /camera/camera_container
ros2 topic hz /camera/color/image_raw → 0.764 fps（间隔 1.31 s）
ros2 topic hz /camera/depth/image_raw → 0.5–0.6 fps（间隔 1.31–2.62 s）
```

采三帧验证整条链路（`scripts/polarizer_ab.py capture --state check --frames 3`）：

```text
深度 1280×960，有效点 76.1% / 76.1% / 76.2%，中位深度 1.340 m
内参 fx 931.473 / fy 931.596 / cx 648.695 / cy 449.319 —— 与 config/real/camera_intrinsics.json 完全一致
```

**结论：仓库里"1280×960 深度有效点≈0，所以退回 640×480"的旧结论作废。**
那条是 10-23 测的，`laser_power:=100` 10-24 才落地。今天在激光 100 下 1280×960 有 76% 有效点，
D1 采集没有理由再降分辨率。

另一个观察：深度开着时彩色也被拖到 ~0.76 fps（datasheet 写 16 fps@1280×960，应是只开彩色流的值）。

界面侧：`python3 scripts/camera_view.py`（VS Code 任务 ⑦）已在本机桌面启动并订阅成功。

## 10. 18:21 偏振第一状态：IR1/IR2 已贴同向膜、投射端未贴 —— 无可测效果

所有者问"要撕下来吗"。先测再答：同一机位采 3 帧（`--state recv_only`），与 18:14 的
`check` 组（有效点 76.1%）逐项对比：

```text
有效点比例      76.1% → 76.1%   （0.0%）
中位深度        1.340 → 1.348 m （8 mm，噪声量级）
空洞率          23.4% → 23.4%
飞点比例        3.6% → 3.8%     （略变差）
彩色平均亮度     85.7 → 85.8    （0.1%；彩色镜头没贴膜，本来就不该变）
深度帧间一致性   有效掩膜 IoU 99.2%
```

结论：**接收端单独两片膜对深度没有可测影响**。不要撕（它也没让数据变差，而且此刻的
"贴膜状态"与"未贴状态"数据不可区分，基线没丢）。需要三步对照实验定位原因：

```text
A 窗口认定：纸/手分别挡住三个窗口，深度出现大面积无效的那个才是 IR 接收窗
B 波段确认：膜举到彩色镜头前应变暗约一半（可见光确实是偏振片）；举到 IR 窗前若无变化
            → 该膜在 830 nm 透明/不偏振，换 800–900 nm 专用膜
C 投射端：另一片膜举在投射窗前旋转，看深度飞点是否随角度变化（有变化=交叉成立）
```

工具侧：`polarizer_ab.py compare` 增加了一行「彩色平均亮度」，用来区分"膜在挡光"和
"膜完全没进光路"。

## 11. 18:2x UI 缩放 + RVS→ROS 前置数据对齐核查

### 11.1 4K 屏上界面太小

处置：`camera_view.py` 增加 DPI/分辨率自适应与 `--ui-scale`：

```text
detect_scale()：按主屏逻辑分辨率取 max(w/1920, h/1080)，夹在 1.0–2.5
窗口默认尺寸：可用屏幕的 72%，并设最小尺寸（随缩放），避免被压扁
字体/内边距/按钮高度/日志高度：全部乘 scale；样例 1.6 倍时按钮 30→48 px、正文 10→16 pt
Qt 高 DPI 属性（AA_EnableHighDpiScaling / AA_UseHighDpiPixmaps）在 QApplication 之前设置
显式覆盖：python3 scripts/camera_view.py --ui-scale 2.0
```

### 11.2 RVS 的前置数据，ROS 侧对齐了吗（逐项核实，见 agent_memory §129）

```text
✅ 内参：同一设备同一档；2026-11-09 实测 color/depth camera_info 的 K 与
        config/real/camera_intrinsics.json 完全一致（深度已配准到彩色，两者 K 相同）
✅ 手眼：已由 scripts/sync_real_handeye.py 从 RVS 三个标定文件导入，带 provenance
⚠️ 手眼未验证：T_tcp_cam vs T_flange_cam 两种口径差 0.16 m；本月按所有者决定不验
✅ 激光功率：laser_power:=100 与 RVS 一致
❌ laser_auto_ctrl：RVS=1，驱动只写 TY_INT_LASER_POWER，不设自动控制
❌ triggerMode22_ledGainRatio：驱动没暴露
?  曝光/增益：RVS 工程无显式值（设备 AE），ROS 也没设；设备里是否残留 RVS 用户集无法确认
❌ RVS 私有后处理（滤波/掩膜/3D 模板匹配）：不可复现，只能自研替代
```

结论：几何前置对齐且有证据；标定前置导入但未验证；设备参数只对齐了激光功率；
RVS 的后处理链不可复现——这是"做不到"，不是"没做"，如实写进 memory。

## 12. 18:33–18:50 D0 相机档位普查（初步实验，含图像记录与数据分析）

所有者要求把普查当作初步实验来做，要有图像记录和数据分析。

### 12.1 新增工具

```text
A scripts/camera_census.py
  逐档换档（复用 camera_mode.py）→ 采 N 帧 → 算指标 → 存图/存深度 → 出 summary.md + metrics.csv
  指标复用 scripts/polarizer_ab.py（有效点、飞点、点距、空洞率），避免两套口径
```

### 12.2 结果（2026-11-09 18:47，laser_power:=100，真机静态工位，未动机械臂/夹爪）

| 档位 | 分辨率 | 彩色 fps | 深度 fps | 有效点 | 中位深度 | 点距 | 飞点 | 空洞率 |
|---|---|---|---|---|---|---|---|---|
| fast | 640×480 | 25.0 | —（该档只开彩色） | — | — | — | — | — |
| modeling | 1280×960 | 0.801 | 0.801 | 76.1% | 1.351 m | 1.48 mm | 3.7% | 23.4% |
| high | 2560×1920 | 0.801 | 0.799 | 76.1% | 1.351 m | 0.73 mm* | 3.6% | 23.4% |

\* high 档的深度被重采样到彩色分辨率（K 同步放大 2 倍，fx 931.47 → 1862.95），点距变小是插值结果。

### 12.3 过程中修正的两件事

1. **HIGH_SPEC 原本是错的**：`color=2560x1440 / depth=2560x1440` 被驱动直接拒绝
   （`Unsupported Depth stream resolution: 2560x1440`、`Unsupported Texture stream resolution: 2560x1440`）。
   按设备真实能力改成 **彩色 2560×1920 + 深度 1280×960**，实测可用（与 RVS 工程用的彩色档一致）。
2. 普查脚本最初把"fast 无深度"当失败，且会空等满 90 s 并累积彩色帧（约 2 GB）。
   已改成"彩色够帧后 6 s 无深度即判该档无深度"，并限制缓存帧数。

### 12.4 产物与归档

```text
runtime/experiments/camera_census_20261109_184714/
    summary.md / metrics.csv
    compare_colour.png / compare_depth.png     三档同场对比图
    fast/color_000.png + meta.json
    modeling/color_000.png + depth_000/001.npy + meta.json
    high/color_000.png + depth_000/001.npy + meta.json
~/cs625_experiment_archive/camera_census_20261109_184714/   （含 README.md）
```

### 12.5 数据分析结论

```text
1. 深度帧率是硬瓶颈：modeling 与 high 都锁在 ~0.8 fps（Default 档），换分辨率不改善。
2. 开深度时彩色同速：fast 档 25 fps 是因为它【只开彩色】；datasheet 的 16 fps@1280x960 同理。
3. high 不带来深度信息增量：有效点/中位深度/空洞率与 modeling 完全一致，只是插值放大；
   代价是每帧 4.4 MB 彩色 + 9.8 MB 深度（uint16）。→ D1 默认 modeling。
4. 三档有效点都是 76.1%，且与 18:14、18:21 两次独立采集一致 → 有效点比例是可复现的指标。
5. 中位深度在同一静态场景下 1.340 → 1.351 m（相机重启后差约 1 cm）→ 跨重启有约 1 cm 的系统偏移，
   位姿对比必须在同一会话内闭合，不能跨重启直接比绝对值。
```

## 13. 19:0x–19:16 查看器三个真实 bug（"等待图像"与"点深度没反应"）

所有者反馈：窗口里图像是有，但状态栏一直"彩色：等待图像"；点"深度"没反应。
排查后发现是三个独立缺陷：

### 13.1 相机换档/重启后，旧订阅端点不会重新匹配

现象：画面停在最后一帧、状态一直是"等待图像"，看起来像没在采集。
原因：`camera_mode.py` 换档会重启 launch；WSL + Fast DDS 下旧订阅端点有时不会重新匹配，
订阅端继续挂在已消失的发布者上。新建进程（新参与者）反而一切正常——所以"重开就好"。
处置：worker 增加看门狗——**收到过帧、随后连续 8 s 没有新帧 → 销毁节点、重建订阅**；
状态里同时显示"已 N s 无新帧"，不再让人误以为在采集。

### 13.2 切到"深度"时 NameError，直接把 worker 线程打死

现象：一点"深度"，之后再也没有任何图像，按钮也没反应。
原因：切换后深度第一帧还没到，状态里拼"已 N s 无新帧"时写成了未定义变量 `v`
（应为候选话题名），抛 `NameError`；异常被外层 `except` 捕获后线程结束 → 再也没有帧。
处置：修正变量；切换时立刻显示"深度：切换到该路，等待第一帧…"，给即时反馈。

### 13.3 同一帧被重复绘制（10 Hz）

现象：看丢弃计数才知道——原来 worker 每 100 ms 就把"最新一帧"重发一次，
0.8 fps 的流等于同一张图被画十几遍。
处置：按 `header.stamp` 去重，只有**新帧**才发信号；fps 统计窗口放宽到 5 s（低帧率下更稳）。

### 13.4 顺带修掉"只能 kill -9"

Qt 事件循环在 C++ 里跑，Python 信号处理器得不到执行，`kill` 无效。
处置：注册 SIGINT/SIGTERM 处理器 + 一个 250 ms 的空定时器，让 Python 有机会处理信号。
现在 `kill` 能干净退出。

### 13.5 验证

```text
离屏（QT_QPA_PLATFORM=offscreen）接真机：
  5 s 彩色：2 帧，状态 1280×960·bgr8·0.8 fps
  切深度后 8 s：6 帧，状态 1280×960·16UC1·0.8 fps·有效点 76.1%·中位深度 1.354 m
  切回彩色正常
旧窗口已关闭，新窗口（pid 166185）已按新代码重启；ROS 图上 /cs625_camera_view 正常订阅。
```
