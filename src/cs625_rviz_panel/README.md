# cs625_rviz_panel

RViz2 任务控制面板：把 `scripts/run_task.py` 的 scope / mode / world / action 做成紧凑的
分段卡片，用 QProcess 启动入口脚本，并显示运行状态、已用时间、最新阶段和日志尾部。

新版本在原有按钮之上加了两块内容：

1. **当前流程卡片（顶部）**：订阅 `/task_state`（`std_msgs/String`，JSON），显示
   `装入 · 安全 · 真机`、阶段名、`7/19` 和一条细进度条；话题中断时每 500 ms 读
   `runtime/task_state.json` 兜底，超过 5 s 没有新状态就显示“空闲 / 等待”。
2. **控制自检卡片**：`检查控制链 / 激活控制器 / 恢复控制 / 环境模型` 四个按钮，
   结果全部写进日志区；ROS 服务调用是异步的（3 s 超时），UI 线程不阻塞。

面板**不含任何运动逻辑**，也不会自动运行任务；`run_task.py`、`run_pick_and_place.py`
和 `config/tasks/` 都没有修改。

## 构建、加载、使用（三行版）

```bash
source scripts/source_dev_env.sh --full && colcon build --symlink-install --packages-select cs625_rviz_panel
source scripts/source_dev_env.sh --full && rviz2 -d src/cs625_rviz_panel/config/cs625_task.rviz
# 右侧 “CS625 Task” 停靠面板：选参数 → “开始”；“执行”会先弹确认框
```

`rviz2` 建议从**仓库根目录**启动：面板先看 `CS625_REPO_ROOT`，没有就沿
`QDir::currentPath()` 向上逐级查找 `scripts/run_task.py`；两者都失败时，“开始”按钮被
禁用，日志卡会说明原因。工作目录解析成功后，子进程的 cwd 固定为这个仓库根目录。

## 界面结构（7 张卡片，400 × 约 715 px）

| 顺序 | 卡片 | 内容 |
| --- | --- | --- |
| 0 | 当前流程 | 流程/模式/环境 + 阶段名 + `7/19` + 4 px 进度条 + 状态点 |
| 1 | 任务范围 | `装入 / 取出 / 全流程`（互斥 pill） |
| 2 | 模式与环境 | `安全 / 效率` + `真机 / 仿真`（两列紧凑排列） |
| 3 | 执行动作 | `预演 / 只规划 / 执行` + 大号 `开始` / `停止` |
| 4 | 控制自检 | `检查控制链 / 激活控制器 / 恢复控制 / 环境模型`（2 × 2） |
| 5 | 任务进程 | 面板启动的 run_task.py：状态点 + 空闲/运行中/失败 + 已用时间 + phase_log 最新阶段 |
| 6 | 日志 | 等宽字体，最近 8 行，自动滚动，行首按级别着色 |

卡片间距 6 px，卡片内边距 8 px，圆角 8 px，1 px 描边。面板宽度固定 400 px。
`config/cs625_task.rviz` 中 dock 宽度也是 400 px（`QMainWindow State` 里右侧 dock 的
`0x190`），1K 窗口下中间 3D 视图仍保留约 968 px。

## 配色：跟随 RViz / Qt 调色板（不再硬编码全黑）

| 用途 | 来源 |
| --- | --- |
| 面板背景 | `palette().color(QPalette::Window)`，通过 `autoFillBackground(true)` 绘制 |
| 卡片背景 | `QPalette::Base` |
| 文字 | `QPalette::WindowText` / `QPalette::Text` |
| 1 px 边框 | `QPalette::Mid` |
| 选中/强调 | `QPalette::Highlight`（缺省回落到 RViz 高亮蓝 `#3daee9`） |
| 运行绿 / 警告琥珀 / 错误红 | `#2ecc71` / `#f39c12` / `#e74c3c`（只用于状态点、状态文字和日志行首） |

主题由 `PanelTheme::fromPalette()` 一次算出，样式表只用在**子容器**上：给面板本身设
`setStyleSheet()` 会重置面板的 palette，进而让下一次主题推导拿到被改坏的调色板。面板
收到 `QEvent::PaletteChange` 时重新推导并重绘，所以 RViz 换主题（或桌面切深浅色）后
面板与两侧 Displays / Tool Properties 保持同一灰阶。启动时面板会把实际取值打印到终端：

```text
[cs625_task_panel] palette Window=#35393e Base=#2a2e32 Mid=#4a4f55 Highlight=#3daee9 Text=#eff0f1
```

`config/cs625_task.rviz` 不设置任何 Qt style / palette / theme 字段，只给 3D 视图一个
`Background Color: 48;48;48`，不影响面板配色。

## 当前流程 / 当前阶段

数据流：

```text
scripts/run_pick_and_place.py
  → /task_state (std_msgs/String, JSON) ─┐
  → runtime/task_state.json ─────────────┴→ TaskRosBridge → TaskPanel → “当前流程”卡片
```

字段：`flow`(load|unload|full)、`mode`、`world`、`phase`、`phase_index`、`phase_total`、
`status`(running|ok|failed)、`detail`、`plan_only`、`stamp`。

- `running`：绿点呼吸（`QPropertyAnimation`，1600 ms），进度条按 `phase_index/phase_total`；
- `ok`：绿点常亮；`failed`：红点；
- 距 `stamp` 超过 5 s（或从没收到）：显示“空闲 / 等待任务”，进度归零——阶段执行器只在
  阶段边界发布，UI 不能假装阶段还在跑；
- 话题 5 s 内没有消息时，500 ms 定时器读 `runtime/task_state.json` 兜底（UI 比节点先起、
  或 RViz 不在同一 DDS 域时仍能看到状态）；文件里的 `stamp` 优先，缺失时用文件 mtime。

## 控制自检

| 按钮 | 行为 | 输出 |
| --- | --- | --- |
| 检查控制链 | 异步查询 `<manager>/list_controllers`（3 s 超时），并读缓存的 safety/robot mode、`/joint_states` 时间戳 | `arm_controller: active`、`safety_mode=1 (NORMAL)`、`robot_mode=7 (RUNNING)`、`joint_states: 12 ms 前更新（正常）`；reverse control 端口提示用户跑 `python3 scripts/control_health.py` |
| 激活控制器 | 二次确认后异步调 `<manager>/switch_controller`，activate `arm_controller`（`BEST_EFFORT`，timeout 5 s） | `ok=true/false`、`message=...`；失败打印 `REJECTED / TIMEOUT / SERVICE_UNAVAILABLE / CALL_FAILED` |
| 恢复控制 | `QProcess` 起 `bash scripts/recover_control.sh`（不看 ROS 节点状态） | 结束后打印输出尾部 20 行和退出码；120 s 兜底 terminate |
| 环境模型 | 读 `runtime/environment/latest.json` → `model.json` 的 `created_at` | `可用（0.6 h 前，阈值 2.0 h）` / `已过期（2.3 h 前…）` / `缺失 — 先运行 scripts/model_environment.py` |

控制器服务名按顺序尝试 `/eli_ros2_control_node`、`/controller_manager`；可用环境变量
`CS625_CONTROLLER_MANAGER` 覆盖首选，`CS625_ARM_CONTROLLER` 覆盖控制器名。

**不阻塞**：`rclcpp` 节点和自己的 `SingleThreadedExecutor` 跑在独立线程；service 用
`async_send_request` + `QTimer` 3 s 截止；超时后迟到的回复按 request id 丢弃。

## 轻量动画（共 3 个）

1. **状态点呼吸**（任务进程卡）：1600 ms、`InOutSine`、`loopCount=-1`。
2. **流程绿点呼吸**（当前流程卡）：只在 `status=running` 时启动，ok/failed/空闲立即停止。
3. **日志新行高亮**：`QVariantAnimation`（400 ms，`OutCubic`）把最后一行的背景从强调色
   25% 淡到透明。

全局只有一个 500 ms 的 `QTimer`；ROS 订阅回调通过队列信号回到 UI 线程，不在回调里碰
QWidget。

## 命令与工作目录

面板实际执行的命令形如：

```text
python3 scripts/run_task.py --scope load --mode safe --dry-run --world real
python3 scripts/run_task.py --scope full --mode efficient --confirm --world sim
```

工作目录固定为面板解析到的仓库根目录。`python3` 从 `PATH` 取；子进程环境设置
`PYTHONUNBUFFERED=1`、`PYTHONIOENCODING=utf-8`，中文输出实时显示。

## 测试（不启动任务、不动机械臂）

```bash
source scripts/source_dev_env.sh --full
colcon build --symlink-install --packages-select cs625_rviz_panel
colcon test --packages-select cs625_rviz_panel
colcon test-result --verbose
```

- `test/test_task_command.cpp`（15 个纯函数用例）：默认预演、动作→命令行、
  scope/mode/world 透传、`--allow-release` 可选、非法组合逐个拒绝、全部合法组合接受、
  阶段日志文件名、已用时间格式、阶段名解析（完整文件/尾部截断/不完整回退/转义/Unicode）、
  日志分级。
- `test/test_task_state.cpp`（12 个纯函数用例）：`/task_state` JSON 解析（完整/中文/
  plan_only/时间戳/损坏输入）、进度与 `7/19` 计算、5 s 空闲映射、流程/模式/环境中文名、
  `runtime/task_state.json` 兜底读取、ISO-8601 UTC 解析、环境模型年龄/过期/缺失、
  safety_mode/robot_mode 数字含义、控制器行格式、日志尾部 20 行。
- `test/test_task_widgets.cpp`（7 个 Qt Widgets 用例，offscreen）：pill 互斥/锁定、
  `PanelTheme` 从 `QPalette` 取值、强调色回落、颜色混合与可读文字色、样式表不含旧黑色。
- `test/test_task_panel_render.cpp`（2 个 offscreen 用例）：真实 `TaskPanel` 在
  light/dark 两种调色板下都能构建并 `grab()`，断言 400 px 宽、`autoFillBackground()`、
  `7/19` 进度、5 s 后回到空闲；另一个用例直接调用“环境模型/检查控制链”槽并断言日志内容
  （无 ROS 时也必须给出明确结果）。保存
  `/tmp/rviz_panel_{default_idle,dark_idle,final,selfcheck}.png`（可用
  `CS625_PANEL_SHOT_DIR` 改目录）。
- `test/test_task_ros_timeout.cpp`（1 个 ROS 用例）：真实 `TaskRosBridge` 无服务时必须在
  3 s 左右返回 `SERVICE_UNAVAILABLE`，证明 UI 不会卡在 service 调用上。
- `test/test_task_ros_services.cpp`（1 个 ROS 集成用例）：把 bridge 指向私有服务基名
  `/cs625_test_cm_<pid>`，用进程内的假 controller manager 验证成功路径——
  `arm_controller: active` 与 `switch_controller … ok=true；message=activated`；不碰真机。

## 手动验收

1. 启动上面的 `rviz2` 命令，右侧出现 400 px 的“CS625 Task”，日志显示
   `任务面板就绪。工作目录: /.../cs625-active-perception`。
2. 保持“预演”，点“开始”：日志出现完整命令和 `dry run: no motion, no planning`。
3. 切“只规划”再点：打印各阶段规划结果，机械臂不动。
4. 切“执行”，点“开始”：出现确认框；点“否”，日志只记录取消。
5. 任务运行中点“停止”：日志显示停止请求，状态回到空闲；再点“开始”可重新运行。
6. 发布一条状态，顶部卡片应在 500 ms 内更新：

   ```bash
   ros2 topic pub --once /task_state std_msgs/msg/String \
     "{data: '{\"flow\":\"load\",\"mode\":\"safe\",\"world\":\"real\",\"phase\":\"d2b_entrance_tilted\",\"phase_index\":7,\"phase_total\":19,\"status\":\"running\",\"detail\":\"points=1200 dur=14.2s\",\"plan_only\":false}'}"
   ```

7. 没有驱动时点“检查控制链”：约 3 s 后日志出现
   `控制器清单查询失败（错误码 SERVICE_UNAVAILABLE）`，面板不卡。

## 已知限制

- 面板宽度固定 400 px：把 dock 拖得更宽时，卡片右侧会留出面板外空白。
- 日志卡只保留 8 行；需要完整日志时看启动 RViz 的终端（`RCLCPP_INFO` 与 `qInfo` 都在那里）。
- “任务进程”卡描述的是面板用 QProcess 启动的 `run_task.py`，阶段仍读
  `runtime/phase_log_<scope>.json`（旧行为）；顶部“当前流程”卡才是实时的 `/task_state`。
  两者可以不同步：例如任务由外部终端启动时，顶部在跑而“任务进程”显示空闲。
- safety_mode / robot_mode 依赖 underlay 的 `eli_common_interface`；没有它时插件仍可编译
  和加载，但这两个数会显示“未收到”。源码里用 `find_package(... QUIET)` 处理。
- 激活控制器用 `BEST_EFFORT`：控制器已经 active 时也返回 `ok=true`；需要严格
  重启控制链时用“恢复控制”（脚本里是 strict deactivate/activate）。
- 面板截图用 `QWidget::grab()` 离屏渲染；WSLg 下抓 RViz 整个窗口仍可能是黑屏，所以验收
  以面板 PNG + 停靠几何为准。设置 `CS625_PANEL_SCREENSHOT=/tmp/x.png` 可让运行中的面板
  在收到第一条 `/task_state` 后自动抓一张自己的图。
- 验证环境：Ubuntu 24.04 / ROS 2 Jazzy / Qt 5.15 / rviz_common 14.1。没有新增需要安装的
  依赖；`rclcpp`、`std_msgs`、`sensor_msgs`、`controller_manager_msgs` 都在 ROS 基础
  安装里。若换到 Qt6 版 RViz，需要按 Qt6 重新编译。
