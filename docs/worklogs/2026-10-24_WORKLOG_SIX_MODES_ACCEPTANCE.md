# 2026-10-24 WORKLOG — 六模式全流程只规划验收、真机夹具几何、双 UI、数据采集与工作区重整

> 接 `2026-10-24_WORKLOG_SIX_MODES_AND_ENV_MODEL.md`。本轮按所有者要求把"不需要动真机的事全部做完"，
> 并对仿真做了**从真机实测生成**的 1:1 重建。真机今晚网络不可达（`No route to host`），
> 因此**没有任何真机运动**；所有结论来自只规划、仿真、单测与相机只读。

---

## 1. 一句话现状

**六类模式在真机夹具碰撞体存在的前提下全部只规划通过（19/19、38/38、57/57），
仿真世界由实测数据生成、夹具换成真机 RVS 几何，两套 UI（Qt 独立相机窗口 + RViz 面板）
可用，实验数据自动采集/裁剪/归档已接入。真机执行仍等机器人恢复。**

```text
scope/mode      plan-only 段数        规划运动时长
load/safe       19/19 通过            335 s
load/efficient  19/19 通过            229 s   ← 安全模式的 68%
unload/safe     38/38 通过            288 s
unload/efficient 38/38 通过           259 s
full/safe       57/57 通过            618 s
full/efficient  57/57 通过            482 s
```

---

## 2. 本轮交付（按所有者需求逐条）

### 2.1 六模式流程按"观测前缀 + 执行体 + 终点"重排

| scope | 阶段序列（摘要） |
|---|---|
| load | 工件粗观测 → 工件精观测 → 桌子扫描 → 插槽粗观测 → 插槽精观测 → baseline → pre_grasp → … → withdraw → **插槽粗观测** |
| unload | 插槽粗观测 → 插槽精观测 → 桌子扫描 → d1…d9（进槽/抓取/抽出/搬运/放置/撤回）→ **工件粗观测** |
| full | 环境建模前缀（一次）→ 装入执行体 → 插槽粗观测 → 补插槽精观测 → 取出执行体 → **工件粗观测** |

- `observe:<waypoint>` 是新的阶段类型：关节规划到观测位 → 停稳 → 抓过程图 → 执行校验命令；
- 观测点写在 `config/tasks/common.yaml` 的 `observe:` 段（粗观测位来自实测定标，
  精观测位由 `capture_dataset.camera_target()` 以 0.68 m / 俯角 20° / 方位 +10° 算出，夹爪间距 0.449 m）；
- 校验失败即中止该流程（`check` 字段是 shell 命令，退出码 0 才算通过）。

### 2.2 撤出/进入只在扶手区慢

- `SpeedProfile` 新增 `slow_band_mm`：以"离座入位的轴向回退距离"定义慢速窗口；
- `build_cartesian_legs()` 把一段拆成"快 → 慢窗口 → 快"最多三段，执行与只规划共用同一套拆分；
- 适用于 `withdraw`（load）、`d2b_entrance_tilted`、`d9a_withdraw`（unload），
  暂定窗口 `[0, 80] mm`，**待所有者按真机扶手高度确认**；
- 效果：`withdraw` 由"全段慢"变成 3 段（`legs=3 band=(0.0, 80.0)`），紧间隙段速度不变。

### 2.3 仿真世界：从真机实测生成（1:1）

```text
runtime/environment/<ts>/model.json + config/tasks/common.yaml
  → scripts/build_sim_world.py → cs625_real_cell.sdf + cs625_real_cell_scene.yaml
  → scripts/apply_real_cell_scene.py → MoveIt 规划场景
```

- 两张桌子按实测（工件桌顶面 −0.0910、插槽桌 −0.0407，xy 用实测范围）；
- **夹具换成真机 RVS 几何**（`analyze_fixture_geometry.py` 判定：旧仿真夹具与真机不符——
  槽宽 447.5 vs 真机 341.6 mm，且只有 2/9 层贯通插入路径）；
- 夹具按 RVS 语义放置：**模型原点 = slot_origin**（不是 bbox 中心），只取 yaw；
- 工件直接使用实测 CAD 位姿（模型系 == CAD 系，定义点残差 0.7 µm）；
- 碰撞体默认只发夹具（桌子厘米级精度会产生假碰撞；工件必须能被夹爪碰到）。

### 2.4 槽内紧间隙的碰撞策略（本轮最重要的工程决定）

单边间隙 ~1 mm < 网格误差，照常做碰撞检查会把真机每天都能做的动作判成碰撞。因此把
**速度规则**与**碰撞规则**分开：

```text
TIGHT_CLEARANCE_PHASES      insert / straighten / d2c / d4 / d5_extract   → 速度 ≤0.005
NO_COLLISION_CHECK_PHASES   = 上面这些 + d2b_entrance_tilted             → 规划不做碰撞检查
```

其余所有阶段仍然开启碰撞检查。报告里必须写清：仿真里"插入可行"是几何导向下的可行。

### 2.5 相机：精细 / 高帧率双模式

`scripts/camera_mode.py {status|fast|modeling|probe}`。实测结论：

```text
modeling 1280x960 彩色+深度   0.79 fps
640x480 彩色+深度            0.797 fps   ← 深度流把整机限速在 ~0.8 fps
640x480 仅彩色               23.1 fps    ← fast 模式
320x240                      硬件不存在
```

- 切换只能重启 launch（`/camera/dynamic_config` 是 GenICam XML，不是分辨率接口）；
- **重要发现**：1280×960 BGR8 图像约 3.7 MiB，大于 Fast DDS 默认共享内存段（~512 KiB），
  `SENSOR_DATA`（best-effort）下会丢 60–80% 图像（`image_raw` 只数到 0.2–0.4 fps，
  而 `camera_info` 是 0.73）。脚本改用 RELIABLE 后计数恢复正常；
- 流程钩子：环境门禁通过后切 fast；每个带校验的观测阶段切回 modeling；流程结束回 modeling。

### 2.6 实验数据自动采集 / 裁剪 / 归档

- `experiment_recorder.py`：20 Hz 时序（34 列：时间、关节位置/速度/effort、TCP、力/力矩、阶段），
  阶段标记、过程图、`run.json`、中文 `summary.md`；
- `archive_experiment.py`：三条裁剪规则（首尾空闲 / 连续静止 >3 s / 重复帧），
  归档到 `~/cs625_experiment_archive/<id>/`，并追加 `docs/evidence/experiment_index.md`；
- `run_task.py --confirm` 自动 start → stop → archive（`--no-record` 可关）；
- 阶段边界通过 `run_pick_and_place.py` 的 `notify_recorder()` 打点。

### 2.7 双 UI

**Qt 面板（`scripts/cs625_ui.py`，468×960，图像独立窗口 1280×960）**
- 状态卡（当前任务/机器人/相机/环境模型/运行状态，全零关节显示"未使能"、环境模型显示年龄与 PASS/REJECT）；
- 任务模式卡（装入/取出/全流程 × 安全/效率 × 预演/只规划/执行 × 真机/仿真）；
- **当前流程卡**：`装入 · 安全 · 真机` + 阶段 `7/19` + 进度条 + 呼吸状态点（订阅 `/task_state`，文件兜底）；
- 监视卡四个页签：关节 / 相机 / 夹爪 / **实时曲线**（关节、TCP、力力矩三组，20 s 滚动，
  暂停/清空/导出 CSV）；
- 日志卡：分级着色、新行淡入、折叠。

**RViz 面板（`src/cs625_rviz_panel`）**：400×530，六模式按钮 + 开始/停止 + 运行状态 + 日志。

### 2.8 工作区整理

- 旧文档归档到 `docs/archive/2026-10-24-pre-six-modes/`（含旧的 `simulation.md`、Phase 0/1 交接、
  P5/P6/P7 协议与验收、旧 diagnostics）；
- 旧仿真资产只读快照到 `~/cs625_legacy_sim_snapshot_20261024/`（39 个文件 + SHA-256SUMS + README）；
- 重写 `README.md`、`AGENTS.md`（12 个包、六模式、环境建模、实验数据、仿真 1:1、源码保留）、
  `docs/README.md`、`docs/simulation.md`、`docs/source_authority.md`、`scripts/README.md`、
  `docs/archive/README.md`；`.vscode/tasks.json` 重写为 00–57 号任务。

---

## 3. 验收证据（原始输出）

### 3.1 六模式只规划（真机夹具碰撞体已发布）

```text
load/safe        exit=0 ok=19 fail=0
load/efficient   exit=0 ok=19 fail=0
unload/safe      exit=0 ok=38 fail=0
unload/efficient exit=0 ok=38 fail=0
full/safe        exit=0 ok=57 fail=0
full/efficient   exit=0 ok=57 fail=0
```

### 3.2 单测

```text
python3 -m pytest test/ -q → 185 passed
```

覆盖：六模式配置解析、环境门禁四种拒绝、plan-only 不发 goal、仿真世界生成与拟合、
夹具几何判定、相机模式选择、实验记录与裁剪、曲线缓冲、UI 命令拼装与状态判定。

### 3.3 UI

```text
主面板 468x960 · 日志卡 213 px · 图像窗口 1280x960 · 无 >200x200 空白
pytest test/test_ui_command.py → 17 passed
曲线控件 demo → 19/19 checks，5000 点 push 平均 0.002 ms
```

### 3.4 相机

```text
modeling 0.793 fps 1280x960（彩色+深度）
fast     23.147 fps 640x480（仅彩色）
```

---

## 4. 未完成 / 需要所有者

```text
1. 真机执行六模式：等机器人网络恢复（No route to host）
2. 扶手区慢速窗口 [0, 80] mm 是暂定值，需要按真机扶手实际高度确认
3. RViz 面板的配色统一 / 控制器自检按钮 / 流程显示：由另一条线在做（本轮未验收）
4. 仿真里的工件比工件桌顶面低 14.8 mm（实测位姿原样保留），作为动力学体需抬高
5. 旧仿真/旧文档归档已按所有者指示完成，归档清单见 docs/archive/README.md
```

---

## 5. Capability layer

```text
- environment and dependencies      PASS    相机双模式实测；camport 工作区；ROS Jazzy 链
- robot model and simulation        PASS    仿真世界由实测生成，夹具用真机 RVS 几何
- kinematics, control and planning  PASS    六模式 231 段规划全通过（含夹具碰撞体）
- vision, hand-eye calibration/TF   PASS    精观测位 3 帧共识 1.0 mm（上一轮）；相机 0.79 fps
- active perception / NBV           NOT STARTED  前置链路已验收，可以开始
- real-hardware integration/safety  BLOCKED  机器人网络不可达；真机执行全部暂缓
```

---

## 6. 下一步唯一推荐任务

**机器人恢复后，先跑一次 `run_task --scope load --mode safe --world real --confirm --allow-release`，
用归档里的 summary.md 核对每阶段时长与 TCP 行程，作为"真机 vs 只规划"的第一份对照数据。**
