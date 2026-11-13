# CS625 视触融合精密装配 / 遮挡目标定位与可达性观测规划

> 课题：**面向遮挡目标定位的机械臂可达性规划方法研究**（毕业论文主线）
> 真机：Elite CS625 六轴协作臂 + 平行夹爪 + Percipio PS800 RGB-D + 插槽夹具 + 弹仓工件
> 环境：WSL2 Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic
> 更新：**2026-11-13**（上一版状态表停在 2026-11-08，已过期；更早的六模式版在 `docs/archive/`）

本仓库做一件事：**让机械臂先看清楚再动手** —— 在遮挡/不确定条件下主动选择观测位姿把目标定位到
可抓取/可装配的精度，再把它交给一个可复现、可测量的装配流程。

---

## 1. 现在真实的状态（2026-11-13）

### 视觉与观测（当前主线，测量链已可依赖）

| 能力 | 状态 | 证据 |
|---|---|---|
| 工件粗观测（带先验） | ✅ 真机，覆盖率 0.98，yaw 可检出 | `scripts/measure_object.py --stage coarse --prior auto` |
| 工件精观测（带先验） | ✅ **真机 0.14–0.5 mm / ≤0.2°** | 同上 `--stage fine --prior <coarse.json>` |
| 插槽精观测 | ✅ 真机 0.64 mm / 0.063° | `measure_object.py --preset slot --stage fine` |
| 观测前缀接入面板效率页 | ✅ 约 1 分钟跑通（该页走 `run_task.py --mode efficient`，不是 ADR-005 的 `teaching_debug` 效率模式） | `run_task.py ... --until measure:workpiece_fine` |
| 预教站位自动选站（方案 A） | 🆕 代码完成 + 离线验证；**真机未跑** | `scripts/nbv_from_reference.py --auto-station` |
| 自动 NBV（几何打分搜索换位） | ⛔ 已放弃：需大幅改构型或越界，收窄为方案 A | `docs/archive/2026-11-13-p7-nbv-legacy-code/` |

### 运动与装配

| 能力 | 状态 | 证据 |
|---|---|---|
| 示教链路：观测 → 抓取 → 抬升 → 转运 → 分级 → 旋转 → 预插入 | ✅ 真机整链跑通（人工示教后回放） | `docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md` |
| 效率模式的抓取 / 插入 | 🚧 **仅 plan-only**，真机未验证 | `scripts/run_pick_and_place.py` |
| 插入到底 | 🚧 371–372 mm 被机器人自身力矩保护挡住（重复性 1.1 mm），不是物理底部 | 同上 第 21 节 |
| 槽内转 10° 转平 | 🚧 未在真机跑通（源码依据已确认） | `scripts/teaching/straighten_in_slot.py` |
| 规划场景里的桌子 / 夹具 | ⛔ MoveIt 查询返回 0 个碰撞对象 → **自动运动前必须先修** | 交接文档 §6 |
| 六维力传感与力控 | ⛔ 传感器 12 月到位，指标全部记"未采集" | `docs/reference/experiment_metrics.md` |

### 平台与工具

| 能力 | 状态 | 证据 |
|---|---|---|
| 两种运行模式（效率 / 示教） | ✅ 已收敛为两条入口：效率模式在开发，示教模式冻结 | `scripts/teaching/teaching_debug.py`、`scripts/resume_teaching.sh` |
| 操作面板（PyQt5） | ✅ 阶段/遥测/采集/相机 1:1/夹爪/RViz 状态卡 | `scripts/cs625_panel.py` |
| RViz 3D（机械臂/点云/规划场景） | ✅ 复用 MoveIt 参数启动 | `scripts/open_rviz.sh` |
| 实验指标与归档 | ✅ 六类 18 项字段，`runtime/debug_runs/` | `scripts/teaching/teaching_debug.py` |
| 六模式（scope × mode）矩阵 | 🗄 已归类（ADR-005）：源码冻结保留，不作为日常入口 | `docs/decisions/ADR-005-two-modes-and-insert-authority.md` |
| P7 五道门仿真流水线 | 🗄 2026-11-13 归档（代码 + 证据），只读溯源 | `docs/archive/2026-11-13-p7-nbv-legacy-code/` |

**一句话**：观测（测量）这条链已经可以依赖；接下来是插入段、规划场景可见性，以及方案 A 的真机验收。

---

## 2. 三分钟上手

```bash
cd ~/projects/cs625-active-perception

# ① 唯一支持的环境入口（每个 shell 必做）
source scripts/source_dev_env.sh --full

# ② 离线自检（不碰机器人）
python3 -m pytest test/ -q            # 275 passed
python3 test/contract_checks.py       # 仓库治理合同
python3 scripts/check_docs.py         # 文档一致性

# ③ 六模式源码的离线 dry-run（已归类，仅作配置合并 + 相位表 + 门检自检，不动臂）
python3 scripts/run_task.py --scope load --mode efficient --world sim --dry-run

# ④ 动臂前的四项门
python3 scripts/control_health.py
python3 scripts/preflight_motion.py

# ⑤ 方案 A 选站（纯数学，不动臂）
python3 scripts/nbv_from_reference.py --auto-station \
    --reference-measurement runtime/measurements/workpiece_zero.json \
    --current-measurement runtime/measurements/workpiece_coarse.json
```

### 两种运行模式（当前主线）

```text
效率模式  scripts/teaching/teaching_debug.py     ← 正在开发（视觉算法 / 力控实验）
示教模式  scripts/resume_teaching.sh（安全模式）   ← 已冻结，只维护不再开发
两者跑的是同一条示教链路（scripts/teaching/teaching_sequence.py），效率模式只是它的调试外壳。
```

```bash
# 效率模式：可分阶段、可改速度、可跳静置/门检，每次运行自动归档
python3 scripts/teaching/teaching_debug.py --stage 5            # 只跑预抓取
python3 scripts/teaching/teaching_debug.py --from 12 --to 13    # 旋转 + 下降
python3 scripts/teaching/teaching_debug.py --stage 14 --repeat 3

# 示教模式（冻结）：门检全开、静置、位姿库、失败自动重启并续跑
bash scripts/resume_teaching.sh
```

> **六模式（load/unload/full × safe/efficient）已按 ADR-005 归类**：源码
> （`run_task.py` / `run_pick_and_place.py` / `cs625_ui.py`）冻结保留、不再作为日常入口。
> 面板「运行」按钮与部分效率流程会调用 `run_task.py --scope load --mode efficient`
> （复用其中 load+efficient 的配置与观测前缀），这不代表六模式矩阵恢复为入口。
> 该路径运行前必须有新鲜环境模型（`runtime/environment/<ts>/model.json`），过期或超差 → 拒绝执行。

### 周边工具

```bash
python3 scripts/cs625_panel.py                 # 操作面板（两标签页 + 相机 + 夹爪）
scripts/open_rviz.sh                           # RViz：机械臂 / TF / 点云 / 规划场景
python3 scripts/show_targets_in_scene.py       # 已知位姿的工件与插槽显示进 RViz
python3 scripts/camera_mode.py {status,fast,modeling,probe,high,stop}
```

### VS Code 任务（8 个）

```text
① 开工 · 真机        驱动栈 + MoveIt + 四项门
② 安全模式 · 示教    scripts/resume_teaching.sh
③ 效率模式 · 调试    scripts/teaching/teaching_debug.py
④ 面板 · 两模式      scripts/cs625_panel.py
⑤ 自检 · 离线        pytest + check_docs + 环境校验
⑥ DSH · 启动/打开    打开正在运行的 DSH Web；没有才后台启动
⑦ 相机 · 查看/换档   scripts/camera_view.py（只碰相机）
⑧ 收工 · 关闭驱动    scripts/stop_all_drivers.sh
```

---

## 3. 目录与文档

```text
src/            12 个 ROS 2 包（接口/描述/传感/感知/观测/规划/执行/实验/仿真/bringup/面板）
scripts/        入口脚本；示教链路在 scripts/teaching/
config/         任务常量（插入权威位姿、抓取几何、RVS 视图）、RViz 配置
docs/           文档（入口见下）
runtime/        运行产物（环境模型、采集、位姿库、debug_runs）——不入版本控制
```

文档入口：[`docs/README.md`](docs/README.md)

```text
必读规则        docs/agent_memory.md
真机经验 29 条  docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md
当前架构与链路  docs/reference/architecture_current.md
指标体系 18 项  docs/reference/experiment_metrics.md
真机参考/排障   docs/reference/real_cell.md
仿真保真度      docs/reference/simulation.md
视觉研究交接    docs/worklogs/2026-11/2026-11-13_HANDOFF_VISION_MILESTONE.md
最近工作日志    docs/worklogs/2026-11/2026-11-13_WORKLOG_P7_ARCHIVE_AND_PLAN_A.md
```

### 分支与提交（2026-11-13 起）

- **`main`**：唯一活跃分支，此后所有提交直接进 `main`。
- **`teaching-rvs-mode`**：已冻结。它只服务于 2026-10 前后"示教模式 + RVS 视角"
  更新阶段；远程该分支保持原样，不再向其提交或推送。

---

## 4. 几条硬规则（摘自 `docs/agent_memory.md`）

```text
· 插入权威 = runtime/slot_reference_verified.json（当天 R1 与它差 33 mm，R1 只作对比、不能用来算目标）
· 保护停止后不自动重试；连续两次 NO_MOTION 才允许一次驱动栈自动重启
· 工件附近禁止横向移动，必须先竖直抬到 SAFE_HEIGHT_Z = 0.53 m 或沿工具轴后撤
· 持件巡航硬上限 0.013 m/s（示教链路 CARRY_SPEED_CAP）；释放必须显式 --allow-release
· 相机与 RVS 互斥；反控端口 50001/50003/50004 由机器人主动拨入
· 实验缺的通道写"未采集"，绝不写 0
· 大幅改构型前必须打印 6 个关节角并经所有者同意（agent_memory 第 133 条，2026-11-13）
```

---

## 5. 当前重心（2026-11-13 起）

```text
1. 修规划场景：让桌子/夹具对 MoveIt 可见（自动运动的前置条件）
2. 方案 A 真机验收：选站 → 换位 → 带先验精测，整链跑通
3. 效率模式的抓取 / 插入真机验证
4. 六维力 12 月到位后：三维模型 / URDF / TCP 更新 + 力控实验数据
```

---

## 6. 许可与来源

```text
· 官方 Elite CS625 驱动 / CS SDK / MoveIt 2 / ros2_control / 相机驱动均为上游依赖，不原地修改
· 师兄的自动化栈冻结在 ~/cs625_legacy_underlay_jazzy/（只读），见 docs/decisions/ADR-004
· 旧文档与旧仿真资产保留在 docs/archive/ 与 ~/cs625_legacy_sim_snapshot_20261024/
· P7 旧仿真流水线（代码 + 运行证据）2026-11-13 归档于 docs/archive/2026-11-13-p7-nbv-legacy-code/
```
