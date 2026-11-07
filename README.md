# CS625 视触融合精密装配 / 遮挡目标定位与可达性观测规划

> 课题：**面向遮挡目标定位的机械臂可达性规划方法研究**（毕业论文主线）
> 真机：Elite CS625 六轴协作臂 + 平行夹爪 + Percipio PS800 RGB-D + 插槽夹具 + 弹仓工件
> 环境：WSL2 Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic
> 更新：**2026-11-08**（上一次 README 停留在 2026-10-25 的六模式阶段，已失效）

本仓库做一件事：**让机械臂先看清楚再动手** —— 在遮挡/不确定条件下主动选择观测位姿把目标定位到
可抓取/可装配的精度，再把它交给一个可复现、可测量的装配流程。

---

## 1. 现在真实的状态（2026-11-08）

| 能力 | 状态 | 证据 |
|---|---|---|
| 观测 → 抓取 → 抬升 → 转运 → 分级 → 旋转 → 预插入 | ✅ **真机整链跑通** | `docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md` |
| 插入到底 | 🚧 未完成（372 mm 不是物理底部，工件与槽三面接触） | 同上 第 4/5 节 |
| 槽内转 10° 转平 | 🚧 未在真机跑通（源码依据已确认） | `scripts/teaching/straighten_in_slot.py` |
| 六维力传感与力控 | ⛔ 传感器 12 月到位，指标全部记"未采集" | `docs/reference/experiment_metrics.md` |
| 自研视觉管线 | 🚧 **十一月主线**，RVS 目前只作一次性位姿采集 | `docs/worklogs/2026-11/2026-11-09_HANDOFF_NEXT_SESSION_VISION_ALGORITHM.md` |
| 两模式运行（安全 / 效率） | ✅ 已收敛，六模式源码冻结保留 | `docs/decisions/ADR-005-two-modes-and-insert-authority.md` |
| 操作面板（PyQt5） | ✅ 阶段/遥测/采集/相机 1:1/夹爪/RViz 状态卡 | `scripts/cs625_panel.py` |
| RViz 3D（机械臂/点云/规划场景） | ✅ 复用 MoveIt 参数启动 | `scripts/open_rviz.sh` |
| 实验指标与归档 | ✅ 六类 18 项字段，`runtime/debug_runs/` | `scripts/teaching/teaching_debug.py` |

**一句话**：装入流程跑到预插入，插入段与视觉算法是接下来两个月的主战场。

---

## 2. 三分钟上手

```bash
cd ~/projects/cs625-active-perception

# ① 唯一支持的环境入口（每个 shell 必做）
source scripts/source_dev_env.sh --full

# ② 离线自检（不碰机器人）
python3 -m pytest test/ -q            # 293 passed
python3 scripts/check_docs.py         # 文档一致性

# ③ 动臂前的四项门
python3 scripts/control_health.py
python3 scripts/preflight_motion.py
```

### 两种模式（唯一入口）

```bash
# 安全模式：门检全开、静置、fitness、位姿库；失败自动重启驱动栈并续跑
bash scripts/resume_teaching.sh

# 效率模式：分阶段执行、可改速度、可跳静置/门检，每次运行自动归档
python3 scripts/teaching/teaching_debug.py --stage 5            # 只跑预抓取
python3 scripts/teaching/teaching_debug.py --from 12 --to 13    # 旋转 + 下降
python3 scripts/teaching/teaching_debug.py --stage 14 --repeat 3
```

### 周边工具

```bash
python3 scripts/cs625_panel.py                 # 操作面板（两标签页 + 相机 + 夹爪）
scripts/open_rviz.sh                           # RViz：机械臂 / TF / 点云 / 规划场景
python3 scripts/show_targets_in_scene.py       # 已知位姿的工件与插槽显示进 RViz
python3 scripts/camera_mode.py {status,fast,modeling,high,stop}
```

### VS Code 任务（只有 5 个）

```text
① 开工 · 真机      驱动栈 + MoveIt + 四项门
② 安全模式 · 示教  scripts/resume_teaching.sh
③ 效率模式 · 调试  scripts/teaching/teaching_debug.py
④ 面板 · 两模式    scripts/cs625_panel.py
⑤ 自检 · 离线      pytest + check_docs + 环境校验
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
视觉研究交接    docs/worklogs/2026-11/2026-11-09_HANDOFF_NEXT_SESSION_VISION_ALGORITHM.md
最近工作日志    docs/worklogs/2026-11/2026-11-07_WORKLOG_MODES_PANEL_DOCS_REORG.md
```

---

## 4. 几条硬规则（摘自 `docs/agent_memory.md`）

```text
· 插入权威 = runtime/slot_reference_verified.json（当天 R1 实测差 33 mm，不能用）
· 保护停止后不自动重试；连续两次 NO_MOTION 才允许一次驱动栈自动重启
· 工件附近禁止横向移动，必须先竖直抬升 20 mm
· 持件巡航速度硬上限 0.013 m/s；释放必须显式 --allow-release
· 相机与 RVS 互斥；反控端口 50001/50003/50004 由机器人主动拨入
· 实验缺的通道写"未采集"，绝不写 0
```

---

## 5. 十一月计划（当前重心）

```text
主线：自研视觉算法 + 仿真平台搭建（配合论文模块 2 主动视觉、模块 1 力控的前期准备）
副线：专利与论文投递、六维力安装后的三维模型/URDF/TCP 更新、纯视觉实验填充数据

视觉第一步见交接文档：停 RVS → 采 20 组静态点云 → 量化帧率/点距/反射 →
出"帧数 vs 精度 vs 耗时"曲线；运行期继续用固化位姿 + ROS 高帧率彩色流。
```

---

## 6. 许可与来源

```text
· 官方 Elite CS625 驱动 / CS SDK / MoveIt 2 / ros2_control / 相机驱动均为上游依赖，不原地修改
· 师兄的自动化栈冻结在 ~/cs625_legacy_underlay_jazzy/（只读），见 docs/decisions/ADR-004
· 旧文档与旧仿真资产保留在 docs/archive/ 与 ~/cs625_legacy_sim_snapshot_20261024/
```
