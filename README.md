# CS625 主动感知 / 遮挡目标定位与可达性观测规划

> 课题：**面向遮挡目标定位的机械臂可达性规划方法研究**
> 真机：Elite CS625 六轴协作臂 + HITBOT 夹爪 + Percipio RGB-D + 插槽夹具 + 弹仓工件
> 环境：WSL2 Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic

本仓库只做一件事：**让机械臂"先看清楚再动手"**——在遮挡/不确定的条件下，通过主动选择观测位姿，
把目标定位到可以抓取/装配的精度，然后把定位结果交给一个可复现的装配流程。

---

## 1. 现在能做什么（2026-10-24 状态）

| 能力 | 状态 | 证据 |
|---|---|---|
| 精观测位定位 | ✅ 3 帧一致，位置离散 **1.0 mm**、转角 0.285° | `docs/worklogs/2026-10-24_WORKLOG_EXECUTION_ESTOP_ROOT_CAUSE.md` |
| 六类任务模式（3 scope × 2 mode） | ✅ 只规划全流程通过：load 19/19、unload 38/38、full 57/57 段 | `runtime/planonly_*.json` |
| 强制环境建模 + 门禁 | ✅ 四种拒绝（插槽漂移/工件漂移/点云不足/模型超龄） | `scripts/model_environment.py` |
| 仿真与真机 1:1 场景 | 🚧 桌面/工件已按实测对齐；夹具几何按真机数据重建中 | `scripts/build_sim_world.py` |
| 双 UI（Qt 面板 + RViz 面板） | ✅ 六模式按钮 + 日志 + 状态 | `scripts/cs625_ui.py`、`src/cs625_rviz_panel` |
| 实验数据自动采集/裁剪/归档 | ✅ 20 Hz 时序 + 阶段标记 + 过程图 → `~/cs625_experiment_archive` | `scripts/experiment_recorder.py` |
| 真机自动执行 | ⛔ 待真机恢复（2026-10-24 机器人网络不可达） | — |

---

## 2. 三分钟上手

```bash
cd ~/projects/cs625-active-perception

# ① 唯一支持的环境入口（每个 shell 必做）
source scripts/source_dev_env.sh --full

# ② 环境校验 + 全部单测（约 15 s）
source scripts/source_dev_env.sh --verify
python3 -m pytest test/ -q

# ③ 建立/检查环境模型（每次任务前必须）
python3 scripts/model_environment.py --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw \
    --out runtime/environment
python3 scripts/model_environment.py --out runtime/environment --verify-only

# ④ 六类模式：预演（打印阶段表与速度，不动臂）
python3 scripts/run_task.py --scope load --mode safe --world real --dry-run

# ⑤ 只规划（不动臂，可串起整条流程验证可达性）
python3 scripts/run_task.py --scope full --mode efficient --plan-only --handover auto

# ⑥ 仿真里跑（需先起 sim_cell；不动真机）
ros2 launch cs625_bringup sim_cell.launch.py
python3 scripts/run_task.py --scope load --mode safe --world sim --confirm

# ⑦ 真机执行（确认急停可达、夹爪 init=5 之后）
python3 scripts/run_task.py --scope load --mode safe --world real --confirm --allow-release

# ⑧ 图形界面（二选一或都开）
python3 scripts/cs625_ui.py                                   # Qt 面板
rviz2 -d src/cs625_rviz_panel/config/cs625_task.rviz          # RViz 面板
```

VS Code 里用 `.vscode/tasks.json` 的一键任务（编号 00–57）做同样的事，不用记命令。

---

## 3. 六类任务模式

```text
scope=load    抓取工件 → 装入插槽          scope=unload  从插槽取出 → 放回工件桌
scope=full    环境建模 → 装入 → 取出 → 回停车位

mode=safe      分段多、dwell 1 s、time_scale 2.0、每阶段实测验收
mode=efficient 单段执行、无 dwell、巡航 0.12 m/s；槽内紧间隙仍 0.005 m/s
```

每条流程都自带**观测前缀**（工件粗/精观测 → 两张桌子扫描 → 插槽粗/精观测），
观测结果写回环境模型，后续段复用；流程终点是下一段流程的起点（插槽粗观测 ↔ 工件粗观测交替）。

细节：`config/tasks/`（配置分层）、`docs/source_authority.md`（哪些数值可信）。

---

## 4. 仓库结构

```text
src/            12 个 ROS 2 包（接口 / 描述 / 传感器 / 感知 / 视角生成 / 评估 /
                运动 / 编排 / 实验 / 仿真 / bringup / RViz 面板）→ docs/project_layout.md
scripts/        任务入口 run_task.py + 阶段执行器 + 感知/标定/建模/记录脚本 → scripts/README.md
config/         tasks/（六类模式，唯一权威）· handeye/ · real/ · verified 基线
test/           离线单测（124+ 用例）与契约检查
docs/           架构、接口、权威性分级、工作日志；旧文档在 docs/archive/
runtime/        生成物（环境模型、实验原始数据、有效配置），不入库
```

---

## 5. 数据在哪

| 数据 | 位置 | 说明 |
|---|---|---|
| 环境模型 | `runtime/environment/<时间戳>/model.json` + `scene.ply` | 每次任务前重建；门禁不过不许动 |
| 实验原始数据 | `runtime/experiments/<id>/` | 20 Hz 时序 + 阶段 + 图像 |
| **实验归档（长期）** | `~/cs625_experiment_archive/<id>/` | 裁剪后 + README + summary，索引见 `docs/evidence/experiment_index.md` |
| 过程图（训练用） | `runtime/process_images/<run>/` | 每阶段进/出各一张，带 TCP 与时间 |
| YOLO 数据集/权重 | `~/cs625_yolo_seg/` | 两类分割模型，mask mAP50 0.861 |
| 标定 | `config/handeye/`、`config/real/camera_intrinsics.json` | 手眼外参、相机内参 |

---

## 6. 文档索引

| 想知道 | 看 |
|---|---|
| 系统怎么分层、包边界 | [`docs/architecture.md`](docs/architecture.md)、[`docs/project_layout.md`](docs/project_layout.md) |
| 话题/服务/动作契约 | [`docs/interfaces.md`](docs/interfaces.md) |
| 坐标系与 TF | [`docs/frames_and_topics.md`](docs/frames_and_topics.md) |
| 哪些数值可信（A/B/C 分级） | [`docs/source_authority.md`](docs/source_authority.md) |
| 仿真怎么起、和真机怎么对齐 | [`docs/simulation.md`](docs/simulation.md) |
| 依赖与第三方边界 | [`docs/dependencies.md`](docs/dependencies.md) |
| 真机安全与就绪度 | [`docs/real_hardware_readiness.md`](docs/real_hardware_readiness.md) |
| 每天做了什么、踩了什么坑 | [`docs/worklogs/`](docs/worklogs/)（按日期命名） |
| 长期记忆（位姿基准、教训） | [`docs/agent_memory.md`](docs/agent_memory.md) |
| 旧文档（2026-10 之前） | [`docs/archive/`](docs/archive/) |

---

## 7. 三条铁律

1. **真机安全**：`execute:=false` / `require_confirmation:=true` 是默认；动臂前必须
   `safety_mode=1` + `robot_mode=7` + `arm_controller` active + `control_health.py` 全 True。
   释放夹爪永远需要 `--allow-release`。
2. **观测先于运动**：每次流程前先建环境模型并过门禁；看不懂的位姿不许执行。
3. **源码不删**：旧仿真、旧文档、师兄代码一律只读保留（快照 + 索引），归档要所有者批准。

工作方式与详细规则见 [`AGENTS.md`](AGENTS.md)。
