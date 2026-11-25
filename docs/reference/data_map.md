# 数据地图：我们到底有哪些数据、放在哪里、怎么读

> 建立：2026-11-25（所有者："我对能拿到什么数据、放在哪里完全没有掌控感"）。
> **每次开工/每次实验后都应该先看这一页**，不要凭记忆猜。
> 一条命令看清现状：`python3 scripts/data_report.py`

## 0. 一句话总览

```text
每次真机运行（run_task.py --confirm / teaching_debug.py）
  → 实时写入 runtime/experiments/<id>/      （原始，20 Hz）
  → 运行结束自动裁剪并归档到 ~/cs625_experiment_archive/<id>/（长期保存）
  → 一行记录追加到 docs/evidence/experiment_index.md

每次视觉测量（measure_object.py）
  → runtime/measurements/<名字>.json        （位姿/覆盖率/重复性/相机位姿）
  → runtime/vision_dataset/measure_<stamp>/ （点云/深度/彩色原始数据）

每次环境建模（model_environment.py）
  → runtime/environment/<时间戳>/model.json + scene.ply
```

## 1. 数据清单（位置 · 格式 · 频率 · 怎么读）

| 数据 | 位置 | 格式 | 频率/时机 | 怎么读 |
|---|---|---|---|---|
| **关节位置** | 归档 `timeseries.csv` 的 `joint_pos_1..6` | CSV（弧度） | 20 Hz，真机运行时 | `awk -F, 'NR>1{print $3}'` 或 pandas |
| **关节速度** | 同上 `joint_vel_1..6` | CSV（rad/s） | 20 Hz | 同上 |
| **关节力矩（effort）** | 同上 `joint_effort_1..6` | CSV（N·m） | 20 Hz | 同上；**这是驱动器由电流反推的力矩**，不是六维力传感器 |
| **TCP 位姿** | 同上 `tcp_x/y/z`、`tcp_qx/qy/qz/qw` | CSV | 20 Hz | 同上 |
| **六维力/力矩** | 同上 `fx fy fz tx ty tz` | CSV（N / N·m） | 20 Hz | **列已存在但当前为空**；接入 KWR75 后自动填充（见手册第 70 条） |
| **阶段标记** | 同目录 `phases.json` | JSON | 每阶段进入/退出 | `python3 -m json.tool phases.json` |
| **相机帧（过程图）** | 同目录 `images/` + `images.jsonl` | PNG + JSONL | 每阶段 1–6 张 | 直接看图；`images.jsonl` 有文件名↔阶段对应 |
| **实验摘要** | 同目录 `summary.md` / `README.md` | Markdown | 归档时生成 | 直接看 |
| **实验索引** | `docs/evidence/experiment_index.md` | Markdown 表格 | 每次归档 1 行 | 直接看，用来找"哪天那次实验" |
| **测量结果（工件/插槽）** | `runtime/measurements/<名字>.json` | JSON | 每次 `measure_object.py` | 见 §2 字段说明 |
| **原始点云/深度/彩色** | `runtime/vision_dataset/measure_<时间戳>/g00_m/` | `cloud.npy`（N×3 点云）、`depth.npy`（米，float32）、`color.png`、`meta.json`、`metadata.jsonl`（含 T_base_tcp，供环境建模） | 每次采集（每组 2–3 帧） | `np.load('cloud.npy')` |
| **环境模型** | `runtime/environment/<时间戳>/model.json` + `scene.ply` | JSON + PLY | 每次 `model_environment.py` | `model.json` 里有桌面高度/足迹/碰撞体素/slot/workpiece |
| **手眼标定** | `config/handeye/tcp_to_camera.json` | JSON（4×4） | 标定变更时 | `T_base_cam = T_base_tcp @ T_tcp_cam` |
| **插入标定（新）** | `runtime/insert_calibration.json` | JSON | 手动标定时 | 手册第 69 条 |
| **夹爪状态** | 不在归档里（Windows 串口） | — | 查询时 | `bash scripts/gripper_link.sh --status` |
| **运行日志** | `runtime/logs/<名字>.log` | 文本 | 每次运行 | 直接看 |
| **六维力实时话题** | `/ft_sensor/wrench` | `WrenchStamped`（N/N·m） | 1 kHz（接入后） | `ros2 topic echo /ft_sensor/wrench` |
| **关节/ TCP 实时话题** | `/joint_states`、`/tcp_pose_broadcaster/pose` | ROS 消息 | 实时 | `ros2 topic hz/eceho` |

## 2. 测量 JSON 的字段（`runtime/measurements/*.json`）

```text
stamp              测量时间戳
dataset            原始数据目录（**注意：目录名与 stamp 不一定相同**，找数据一律用它）
verdict            OK / REFUSE（REFUSE 时下游不得使用）
reasons            REFUSE 的原因列表（覆盖低/重复性差/…）
coverage_median    覆盖率中位数（工件门槛 0.95，插槽判据不同）
repeatability      {translation_mm, rotation_deg} 组间重复性
workpiece_yaw_deg  工件偏转角（选站用）
pose_base_centre_mm 工件中心（base 系，mm）
T_base_cam         采集时相机位姿（4×4）；没有它无法离线复算
per_group[].pose_base.translation   **CAD 模板原点**在 base 系的位置（位姿推导用这个）
per_group[].pose_base.centre_mm     CAD 包围盒中心（只用于碰撞体/范围，不用于位姿推导）
```

## 3. 三条"你没注意到但其实一直有"的事实

```text
① 关节力矩一直在采：`joint_effort_1..6` 每行都有值（来自驱动器电流反推）。
   它没有单独的 effort.csv——就在归档的 timeseries.csv 里。
② 六维力的列早就预留好了：`fx fy fz tx ty tz`，当前 100% 为空；
   传感器接上后由 experiment_recorder 自动填充，不用改表结构。
③ 点云不只在测量 JSON 里有：原始 `cloud.npy`/`depth.npy`/`color.png` 都在
   runtime/vision_dataset/ 下，可以重新做算法、重新出图。
```

## 4. 出图/分析的现成入口

```bash
# 一条命令看清"现在有哪些数据、多新、多大"
python3 scripts/data_report.py

# 看某次归档实验
ls ~/cs625_experiment_archive/20261125_203759_real_load_efficient/
python3 -m json.tool ~/cs625_experiment_archive/<id>/phases.json | head -40

# 直接画力矩曲线（20 Hz）
python3 - <<'PY'
import pandas as pd, matplotlib.pyplot as plt
d = pd.read_csv("~/cs625_experiment_archive/<id>/timeseries.csv".replace("~", __import__("os").path.expanduser("~")))
d[["joint_effort_1","joint_effort_2","joint_effort_3"]].plot(); plt.show()
PY

# 看原始点云
python3 -c "import numpy as np, open3d as o3d; p=o3d.geometry.PointCloud(); \
p.points=o3d.utility.Vector3dVector(np.load('runtime/vision_dataset/<stamp>/g00_m/cloud.npy')[:,:3]); \
o3d.visualization.draw_geometries([p])"
```

## 5. 面板（4K）为什么"看不到数据"

```text
面板的"视觉实验数据"卡片只读两个文件：
    runtime/measurements/workpiece_latest.json
    runtime/measurements/slot_latest.json
它**不读** timeseries、不读点云、不读环境模型。所以：
  · 没跑过对应测量 → 显示"—"；
  · 跑了别的名字的测量（如 eff4_coarse.json）→ 卡片仍是空的。
要"看到全部数据"，用 `python3 scripts/data_report.py`（覆盖上面所有数据项）。
```

## 6. 每次实验后的自检（30 秒）

```text
1. python3 scripts/data_report.py            # 数据在不在、多新
2. 打开 ~/cs625_experiment_archive/<最新>/summary.md
3. 确认 timeseries.csv 行数 > 0、images/ 有图、phases.json 有本次阶段
4. 需要论文图时：从 timeseries.csv（力矩/位置/时间）+ images/ 直接出
```
