# 2026-11-10 WORKLOG D1 数据入口：vision_capture.py 与首次干跑

> 相关：`docs/plans/2026-11.md` D1；`docs/manuals/debug/camera.md`
> 真机使用：相机（真机，已启动）。**未发机械臂指令、未启动夹爪、未发生任何运动。**
> 仿真使用：无。

## 1. 本轮要解决的问题

D1 的第一件交付：把"相机给的数据"按固定格式、可复现地存下来，并当场给出重复性——
后面粗测量/精配准的每个结论都建立在这批数据上。

## 2. 新增文件

```text
A scripts/vision_capture.py   按组采 RGB-D → 落盘 depth.npy / cloud.npy / color.png / meta.json
                              → 组内逐像素时间重复性 + 汇总 summary/metrics
```

- 订阅用 **reliable QoS**（大图 best-effort 会被静默丢弃，昨天踩过）。
- 只收新帧（按 `header.stamp` 去重）。
- 不动机械臂；组间停顿留给人工摆放工件。
- 每次运行的 csv/summary 带 tag 后缀，同一 `--out` 下可以采多个朝向而不互相覆盖。

## 3. 首次干跑（当前姿态，3 组 × 5 帧）

```bash
python3 scripts/vision_capture.py --groups 3 --frames 5 --tag dryrun --delay 1.0
```

```text
[00] 有效  76.0%  中位 1.355 m  点距 1.491 mm  重复性 1.497 mm
[01] 有效  76.1%  中位 1.355 m  点距 1.488 mm  重复性 1.497 mm
[02] 有效  76.0%  中位 1.355 m  点距 1.493 mm  重复性 1.549 mm
组内重复性中位数 1.497 mm（验收 ≤ 2 mm）
```

单组 meta 关键字段：`valid_ratio 0.7604 / median_m 1.355 / repeatability_mm 1.497 /
repeat_p95_mm 5.528 / spacing_mm 1.491 / flying 0.0305 / hole_ratio 0.2358`。

结论：**当前姿态满足 D1 验收线**（重复性 1.5 mm ≤ 2 mm）。p95 5.5 mm 说明边缘/背景处抖动偏大，
统计时应按工件区域取，而不是整幅图。

## 4. 现场状态（开工前检查）

```text
相机：在线。pid 166428，modeling 1280×960，发布者 1，0.8 fps
机械臂 ROS 驱动栈：不在线（ros2 node list 只有 /camera/camera、/camera/camera_container）
  → 现在读不到 TCP/TF、也不能规划移动；若只需"停着采"不受影响
夹爪：未启动
```

## 5. 观测位判断

当前相机姿态的画面：工件斜视占画面约 35–40%，扶手可见，深度有效 76%、点距 1.49 mm。
与 `config/tasks/common.yaml` 里 `observe.workpiece_fine`（az≈25°、el≈30°、投影面积 37%）的
描述一致 → **当前姿态可以直接当精观测位用**，不需要先动机械臂。

## 6. 下一步

5 个朝向 × 4 组（每组 5 帧），每次用一个 tag：

```bash
OUT=runtime/vision_dataset/$(date +%Y%m%d_%H%M)_orient5
python3 scripts/vision_capture.py --groups 4 --frames 5 --tag rot0   --out $OUT --delay 2
# 工件转 +30° 后重复 --tag rotp30；+60° → rotp60；+90° → rotp90；-30° → rotm30
```

## 7. 11:3x 开工检查（失败）+ 三维度问题回答

### 7.1 机械臂驱动栈仍未启动

```text
ros2 node list          → 只有 /camera/* 与 /cs625_camera_view、/control_health_activate
ros2 topic list         → 没有 /joint_states、没有 /tcp_pose_broadcaster/pose
pgrep cs625_bringup     → 无进程
runtime/logs/driver_*.log → 不存在（说明 launch 根本没跑起来）
python3 scripts/control_health.py → sockets False / controller False / joint_states False
ping 192.168.1.200      → 通（0.8 ms）
```

结论：机器人网络可达，但驱动栈没在跑 → **六轴位姿记录做不了**（`scripts/record_pose.py` 已就绪，
驱动起来一条命令即可写入 `docs/reference/observation_poses_recorded.yaml`）。

### 7.2 粗精配准开发的前置条件核对

```text
数据：runtime/vision_dataset/20261110_1115_orient5（20 组，重复性 1.60 mm）  ✅
内参：/camera/color/camera_info = config/real/camera_intrinsics.json        ✅
模板：~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL（13.8 MB）      ✅
库：Open3D 0.20.0（含 GICP）+ scipy 1.11.4                                   ✅
手眼：不需要（本月全部在相机系内验收）                                       ✅
→ 可以开始 D2（粗测量）
```

### 7.3 六模式偏差的历史根因（按量级排序，全部有文档出处）

| 量级 | 原因 | 出处 |
|---|---|---|
| **353 mm** | RVS 的 T1/T3 是**法兰**位姿，我们发了**工具 TCP**（差 -46,0,353 mm） | `2026-10-20_WORKLOG_P4_REALTIME_TCP_POSE.md` §16 |
| **~160 / 106 mm** | RVS 返回**几何中心**，而 `T_b_tcp` 按 box.ply **模型原点**写；中心偏移 (10.5,-215,-105.2) mm | 同上 §15 |
| **~100 mm** | ICP"稳定但错"：近正方形面四重对称，自洽的错误解 | `docs/agent_memory.md`（稳定性≠正确性） |
| **35 mm** | 复用 74 小时前的旧 R1 插槽位姿 → 2026-11-03 三次碰撞 | `2026-11-03_WORKLOG_COLLISION_INCIDENT_AND_MOTION_GATES.md` |
| **33 mm** | RVS R1 本身的跨会话漂移（系统偏置，单次测量不可靠） | 同上 / 交接文档 |
| **20 mm** | 规划终点关节容差 0.02 rad ≈ 1.15°（0.6 m 力臂） | `config/tasks/common.yaml` 注释 |
| **13 mm** | 示教与六模式两套插入常量并存、会漂移 | `2026-11-02_WORKLOG_IMPROVED_AND_REMAINING.md` #16 |
| **毫米级** | 附着碰撞体用简化 `box_collision.STL`（无扶手），低估真实外廓 | `docs/agent_memory.md` |

**共同模式：几乎全部是坐标口径 / 参考系 / 旧常量复用，而不是算法精度不足。**
这也是"总是偏"的原因——每一处口径错误都是几十到几百毫米的**系统偏差**，
不消除就一直在，且会被下一个环节继承。
