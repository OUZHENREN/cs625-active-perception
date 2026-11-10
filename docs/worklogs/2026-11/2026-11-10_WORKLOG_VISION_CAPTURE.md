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
