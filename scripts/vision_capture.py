#!/usr/bin/env python3
"""D1 数据入口：在固定观测位按组采集工件 RGB-D，落盘并给出重复性统计。

为什么单独一个脚本
------------------
D0 普查已经确定了默认档（modeling，1280×960，深度 0.8 fps、有效点约 76%）。
这一步只做一件事：把"相机给的数据"按固定格式、可复现地存下来，并当场算出
重复性——后面粗测量/精配准的每一个结论都建立在这批数据上。

它不动机械臂、不碰夹爪：机械臂停在观测位，人负责转动/摆放工件，脚本只管采。

落盘格式（每个组一个目录）
--------------------------
    runtime/vision_dataset/<时间戳>/
        summary.md          人读的结论表
        metrics.csv         机读的指标
        g00/color.png       一组里的第一帧彩色
        g00/depth.npy       N 帧的中位深度（float32，米）
        g00/cloud.npy       由中位深度反投影的点云（N×3，相机光学系，米）
        g00/meta.json       内参、帧数、逐帧统计、时间戳
        g01/ … gNN/

重复性怎么算
------------
同一组内 N 帧逐像素比较：只在"每帧都有效"的像素上求时间方向的标准差，取中位数。
这个数就是"相机停着不动时，深度读数抖多少毫米"——D1 的验收线是 ≤ 2 mm。
"""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import polarizer_ab as pab                    # 复用反投影/点距/空洞率，避免两套口径

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO / "runtime" / "vision_dataset"


def grab_frames(count: int, timeout: float) -> dict:
    """采 count 帧（彩色 + 深度 + 内参）；只收新帧，按 header 时间去重。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=None)
    node = rclpy.create_node("vision_capture")
    qos = QoSProfile(depth=3, history=HistoryPolicy.KEEP_LAST)
    qos.reliability = ReliabilityPolicy.RELIABLE      # 大图必须 reliable，否则被静默丢弃

    state: dict = {"depth": [], "colour": [], "stamps": [], "info": None, "encoding": ""}
    seen: set[int] = set()

    def on_info(message):
        if state["info"] is None:
            state["info"] = {"k": list(message.k), "width": message.width, "height": message.height}

    def on_depth(message):
        key = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if key in seen:
            return
        seen.add(key)
        state["encoding"] = message.encoding
        if message.encoding == "16UC1":
            depth = np.frombuffer(bytes(message.data), dtype=np.uint16).reshape(
                message.height, message.width).astype(np.float32) / 1000.0
        elif message.encoding == "32FC1":
            depth = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
                message.height, message.width).copy()
        else:
            return
        state["depth"].append(depth)
        state["stamps"].append(float(message.header.stamp.sec)
                               + float(message.header.stamp.nanosec) * 1e-9)

    def on_colour(message):
        if len(state["colour"]) >= 1:
            return
        try:
            state["colour"].append(np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
                message.height, message.width, 3).copy())
        except ValueError:
            pass

    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
    node.create_subscription(Image, "/camera/color/image_raw", on_colour, qos)
    node.create_subscription(CameraInfo, "/camera/color/camera_info", on_info, qos)

    deadline = time.time() + timeout
    while time.time() < deadline and len(state["depth"]) < count:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return state


def group_metrics(frames: list[np.ndarray], k: list[float]) -> dict:
    """一组的统计：有效点、中位深度、点距、空洞率，以及逐像素时间重复性。"""
    stack = np.stack(frames)
    valid = stack > 0
    always = valid.all(axis=0)
    reference = np.median(stack, axis=0)
    result = {
        "frames": len(frames),
        "valid_ratio": round(float(valid.mean()), 4),
        "median_m": round(float(np.median(reference[reference > 0])), 4) if (reference > 0).any() else None,
    }
    if always.any():
        spread = stack[:, always].std(axis=0)          # 逐像素时间标准差（米）
        result["repeatability_mm"] = round(float(np.median(spread) * 1000.0), 3)
        result["repeat_p95_mm"] = round(float(np.percentile(spread, 95) * 1000.0), 3)
    points = pab.back_project(reference, k)
    if len(points) > 1000:
        result["spacing_mm"] = round(pab.spacing_mm(points), 3)
        result["flying"] = round(pab.flying_stats(points)[0], 4)
    result["hole_ratio"] = round(pab.hole_ratio(reference, k, None), 4)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--groups", type=int, default=20)
    parser.add_argument("--frames", type=int, default=5, help="每组采几帧（用于组内重复性）")
    parser.add_argument("--tag", default="workpiece", help="组名前缀，例如朝向标签")
    parser.add_argument("--out", default=None)
    parser.add_argument("--delay", type=float, default=2.0, help="组间停顿，留给人摆放工件")
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    root = pathlib.Path(args.out) if args.out else DEFAULT_OUT / stamp
    root.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for index in range(args.groups):
        state = grab_frames(args.frames, args.timeout)
        if not state["depth"] or state["info"] is None:
            print(f"[{index:02d}] 没收到深度或内参（相机没起 / QoS 不匹配），停止")
            break
        metrics = group_metrics(state["depth"], state["info"]["k"])
        directory = root / f"g{index:02d}_{args.tag}"
        directory.mkdir(exist_ok=True)
        reference = np.median(np.stack(state["depth"]), axis=0).astype(np.float32)
        np.save(directory / "depth.npy", reference)
        points = pab.back_project(reference, state["info"]["k"])
        np.save(directory / "cloud.npy", points.astype(np.float32))
        if state["colour"]:
            import cv2
            cv2.imwrite(str(directory / "color.png"), state["colour"][0])
        meta = dict(metrics)
        meta.update(camera_info=state["info"], encoding=state["encoding"],
                    stamps=state["stamps"], captured_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
        (directory / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
        row = {"group": directory.name, **metrics}
        rows.append(row)
        print(f"[{index:02d}] 有效 {metrics['valid_ratio'] * 100:5.1f}%  "
              f"中位 {metrics['median_m']} m  点距 {metrics.get('spacing_mm')} mm  "
              f"重复性 {metrics.get('repeatability_mm')} mm", flush=True)
        if index + 1 < args.groups and args.delay > 0:
            time.sleep(args.delay)

    if not rows:
        return 1
    with (root / f"metrics_{args.tag}.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    repeats = [row.get("repeatability_mm") for row in rows if row.get("repeatability_mm")]
    lines = ["# 视觉数据集采集（D1）", "",
             f"采集时间：{time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"组数：{len(rows)}，每组 {args.frames} 帧，标签：{args.tag}", "",
             "| 组 | 有效点 | 中位深度 | 点距 | 重复性(中位) | 重复性(p95) |",
             "|---|---|---|---|---|---|"]
    for row in rows:
        lines.append(f"| {row['group']} | {row['valid_ratio'] * 100:.1f}% | {row['median_m']} m | "
                     f"{row.get('spacing_mm')} mm | {row.get('repeatability_mm')} mm | "
                     f"{row.get('repeat_p95_mm')} mm |")
    if repeats:
        lines += ["", f"组内重复性中位数：**{np.median(repeats):.3f} mm**（验收线 ≤ 2 mm）"]
    lines.append("")
    (root / f"summary_{args.tag}.md").write_text("\n".join(lines))
    print(f"\n产物：{root}")
    if repeats:
        print(f"组内重复性中位数 {np.median(repeats):.3f} mm（验收 ≤ 2 mm）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
