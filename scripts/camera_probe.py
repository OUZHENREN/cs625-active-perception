#!/usr/bin/env python3
"""相机实时探针：每 0.5 s 打印一行彩色/深度统计，用数字看相机在不在、挡哪个窗有用。

为什么不用肉眼看窗口
--------------------
0.8 fps 的画面每 1.25 s 才更新一次，肉眼很难判断"到底是没接收到新帧，还是画面本来就是静止的"。
这个探针把**每帧的到达情况**变成数字：

    时间  彩色帧数  彩色均值  深度帧数  有效点  中位深度
     3.0s       2     85.6        2   76.1%   1.352 m

· 数字一直在涨 → 数据在流；
· 挡住彩色镜头 → `彩色均值` 掉到接近 0；
· 挡住红外投射器/接收窗 → `有效点` 掉下去（彩色均值不变，因为那是另一条光路）；
· 完全不动 → `帧数` 不涨，说明订阅端没收到（QoS/graph 问题）。

用法
----
    source scripts/source_dev_env.sh --full
    python3 scripts/camera_probe.py --duration 60
    python3 scripts/camera_probe.py --duration 60 --out runtime/probe.csv
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import time

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--out", default=None, help="可选：把每次打印写进 CSV")
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init(args=None)
    node = rclpy.create_node("camera_probe")
    # 大图（3.7 MiB / 2.4 MiB）超过 Fast DDS 默认共享内存段，best-effort 会被静默丢弃
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST)
    qos.reliability = ReliabilityPolicy.RELIABLE

    state = {"c": 0, "d": 0, "cmean": float("nan"), "dvalid": float("nan"),
             "dmedian": float("nan"), "cstamp": 0.0, "dstamp": 0.0}

    def on_colour(message):
        try:
            pixels = np.frombuffer(bytes(message.data), dtype=np.uint8)
        except Exception:                       # noqa: BLE001
            return
        if pixels.size == 0:
            return
        state["c"] += 1
        state["cmean"] = float(pixels[::7].mean())
        state["cstamp"] = time.time()

    def on_depth(message):
        try:
            values = np.frombuffer(bytes(message.data), dtype=np.uint16)
        except Exception:                       # noqa: BLE001
            return
        if values.size == 0:
            return
        valid = values > 0
        state["d"] += 1
        state["dvalid"] = float(valid.mean()) * 100.0
        if valid.any():
            state["dmedian"] = float(np.median(values[valid])) / 1000.0
        state["dstamp"] = time.time()

    node.create_subscription(Image, "/camera/color/image_raw", on_colour, qos)
    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)

    rows: list[list] = []
    print(f"{'时间':>6} {'彩色帧':>6} {'彩色均值':>9} {'深度帧':>6} {'有效点':>7} {'中位深度':>9}   备注")
    started = time.time()
    last = 0.0
    try:
        while time.time() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last < args.period:
                continue
            last = now
            age_c = now - state["cstamp"] if state["cstamp"] else 999.0
            age_d = now - state["dstamp"] if state["dstamp"] else 999.0
            note = []
            if state["c"] == 0:
                note.append("彩色一帧没收到")
            if state["d"] == 0:
                note.append("深度一帧没收到")
            if 0 < age_c <= 3:
                note.append(f"彩色 {age_c:.1f}s 前")
            if 0 < age_d <= 3:
                note.append(f"深度 {age_d:.1f}s 前")
            line = (f"{now - started:6.1f} {state['c']:6d} {state['cmean']:9.1f} "
                    f"{state['d']:6d} {state['dvalid']:6.1f}% {state['dmedian']:8.3f}m   "
                    + " · ".join(note))
            print(line, flush=True)
            rows.append([round(now - started, 2), state["c"], round(state["cmean"], 2),
                         state["d"], round(state["dvalid"], 2), round(state["dmedian"], 4)])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.out:
        path = pathlib.Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s", "colour_frames", "colour_mean", "depth_frames",
                             "depth_valid_pct", "depth_median_m"])
            writer.writerows(rows)
        print(f"已写入 {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
