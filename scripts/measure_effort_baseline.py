#!/usr/bin/env python3
"""Measure the joint-effort baseline for the compliant-insert contact detector.

专项 3 / A 期标定工具（2026-11-03）。师兄的接触检测用 Σ|τ|（关节力矩绝对值之和）
的残差（快滤波减慢基线），阈值 0.8/2.0 是他那套状态监控的单位；我们的信号源是
/joint_states 的 effort 字段，单位与噪声都需要在本机标定，不能照搬数值。

流程：
  1. 静止采样 N 秒（--still，默认 8）→ Σ|effort| 的 mean/std/峰峰
  2. 可选微动采样：写一段 J6 小幅三角轨迹（--wiggle-deg 默认 4°）交给
     execute_assembly_chunks 执行（time_scale 2 慢速），同时连续采样 →
     运动中的 Σ|effort| 统计与相对静止均值的最大残差
  3. 打印统计，并给出接触/安全阈值的【建议值】（需要所有者确认后才写入
     compliant_insert.py）

安全：微动只动 wrist_3，幅度默认 4°（TCP 端 ~24 mm），默认 --no-wiggle。

Usage:
    python3 scripts/measure_effort_baseline.py --still 8
    python3 scripts/measure_effort_baseline.py --still 8 --wiggle-deg 4
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def stats(samples: list[float], label: str) -> dict:
    arr = np.array(samples)
    result = {
        "label": label,
        "n": int(len(arr)),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "max": float(arr.max()),
        "p2p": float(arr.max() - arr.min()),
    }
    print(f"{label}: n={result['n']} mean={result['mean']:.3f} "
          f"std={result['std']:.3f} min={result['min']:.3f} max={result['max']:.3f} "
          f"peak-peak={result['p2p']:.3f}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--still", type=float, default=8.0,
                        help="stationary sampling duration in seconds")
    parser.add_argument("--wiggle-deg", type=float, default=0.0,
                        help="if > 0, run a slow wrist_3 wiggle of this amplitude "
                             "and sample during it")
    parser.add_argument("--rate-hz", type=float, default=125.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("measure_effort_baseline")
    samples: list[tuple[float, float]] = []

    def on_state(msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        eff = dict(zip(msg.name, msg.effort))
        if all(k in pos and k in eff for k in ORDER):
            samples.append((time.time(), float(sum(abs(eff[k]) for k in ORDER))))

    node.create_subscription(JointState, "/joint_states", on_state, 10)

    deadline = time.time() + args.still
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    still_samples = [v for _, v in samples]
    still = stats(still_samples, "still")
    samples.clear()

    if args.wiggle_deg <= 0:
        print("no wiggle requested; done")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # Build a small triangle trajectory on wrist_3 (the pose is safe: the tool
    # hangs above the tables and only the last joint moves).
    latest: dict = {}
    sub = node.create_subscription(JointState, "/joint_states",
                                   lambda m: latest.__setitem__("j", m), 10)
    wait_deadline = time.time() + 8.0
    while "j" not in latest and time.time() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "j" not in latest:
        print("no joint states; cannot build the wiggle")
        return 2
    pos = dict(zip(latest["j"].name, latest["j"].position))
    q0 = [pos[k] for k in ORDER]
    q1 = list(q0)
    q1[5] += np.radians(args.wiggle_deg)
    trajectory = {
        "joint_names": ORDER,
        "points": [
            {"q": q0, "t": 0.0},
            {"q": [(a + b) / 2 for a, b in zip(q0, q1)], "t": 1.5},
            {"q": q1, "t": 3.0},
        ],
    }
    path = "/tmp/effort_baseline_wiggle.json"
    json.dump(trajectory, open(path, "w"))
    print(f"wiggle: wrist_3 +{args.wiggle_deg:.1f} deg and back (time_scale 2)")
    done = subprocess.run(
        [sys.executable, "scripts/execute_assembly_chunks.py", path,
         "--time-scale", "2.0", "--goal-tolerance", "0.002",
         "--verify-final", "0.002", "--fault-poll", "0.2"],
        capture_output=True, text=True, timeout=600,
    )
    for line in (done.stdout or "").strip().splitlines()[-2:]:
        print("  exec:", line)
    # keep sampling 2 s after the executor returns so the record includes settle
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    motion_samples = [v for _, v in samples]
    motion = stats(motion_samples, "motion")

    above = [v - motion["mean"] for v in motion_samples]
    above = [v for v in above if v > 0]
    spread = max(motion["p2p"], (max(above) if above else 0.0))
    print(f"motion-internal spread (p2p vs mean): "
          f"max-above-mean={max(above) if above else 0:.3f} p2p={motion['p2p']:.3f}")
    print("\nproposed thresholds (OWNER CONFIRMATION REQUIRED):")
    print(f"  contact = still_mean + 5 * spread"
          f" = {still['mean'] + 5 * spread:.2f}")
    print(f"  safety  = still_mean + 12 * spread"
          f" = {still['mean'] + 12 * spread:.2f}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
