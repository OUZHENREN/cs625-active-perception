#!/usr/bin/env python3
"""Verify the measured TCP pose against a target and report the residual error.

Why this exists: a MoveIt pose goal only guarantees its constraint tolerances.
With the default 0.05 rad (~2.9 deg) orientation tolerance the planner reported
success while the workpiece was still visibly skewed.  Every assembly-relevant
motion must therefore be checked against the *measured* pose afterwards.

Usage:
  python3 scripts/verify_pose.py --xyz 1.0977 -0.0138 0.3758 \
      --quat -0.004 0.997 -0.001 0.083 --tol-pos 0.003 --tol-rot 0.01

Exit codes:
  0 within tolerance, 1 residual too large, 2 no pose received
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)


def measure(node: Node, timeout_sec: float = 4.0) -> tuple[np.ndarray, np.ndarray] | None:
    latest: dict = {}
    sub = node.create_subscription(
        PoseStamped, "/tcp_pose_broadcaster/pose",
        lambda msg: latest.__setitem__("m", msg), 10,
    )
    deadline = time.time() + timeout_sec
    while "m" not in latest and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    msg = latest.get("m")
    if msg is None:
        return None
    p = msg.pose.position
    q = msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xyz", nargs=3, type=float, required=True)
    parser.add_argument("--quat", nargs=4, type=float, required=True)
    parser.add_argument("--tol-pos", type=float, default=0.0005,
                        help="metres, default 0.5 mm")
    parser.add_argument("--tol-rot", type=float, default=0.002,
                        help="radians, default 0.002 rad (0.11 deg)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("verify_pose")
    try:
        measured = measure(node)
        if measured is None:
            print("NO_POSE")
            return 2
        xyz, quat = measured
        target_xyz = np.array(args.xyz, dtype=float)
        target_quat = np.array(args.quat, dtype=float)
        pos_err = float(np.linalg.norm(xyz - target_xyz))
        rot_err = float(
            (R.from_quat(target_quat) * R.from_quat(quat).inv()).magnitude()
        )
        ok = pos_err <= args.tol_pos and rot_err <= args.tol_rot
        print(f"position {np.round(xyz, 4).tolist()} error {pos_err * 1000:.2f} mm")
        print(f"rotation error {np.degrees(rot_err):.2f} deg")
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
