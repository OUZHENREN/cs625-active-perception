#!/usr/bin/env python3
"""Verify the TCP is within tolerance of a target pose before a tight move.

Assembly phases need the tool to be lined up to a fraction of a millimetre before
entering the slot.  This reads the TCP pose topic (20 samples, averaged), compares
it against a target pose passed on the command line, and exits non-zero when the
error exceeds the owner's acceptance gate.

Gate (2026-10-27, owner-confirmed):
    position error norm  <= 0.1 mm
    orientation error    <= 0.1 deg

The grasp approach (2026-11-02 audit) uses a wider gate (0.5 mm / 0.05 deg),
because the joint-space planner stops a few tenths of a millimetre short of the
nominal TCP; pass --pos-tol-mm / --rot-tol-deg to override.

Usage
-----
    python3 scripts/teaching/verify_pose.py --x .. --y .. --z .. --qx .. --qy .. --qz .. --qw ..
        [--pos-tol-mm 0.1] [--rot-tol-deg 0.1]

Exit codes
----------
    0  within tolerance
    1  position error too large
    2  orientation error too large
    3  could not read the TCP pose
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

POS_TOL_MM = 0.1
ROT_TOL_DEG = 0.1


def read_tcp(node: Node, samples: int = 20) -> np.ndarray | None:
    latest: list[PoseStamped] = []
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: latest.append(m), 10)
    deadline = time.time() + 8.0
    while len(latest) < samples and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not latest:
        return None
    rows = [[m.pose.position.x, m.pose.position.y, m.pose.position.z,
             m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
             m.pose.orientation.w] for m in latest]
    return np.array(rows).mean(axis=0)


def main() -> int:
    parser = argparse.ArgumentParser()
    for name in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        parser.add_argument(f"--{name}", type=float, required=True)
    parser.add_argument("--pos-tol-mm", type=float, default=POS_TOL_MM)
    parser.add_argument("--rot-tol-deg", type=float, default=ROT_TOL_DEG)
    args = parser.parse_args()
    pos_tol_mm = args.pos_tol_mm
    rot_tol_deg = args.rot_tol_deg

    target_pos = np.array([args.x, args.y, args.z])
    target_quat = np.array([args.qx, args.qy, args.qz, args.qw])

    rclpy.init()
    node = Node("verify_pose")
    row = read_tcp(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if row is None:
        print("no TCP pose")
        return 3

    pos = row[:3]
    quat = row[3:]
    pos_err_mm = np.linalg.norm(pos - target_pos) * 1000.0

    def norm(q):
        return q if q[3] >= 0 else -q
    rot_err_deg = float(np.degrees(np.linalg.norm(
        (R.from_quat(norm(quat)).inv() * R.from_quat(norm(target_quat))).as_rotvec())))

    print(f"position error = {pos_err_mm:.4f} mm (gate {pos_tol_mm} mm)")
    print(f"orientation error = {rot_err_deg:.4f} deg (gate {rot_tol_deg} deg)")

    if pos_err_mm > pos_tol_mm:
        print("POSITION_OUT_OF_TOLERANCE")
        return 1
    if rot_err_deg > rot_tol_deg:
        print("ORIENTATION_OUT_OF_TOLERANCE")
        return 2
    print("POSE_WITHIN_TOLERANCE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
