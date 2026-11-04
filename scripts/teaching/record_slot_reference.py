#!/usr/bin/env python3
"""Record a visually-aligned pose as the reference slot pose for this run.

Owner's workflow (2026-11-04): the stored calibration is out of range and RVS's live
measurement carries a tens-of-mm bias, so the operator aligns the arm BY EYE once, that
pose becomes today's reference, and every later RVS capture is compared against the
recorded successes before anything moves.

What "aligned" means here: the workpiece in the jaws is visually centred over the slot
mouth, at any height above the fixture (the tool is not descending).  That pose is a
pre-insert pose, so the slot pose is recovered by inverting the same chain the chain uses:

    pre_insert = rotated @ translate(z -350 mm)
    rotated    = insert   @ Ry(-10 deg)
    insert     = slot     @ T_s_tcp

    => rotated = pre_insert @ translate(z +350 mm)
    => insert  = rotated    @ Ry(+10 deg)
    => slot    = insert     @ inv(T_s_tcp)

Usage:
    python3 scripts/teaching/record_slot_reference.py [--note "..."] [--show]

Exit codes: 0 recorded; 2 could not read the TCP pose; 3 argument error.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
STORE = REPO / "runtime" / "slot_reference.json"
PRE_INSERT_BACKOFF_M = 0.350
TILT_DEG = -10.0
T_S_TCP = np.array([[-1.0, 0.0, 0.0, -0.082],
                    [0.0, 1.0, 0.0, -0.193],
                    [0.0, 0.0, -1.0, -0.4125],
                    [0.0, 0.0, 0.0, 1.0]])


def to_matrix(xyz, quat) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(quat).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def current_tcp(timeout: float = 8.0):
    rclpy.init()
    node = Node("record_slot_reference")
    latest: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: latest.__setitem__("m", m), 10)
    end = time.time() + timeout
    while "m" not in latest and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if "m" not in latest:
        return None
    pose = latest["m"].pose
    return ([pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])


def slot_from_pre_insert(pre_xyz, pre_quat) -> tuple[list[float], list[float]]:
    """Invert the chain: pre-insert pose -> slot pose."""
    pre = to_matrix(pre_xyz, pre_quat)
    # rotated = pre_insert @ translate(z +350 mm)
    undo_backoff = np.eye(4)
    undo_backoff[2, 3] = PRE_INSERT_BACKOFF_M
    rotated = pre @ undo_backoff
    # insert = rotated @ Ry(+10 deg)
    undo_tilt = np.eye(4)
    undo_tilt[:3, :3] = R.from_euler("y", -TILT_DEG, degrees=True).as_matrix()
    insert = rotated @ undo_tilt
    slot = insert @ np.linalg.inv(T_S_TCP)
    return ([float(v) for v in slot[:3, 3]],
            [float(v) for v in R.from_matrix(slot[:3, :3]).as_quat()])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--note", default="", help="what makes this reference valid")
    parser.add_argument("--show", action="store_true", help="print the stored references")
    args = parser.parse_args()

    if args.show:
        if not STORE.exists():
            print("no references recorded yet")
            return 0
        for entry in json.loads(STORE.read_text()):
            print(f"{entry['stamp']}  slot=({entry['slot'][0]:.4f}, "
                  f"{entry['slot'][1]:.4f}, {entry['slot'][2]:.4f})  {entry.get('note', '')}")
        return 0

    pose = current_tcp()
    if pose is None:
        print("REFUSED: cannot read /tcp_pose_broadcaster/pose")
        return 2
    pre_xyz, pre_quat = pose
    print(f"current TCP (treated as a pre-insert pose): {np.round(pre_xyz, 4)}")
    slot_xyz, slot_quat = slot_from_pre_insert(pre_xyz, pre_quat)
    print(f"  -> slot pose {np.round(slot_xyz, 4)}")

    entries = json.loads(STORE.read_text()) if STORE.exists() else []
    entries.append({
        "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "pre_insert_tcp": [float(v) for v in pre_xyz],
        "pre_insert_quat": [float(v) for v in pre_quat],
        "slot": slot_xyz,
        "slot_quat": slot_quat,
        "note": args.note,
    })
    STORE.parent.mkdir(parents=True, exist_ok=True)
    STORE.write_text(json.dumps(entries, indent=2))
    print(f"recorded as reference #{len(entries)} in {STORE.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
