#!/usr/bin/env python3
"""Fine-pose adjustment with feedback: show the error, then nudge, then show it again.

Why (owner, 2026-11-04): with 1-2 mm slot clearance the last centimetres are adjusted by
eye, in small steps, and each step needs an immediate numeric readout of how far the tool
still is from the reference and in which direction -- otherwise it is guesswork.

The reference is stored in runtime/fine_target.json and holds the tool pose that the owner
confirmed works (x, y and attitude; the height belongs to the insert plan, not to the
alignment, so it is not part of the reference by default).

Usage:
    # record the current pose as the reference (height ignored by default)
    python3 scripts/teaching/fine_adjust.py --set-target [--with-z]

    # show the current error against the reference
    python3 scripts/teaching/fine_adjust.py --show

    # nudge and immediately show the new error (millimetres, degrees)
    python3 scripts/teaching/fine_adjust.py --dx -20 --dy 0 --droll 0.5

Exit codes: 0 ok; 2 could not read the TCP pose; 3 argument error.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
STORE = REPO / "runtime" / "fine_target.json"
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"


def current_tcp(timeout: float = 8.0):
    rclpy.init()
    node = Node("fine_adjust")
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


def report(current_xyz, current_quat, target) -> np.ndarray:
    """Print the error and return the current position as a numpy array."""
    tgt_xyz = np.array(target["tcp"])
    tgt_quat = np.array(target["quat"])
    d = np.array(current_xyz) - tgt_xyz
    rot_err = R.from_quat(tgt_quat).inv() * R.from_quat(current_quat)
    angle = float(np.degrees(rot_err.magnitude()))
    axes = rot_err.as_euler("xyz", degrees=True)
    print(f"  当前 ({current_xyz[0]:.4f}, {current_xyz[1]:.4f}, {current_xyz[2]:.4f})")
    print(f"  参考 ({tgt_xyz[0]:.4f}, {tgt_xyz[1]:.4f}, {tgt_xyz[2]:.4f})"
          f"   [z {'参与比较' if target.get('use_z') else '忽略（对齐与高度无关）'}]")
    print(f"  偏差 dx={d[0] * 1000:+7.2f}  dy={d[1] * 1000:+7.2f}  dz={d[2] * 1000:+7.2f} mm"
          f"   水平 {np.linalg.norm(d[:2]) * 1000:.2f} mm")
    print(f"  姿态差 {angle:6.2f}°   (绕 x/y/z 分量 {axes[0]:+.2f} / {axes[1]:+.2f} / {axes[2]:+.2f}°)")
    # A nudge hint: which direction and how much.
    if np.linalg.norm(d[:2]) > 5e-4:
        print(f"  建议: x 需要 {d[0] * 1000:+.1f} mm 反向修正, "
              f"y 需要 {d[1] * 1000:+.1f} mm 反向修正")
    else:
        print("  水平已在对位精度内（<0.5 mm）")
    return np.array(current_xyz)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--set-target", action="store_true",
                        help="record the current pose as the reference")
    parser.add_argument("--with-z", action="store_true",
                        help="include z in the comparison (default: height is ignored)")
    parser.add_argument("--show", action="store_true", help="only print the error")
    parser.add_argument("--dx", type=float, default=0.0, help="nudge along world x (mm)")
    parser.add_argument("--dy", type=float, default=0.0, help="nudge along world y (mm)")
    parser.add_argument("--dz", type=float, default=0.0, help="nudge along world z (mm)")
    parser.add_argument("--droll", type=float, default=0.0,
                        help="nudge the roll about the tool z axis (deg)")
    parser.add_argument("--velocity", type=float, default=0.006)
    args = parser.parse_args()

    pose = current_tcp()
    if pose is None:
        print("REFUSED: cannot read /tcp_pose_broadcaster/pose")
        return 2
    xyz, quat = pose

    if args.set_target:
        STORE.parent.mkdir(parents=True, exist_ok=True)
        STORE.write_text(json.dumps({
            "tcp": [float(v) for v in xyz],
            "quat": [float(v) for v in quat],
            "use_z": bool(args.with_z),
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }, indent=2))
        print(f"reference recorded: xyz={np.round(xyz, 4)} (z "
              f"{'included' if args.with_z else 'ignored'})")
        return 0

    if not STORE.exists():
        print("no reference yet; run with --set-target first")
        return 3
    target = json.loads(STORE.read_text())
    target["use_z"] = target.get("use_z", False) or args.with_z

    print("before:")
    report(xyz, quat, target)

    if args.show:
        return 0
    if not any((args.dx, args.dy, args.dz, args.droll)):
        return 0

    new_xyz = np.array(xyz) + np.array([args.dx, args.dy, args.dz]) / 1000.0
    rot = R.from_quat(quat)
    if args.droll:
        rot = rot * R.from_euler("z", args.droll, degrees=True)
    new_quat = rot.as_quat()

    plan = pathlib.Path("/tmp/fine_nudge.json")
    made = subprocess.run([sys.executable, str(PLAN),
                           *[f"{v:.6f}" for v in (*new_xyz, *new_quat)],
                           "--velocity", str(args.velocity),
                           "--goal-tol", "0.0006", "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if made.returncode != 0:
        print("plan refused:", (made.stdout or made.stderr).strip().splitlines()[-1])
        return 3
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", "3.0", "--goal-tolerance", "0.0006",
                          "--verify-final", "0.002", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=600)
    if "ALL_ASSEMBLY_CHUNKS_OK" not in (run.stdout or ""):
        print("move failed:", (run.stdout or "").strip().splitlines()[-1])
        return 3

    pose = current_tcp()
    if pose is None:
        print("moved, but the TCP pose cannot be read back")
        return 2
    print("\nafter:")
    report(pose[0], pose[1], target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
