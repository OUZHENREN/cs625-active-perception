#!/usr/bin/env python3
"""Promote the pose of a SUCCESSFUL fine capture into the coarse baseline chain.

Owner request (2026-11-05): the view pose at which the fine workpiece capture (T3) just
succeeded should become the next coarse baseline B, and the previous B should become C.
The idea is that the next run starts closer to where the part actually is, instead of
always starting from the A stance.

Baseline order becomes A -> B -> C, where C is the oldest but still valid stance.

This only rewrites ``config/tasks/common.yaml`` -> ``teaching_observation_baselines`` and
keeps the replaced value as a dated comment, so every change is auditable.

Usage:
    python3 scripts/teaching/promote_baseline.py --from-current
    python3 scripts/teaching/promote_baseline.py --position ... --quat ...   (explicit)
    python3 scripts/teaching/promote_baseline.py --show

Exit codes: 0 written/shown; 2 cannot read the live pose; 3 bad arguments.
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
CONFIG = REPO / "config" / "tasks" / "common.yaml"
KEY = "teaching_observation_baselines"
ORDER = ("A", "B", "C")


def current_pose(timeout: float = 8.0):
    rclpy.init()
    node = Node("promote_baseline")
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


def read_baselines() -> dict:
    data = yaml.safe_load(CONFIG.read_text()) or {}
    return dict(data.get(KEY) or {})


def write_baselines(baselines: dict) -> None:
    """Replace the whole block so ordering and comments stay consistent."""
    text = CONFIG.read_text()
    lines = text.splitlines()
    start = next(index for index, line in enumerate(lines) if line.startswith(KEY + ":"))
    end = len(lines)
    for index in range(start + 1, len(lines)):
        if lines[index] and not lines[index].startswith((" ", "\t")):
            end = index
            break
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    body = [f"{KEY}:", f"  # {stamp} rewritten by promote_baseline.py "
                       "(successful fine view -> B, old B -> C)"]
    for name in ORDER:
        entry = baselines.get(name)
        if not entry:
            continue
        body.append(f"  {name}:")
        if entry.get("note"):
            body.append(f"    note: {entry['note']}")
        body.append(f"    position: [{', '.join(f'{float(v):.4f}' for v in entry['position'])}]")
        body.append(f"    quat: [{', '.join(f'{float(v):.4f}' for v in entry['quat'])}]")
        if entry.get("joints_deg"):
            body.append(
                f"    joints_deg: [{', '.join(f'{float(v):.1f}' for v in entry['joints_deg'])}]")
    updated = lines[:start] + body + lines[end:]
    CONFIG.write_text("\n".join(updated).rstrip() + "\n")
    # Fail loudly here rather than later in the chain.
    yaml.safe_load(CONFIG.read_text())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--from-current", action="store_true")
    parser.add_argument("--position", nargs=3, type=float)
    parser.add_argument("--quat", nargs=4, type=float)
    parser.add_argument("--note", default="promoted from a successful fine capture")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    baselines = read_baselines()
    if args.show or not (args.from_current or args.position):
        for name in ORDER:
            entry = baselines.get(name)
            if entry:
                print(f"{name}: position {entry.get('position')} quat {entry.get('quat')}")
            else:
                print(f"{name}: (empty)")
        return 0

    if args.from_current:
        pose = current_pose()
        if pose is None:
            print("REFUSED: cannot read /tcp_pose_broadcaster/pose")
            return 2
        position, quat = pose
    else:
        if not args.quat:
            print("need --quat together with --position")
            return 3
        position, quat = args.position, args.quat

    # Rotation: old B -> C, old C dropped, new pose -> B.  A is untouched.
    if baselines.get("B"):
        baselines["C"] = dict(baselines["B"], note="previous B (demoted)")
    baselines["B"] = {"position": [float(v) for v in position],
                      "quat": [float(v) for v in quat],
                      "note": args.note,
                      "joints_deg": None}
    write_baselines(baselines)

    check = read_baselines()
    print("promoted the successful view to baseline B (previous B is now C)")
    for name in ORDER:
        entry = check.get(name)
        if entry:
            print(f"  {name}: [{', '.join(f'{float(v):.4f}' for v in entry['position'])}]")
        else:
            print(f"  {name}: (empty)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
