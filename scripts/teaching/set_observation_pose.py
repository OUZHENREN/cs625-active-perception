#!/usr/bin/env python3
"""Register a manually taught coarse-observation pose as the new baseline.

Why (owner workflow, 2026-11-03): RVS's coarse match needs the part inside the
camera's working region.  When the workpiece is placed somewhere the fixed
observation pose cannot see, the owner moves the arm by hand to a pose where RVS
DOES recognise it -- that pose becomes the new coarse-observation baseline.

The baseline lives in ``config/tasks/common.yaml`` ->
``teaching_observation_poses``.  This script rewrites only that block and keeps the
previous value as a dated comment, so the change is auditable and reversible.

Usage:
    python3 scripts/teaching/set_observation_pose.py --name workpiece_coarse --from-current
    python3 scripts/teaching/set_observation_pose.py --name slot_coarse --show
    python3 scripts/teaching/set_observation_pose.py --name workpiece_coarse \
        --position 0.59 0.35 0.71 --quat 0.29 -0.70 -0.25 -0.60

Exit codes: 0 written/shown; 2 could not read the TCP pose; 3 bad arguments.
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import re
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
CONFIG = REPO / "config" / "tasks" / "common.yaml"
NAMES = ("slot_coarse", "workpiece_coarse")
BLOCK_KEY = "teaching_observation_poses:"


def current_tcp(timeout: float = 8.0):
    rclpy.init()
    node = Node("set_observation_pose")
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


def rewrite_block(text: str, name: str, position, quat) -> str:
    """Rewrite <name>'s position/quat inside the teaching_observation_poses block.

    Scoped on purpose: the file holds many other position lists, so a global
    substitution would silently corrupt unrelated keys.
    """
    start = text.find(BLOCK_KEY)
    if start < 0:
        raise SystemExit(f"{BLOCK_KEY} not found in {CONFIG}")
    # The block ends at the first line that starts at column 0 after it.
    tail = text[start + len(BLOCK_KEY):]
    end_match = re.search(r"\n(?=\S)", tail)
    end = start + len(BLOCK_KEY) + (end_match.start() if end_match else len(tail))
    block = text[start:end]

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    entry_pattern = re.compile(
        rf"(  {name}:\n    position: \[)([^\]]*)(\]\n    quat: \[)([^\]]*)(\])")
    entry = entry_pattern.search(block)
    if not entry:
        raise SystemExit(f"cannot locate {name} inside {BLOCK_KEY}")
    old_position, old_quat = entry.group(2).strip(), entry.group(4).strip()

    new_entry = (f"  {name}:\n"
                 f"    position: [{', '.join(f'{v:.4f}' for v in position)}]\n"
                 f"    quat: [{', '.join(f'{v:.4f}' for v in quat)}]")
    history = (f"  # {stamp} replaced {name}: was [{old_position}] / [{old_quat}]\n")
    new_block = entry_pattern.sub(history + new_entry, block, count=1)
    return text[:start] + new_block + text[end:]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", required=True, choices=NAMES)
    parser.add_argument("--from-current", action="store_true",
                        help="read the pose from the live TCP topic")
    parser.add_argument("--position", nargs=3, type=float)
    parser.add_argument("--quat", nargs=4, type=float)
    parser.add_argument("--show", action="store_true", help="print the stored value")
    args = parser.parse_args()

    text = CONFIG.read_text()
    stored = (yaml.safe_load(text).get("teaching_observation_poses") or {})
    if args.show or not (args.from_current or args.position):
        entry = stored.get(args.name, {})
        print(f"{args.name}: position {entry.get('position')} quat {entry.get('quat')}")
        return 0

    if args.from_current:
        pose = current_tcp()
        if pose is None:
            print("REFUSED: cannot read /tcp_pose_broadcaster/pose")
            return 2
        position, quat = pose
    else:
        if not args.quat:
            print("need --quat together with --position")
            return 3
        position, quat = args.position, args.quat

    CONFIG.write_text(rewrite_block(text, args.name, position, quat))
    # Read back through YAML so a malformed edit fails loudly here, not later.
    check = yaml.safe_load(CONFIG.read_text())["teaching_observation_poses"][args.name]
    print(f"registered {args.name}:")
    print(f"  position [{', '.join(f'{v:.4f}' for v in position)}]")
    print(f"  quat     [{', '.join(f'{v:.4f}' for v in quat)}]")
    print(f"  read back {check['position']} / {check['quat']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
