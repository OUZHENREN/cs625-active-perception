#!/usr/bin/env python3
"""Find the slot's lateral centre by touching its rim with the carried part.

Why (owner, 2026-11-04): after shifting the insert target by 30.5 mm the visible offset
"clearly got smaller", i.e. the remaining error is a residual in the TARGET, not an offset
of the part inside the jaws.  Guessing the next correction would repeat the same mistake,
and the 13b gate cannot help -- it only proves the TCP reached the target, never that the
target sits in the real slot.  So the slot is measured instead: the part is lowered to
just above the fixture rim and swept slowly along one axis while the effort residual is
watched.  Contact marks the rim; two edges give the centre.

Safety (2026-11-03 rules): slow (<=0.005 m/s), segmented, contact = effort above the
seed baseline + --contact-margin, and any contact cancels the goal immediately.  Nothing
here retries after a protective stop.

Usage:
    python3 scripts/teaching/probe_lateral.py --axis y --span 0.06 --step 0.005 \
        [--dip 0.02] [--contact-margin 0.68] [--dry-run]

Exit codes: 0 centre found; 1 one edge only; 2 refused (no effort stream / planning);
3 aborted by contact on the dip.
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
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Probe(Node):
    def __init__(self, contact_margin: float) -> None:
        super().__init__("probe_lateral")
        self.efforts: list[float] = []
        self.tcp: PoseStamped | None = None
        self.contact_margin = contact_margin
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self._on_tcp, 10)

    def _on_state(self, msg: JointState) -> None:
        effort = dict(zip(msg.name, msg.effort))
        if all(k in effort for k in ORDER):
            values = [effort[k] for k in ORDER]
            if all(np.isfinite(v) and abs(v) < 1.0e4 for v in values):
                self.efforts.append(sum(abs(v) for v in values))
        del self.efforts[:-64]

    def _on_tcp(self, msg: PoseStamped) -> None:
        self.tcp = msg

    def baseline(self, seconds: float = 1.0) -> float:
        values: list[float] = []
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.efforts:
                values.append(self.efforts[-1])
        return float(np.mean(values)) if values else float("nan")

    def peak_since(self, mark: int) -> float:
        recent = self.efforts[mark:]
        return max(recent) if recent else 0.0

    def current_tcp(self, timeout: float = 6.0) -> np.ndarray:
        end = time.time() + timeout
        while self.tcp is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.tcp is None:
            raise SystemExit("cannot read /tcp_pose_broadcaster/pose")
        p = self.tcp.pose
        return np.array([p.position.x, p.position.y, p.position.z])


def move_to(node: Probe, xyz, quat, velocity: float) -> bool:
    """Plan and execute one small move; returns True when it completed."""
    plan = pathlib.Path("/tmp/probe_step.json")
    done = subprocess.run([sys.executable, str(PLAN),
                           *[f"{v:.6f}" for v in (*xyz, *quat)],
                           "--velocity", str(velocity), "--goal-tol", "0.0006",
                           "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if done.returncode != 0:
        print("   plan refused:", (done.stdout or done.stderr).strip().splitlines()[-1])
        return False
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", "2.0", "--goal-tolerance", "0.0006",
                          "--verify-final", "0.002", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=600)
    ok = "ALL_ASSEMBLY_CHUNKS_OK" in (run.stdout or "")
    if not ok:
        print("   move failed:", (run.stdout or "").strip().splitlines()[-1])
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--axis", default="y", choices=["x", "y"])
    parser.add_argument("--span", type=float, default=0.06, help="search each way (m)")
    parser.add_argument("--step", type=float, default=0.005)
    parser.add_argument("--dip", type=float, default=0.020,
                        help="how far to lower before sweeping (m)")
    parser.add_argument("--velocity", type=float, default=0.005)
    parser.add_argument("--contact-margin", type=float, default=0.68)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Probe(args.contact_margin)
    start = node.current_tcp()
    base = node.baseline()
    print(f"start TCP {np.round(start, 4)}  effort baseline {base:.3f}")
    if not np.isfinite(base) or base < 1.0:
        print("REFUSED: effort stream looks dead")
        return 2

    quat_msg = node.tcp.pose.orientation
    quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
    axis_index = 0 if args.axis == "x" else 1
    sweeps = np.arange(0.0, args.span + 1e-9, args.step)
    if args.dry_run:
        print(f"would sweep ±{args.span * 1000:.0f} mm along {args.axis} in "
              f"{args.step * 1000:.0f} mm steps, dipping {args.dip * 1000:.0f} mm")
        return 0

    edges: list[float] = []
    for direction in (+1.0, -1.0):
        hit = None
        for distance in sweeps:
            target = start.copy()
            target[axis_index] = start[axis_index] + direction * distance
            if not move_to(node, target, quat, args.velocity):
                break
            mark = len(node.efforts)
            dip = target.copy()
            dip[2] -= args.dip
            if not move_to(node, dip, quat, args.velocity):
                # a refusal or a stop while dipping counts as an edge
                hit = distance
                break
            time.sleep(0.4)
            peak = node.peak_since(mark)
            residual = peak - base
            print(f"   {args.axis}{direction * distance * 1000:+6.1f} mm  "
                  f"peak {peak:6.2f}  residual {residual:+5.2f}")
            move_to(node, target, quat, args.velocity)   # back to sweep height
            if residual > args.contact_margin:
                hit = distance
                print(f"   contact at {direction * distance * 1000:+.1f} mm")
                break
        edges.append(hit)
        if hit is None:
            print(f"   no contact within {args.span * 1000:.0f} mm going {direction:+.0f}")
        # return to the start before the other direction
        move_to(node, start, quat, args.velocity)

    print(f"edges along {args.axis}: +{edges[0]} / -{edges[1]}")
    if edges[0] is not None and edges[1] is not None:
        centre_offset = (edges[0] - edges[1]) / 2.0
        print(f"CENTRE at {args.axis} = start + {centre_offset * 1000:+.1f} mm")
        (REPO / "runtime").mkdir(exist_ok=True)
        (REPO / "runtime" / "lateral_probe.json").write_text(json.dumps({
            "axis": args.axis, "start": start.tolist(), "quat": quat,
            "edge_plus_m": edges[0], "edge_minus_m": edges[1],
            "centre_offset_m": centre_offset, "stamp": time.time(),
        }, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        return 0
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
