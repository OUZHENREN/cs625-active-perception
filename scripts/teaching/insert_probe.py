#!/usr/bin/env python3
"""Descend the insert in steps and record the torque signature against height.

Why (owner, 2026-11-04): the insert must not simply run to the computed end point --
where the workpiece actually seats has to be read from the torque curve.  This script
walks the descent in fixed steps, samples the resting effort after each step, and prints
a height-vs-effort table so the seating point (a step change / plateau) can be identified
from data instead of assumed.

Rules that are deliberately NOT in here (owner, 2026-11-04):
  * no automatic retreat -- a safety trip cancels and stops, the operator decides;
  * no retry after a protective stop.

Usage:
    python3 scripts/teaching/insert_probe.py --steps 4 --step-mm 25 \
        [--contact-margin 0.68] [--safety-margin 1.63] [--velocity 0.002]

Exit codes: 0 all steps done; 1 safety trip (stopped); 2 cannot read state; 3 refused.
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
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class State(Node):
    def __init__(self) -> None:
        super().__init__("insert_probe")
        self.efforts: list[float] = []
        self.tcp: PoseStamped | None = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._on_tcp, 10)

    def _on_state(self, msg: JointState) -> None:
        effort = dict(zip(msg.name, msg.effort))
        if all(k in effort for k in ORDER):
            values = [effort[k] for k in ORDER]
            if all(np.isfinite(v) and abs(v) < 1.0e4 for v in values):
                self.efforts.append(sum(abs(v) for v in values))
        del self.efforts[:-80]

    def _on_tcp(self, msg: PoseStamped) -> None:
        self.tcp = msg

    def settle(self, seconds: float = 1.5) -> tuple[float, float]:
        """Spin for `seconds`, return (mean effort, peak effort) of that window."""
        self.efforts.clear()
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.efforts:
            return float("nan"), float("nan")
        return float(np.mean(self.efforts)), float(np.max(self.efforts))

    def pose(self, timeout: float = 6.0):
        end = time.time() + timeout
        while self.tcp is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.tcp is None:
            return None
        p = self.tcp.pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def move(node: State, xyz, quat, velocity: float) -> bool:
    plan = pathlib.Path("/tmp/insert_probe_step.json")
    made = subprocess.run([sys.executable, str(PLAN),
                           *[f"{v:.6f}" for v in (*xyz, *quat)],
                           "--velocity", str(velocity), "--goal-tol", "0.0006",
                           "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if made.returncode != 0:
        print("   plan refused:", (made.stdout or made.stderr).strip().splitlines()[-1])
        return False
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", "2.0", "--goal-tolerance", "0.0006",
                          "--verify-final", "0.002", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=900)
    if "ALL_ASSEMBLY_CHUNKS_OK" not in (run.stdout or ""):
        last = (run.stdout or "").strip().splitlines()[-1] if run.stdout else "?"
        print("   move failed:", last)
        # A protective stop or a dead effort stream must stop the whole probe.
        if "PROTECTIVE" in last or "EFFORT_DEAD" in last:
            return False
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", type=int, default=4)
    parser.add_argument("--step-mm", type=float, default=25.0)
    parser.add_argument("--velocity", type=float, default=0.002)
    parser.add_argument("--contact-margin", type=float, default=0.68)
    parser.add_argument("--safety-margin", type=float, default=1.63)
    parser.add_argument("--min-z", type=float, default=-0.05,
                        help="never command below this TCP z (safety floor)")
    parser.add_argument("--csv", default="runtime/insert_probe.csv")
    args = parser.parse_args()

    rclpy.init()
    node = State()
    pose = node.pose()
    if pose is None:
        print("REFUSED: cannot read the TCP pose")
        return 2
    start_xyz, quat = pose
    base_mean, _ = node.settle()
    if not np.isfinite(base_mean) or base_mean < 1.0:
        print("REFUSED: effort stream looks dead")
        return 2
    print(f"start z={start_xyz[2]:.4f}  baseline effort {base_mean:.3f}  "
          f"contact>{base_mean + args.contact_margin:.2f}  "
          f"safety>{base_mean + args.safety_margin:.2f}")
    print(f"{'step':>4}{'z':>10}{'mean':>9}{'peak':>9}{'residual':>10}{'note':>16}")

    rows = [(0, start_xyz[2], base_mean, base_mean, 0.0, "start")]
    out = pathlib.Path(args.csv)
    out.parent.mkdir(parents=True, exist_ok=True)
    for step in range(1, args.steps + 1):
        target = start_xyz.copy()
        target[2] = start_xyz[2] - (args.step_mm * step) / 1000.0
        if target[2] < args.min_z:
            print(f"{step:>4}   refused: below the safety floor {args.min_z}")
            break
        if not move(node, target, quat, args.velocity):
            rows.append((step, target[2], float("nan"), float("nan"), float("nan"), "STOPPED"))
            break
        mean, peak = node.settle()
        residual = mean - base_mean
        note = "contact" if residual > args.contact_margin else "free"
        if residual > args.safety_margin:
            note = "SAFETY TRIP"
        print(f"{step:>4}{target[2]:>10.4f}{mean:>9.2f}{peak:>9.2f}{residual:>+10.2f}{note:>16}")
        rows.append((step, target[2], mean, peak, residual, note))
        if note == "SAFETY TRIP":
            print("stopping: safety margin exceeded (no retreat by design)")
            break

    with out.open("w", encoding="utf-8") as handle:
        handle.write("step,z,mean_effort,peak_effort,residual,note\n")
        for row in rows:
            handle.write(",".join(str(v) for v in row) + "\n")
    print(f"\ntable written to {out.relative_to(REPO)}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(r[5] != "STOPPED" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
