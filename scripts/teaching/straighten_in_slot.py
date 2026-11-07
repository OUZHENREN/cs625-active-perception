#!/usr/bin/env python3
"""Straighten the workpiece inside the slot: rotate as far as the slot allows, never force.

Why this exists (owner, 2026-11-07): the senior's flow ends with a 10 deg rotation of the
workpiece about its TCP.  A single 10 deg move jams: the jaws sit ~370 mm inside the slot and
sweeping them 10 deg needs ~35 mm of room that is not there (measured 2026-11-04: 1 deg at
depth already tripped the protective stop).

The owner's own observation gives the right formulation: a correctly seated part still has
about 1.5 deg of free rotation.  So the achievable rotation angle is not an obstacle, it is
the MEASUREMENT -- this script rotates in small steps about the TCP's own Y axis and stops at
the first step that shows a resistance jump, then rotates back to the last good attitude.

Outcome is one of:
  * reached the requested angle  -> the slot is looser than expected (report it);
  * stopped at N deg             -> N is the free rotation space inside the slot;
  * protective stop              -> revert and report the last good angle.

Usage:
    python3 scripts/teaching/straighten_in_slot.py [--total-deg 10] [--step-deg 0.5] \
        [--effort-jump 1.0] [--expect X Y Z QX QY QZ QW] [--dry-run]

Exit codes: 0 finished (whatever the angle); 2 cannot read state; 3 planning/execution issue.
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("straighten_in_slot")
        self.pose_msg: PoseStamped | None = None
        self.efforts: list[float] = []
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._on_pose, 10)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _on_state(self, msg: JointState) -> None:
        effort = dict(zip(msg.name, msg.effort))
        if all(k in effort for k in ORDER):
            values = [effort[k] for k in ORDER]
            if all(np.isfinite(v) and abs(v) < 1.0e4 for v in values):
                self.efforts.append(float(sum(abs(v) for v in values)))
        del self.efforts[:-80]

    def pose(self, timeout: float = 6.0):
        end = time.time() + timeout
        while self.pose_msg is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pose_msg is None:
            return None
        p = self.pose_msg.pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def effort(self, seconds: float = 1.2) -> float:
        values: list[float] = []
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.efforts:
                values.append(self.efforts[-1])
        return float(np.mean(values)) if values else float("nan")


def plan_and_run(xyz, quat, velocity: float) -> str:
    plan = pathlib.Path("/tmp/straighten_step.json")
    made = subprocess.run([sys.executable, str(PLAN),
                           *[f"{float(v):.6f}" for v in (*xyz, *quat)],
                           "--velocity", str(velocity), "--goal-tol", "0.001",
                           "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if made.returncode != 0:
        text = (made.stdout or made.stderr or "").strip().splitlines()
        return f"PLAN_REFUSED: {text[-1] if text else '?'}"
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", "2.0", "--goal-tolerance", "0.001",
                          "--verify-final", "0.004", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=600)
    output = run.stdout or ""
    if "ALL_ASSEMBLY_CHUNKS_OK" in output:
        return "OK"
    last = output.strip().splitlines()[-1] if output.strip() else "?"
    if "PROTECTIVE" in last or "EMERGENCY" in last:
        return "STOPPED"
    if "NO_MOTION" in last:
        return "NO_MOTION"
    return f"FAILED: {last}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--total-deg", type=float, default=10.0)
    parser.add_argument("--step-deg", type=float, default=0.5)
    parser.add_argument("--effort-jump", type=float, default=1.0,
                        help="resistance jump (sum|effort|) that counts as binding")
    parser.add_argument("--expect", nargs=7, type=float, default=None,
                        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--pos-tol-mm", type=float, default=3.0)
    parser.add_argument("--velocity", type=float, default=0.004)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    measured = node.pose()
    if measured is None:
        print("REFUSED: cannot read the TCP pose")
        return 2
    position, quat = measured

    if args.expect is not None:
        want_xyz = np.array(args.expect[:3], dtype=float)
        want_quat = np.array(args.expect[3:], dtype=float)
        pos_err = float(np.linalg.norm(position - want_xyz) * 1000.0)
        rot_err = float(np.degrees((R.from_quat(want_quat).inv()
                                    * R.from_quat(quat)).magnitude()))
        print(f"[gate] {pos_err:.2f} mm / {rot_err:.2f} deg from the expected pose "
              f"(limit {args.pos_tol_mm:.1f} mm)")
        if pos_err > args.pos_tol_mm:
            print("   GATE FAILED: not at the expected pose; refusing to rotate")
            node.destroy_node()
            rclpy.shutdown()
            return 2
    print(f"start attitude quat {np.round(quat, 4)}  position {np.round(position, 4)}")
    baseline = node.effort()
    steps = max(1, int(round(args.total_deg / args.step_deg)))
    print(f"rotating about the TCP's own Y axis in {steps} steps of {args.step_deg:.2f} deg "
          f"(binding threshold: +{args.effort_jump:.1f} residual)")
    if args.dry_run:
        print("dry run: nothing will move")
        return 0

    print(f"\n{'step':>4}{'angle':>8}{'effort':>9}{'residual':>10}  result")
    best: tuple[np.ndarray, float] | None = None
    previous = baseline
    reached = 0.0
    for index in range(1, steps + 1):
        angle = index * args.step_deg
        # Right-multiply = rotate in the TCP frame, so the TCP point stays put.
        stepped = (R.from_quat(quat) * R.from_euler("y", args.step_deg, degrees=True)).as_quat()
        status = plan_and_run(position, stepped, args.velocity)
        mean = node.effort()
        residual = mean - baseline
        print(f"{index:>4}{angle:>8.2f}{mean:>9.2f}{residual:>+10.2f}  {status}")
        if status != "OK":
            print(f"   stopped at {reached:.2f} deg ({status}) -- rotating back")
            if best is not None:
                plan_and_run(position, best[0], args.velocity)
            break
        if residual - (previous - baseline) > args.effort_jump:
            print(f"   resistance jumped by {residual - (previous - baseline):+.2f} -- "
                  f"this is the slot wall; rotating back to {reached:.2f} deg")
            plan_and_run(position, np.array(best[0]), args.velocity) if best else None
            break
        quat = np.asarray(stepped).reshape(4)
        best = (quat, residual)
        previous = mean
        reached = angle
    else:
        print(f"\nreached the full {args.total_deg:.1f} deg: the slot is looser than expected")

    print(f"\nMAX FREE ROTATION: {reached:.2f} deg  (owner expects ~1.5 deg for a seated part)")
    if best is not None and reached < args.total_deg:
        current, _ = node.pose()
        settled = float(np.degrees((R.from_quat(np.array(best[0])).inv()
                                    * R.from_quat(current[1])).magnitude())) if current else float("nan")
        print(f"remaining attitude offset to the last good step: {settled:.2f} deg")
    node.destroy_node()
    rclpy.shutdown()
    print("STRAIGHTEN_IN_SLOT_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
