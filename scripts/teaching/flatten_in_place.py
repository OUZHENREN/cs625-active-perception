#!/usr/bin/env python3
"""Rotate the tool from its tilted entry attitude to flat, about the fixed TCP point.

Why not a single joint-space plan: plan_joint_locked interpolates joints linearly, which
does not control the TOOL orientation path -- the tool can swing sideways while the joints
move, and with the workpiece already seated in the slot that swing jams against a wall
(protective stop twice on 2026-11-04).  Rotating in small Cartesian steps with the TCP held
fixed keeps the tool pivoting in place.

Usage:
    python3 scripts/teaching/flatten_in_place.py --steps 10 --velocity 0.004 [--dry-run]
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
from scipy.spatial.transform import Rotation as R, Slerp
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
FLAT_QUAT = np.array([0.0048, 1.0, -0.0017, 0.0008])
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("flatten_in_place")
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
        del self.efforts[:-64]

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

    def effort(self, seconds: float = 1.0) -> float:
        values: list[float] = []
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.efforts:
                values.append(self.efforts[-1])
        return float(np.mean(values)) if values else float("nan")


def angle_between(a, b) -> float:
    delta = R.from_quat(a).inv() * R.from_quat(b)
    return float(np.degrees(float(delta.magnitude())))


def plan_and_run(xyz, quat, velocity: float, time_scale: float) -> bool:
    plan = pathlib.Path("/tmp/flatten_step.json")
    made = subprocess.run([sys.executable, str(PLAN),
                           *[f"{float(v):.6f}" for v in (*xyz, *quat)],
                           "--velocity", str(velocity), "--goal-tol", "0.0006",
                           "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if made.returncode != 0:
        text = (made.stdout or made.stderr or "").strip().splitlines()
        print("   plan refused:", text[-1] if text else "?")
        return False
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", str(time_scale), "--goal-tolerance", "0.0006",
                          "--verify-final", "0.002", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=900)
    if "ALL_ASSEMBLY_CHUNKS_OK" not in (run.stdout or ""):
        text = (run.stdout or "").strip().splitlines()
        print("   execution stopped:", text[-1] if text else "?")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--velocity", type=float, default=0.004)
    parser.add_argument("--time-scale", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    target_quat = FLAT_QUAT / np.linalg.norm(FLAT_QUAT)
    rclpy.init()
    node = Probe()
    measured = node.pose()
    if measured is None:
        print("REFUSED: cannot read the TCP pose")
        return 2
    start_xyz, start_quat = measured
    total = angle_between(start_quat, target_quat)
    baseline = node.effort()
    print(f"start ({start_xyz[0]:.4f}, {start_xyz[1]:.4f}, {start_xyz[2]:.4f})  "
          f"remaining {total:.2f} deg  effort baseline {baseline:.2f}")
    print(f"plan: {args.steps} steps of {total / args.steps:.2f} deg about the fixed TCP")
    if args.dry_run:
        return 0

    slerp = Slerp([0.0, 1.0], R.from_quat(np.vstack([start_quat, target_quat])))
    for step in range(1, args.steps + 1):
        fraction = step / args.steps
        # Slerp with a LIST returns a (1,) rotation whose as_quat() is (1, 4);
        # pass the scalar so the quaternion is a flat 4-vector.
        quat = np.asarray(slerp(float(fraction)).as_quat(), dtype=float).reshape(4)
        if not plan_and_run(start_xyz, quat, args.velocity, args.time_scale):
            node.destroy_node()
            rclpy.shutdown()
            return 3
        current = node.pose()
        mean = node.effort()
        remaining = angle_between(current[1], target_quat) if current is not None else float("nan")
        moved = float(np.linalg.norm(current[0] - start_xyz) * 1000.0) if current is not None else float("nan")
        print(f"   step {step:2d}  remaining {remaining:6.2f} deg  tcp drift {moved:5.2f} mm  "
              f"effort {mean:6.2f}  residual {mean - baseline:+.2f}")

    node.destroy_node()
    rclpy.shutdown()
    print("FLATTEN_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
