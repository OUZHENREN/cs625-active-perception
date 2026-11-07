#!/usr/bin/env python3
"""Insertion strategy the owner taught on 2026-11-05, executed as a scripted search.

Owner's MANUAL procedure, verbatim (2026-11-07):
    "先将工件大致插入，然后适当沿 TCP Y 轴转动以减小碰撞和摩擦，并且注意插入到最终部分
     应该还能有 1.5 度左右的旋转空间；插入的同时也可以检查 Y 轴两端的空间，正常插入的话
     应该左右各有 2 mm 的间隙；
     一定要注意适当地转动而不是直接插入，否则会导致空间越来越小和力矩阻力越来越大导致无法
     插入；同时也可以检查左右两端的间隙是否平衡，不然在最后的转 10 度环节的阻力也会大很多。"

Two cautions extracted from it:
    * rotate WHILE inserting, never drive straight in: the free space closes and the torque
      resistance grows until the part can no longer move;
    * keep the two side gaps (y, ~2 mm each) BALANCED: an unbalanced part makes the final
      10 deg rotation about the TCP much harder.

The physical rules (owner, 2026-11-05):
  1. while the part is still entering, a small rotation about world Y opens a looser region
     (Y is the slot-opening axis whose two side gaps are only ~2 mm);
  2. the free rotation space closes with depth, so the amplitude shrinks -- down to a
     steady 0.5-1 deg near the bottom;
  3. once it reaches the bottom, sweep laterally in y: the two side gaps are about 2 mm
     each and unbalance easily, which widens the clearance.

What this script CANNOT do: find "the direction of least resistance".  That needs the
DIRECTION of the contact force, and this cell has no 6-DoF sensor (only joint-torque sums),
so the dither is a fixed rhythm, not an adaptive search.  The owner accepted that
limitation and asked for the scripted version.

Stages:
  A  descend step by step toward the recorded insert target, rotating +/- amplitude about
     the TCP's own Y axis on alternating steps (amplitude shrinks with depth, floored
     at --min-rot-deg);
  B  keep descending until the effort residual jumps or a protective stop ends the motion
     -- that is the "bottom";
  C  sweep y by --y-sweep-mm in --y-step-mm steps, both directions, reporting the effort at
     each position.

The software never cancels on effort (owner: only the pendant's protection may stop it);
it records the curve so the seating point can be read afterwards.

Usage:
    python3 scripts/teaching/assembly_search.py --target 1.1460 -0.0548 0.0342 \
        [--quat QX QY QZ QW] [--step-mm 10] [--max-depth-mm 450] [--dry-run]

Exit codes: 0 finished; 2 cannot read the state; 3 planning/execution failure.
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
        super().__init__("assembly_search")
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


def plan_and_run(node: Probe, xyz, quat, velocity: float) -> str:
    """Plan and execute one pose; returns 'OK', 'STOPPED' or a refusal reason."""
    plan = pathlib.Path("/tmp/assembly_step.json")
    made = subprocess.run([sys.executable, str(PLAN),
                           *[f"{float(v):.6f}" for v in (*xyz, *quat)],
                           "--velocity", str(velocity), "--goal-tol", "0.002",
                           "--output", str(plan)],
                          capture_output=True, text=True, timeout=300)
    if made.returncode != 0:
        text = (made.stdout or made.stderr or "").strip().splitlines()
        return f"PLAN_REFUSED: {text[-1] if text else '?'}"
    run = subprocess.run([sys.executable, str(EXECUTE), str(plan),
                          "--time-scale", "2.0", "--goal-tolerance", "0.002",
                          "--verify-final", "0.008", "--fault-poll", "0.1"],
                         capture_output=True, text=True, timeout=900)
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
    parser.add_argument("--target", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"), help="recorded insert target (TCP)")
    parser.add_argument("--quat", nargs=4, type=float, default=None,
                        help="insert attitude (default: keep the current one)")
    parser.add_argument("--step-mm", type=float, default=10.0)
    parser.add_argument("--max-depth-mm", type=float, default=460.0)
    parser.add_argument("--rot-amplitude-deg", type=float, default=2.0,
                        help="initial dither amplitude about the TCP y axis")
    parser.add_argument("--min-rot-deg", type=float, default=1.5,
                        help="amplitude kept near the seat; the owner says the part must "
                             "still have about 1.5 deg of free rotation there")
    parser.add_argument("--y-sweep-mm", type=float, default=2.0)
    parser.add_argument("--y-step-mm", type=float, default=1.0)
    parser.add_argument("--expect", nargs=7, type=float, default=None,
                        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="pose the arm must already be at (the pre-insert gate); "
                             "the run refuses to start if the TCP is outside --pos-tol-mm")
    parser.add_argument("--pos-tol-mm", type=float, default=2.0)
    parser.add_argument("--rot-tol-deg", type=float, default=0.5)
    parser.add_argument("--y-balance-tol-mm", type=float, default=1.0,
                        help="warn when the TCP drifts this far in y from the nominal insert "
                             "axis (the two side gaps should stay ~2 mm each)")
    parser.add_argument("--velocity", type=float, default=0.002)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    measured = node.pose()
    if measured is None:
        print("REFUSED: cannot read the TCP pose")
        return 2
    start_xyz, start_quat = measured
    attitude = R.from_quat(start_quat if args.quat is None else np.array(args.quat, dtype=float))
    target = np.array(args.target, dtype=float)
    total_depth = float(np.linalg.norm(target - start_xyz) * 1000.0)
    steps = max(1, int(round(min(total_depth, args.max_depth_mm) / args.step_mm)))
    print(f"start ({start_xyz[0]:.4f}, {start_xyz[1]:.4f}, {start_xyz[2]:.4f})  "
          f"target {np.round(target, 4).tolist()}  distance {total_depth:.0f} mm  "
          f"steps {steps} x {args.step_mm:.0f} mm")
    print(f"dither: +/-{args.rot_amplitude_deg:.1f} deg about the TCP y axis, shrinking to "
          f"{args.min_rot_deg:.1f} deg; y sweep +/-{args.y_sweep_mm:.1f} mm at the bottom")
    if args.dry_run:
        return 0

    # Pose re-verification before the insert (owner, 2026-11-05): the chain gates the
    # pre-insert pose (13b) but this script is also launched by hand, so it checks itself.
    if args.expect is not None:
        expected_xyz = np.array(args.expect[:3], dtype=float)
        expected_quat = np.array(args.expect[3:], dtype=float)
        pos_err = float(np.linalg.norm(start_xyz - expected_xyz) * 1000.0)
        rot_err = float(np.degrees((R.from_quat(expected_quat).inv()
                                    * R.from_quat(start_quat)).magnitude()))
        print(f"\n[gate] TCP vs expected pre-insert: {pos_err:.2f} mm / {rot_err:.2f} deg "
              f"(limits {args.pos_tol_mm:.1f} mm / {args.rot_tol_deg:.2f} deg)")
        if pos_err > args.pos_tol_mm or rot_err > args.rot_tol_deg:
            print("   GATE FAILED: refusing to start the insert from the wrong pose")
            node.destroy_node()
            rclpy.shutdown()
            return 4
        print("   GATE OK")

    baseline = node.effort()
    print(f"effort baseline {baseline:.2f}")
    direction = (target - start_xyz) / max(np.linalg.norm(target - start_xyz), 1e-9)
    print(f"\n{'step':>4}{'z':>9}{'dither':>8}{'effort':>9}{'residual':>10}")
    bottom_reached = False
    for index in range(1, steps + 1):
        fraction = index / steps
        point = start_xyz + direction * (args.step_mm * index / 1000.0)
        amplitude = max(args.min_rot_deg,
                        args.rot_amplitude_deg * (1.0 - 0.7 * fraction))
        sign = 1.0 if index % 2 else -1.0
        # Rotate about the TCP'S OWN Y axis (owner, 2026-11-07): the senior's chain
        # right-multiplies (T_world_insert @ Ry(-10)), i.e. the rotation is expressed in the
        # TCP frame, so the TCP point -- the gripped end of the workpiece -- stays fixed and
        # the far end swings about it.  Right-multiplication is what makes the axis belong to
        # the tool instead of to the world.
        quat = (attitude * R.from_euler("y", sign * amplitude, degrees=True)).as_quat()
        status = plan_and_run(node, point, quat, args.velocity)
        mean = node.effort()
        # Gap-balance proxy: how far the TCP sits from the nominal insert axis in y.  The
        # owner expects ~2 mm each side, so a drift beyond --y-balance-tol-mm means the part
        # is hugging one wall -- which makes the final 10 deg rotation much harder.
        y_drift = float(point[1] - target[1]) * 1000.0
        flag = "  <-- Y UNBALANCED" if abs(y_drift) > args.y_balance_tol_mm else ""
        print(f"{index:>4}{point[2]:>9.4f}{sign * amplitude:>+8.2f}"
              f"{mean:>9.2f}{mean - baseline:>+10.2f}   y-drift {y_drift:+.1f} mm{flag}")
        if status != "OK":
            print(f"   -> {status}")
            if status == "STOPPED":
                print("   protective stop: the pendant protection fired; this IS the seat "
                      "according to the owner's rule -- stopping here")
                bottom_reached = True
                break
            if status == "NO_MOTION":
                print("   the arm stopped executing (robot-side state); aborting")
                return 3
            return 3
        if mean - baseline > 3.0:
            print(f"   residual {mean - baseline:+.2f} > 3.0: treating this as the bottom")
            bottom_reached = True
            break

    if bottom_reached:
        print(f"\nstage C: y sweep +/-{args.y_sweep_mm:.1f} mm")
        here, here_quat = node.pose()
        for offset in np.arange(-args.y_sweep_mm, args.y_sweep_mm + 1e-9, args.y_step_mm):
            point = here + np.array([0.0, float(offset) / 1000.0, 0.0])
            status = plan_and_run(node, point, here_quat, args.velocity)
            mean = node.effort()
            print(f"   y{offset:+.1f} mm  {status}  effort {mean:.2f}  "
                  f"residual {mean - baseline:+.2f}")
            if status != "OK":
                print("   stopping the sweep:", status)
                break
    else:
        print("\nno bottom detected within the requested depth; sweep skipped")
    final, final_quat = node.pose()
    if final is not None:
        to_target = float(np.linalg.norm(final - target) * 1000.0)
        print(f"\nfinal TCP ({final[0]:.4f}, {final[1]:.4f}, {final[2]:.4f})  "
              f"{to_target:.1f} mm from the recorded insert target")
    node.destroy_node()
    rclpy.shutdown()
    print("ASSEMBLY_SEARCH_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
