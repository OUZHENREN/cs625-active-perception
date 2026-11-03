#!/usr/bin/env python3
"""Supervised insertion for the teaching chain (专项 3, A 期).

Modelled on the senior's cs625_compliant_placement, adapted to this cell's
execution chain (MoveIt + arm_controller, NOT the robot's internal motion):

  senior                               this implementation
  ----------------------------------------------------------------
  contact = residual(joint_torques)    contact/safety on the FAST-filtered
  speed slider throttling              per-segment time_scale control
  (robot internal motion only)         (our SEGMENTED executor)
  C++ 50 Hz wall-timer loop            Python 100 Hz loop over /joint_states
                                       (topic carrier 125 Hz; effort refresh
                                       measured at ~72 Hz -- still enough for
                                       0.005 m/s insertion)

Flow:
  1. plan the insert trajectory (plan_joint_locked: collision-checked IK,
     joint-space plan -- the branch cannot flip)
  2. sample the stationary effort for 1 s and seed the filters
  3. execute the trajectory in --segments segments (shared seams, re-timed)
     while a 100 Hz loop watches the FAST-filtered sum |effort|:
       fast >= --contact  -> slow the NEXT segment (time_scale doubles)
       fast >= --safety   -> cancel the controller goal immediately, then
                             retreat --retreat-mm mm back along the tool axis
                             (reverse of the insertion direction, no replanning
                             of the failed path -- 10-27 lesson)
  4. DONE when the measured TCP is within --tol-mm of the target

Defaults are the 2026-11-03 measured baseline proposals (owner-confirmed):
contact = 16.68, safety = 17.63 (sum of |joint effort|).

Usage:
    python3 scripts/compliant_insert.py X Y Z QX QY QZ QW \
        [--segments 6] [--tol-mm 3] [--contact 16.68] [--safety 17.63] \
        [--retreat-mm 20] [--dry-run]

Exit codes: 0 DONE; 1 target not reached; 2 plan refused; 3 safety abort
(retreated); 4 execution failure; 5 no effort stream.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parent.parent
PLAN_LOCKED = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTOR = REPO / "scripts" / "execute_assembly_chunks.py"
sys.path.insert(0, str(REPO / "scripts" / "teaching"))

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ACTION = "/arm_controller/follow_joint_trajectory"
MONITOR_HZ = 100.0


def run(args: list[str], timeout: float = 600.0) -> tuple[int, str]:
    done = subprocess.run([sys.executable, "-u", *args],
                          capture_output=True, text=True, timeout=timeout)
    return done.returncode, (done.stdout or "") + (done.stderr or "")


class Monitor(Node):
    """100 Hz effort watchdog + TCP reader + goal canceller."""

    def __init__(self, contact: float, safety: float) -> None:
        super().__init__("compliant_insert_monitor")
        self.latest_effort: float | None = None
        self.latest_tcp: PoseStamped | None = None
        self.slow = 0.0
        self.fast = 0.0
        self.seeded = False
        self.contact = contact
        self.safety = safety
        self.tripped: str | None = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self._on_tcp, 10)

    def _on_state(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        eff = dict(zip(msg.name, msg.effort))
        if all(k in pos and k in eff for k in ORDER):
            self.latest_effort = float(sum(abs(eff[k]) for k in ORDER))

    def _on_tcp(self, msg: PoseStamped) -> None:
        self.latest_tcp = msg

    def seed(self, seconds: float = 1.0) -> bool:
        values: list[float] = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_effort is not None:
                values.append(self.latest_effort)
        if not values:
            return False
        self.slow = self.fast = float(np.mean(values))
        self.seeded = True
        print(f"   effort baseline seeded: {self.slow:.3f} ({len(values)} samples)")
        return True

    def evaluate(self) -> tuple[bool, bool, float]:
        """(contact, safety, fast_value) at the current sample."""
        if not self.seeded or self.latest_effort is None:
            return False, False, 0.0
        x = self.latest_effort
        self.slow += 0.01 * (x - self.slow)
        self.fast += 0.2 * (x - self.fast)
        return self.fast >= self.contact, self.fast >= self.safety, self.fast

    def cancel_goal(self) -> bool:
        # Empty goal_info cancels ALL goals of the action -- exactly what a
        # safety abort wants (there is at most one in flight here anyway).
        raw = self.create_client(CancelGoal, f"{ACTION}/_action/cancel_goal")
        if not raw.wait_for_service(timeout_sec=5.0):
            print("   [monitor] cancel service not available")
            return False
        future = raw.call_async(CancelGoal.Request())
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result()
        ok = bool(result is not None
                  and all(r.return_code == 0 for r in result.return_codes)) \
            if result is not None else False
        print(f"   [monitor] cancel_goal -> {'OK' if ok else 'FAILED'} "
              f"({result.return_codes if result else 'timeout'})")
        return ok

    def mean_tcp(self, samples: int = 20) -> tuple[np.ndarray, np.ndarray] | None:
        rows: list[np.ndarray] = []
        deadline = time.time() + 8.0
        while len(rows) < samples and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_tcp is not None:
                pose = self.latest_tcp.pose
                rows.append(np.array([pose.position.x, pose.position.y, pose.position.z,
                                      pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w]))
        if not rows:
            return None
        mean = np.array(rows).mean(axis=0)
        return mean[:3], mean[3:]


def plan_insert(target_xyz, target_quat) -> tuple[int, str]:
    return run([str(PLAN_LOCKED), *[f"{v:.6f}" for v in (*target_xyz, *target_quat)],
                "--velocity", "0.005", "--goal-tol", "0.0003",
                "--output", "/tmp/compliant_insert_plan.json"], timeout=300.0)


def segment_plan(plan_path: pathlib.Path, segments: int,
                 time_scale: float) -> list[pathlib.Path]:
    data = json.loads(plan_path.read_text())
    points = data["points"]
    step = max(2, math.ceil((len(points) - 1) / segments))
    paths: list[pathlib.Path] = []
    for index in range(segments):
        start = index * step
        end = min((index + 1) * step + 1, len(points))
        t0 = points[start]["t"]
        seg = [{"q": p["q"], "t": p["t"] - t0} for p in points[start:end]]
        out = pathlib.Path(f"/tmp/compliant_insert_seg{index:02d}.json")
        out.write_text(json.dumps({"joint_names": data["joint_names"], "points": seg}))
        paths.append(out)
    return paths


def retreat_along_tool(retreat_mm: float, monitor: Monitor) -> bool:
    measured = monitor.mean_tcp()
    if measured is None:
        print("   [retreat] cannot read the TCP; leaving the arm where it is")
        return False
    xyz, quat = measured
    axis = R.from_quat(quat).as_matrix() @ np.array([0.0, 0.0, 1.0])
    target = xyz + axis * (retreat_mm / 1000.0)
    print(f"   [retreat] moving {retreat_mm} mm back along the tool axis to "
          f"{np.round(target, 4).tolist()}")
    code, text = run([str(PLAN_LOCKED), *[f"{v:.6f}" for v in (*target, *quat)],
                      "--velocity", "0.005", "--goal-tol", "0.004",
                      "--output", "/tmp/compliant_retreat.json"], timeout=300.0)
    if code != 0:
        print("   [retreat] plan refused; the arm stays put (owner intervention)")
        return False
    code, text = run([str(EXECUTOR), "/tmp/compliant_retreat.json",
                      "--time-scale", "3.0", "--goal-tolerance", "0.004",
                      "--verify-final", "0.004", "--fault-poll", "0.2"], timeout=600.0)
    ok = "ALL_ASSEMBLY_CHUNKS_OK" in text
    print(f"   [retreat] {'OK' if ok else 'FAILED'}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--segments", type=int, default=6)
    parser.add_argument("--tol-mm", type=float, default=3.0)
    parser.add_argument("--contact", type=float, default=16.68)
    parser.add_argument("--safety", type=float, default=17.63)
    parser.add_argument("--retreat-mm", type=float, default=20.0)
    parser.add_argument("--no-retreat", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    target_xyz = np.array(args.target[:3])
    target_quat = np.array(args.target[3:])
    print(f"target TCP {np.round(target_xyz, 4).tolist()} "
          f"contact={args.contact} safety={args.safety} tol={args.tol_mm} mm")

    code, text = plan_insert(target_xyz, target_quat)
    if code != 0:
        for line in text.splitlines():
            print("   ", line)
        print("PLAN_REFUSED")
        return 2
    for line in text.splitlines()[-2:]:
        print("   ", line)
    if args.dry_run:
        print("dry run: plan only, no motion, no monitoring")
        return 0

    rclpy.init()
    monitor = Monitor(args.contact, args.safety)
    if not monitor.seed(1.0):
        print("no effort stream; aborting")
        return 5

    paths = segment_plan(pathlib.Path("/tmp/compliant_insert_plan.json"),
                         args.segments, 2.0)
    time_scale = 2.0
    current: subprocess.Popen | None = None
    result = 0
    for index, path in enumerate(paths):
        print(f"segment {index + 1}/{len(paths)}  time_scale={time_scale}")
        current = subprocess.Popen(
            [sys.executable, "-u", str(EXECUTOR), str(path),
             "--time-scale", str(time_scale), "--goal-tolerance", "0.0003",
             "--verify-final", "0.0003", "--fault-poll", "0.2"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            cwd=str(REPO))
        safety_hit = False
        contact_hit = False
        period = 1.0 / MONITOR_HZ
        while current.poll() is None:
            deadline = time.time() + period
            contact_now, safety_now, fast = monitor.evaluate()
            if safety_now:
                safety_hit = True
                print(f"   [monitor] SAFETY trip: fast={fast:.3f} >= {args.safety}")
                break
            if contact_now:
                contact_hit = True
            while time.time() < deadline:
                rclpy.spin_once(monitor, timeout_sec=0.01)
        output, _ = current.communicate()
        ok = "ALL_ASSEMBLY_CHUNKS_OK" in output
        if safety_hit:
            monitor.cancel_goal()
            if not args.no_retreat:
                retreat_along_tool(args.retreat_mm, monitor)
            result = 3
            break
        if not ok:
            print("   segment FAILED; stopping")
            result = 4
            break
        for line in output.splitlines()[-1:]:
            print("   ", line)
        if contact_hit:
            time_scale = min(time_scale * 2.0, 8.0)
            print(f"   contact detected during segment; next time_scale={time_scale}")
    if result:
        monitor.destroy_node()
        rclpy.shutdown()
        return result

    measured = monitor.mean_tcp()
    if measured is None:
        print("cannot verify the final TCP")
        result = 1
    else:
        xyz, quat = measured
        pos_err = float(np.linalg.norm(xyz - target_xyz) * 1000.0)
        rot_err = float((R.from_quat(quat).inv()
                         * R.from_quat(target_quat)).magnitude() * 180 / np.pi)
        print(f"final TCP err {pos_err:.2f} mm / {rot_err:.2f} deg (tol {args.tol_mm} mm)")
        result = 0 if pos_err <= args.tol_mm else 1
    monitor.destroy_node()
    rclpy.shutdown()
    print("INSERT_DONE" if result == 0 else f"INSERT_INCOMPLETE (code {result})")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
