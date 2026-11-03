#!/usr/bin/env python3
"""Trigger an RVS capture command and capture the pose it returns.

Why this exists
---------------
The first step of the task is "where is the workpiece / slot".  RVS answers that
through its command server (an ASCII line ``CMD,x,y,z,rx,ry,rz`` in millimetres
and degrees, flange pose), and the senior's ``tcp_server`` bridge republishes the
answer on ROS pose topics.  This script is the thin client for that round trip,
so the teaching flow does not depend on typing socket commands by hand.

Command -> topic that carries the answer
---------------------------------------
    T2  -> /box_precision_view_tcp_pose   (coarse workpiece observation pose)
    T3  -> /box_target_pose               (fine workpiece grasp pose)
    T1  -> /slot_target_pose              (slot pose; refine with /slot_precision_view_tcp_pose)
    T0  -> /slot_precision_view_tcp_pose  (coarse slot observation pose)

Data flow
---------
    TF base_link->flange  --+
                            +--> "CMD,x,y,z,rx,ry,rz" --7000--> RVS
    (flange pose, mm/deg)   |                                     |
                            |                                     v
    ROS pose topics  <--9999-- tcp_server  <----------------- RVS pose answer

The script never moves the robot and never plans: it only speaks to RVS and
records what came back, which makes it safe to run before any motion step.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

RVS_HOST = "127.0.0.1"
RVS_PORT = 7000

ANSWER_TOPIC = {
    "T2": "/box_precision_view_tcp_pose",
    "T3": "/box_target_pose",
    "T1": "/slot_target_pose",
    "T0": "/slot_precision_view_tcp_pose",
}

REPO = Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "runtime" / "rvs_teaching"


class PoseCatcher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("rvs_pose_client")
        self.received: list[PoseStamped] = []
        self.create_subscription(PoseStamped, topic,
                                 lambda msg: self.received.append(msg), 10)

    def wait_for_pose(self, timeout_sec: float) -> PoseStamped | None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.received:
                return self.received[-1]
        return None


def flange_pose(node: Node) -> tuple[list[float], list[float]] | None:
    """Flange pose in the base frame, millimetres and degrees (what RVS expects).

    The cell publishes several unconnected TF trees, so ``base_link -> flange``
    lookups are unreliable.  The URDF makes the flange and the TCP tool link
    coincide (``flange_to_eef_joint`` has a zero offset), therefore the TCP pose
    topic is the flange pose and is used first; TF stays as a fallback.
    """
    latest: dict = {}
    sub = node.create_subscription(
        PoseStamped, "/tcp_pose_broadcaster/pose",
        lambda msg: latest.__setitem__("m", msg), 10)
    deadline = time.time() + 6.0
    while "m" not in latest and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" in latest:
        msg = latest["m"]
        p = msg.pose.position
        q = msg.pose.orientation
        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
        return [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0], [float(v) for v in rpy]
    print("no TCP pose topic, falling back to TF")
    import tf2_ros
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    last_error: Exception | None = None
    deadline = time.time() + 15.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = buffer.lookup_transform("base_link", "flange", rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as exc:
            last_error = exc
            continue
        t = tf.transform.translation
        q = tf.transform.rotation
        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
        return [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0], [float(v) for v in rpy]
    print(f"NO_FLANGE_TF last_error={last_error}")
    return None


# URDF (cs625_active_perception.urdf.xacro): flange -> tool0 is identity, and the
# controller TCP (my_end_effector_link) sits at this offset in the flange frame.
# RVS is fed `tool0`, the tcp_pose_broadcaster topic carries the TCP, so the two
# differ by exactly this vector.
FLANGE_TO_TCP = (-0.046, 0.0, 0.353)


def tcp_to_tool0(xyz_mm: list[float], rpy_deg: list[float]) -> list[float]:
    """Convert the controller TCP pose into the tool0 pose RVS expects."""
    rot = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    p_tcp = np.array(xyz_mm, dtype=float)
    p_tool0 = p_tcp - rot @ np.array(FLANGE_TO_TCP) * 1000.0
    return [float(v) for v in p_tool0]


def shift_along_tool_z(xyz_mm: list[float], rpy_deg: list[float],
                       offset_m: float) -> list[float]:
    """Move the reported point backwards along the tool z axis.

    RVS models the arm together with its tool, so the pose it expects is the
    flange, which sits ``offset_m`` behind the TCP along tool z.  Sending the TCP
    directly makes every returned pose short by that offset.
    """
    if abs(offset_m) < 1e-9:
        return xyz_mm
    rot = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    axis = rot @ np.array([0.0, 0.0, 1.0])
    point = np.array(xyz_mm, dtype=float) - axis * offset_m * 1000.0
    return [float(v) for v in point]


def send_command(command: str, xyz_mm: list[float], rpy_deg: list[float]) -> str:
    payload = ",".join([command] + [f"{v:.3f}" for v in (*xyz_mm, *rpy_deg)]) + "\n"
    with socket.create_connection((RVS_HOST, RVS_PORT), timeout=5) as sock:
        sock.sendall(payload.encode())
        sock.shutdown(socket.SHUT_WR)
    return payload.strip()


# --- multi-trigger policy (owner, 2026-10-30) ---------------------------------
# RVS rarely succeeds on the first trigger, so collect at least MIN_FRAMES
# plausible answers and stop early once enough are seen, but never try more
# than MAX_ATTEMPTS times.  RVS occasionally answers with garbage
# (2026-10-30 T0: x = 4.41 m twice before a good answer); implausible frames
# are saved for the record but do not count.
MIN_FRAMES_DEFAULT = 3
MAX_ATTEMPTS_DEFAULT = 10

# Plausibility box for an answer (metres, base frame).
WORKSPACE = {"x": (-0.6, 1.8), "y": (-1.2, 1.6), "z": (-0.9, 1.6)}


def plausible(record: dict) -> bool:
    p = record["position"]
    for i, axis in enumerate("xyz"):
        if not (WORKSPACE[axis][0] <= p[i] <= WORKSPACE[axis][1]):
            return False
    norm = math.sqrt(sum(v * v for v in record["orientation"]))
    return 0.99 <= norm <= 1.01


def spread_report(frames: list[dict]) -> tuple[float, float]:
    """Max pairwise position distance (mm) and orientation angle (deg)."""
    max_mm = 0.0
    max_deg = 0.0
    for a in frames:
        for b in frames:
            dp = math.dist(a["position"], b["position"]) * 1000.0
            da = float(np.degrees((R.from_quat(a["orientation"]).inv()
                                   * R.from_quat(b["orientation"])).magnitude()))
            max_mm = max(max_mm, dp)
            max_deg = max(max_deg, da)
    return max_mm, max_deg


# Target poses (R1/S3) describe FIXED objects, so their frames must agree
# (2026-11-02 audit, hole 6).  View poses (S0/S2) legitimately vary because RVS
# recomputes them per trigger, so no limit applies there.
SPREAD_LIMITS = {"T1": (30.0, 3.0), "T3": (30.0, 3.0)}


def spread_violation(command: str, max_mm: float, max_deg: float) -> str | None:
    """Reason string when the frame agreement is unacceptable, else None."""
    limit = SPREAD_LIMITS.get(command)
    if limit is None:
        return None
    if max_mm <= limit[0] and max_deg <= limit[1]:
        return None
    return (f"SPREAD_TOO_LARGE for {command}: {max_mm:.1f} mm / {max_deg:.2f} deg "
            f"exceeds {limit[0]:.0f} mm / {limit[1]:.1f} deg; "
            "the target may have moved between triggers")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=sorted(ANSWER_TOPIC))
    parser.add_argument("--tool-offset", type=float, default=0.0,
                        help="metres from flange to TCP along tool z; RVS expects the flange")
    parser.add_argument("--timeout", type=float, default=300.0,
                        help="per-attempt wait; T3 was ~17 s on 10-27 but T2 took 229 s "
                             "on 2026-10-30, so 300 s is the safe default")
    parser.add_argument("--min-frames", type=int, default=MIN_FRAMES_DEFAULT,
                        help="stop once this many plausible answers arrived")
    parser.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS_DEFAULT,
                        help="give up after this many triggers (owner: never more than 10)")
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = PoseCatcher(ANSWER_TOPIC[args.command])
    try:
        pose = flange_pose(node)
        if pose is None:
            print("REFUSED: cannot read the flange pose, RVS needs it")
            return 2
        xyz_mm, rpy_deg = pose
        raw = list(xyz_mm)
        if args.tool_offset > 0.0:
            xyz_mm = shift_along_tool_z(xyz_mm, rpy_deg, args.tool_offset)
        else:
            xyz_mm = tcp_to_tool0(xyz_mm, rpy_deg)
        print("tcp:", [round(v, 2) for v in raw],
              "-> tool0:", [round(v, 2) for v in xyz_mm],
              "(URDF flange->TCP offset applied)")

        frames: list[dict] = []
        attempt = 0
        for attempt in range(1, args.max_attempts + 1):
            node.received = []
            sent = send_command(args.command, xyz_mm, rpy_deg)
            print(f"[attempt {attempt}/{args.max_attempts}] sent:", sent)
            started = time.time()
            answer = node.wait_for_pose(args.timeout)
            elapsed = time.time() - started
            if answer is None:
                print(f"   no answer after {elapsed:.0f}s")
                continue
            p = answer.pose.position
            q = answer.pose.orientation
            record = {
                "command": args.command,
                "topic": ANSWER_TOPIC[args.command],
                "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "attempt": attempt,
                "elapsed_sec": round(elapsed, 2),
                "position": [p.x, p.y, p.z],
                "orientation": [q.x, q.y, q.z, q.w],
                "cloud_main_tcp": 0.0,
                "frame": answer.header.frame_id,
            }
            out = OUT_DIR / (f"{args.command}_{time.strftime('%Y%m%d_%H%M%S')}"
                             f"_a{attempt:02d}.json")
            out.write_text(json.dumps(record, indent=2))
            if not plausible(record):
                print(f"   implausible frame {[round(v, 3) for v in record['position']]}"
                      " discarded (saved for the record)")
                continue
            frames.append(record)
            print(f"   valid #{len(frames)} {[round(v, 4) for v in record['position']]}"
                  f" ({elapsed:.1f}s) -> {out.name}")
            if len(frames) >= args.min_frames:
                break

        if len(frames) < args.min_frames:
            print(f"INSUFFICIENT_VALID_FRAMES: {len(frames)}/{args.min_frames} "
                  f"after {attempt} attempts")
            return 3
        max_mm, max_deg = spread_report(frames)
        print(f"spread over {len(frames)} frames: {max_mm:.1f} mm / {max_deg:.2f} deg")
        reason = spread_violation(args.command, max_mm, max_deg)
        if reason is not None:
            print(reason)
            return 4
        last = frames[-1]
        print("answer:", [round(v, 4) for v in last["position"]],
              [round(v, 4) for v in last["orientation"]],
              f"({last['elapsed_sec']:.1f}s)")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
