#!/usr/bin/env python3
"""Align the simulated arm to a static P7.2 observation state without MoveIt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--positions", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=12.0)
    parser.add_argument("--terminal-tolerance-rad", type=float, default=0.01)
    args = parser.parse_args()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError("set CS625_P7_SIMULATION_EXECUTION=1 only in isolated P7 simulation")
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {args.output}")
    target_data = yaml.safe_load(args.positions.read_text(encoding="utf-8"))
    target = {name: float(target_data[name]) for name in JOINT_NAMES}

    rclpy.init()
    node = rclpy.create_node("p7_2_align_observation_joints")
    measured: dict[str, float] = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda message: measured.update(zip(message.name, message.position)), 10,
    )
    deadline = time.monotonic() + 5.0
    while any(name not in measured for name in JOINT_NAMES) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if any(name not in measured for name in JOINT_NAMES):
        raise RuntimeError("six-axis joint state unavailable")
    initial = {name: measured[name] for name in JOINT_NAMES}

    client = ActionClient(
        node, FollowJointTrajectory,
        "/joint_trajectory_controller/follow_joint_trajectory",
    )
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("trajectory action unavailable")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [target[name] for name in JOINT_NAMES]
    seconds = int(args.duration_sec)
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = int((args.duration_sec - seconds) * 1e9)
    goal.trajectory.points = [point]
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=8.0)
    handle = sent.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("observation alignment trajectory rejected")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(
        node, result_future, timeout_sec=max(30.0, args.duration_sec + 15.0)
    )
    wrapped = result_future.result()
    if wrapped is None:
        raise RuntimeError("observation alignment result timeout")
    settle_deadline = time.monotonic() + 1.0
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    final = {name: measured[name] for name in JOINT_NAMES}
    errors = {name: abs(final[name] - target[name]) for name in JOINT_NAMES}
    success = wrapped.result.error_code == 0 and max(errors.values()) <= args.terminal_tolerance_rad
    record = {
        "schema": "p7_2_observation_alignment_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "gate": "P7.2_POSE_ESTIMATION_INPUT_SETUP",
        "method": "direct ros2_control joint-space fixture alignment; MoveIt not launched",
        "moveit_called": False,
        "trajectory_commands_sent": 1,
        "initial_positions_rad": initial,
        "target_positions_rad": target,
        "final_positions_rad": final,
        "terminal_errors_rad": errors,
        "maximum_terminal_error_rad": max(errors.values()),
        "terminal_tolerance_rad": args.terminal_tolerance_rad,
        "controller_error_code": int(wrapped.result.error_code),
        "success": success,
        "claim_boundary": "observation fixture setup only; not P7.4 MoveIt execution evidence",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(record, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
