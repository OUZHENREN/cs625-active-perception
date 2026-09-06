#!/usr/bin/env python3
"""Execute one guarded P7 motion phase and atomically capture its receipt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-id", required=True)
    parser.add_argument(
        "--phase", required=True, choices=("view", "pregrasp", "approach", "lift")
    )
    parser.add_argument("--position", required=True, nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--orientation", required=True, nargs=4, type=float,
        metavar=("QX", "QY", "QZ", "QW"),
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=245.0)
    parser.add_argument("--joint-goal-file", type=Path,
                        help="accepted pregrasp branch JSON; adapter still plans and executes")
    parser.add_argument("--command-topic", default="/p7/arm_motion_command")
    parser.add_argument("--status-topic", default="/p7/arm_motion_status")
    arguments = parser.parse_args()
    norm = math.sqrt(sum(value * value for value in arguments.orientation))
    if abs(norm - 1.0) > 1.0e-3:
        parser.error("--orientation must be a unit quaternion")
    if arguments.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be positive")
    return arguments


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    rclpy.init()
    node = Node("p7_execute_pose_capture")
    publisher = node.create_publisher(String, arguments.command_topic, 10)
    receipt: dict | None = None

    def on_status(message: String) -> None:
        nonlocal receipt
        try:
            decoded = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(decoded, dict) and decoded.get("command_id") == arguments.command_id:
            receipt = decoded

    node.create_subscription(String, arguments.status_topic, on_status, 10)
    discovery_deadline = time.monotonic() + 8.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("P7 arm-motion adapter subscription unavailable")

    x, y, z = arguments.position
    qx, qy, qz, qw = arguments.orientation
    command = {
        "command_id": arguments.command_id,
        "phase": arguments.phase,
        "pose": {
            "frame_id": "base_link",
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
    }
    if arguments.joint_goal_file:
        branch = json.loads(arguments.joint_goal_file.read_text(encoding="utf-8"))
        if branch.get("success") is not True or arguments.phase not in ("view", "pregrasp"):
            raise ValueError("accepted joint branch requires view or pregrasp phase")
        command["joint_goal_positions_rad"] = branch["joint_goal_positions_rad"]
    outgoing = String()
    outgoing.data = json.dumps(command, separators=(",", ":"))
    publisher.publish(outgoing)
    deadline = time.monotonic() + arguments.timeout_sec
    while receipt is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if receipt is None:
        raise RuntimeError("P7 arm-motion receipt timeout")

    record = {
        "capture_schema": "p7_phase_receipt_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
        "receipt": receipt,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(arguments.output)
    print(json.dumps(receipt, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    if receipt.get("success") is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
