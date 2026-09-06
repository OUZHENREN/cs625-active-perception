#!/usr/bin/env python3
"""Send one guarded P7 gripper target and atomically capture joint acknowledgement."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-id", required=True)
    parser.add_argument("--target-position-m", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    arguments = parser.parse_args()
    if not 0.0 <= arguments.target_position_m <= 0.04:
        parser.error("--target-position-m must be within [0.0, 0.04]")
    return arguments


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    rclpy.init()
    node = Node("p7_execute_gripper_capture")
    publisher = node.create_publisher(String, "/p7/gripper_command", 10)
    receipt: dict | None = None

    def on_status(message: String) -> None:
        nonlocal receipt
        try:
            decoded = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(decoded, dict) and decoded.get("command_id") == arguments.command_id:
            receipt = decoded

    node.create_subscription(String, "/p7/gripper_status", on_status, 10)
    discovery_deadline = time.monotonic() + 8.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("P7 gripper adapter subscription unavailable")

    command = {
        "command_id": arguments.command_id,
        "target_position_m": arguments.target_position_m,
    }
    outgoing = String()
    outgoing.data = json.dumps(command, separators=(",", ":"))
    publisher.publish(outgoing)
    deadline = time.monotonic() + arguments.timeout_sec
    while receipt is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if receipt is None:
        raise RuntimeError("P7 gripper receipt timeout")

    record = {
        "capture_schema": "p7_gripper_receipt_v1",
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
