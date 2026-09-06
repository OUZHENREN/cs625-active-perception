#!/usr/bin/env python3
"""Execute one P7 camera-view command and record the actual TF path."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-id", required=True)
    parser.add_argument("--position", required=True, nargs=3, type=float)
    parser.add_argument("--orientation", required=True, nargs=4, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--trace-frame", default="camera_depth_optical_frame")
    joint_goal = parser.add_mutually_exclusive_group()
    joint_goal.add_argument("--joint-goal-file", type=Path)
    joint_goal.add_argument(
        "--joint-goal-rad", nargs=6, type=float, metavar=JOINT_NAMES
    )
    parser.add_argument("--sample-rate-hz", type=float, default=20.0)
    parser.add_argument("--timeout-sec", type=float, default=245.0)
    parser.add_argument("--contact-monitor-bin", type=Path)
    arguments = parser.parse_args()
    quaternion_norm = math.sqrt(sum(value * value for value in arguments.orientation))
    if abs(quaternion_norm - 1.0) > 1.0e-3:
        parser.error("--orientation must be a unit quaternion")
    if arguments.sample_rate_hz <= 0.0 or arguments.timeout_sec <= 0.0:
        parser.error("sample rate and timeout must be positive")
    if arguments.joint_goal_rad is not None and any(
        not math.isfinite(value) for value in arguments.joint_goal_rad
    ):
        parser.error("--joint-goal-rad values must be finite")
    return arguments


def load_joint_goal(arguments: argparse.Namespace) -> tuple[dict[str, float] | None, str | None]:
    if arguments.joint_goal_file is not None:
        source = yaml.safe_load(arguments.joint_goal_file.read_text(encoding="utf-8"))
        if not isinstance(source, dict):
            raise ValueError("joint-goal YAML must contain a name-to-position mapping")
        try:
            result = {name: float(source[name]) for name in JOINT_NAMES}
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("joint-goal YAML must define all six CS625 joints") from error
        if any(not math.isfinite(value) for value in result.values()):
            raise ValueError("joint-goal YAML values must be finite")
        return result, str(arguments.joint_goal_file)
    if arguments.joint_goal_rad is not None:
        return dict(zip(JOINT_NAMES, arguments.joint_goal_rad)), "command_line"
    return None, None


def quaternion_error_rad(expected: list[float], observed: list[float]) -> float:
    dot = abs(sum(left * right for left, right in zip(expected, observed)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def main() -> None:
    arguments = parse_arguments()
    joint_goal_positions, joint_goal_source = load_joint_goal(arguments)
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )

    rclpy.init()
    node = Node("p7_execute_view_trace")
    publisher = node.create_publisher(String, "/p7/arm_motion_command", 10)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    receipt: dict | None = None

    def on_status(message: String) -> None:
        nonlocal receipt
        try:
            decoded = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(decoded, dict) and decoded.get("command_id") == arguments.command_id:
            receipt = decoded

    node.create_subscription(String, "/p7/arm_motion_status", on_status, 10)
    discovery_deadline = time.monotonic() + 8.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("P7 arm-motion adapter subscription unavailable")

    samples: list[dict] = []
    tf_miss_count = 0
    start_wall = time.monotonic()

    def sample_tf() -> None:
        nonlocal tf_miss_count
        try:
            transform = tf_buffer.lookup_transform(
                arguments.frame_id, arguments.trace_frame, Time()
            ).transform
        except TransformException:
            tf_miss_count += 1
            return
        samples.append(
            {
                "elapsed_sec": round(time.monotonic() - start_wall, 6),
                "position_m": [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                "orientation_xyzw": [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ],
            }
        )

    tf_deadline = time.monotonic() + 8.0
    while not samples and time.monotonic() < tf_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        sample_tf()
    if not samples:
        raise RuntimeError(
            f"TF unavailable: {arguments.frame_id} -> {arguments.trace_frame}"
        )

    x, y, z = arguments.position
    qx, qy, qz, qw = arguments.orientation
    command = {
        "command_id": arguments.command_id,
        "phase": "view",
        "physical_tool_frame": arguments.trace_frame,
        "pose": {
            "frame_id": arguments.frame_id,
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
    }
    if joint_goal_positions is not None:
        command["joint_goal_positions_rad"] = joint_goal_positions
    outgoing = String()
    outgoing.data = json.dumps(command, separators=(",", ":"))
    monitor = None
    contact_path = arguments.output.with_name(arguments.output.stem + "_contacts.json")
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    if arguments.contact_monitor_bin:
        monitor = subprocess.Popen(
            [str(arguments.contact_monitor_bin), str(arguments.timeout_sec + 20), str(contact_path)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        selector = selectors.DefaultSelector()
        selector.register(monitor.stdout, selectors.EVENT_READ)
        ready = False
        ready_deadline = time.monotonic() + 12
        while time.monotonic() < ready_deadline and monitor.poll() is None:
            if selector.select(timeout=0.1):
                line = monitor.stdout.readline().strip()
                if line == "P7_CONTACT_MONITOR_READY":
                    ready = True
                    break
        selector.close()
        if not ready:
            monitor.terminate()
            monitor.communicate(timeout=5)
            raise RuntimeError("contact monitor did not receive its sensor heartbeat")
    command_issued_at_utc = datetime.now(timezone.utc).isoformat()
    publisher.publish(outgoing)

    period = 1.0 / arguments.sample_rate_hz
    next_sample = time.monotonic()
    deadline = time.monotonic() + arguments.timeout_sec
    while receipt is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, period))
        now = time.monotonic()
        if now >= next_sample:
            sample_tf()
            next_sample = now + period
    sample_tf()
    receipt_received_at_utc = datetime.now(timezone.utc).isoformat()
    contact = None
    if monitor is not None:
        monitor_was_live_at_receipt = monitor.poll() is None
        if monitor_was_live_at_receipt:
            monitor.send_signal(signal.SIGINT)
        monitor_stdout, _ = monitor.communicate(timeout=8)
        contact = {
            "path": str(contact_path),
            "monitor_live_at_receipt": monitor_was_live_at_receipt,
            "monitor_return_code": monitor.returncode,
            "monitor_stdout": monitor_stdout.strip(),
        }
        if contact_path.exists():
            contact["observation"] = json.loads(contact_path.read_text(encoding="utf-8"))
        contact["covers_command_through_receipt"] = bool(
            monitor_was_live_at_receipt and contact.get("observation", {}).get("monitor_available")
        )
    if receipt is None:
        raise RuntimeError("P7 arm-motion receipt timeout")

    cartesian_path_length_m = sum(
        math.dist(previous["position_m"], current["position_m"])
        for previous, current in zip(samples, samples[1:])
    )
    terminal_position_error_m = math.dist(samples[-1]["position_m"], arguments.position)
    terminal_orientation_error_rad = quaternion_error_rad(
        list(arguments.orientation), samples[-1]["orientation_xyzw"]
    )
    record = {
        "capture_schema": "p7_view_motion_trace_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "command_issued_at_utc": command_issued_at_utc,
        "receipt_received_at_utc": receipt_received_at_utc,
        "gazebo_contacts": contact,
        "command": command,
        "joint_goal_source": joint_goal_source,
        "trace": {
            "frame_id": arguments.frame_id,
            "trace_frame": arguments.trace_frame,
            "sample_rate_hz": arguments.sample_rate_hz,
            "sample_count": len(samples),
            "tf_miss_count": tf_miss_count,
            "actual_cartesian_path_length_m": round(cartesian_path_length_m, 6),
            "terminal_position_error_m": round(terminal_position_error_m, 6),
            "terminal_orientation_error_rad": round(terminal_orientation_error_rad, 6),
            "samples": samples,
        },
        "receipt": receipt,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(arguments.output)
    print(json.dumps({"receipt": receipt, "trace": {k: v for k, v in record["trace"].items() if k != "samples"}}, sort_keys=True))

    node.destroy_node()
    rclpy.shutdown()
    if receipt.get("success") is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
