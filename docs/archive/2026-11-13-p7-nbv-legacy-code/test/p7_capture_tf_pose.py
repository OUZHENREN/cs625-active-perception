#!/usr/bin/env python3
"""Capture one live P7 TF transform as an atomic JSON pose record."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent-frame", default="base_link")
    parser.add_argument("--child-frame", default="p7_grasp_center_link")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=12.0)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    rclpy.init()
    node = rclpy.create_node("p7_capture_tf_pose")
    buffer = Buffer()
    TransformListener(buffer, node)
    transform = None
    last_error = ""
    deadline = time.monotonic() + arguments.timeout_sec
    while transform is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                arguments.parent_frame,
                arguments.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            last_error = str(error)
    if transform is None:
        raise RuntimeError(f"P7 TF unavailable: {last_error}")

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    record = {
        "capture_schema": "p7_tf_pose_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "parent_frame": arguments.parent_frame,
        "child_frame": arguments.child_frame,
        "transform_stamp": {
            "sec": transform.header.stamp.sec,
            "nanosec": transform.header.stamp.nanosec,
        },
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
        "pose": {
            "position": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
            },
            "orientation": {
                "x": rotation.x,
                "y": rotation.y,
                "z": rotation.z,
                "w": rotation.w,
            },
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
