#!/usr/bin/env python3
"""Capture one read-only P7.1 optical-axis marker diagnostic."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


TOPICS = {
    "color": "/sensors/camera/color/image",
    "depth": "/sensors/camera/depth/image",
    "camera_info": "/sensors/camera/depth/camera_info",
    "points": "/sensors/camera/points",
}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    return parser.parse_args()


def stamp_sec(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


def color_array(message):
    if message.encoding.lower() != "rgb8":
        raise ValueError(f"expected rgb8, got {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return rows[:, : message.width * 3].reshape(message.height, message.width, 3).copy()


def depth_array(message):
    if message.encoding.upper() != "32FC1":
        raise ValueError(f"expected 32FC1, got {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.step // 4)
    return rows[:, : message.width].copy()


def save_ppm(path, rgb):
    path.write_bytes(f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode() + rgb.tobytes())


def main():
    args = arguments()
    if args.output_dir.exists():
        raise FileExistsError(f"refusing to reuse diagnostic directory: {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node("p7_1_axis_marker_diagnostic")
    latest = {}
    subscriptions = []
    for name, message_type in (
        ("color", Image),
        ("depth", Image),
        ("camera_info", CameraInfo),
        ("points", PointCloud2),
    ):
        subscriptions.append(
            node.create_subscription(
                message_type,
                TOPICS[name],
                lambda message, stream=name: latest.__setitem__(stream, message),
                qos_profile_sensor_data,
            )
        )
    deadline = time.monotonic() + args.timeout_sec
    while len(latest) != len(TOPICS) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        if len(latest) != len(TOPICS):
            raise TimeoutError(f"missing streams: {sorted(set(TOPICS) - set(latest))}")
        rgb = color_array(latest["color"])
        depth = depth_array(latest["depth"])
        xyz = np.asarray(
            point_cloud2.read_points_numpy(
                latest["points"], field_names=["x", "y", "z"], skip_nans=False
            ),
            dtype=float,
        ).reshape((-1, 3))
        save_ppm(args.output_dir / "color.ppm", rgb)
        np.save(args.output_dir / "depth.npy", depth, allow_pickle=False)
        np.save(args.output_dir / "points_xyz_m.npy", xyz, allow_pickle=False)
        red = (
            (rgb[:, :, 0].astype(int) >= 180)
            & (rgb[:, :, 0].astype(int) >= rgb[:, :, 1].astype(int) + 70)
            & (rgb[:, :, 0].astype(int) >= rgb[:, :, 2].astype(int) + 70)
        )
        rows, cols = np.nonzero(red)
        centre = (rgb.shape[0] // 2, rgb.shape[1] // 2)
        centre_depth = float(depth[centre])
        finite_xyz = np.all(np.isfinite(xyz), axis=1)
        record = {
            "schema": "p7_1_axis_marker_diagnostic_v1",
            "evidence_class": "DIAGNOSTIC_ONLY",
            "acceptance_effect": "none",
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "runtime": {
                "ros_domain_id": __import__("os").environ.get("ROS_DOMAIN_ID", ""),
                "gz_partition": __import__("os").environ.get("GZ_PARTITION", ""),
            },
            "streams": {
                name: {
                    "topic": TOPICS[name],
                    "frame_id": message.header.frame_id,
                    "stamp_sec": stamp_sec(message),
                }
                for name, message in latest.items()
            },
            "red_marker": {
                "pixel_count": int(red.sum()),
                "visible": bool(red.any()),
                "centroid_uv_px": (
                    [float(cols.mean()), float(rows.mean())] if red.any() else None
                ),
                "bounds_uv_px": (
                    [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())]
                    if red.any()
                    else None
                ),
            },
            "depth": {
                "centre_value_m": centre_depth,
                "centre_finite": bool(np.isfinite(centre_depth)),
                "finite_pixel_count": int(np.isfinite(depth).sum()),
            },
            "point_cloud": {
                "finite_point_count": int(finite_xyz.sum()),
                "finite_xyz_min_m": np.min(xyz[finite_xyz], axis=0).tolist() if finite_xyz.any() else None,
                "finite_xyz_max_m": np.max(xyz[finite_xyz], axis=0).tolist() if finite_xyz.any() else None,
            },
            "interpretation_rule": (
                "visible marker isolates YCB mesh/resource failure; invisible marker isolates "
                "camera-axis, self-occlusion, or camera rendering failure"
            ),
        }
        (args.output_dir / "diagnostic.json").write_text(
            json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(record, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
