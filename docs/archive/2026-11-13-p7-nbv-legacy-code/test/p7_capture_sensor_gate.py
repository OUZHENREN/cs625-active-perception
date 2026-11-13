#!/usr/bin/env python3
"""Capture the P7.1 RGB-D input gate without commanding robot motion.

Five synchronized acquisition windows are retained as ROS 2 CDR payloads.
Gazebo ground truth is used only to audit whether the rendered target is in
the camera field of view and represented by depth/point-cloud samples; it is
never published or reported as a pose-estimator result.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


CAPTURE_SCHEMA = "p7_1_sensor_gate_v1"
GATE_RULES_VERSION = "p7_1_sensor_gate_rules_v1"
STREAM_TYPES = {
    "color": "sensor_msgs/msg/Image",
    "depth": "sensor_msgs/msg/Image",
    "camera_info": "sensor_msgs/msg/CameraInfo",
    "points": "sensor_msgs/msg/PointCloud2",
}
TOPICS = {
    "color": "/sensors/camera/color/image",
    "depth": "/sensors/camera/depth/image",
    "camera_info": "/sensors/camera/depth/camera_info",
    "points": "/sensors/camera/points",
}
TARGET_LOCAL_CENTER_M = np.array((-0.0091685, 0.0840170, 0.0510065), dtype=float)
TARGET_RADIUS_M = 0.0340
TARGET_HEIGHT_M = 0.101855


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-pose-file", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--expected-windows", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--sync-slop-sec", type=float, default=0.15)
    parser.add_argument("--minimum-window-separation-sec", type=float, default=0.50)
    parser.add_argument("--minimum-target-points", type=int, default=12)
    parser.add_argument("--startup-warmup-sec", type=float, default=0.0,
                        help="Spin TF/sensor subscriptions then clear pre-start sensor buffers.")
    parser.add_argument(
        "--discard-initial-windows",
        type=int,
        default=0,
        help="Discard synchronized warm-up windows before immutable capture.",
    )
    arguments = parser.parse_args()
    if arguments.expected_windows < 1:
        parser.error("--expected-windows must be positive")
    if arguments.timeout_sec <= 0.0 or arguments.sync_slop_sec <= 0.0:
        parser.error("timeout and synchronization slop must be positive")
    if arguments.minimum_window_separation_sec < 0.0:
        parser.error("minimum window separation must be non-negative")
    if arguments.minimum_target_points < 1:
        parser.error("minimum target points must be positive")
    if arguments.discard_initial_windows < 0:
        parser.error("discarded warm-up window count must be non-negative")
    if not 0.0 <= arguments.startup_warmup_sec <= 10.0:
        parser.error("startup warmup must be within [0, 10] seconds")
    return arguments


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=json_scalar) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def json_scalar(value: Any) -> Any:
    """Convert NumPy scalar values without silently flattening arrays."""

    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def lookup_transform_at_exact_stamp(
    node: Any,
    tf_buffer: Buffer,
    parent: str,
    child: str,
    stamp: Any,
    *,
    timeout_sec: float = 0.50,
) -> Any:
    """Wait briefly for a transform at the requested message timestamp.

    Gazebo sensor messages can reach the subscriber a few milliseconds before
    the matching dynamic TF.  Spinning the same node lets that exact transform
    enter the buffer without falling back to the latest transform.
    """

    deadline = time.monotonic() + timeout_sec
    last_error: TransformException | None = None
    requested_time = Time.from_msg(stamp)
    while time.monotonic() < deadline:
        try:
            return tf_buffer.lookup_transform(parent, child, requested_time)
        except TransformException as error:
            last_error = error
            remaining = max(0.0, deadline - time.monotonic())
            if remaining > 0.0:
                rclpy.spin_once(node, timeout_sec=min(0.02, remaining))
    if last_error is not None:
        raise last_error
    raise TransformException(
        f"exact-time transform {parent} <- {child} unavailable after {timeout_sec:.3f}s"
    )


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def stamp_seconds(message: Any) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


def header_valid(message: Any) -> bool:
    return bool(message.header.frame_id) and stamp_seconds(message) > 0.0


def select_synchronized_window(
    buffers: dict[str, deque],
    *,
    after_points_stamp_sec: float,
    sync_slop_sec: float,
) -> dict[str, Any] | None:
    """Return the newest complete tuple around one unseen point-cloud stamp."""

    if any(not buffers[name] for name in STREAM_TYPES):
        return None
    for points in reversed(buffers["points"]):
        anchor = stamp_seconds(points)
        if anchor <= after_points_stamp_sec:
            continue
        selected = {"points": points}
        for name in ("color", "depth", "camera_info"):
            selected[name] = min(
                buffers[name], key=lambda message: abs(stamp_seconds(message) - anchor)
            )
        stamps = [stamp_seconds(message) for message in selected.values()]
        if max(stamps) - min(stamps) <= sync_slop_sec:
            return selected
    return None


def quaternion_matrix(quaternion: dict[str, float] | tuple[float, float, float, float]) -> np.ndarray:
    if isinstance(quaternion, dict):
        x, y, z, w = (float(quaternion[key]) for key in ("x", "y", "z", "w"))
    else:
        x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def transform_components(transform: Any) -> tuple[np.ndarray, np.ndarray]:
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=float,
    )
    rotation = quaternion_matrix(
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
    )
    return rotation, translation


def target_visibility_from_points(
    points_camera_m: np.ndarray,
    *,
    world_from_camera_rotation: np.ndarray,
    world_from_camera_translation: np.ndarray,
    target_model_position_world_m: np.ndarray,
    target_model_rotation_world: np.ndarray,
    radial_margin_m: float = 0.008,
    axial_margin_m: float = 0.008,
) -> dict[str, Any]:
    """Count observed points inside the versioned target collision proxy."""

    points = np.asarray(points_camera_m, dtype=float).reshape((-1, 3))
    finite = points[np.isfinite(points).all(axis=1)]
    if not len(finite):
        return {"finite_point_count": 0, "target_point_count": 0, "target_point_fraction": 0.0}
    world = finite @ world_from_camera_rotation.T + world_from_camera_translation
    local = (world - target_model_position_world_m) @ target_model_rotation_world
    relative = local - TARGET_LOCAL_CENTER_M
    radial = np.linalg.norm(relative[:, :2], axis=1)
    inside = (radial <= TARGET_RADIUS_M + radial_margin_m) & (
        np.abs(relative[:, 2]) <= TARGET_HEIGHT_M / 2.0 + axial_margin_m
    )
    count = int(np.count_nonzero(inside))
    return {
        "finite_point_count": int(len(finite)),
        "target_point_count": count,
        "target_point_fraction": count / float(len(finite)),
        "radial_margin_m": radial_margin_m,
        "axial_margin_m": axial_margin_m,
    }


def project_target_center(
    *,
    target_center_world_m: np.ndarray,
    world_from_camera_rotation: np.ndarray,
    world_from_camera_translation: np.ndarray,
    camera_info: CameraInfo,
) -> dict[str, Any]:
    camera = world_from_camera_rotation.T @ (
        target_center_world_m - world_from_camera_translation
    )
    z = float(camera[2])
    fx, fy, cx, cy = (
        float(camera_info.k[0]),
        float(camera_info.k[4]),
        float(camera_info.k[2]),
        float(camera_info.k[5]),
    )
    if z <= 0.0:
        return {
            "camera_xyz_m": camera.tolist(),
            "in_front": False,
            "inside_image": False,
            "u_px": None,
            "v_px": None,
        }
    u = float(fx * float(camera[0]) / z + cx)
    v = float(fy * float(camera[1]) / z + cy)
    return {
        "camera_xyz_m": camera.tolist(),
        "in_front": True,
        "inside_image": bool(
            0.0 <= u < camera_info.width and 0.0 <= v < camera_info.height
        ),
        "u_px": u,
        "v_px": v,
    }


def image_layout_valid(message: Image) -> bool:
    return (
        message.width > 0
        and message.height > 0
        and message.step > 0
        and bool(message.encoding)
        and len(message.data) >= message.step * message.height
    )


def camera_info_valid(message: CameraInfo) -> bool:
    return (
        message.width > 0
        and message.height > 0
        and len(message.k) == 9
        and math.isfinite(message.k[0])
        and math.isfinite(message.k[4])
        and message.k[0] > 0.0
        and message.k[4] > 0.0
    )


def color_array(message: Image) -> np.ndarray | None:
    channels_by_encoding = {
        "rgb8": (3, (0, 1, 2)),
        "bgr8": (3, (2, 1, 0)),
        "rgba8": (4, (0, 1, 2)),
        "bgra8": (4, (2, 1, 0)),
        "mono8": (1, (0, 0, 0)),
    }
    layout = channels_by_encoding.get(message.encoding.lower())
    if layout is None or not image_layout_valid(message):
        return None
    channels, order = layout
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        (message.height, message.step)
    )
    pixels = rows[:, : message.width * channels].reshape(
        (message.height, message.width, channels)
    )
    return pixels[:, :, list(order)]


def depth_array(message: Image) -> np.ndarray | None:
    formats = {
        "32fc1": np.dtype(">f4" if message.is_bigendian else "<f4"),
        "16uc1": np.dtype(">u2" if message.is_bigendian else "<u2"),
    }
    dtype = formats.get(message.encoding.lower())
    if dtype is None or not image_layout_valid(message):
        return None
    item_size = dtype.itemsize
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        (message.height, message.step)
    )
    packed = np.ascontiguousarray(rows[:, : message.width * item_size])
    return packed.view(dtype).reshape((message.height, message.width))


def rgb_roi_statistics(rgb: np.ndarray | None, projection: dict[str, Any], fx: float) -> dict[str, Any]:
    if rgb is None or not projection.get("inside_image") or projection.get("u_px") is None:
        return {"rgb_roi_available": False, "rgb_roi_sample_count": 0}
    z = max(float(projection["camera_xyz_m"][2]), 1.0e-6)
    half_size = max(4, int(math.ceil(fx * TARGET_RADIUS_M / z)))
    u = int(round(float(projection["u_px"])))
    v = int(round(float(projection["v_px"])))
    x0, x1 = max(0, u - half_size), min(rgb.shape[1], u + half_size + 1)
    y0, y1 = max(0, v - half_size), min(rgb.shape[0], v + half_size + 1)
    roi = rgb[y0:y1, x0:x1]
    if not roi.size:
        return {"rgb_roi_available": False, "rgb_roi_sample_count": 0}
    return {
        "rgb_roi_available": True,
        "rgb_roi_bounds_px": [x0, y0, x1, y1],
        "rgb_roi_sample_count": int(roi.shape[0] * roi.shape[1]),
        "rgb_roi_mean": [float(value) for value in roi.mean(axis=(0, 1))],
        "rgb_roi_std": [float(value) for value in roi.std(axis=(0, 1))],
    }


def save_ppm(path: Path, rgb: np.ndarray) -> None:
    header = f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode("ascii")
    atomic_write_bytes(path, header + np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())


def error_code_list(window: dict[str, Any]) -> list[str]:
    failures = []
    if not window["synchronization"]["within_slop"]:
        failures.append("P7_1_STREAMS_NOT_SYNCHRONIZED")
    if not all(item["header_valid"] for item in window["streams"].values()):
        failures.append("P7_1_INVALID_HEADER")
    if not window["validations"]["color_layout_valid"]:
        failures.append("P7_1_COLOR_INVALID")
    if not window["validations"]["depth_layout_valid"]:
        failures.append("P7_1_DEPTH_INVALID")
    if not window["validations"]["camera_info_valid"]:
        failures.append("P7_1_CAMERA_INFO_INVALID")
    if not window["validations"]["point_cloud_layout_valid"]:
        failures.append("P7_1_POINT_CLOUD_INVALID")
    if not window["tf"]["base_to_color_available"] or not window["tf"]["base_to_depth_available"]:
        failures.append("P7_1_TIMESTAMP_EXACT_TF_UNAVAILABLE")
    if not window["target_visibility"]["projected_inside_image"]:
        failures.append("P7_1_TARGET_OUTSIDE_CAMERA_FOV")
    if not window["target_visibility"].get("rgb_roi_available", False):
        failures.append("P7_1_TARGET_RGB_ROI_UNAVAILABLE")
    if not window["target_visibility"]["point_cloud_support_pass"]:
        failures.append("P7_1_TARGET_NOT_OBSERVED_IN_POINT_CLOUD")
    return sorted(set(failures))


def message_record(name: str, message: Any, path: Path) -> dict[str, Any]:
    payload = serialize_message(message)
    atomic_write_bytes(path, payload)
    record = {
        "topic": TOPICS[name],
        "ros_type": STREAM_TYPES[name],
        "cdr_file": path.name,
        "cdr_size_bytes": len(payload),
        "cdr_sha256": hashlib.sha256(payload).hexdigest(),
        "header_valid": header_valid(message),
        "stamp_sec": stamp_seconds(message),
        "frame_id": message.header.frame_id,
    }
    if isinstance(message, Image):
        record.update(
            {
                "width": int(message.width),
                "height": int(message.height),
                "encoding": message.encoding,
                "step": int(message.step),
            }
        )
    elif isinstance(message, CameraInfo):
        record.update(
            {
                "width": int(message.width),
                "height": int(message.height),
                "distortion_model": message.distortion_model,
                "k": [float(value) for value in message.k],
            }
        )
    else:
        record.update(
            {
                "width": int(message.width),
                "height": int(message.height),
                "point_step": int(message.point_step),
                "row_step": int(message.row_step),
                "fields": [field.name for field in message.fields],
            }
        )
    return record


def transform_record(transform: Any) -> dict[str, Any]:
    value = transform.transform
    return {
        "parent_frame": transform.header.frame_id,
        "child_frame": transform.child_frame_id,
        "stamp_sec": float(transform.header.stamp.sec)
        + float(transform.header.stamp.nanosec) * 1.0e-9,
        "translation_m": {
            "x": value.translation.x,
            "y": value.translation.y,
            "z": value.translation.z,
        },
        "orientation": {
            "x": value.rotation.x,
            "y": value.rotation.y,
            "z": value.rotation.z,
            "w": value.rotation.w,
        },
    }


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_1_SENSOR_GATE") != "1":
        raise RuntimeError(
            "set CS625_P7_1_SENSOR_GATE=1 only for the isolated P7.1 simulation"
        )
    if arguments.output_dir.exists():
        raise FileExistsError(f"refusing to reuse output directory: {arguments.output_dir}")
    arguments.output_dir.mkdir(parents=True)
    started_at = datetime.now(timezone.utc)
    target_source = json.loads(arguments.target_pose_file.read_text(encoding="utf-8"))
    target_pose = target_source.get("pose", target_source)
    target_position = np.array(
        [target_pose["position"][axis] for axis in ("x", "y", "z")], dtype=float
    )
    target_rotation = quaternion_matrix(target_pose["orientation"])
    target_center_world = target_position + target_rotation @ TARGET_LOCAL_CENTER_M

    record: dict[str, Any] = {
        "capture_schema": CAPTURE_SCHEMA,
        "gate_rules_version": GATE_RULES_VERSION,
        "gate": "P7.1_PERCEPTION_INPUT",
        "record_status": "incomplete",
        "gate_pass": False,
        "started_at_utc": started_at.isoformat(),
        "finished_at_utc": None,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
            "real_hardware_connected": False,
            "trajectory_commands_sent": 0,
        },
        "scope": {
            "expected_windows": arguments.expected_windows,
            "sync_slop_sec": arguments.sync_slop_sec,
            "minimum_window_separation_sec": arguments.minimum_window_separation_sec,
            "minimum_target_points": arguments.minimum_target_points,
            "discard_initial_windows": arguments.discard_initial_windows,
            "startup_warmup_sec": arguments.startup_warmup_sec,
            "raw_format": "ROS 2 CDR serialization plus inspectable PPM/NumPy derivatives",
            "ground_truth_use": "visibility_audit_only_not_pose_estimation",
            "p7_2_pose_estimation_evaluated": False,
            "p7_3_nbv_evaluated": False,
            "p7_4_moveit_evaluated": False,
            "p7_5_grasp_evaluated": False,
        },
        "target_ground_truth_source": {
            "path": str(arguments.target_pose_file),
            "sha256": hashlib.sha256(arguments.target_pose_file.read_bytes()).hexdigest(),
            "model_position_world_m": target_position.tolist(),
            "model_orientation": target_pose["orientation"],
            "local_collision_center_m": TARGET_LOCAL_CENTER_M.tolist(),
            "collision_proxy_radius_m": TARGET_RADIUS_M,
            "collision_proxy_height_m": TARGET_HEIGHT_M,
        },
        "windows": [],
        "failure_codes": [],
    }

    rclpy.init()
    node = rclpy.create_node(
        "p7_1_sensor_gate_capture",
        parameter_overrides=[rclpy.Parameter("use_sim_time", value=True)],
    )
    buffers = {name: deque(maxlen=30) for name in STREAM_TYPES}
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
                lambda message, stream=name: buffers[stream].append(message),
                qos_profile_sensor_data,
            )
        )
    tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
    tf_listener = TransformListener(tf_buffer, node)
    deadline = time.monotonic() + arguments.timeout_sec
    last_points_stamp = -math.inf
    last_window_stamp = -math.inf
    discarded_initial_windows = 0

    try:
        # DDS may deliver images before the first dynamic TF. A fixed startup
        # phase followed by buffer clearing prevents old pre-TF timestamps
        # from entering the measured windows; it never filters on visibility
        # or pose success. All subsequent failures remain gate failures.
        warmup_deadline = time.monotonic() + arguments.startup_warmup_sec
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if arguments.startup_warmup_sec > 0:
            record["startup_buffered_messages_cleared"] = {
                name: len(messages) for name, messages in buffers.items()}
            for messages in buffers.values():
                messages.clear()
        while len(record["windows"]) < arguments.expected_windows and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            synchronized = select_synchronized_window(
                buffers,
                after_points_stamp_sec=last_points_stamp,
                sync_slop_sec=arguments.sync_slop_sec,
            )
            if synchronized is None:
                continue
            points_stamp = stamp_seconds(synchronized["points"])
            if points_stamp - last_window_stamp < arguments.minimum_window_separation_sec:
                last_points_stamp = points_stamp
                continue
            last_points_stamp = points_stamp
            last_window_stamp = points_stamp
            if discarded_initial_windows < arguments.discard_initial_windows:
                discarded_initial_windows += 1
                continue
            index = len(record["windows"]) + 1
            window_dir = arguments.output_dir / f"window_{index:02d}"
            window_dir.mkdir()
            stream_records = {
                name: message_record(name, message, window_dir / f"{name}.cdr")
                for name, message in synchronized.items()
            }
            stamps = [item["stamp_sec"] for item in stream_records.values()]
            fields = {field.name for field in synchronized["points"].fields}
            point_layout_valid = (
                synchronized["points"].width > 0
                and synchronized["points"].height > 0
                and synchronized["points"].point_step > 0
                and {"x", "y", "z"}.issubset(fields)
                and len(synchronized["points"].data)
                >= synchronized["points"].row_step * synchronized["points"].height
            )

            color = color_array(synchronized["color"])
            if color is not None:
                save_ppm(window_dir / "color_preview.ppm", color)
            depth = depth_array(synchronized["depth"])
            if depth is not None:
                np.save(window_dir / "depth_raw.npy", depth, allow_pickle=False)
            try:
                xyz = point_cloud2.read_points_numpy(
                    synchronized["points"],
                    field_names=["x", "y", "z"],
                    skip_nans=False,
                    reshape_organized_cloud=False,
                )
                xyz = np.asarray(xyz, dtype=float).reshape((-1, 3))
            except (AssertionError, TypeError, ValueError) as error:
                xyz = np.empty((0, 3), dtype=float)
                point_layout_valid = False
                stream_records["points"]["decode_error"] = str(error)
            np.save(window_dir / "points_xyz_m.npy", xyz, allow_pickle=False)

            tf_results: dict[str, Any] = {
                "base_to_color_available": False,
                "base_to_depth_available": False,
                "world_to_points_available": False,
            }
            base_to_color = None
            base_to_depth = None
            world_to_points = None
            for key, parent, child, stamp in (
                (
                    "base_to_color",
                    "base_link",
                    synchronized["color"].header.frame_id,
                    synchronized["color"].header.stamp,
                ),
                (
                    "base_to_depth",
                    "base_link",
                    synchronized["depth"].header.frame_id,
                    synchronized["depth"].header.stamp,
                ),
                (
                    "world_to_points",
                    "world",
                    synchronized["points"].header.frame_id,
                    synchronized["points"].header.stamp,
                ),
            ):
                try:
                    transform = lookup_transform_at_exact_stamp(
                        node,
                        tf_buffer,
                        parent,
                        child,
                        stamp,
                        timeout_sec=0.50,
                    )
                    tf_results[f"{key}_available"] = True
                    tf_results[key] = transform_record(transform)
                    if key == "base_to_color":
                        base_to_color = transform
                    elif key == "base_to_depth":
                        base_to_depth = transform
                    else:
                        world_to_points = transform
                except TransformException as error:
                    tf_results[f"{key}_error"] = str(error)

            visibility = {
                "projected_inside_image": False,
                "point_cloud_support_pass": False,
                "minimum_target_points": arguments.minimum_target_points,
            }
            if world_to_points is not None:
                world_rotation, world_translation = transform_components(
                    world_to_points.transform
                )
                projection = project_target_center(
                    target_center_world_m=target_center_world,
                    world_from_camera_rotation=world_rotation,
                    world_from_camera_translation=world_translation,
                    camera_info=synchronized["camera_info"],
                )
                point_support = target_visibility_from_points(
                    xyz,
                    world_from_camera_rotation=world_rotation,
                    world_from_camera_translation=world_translation,
                    target_model_position_world_m=target_position,
                    target_model_rotation_world=target_rotation,
                )
                visibility.update(projection)
                visibility.update(point_support)
                visibility.update(
                    rgb_roi_statistics(
                        color, projection, float(synchronized["camera_info"].k[0])
                    )
                )
                visibility["projected_inside_image"] = bool(projection["inside_image"])
                visibility["point_cloud_support_pass"] = (
                    point_support["target_point_count"] >= arguments.minimum_target_points
                )

            window = {
                "window_index": index,
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "streams": stream_records,
                "synchronization": {
                    "minimum_stamp_sec": min(stamps),
                    "maximum_stamp_sec": max(stamps),
                    "spread_sec": max(stamps) - min(stamps),
                    "within_slop": max(stamps) - min(stamps) <= arguments.sync_slop_sec,
                },
                "validations": {
                    "color_layout_valid": image_layout_valid(synchronized["color"]),
                    "depth_layout_valid": image_layout_valid(synchronized["depth"]),
                    "camera_info_valid": camera_info_valid(synchronized["camera_info"]),
                    "point_cloud_layout_valid": point_layout_valid,
                },
                "tf": tf_results,
                "target_visibility": visibility,
                "failure_codes": [],
            }
            window["failure_codes"] = error_code_list(window)
            window["window_pass"] = not window["failure_codes"]
            atomic_write_json(window_dir / "metadata.json", window)
            record["windows"].append(window)
    except Exception as error:  # Preserve an atomic terminal record for infrastructure faults.
        record["failure_codes"].append("P7_1_CAPTURE_INFRASTRUCTURE_ERROR")
        record["infrastructure_error"] = f"{type(error).__name__}: {error}"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(record["windows"]) != arguments.expected_windows:
        record["failure_codes"].append("P7_1_INSUFFICIENT_SYNCHRONIZED_WINDOWS")
    for window in record["windows"]:
        record["failure_codes"].extend(window["failure_codes"])
    record["failure_codes"] = sorted(set(record["failure_codes"]))
    record["record_status"] = (
        "complete" if len(record["windows"]) == arguments.expected_windows else "incomplete"
    )
    record["gate_pass"] = (
        record["record_status"] == "complete"
        and not record["failure_codes"]
        and all(window["window_pass"] for window in record["windows"])
    )
    record["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    record["claim_boundary"] = (
        "P7.1 sensor-input evidence only; no pose estimate, NBV, MoveIt plan, "
        "trajectory execution, grasp, or task success was evaluated"
    )
    atomic_write_json(arguments.output_dir / "gate.json", record)
    print(json.dumps(record, sort_keys=True, default=json_scalar))
    if not record["gate_pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
