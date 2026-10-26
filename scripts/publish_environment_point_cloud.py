#!/usr/bin/env python3
"""Publish the measured environment model (scene.ply) as a PointCloud2 for RViz.

Why this exists: ``scripts/model_environment.py`` produces, per measurement run,

    runtime/environment/<ts>/model.json   tables / slot / workpiece numbers
    runtime/environment/<ts>/scene.ply    merged, downsampled base-frame cloud

and nothing in this repository shows the scene cloud on screen.  The numbers in
model.json say "the slot moved 3 mm"; only the cloud shows *where*.  This script
turns scene.ply into a normal ROS 2 point cloud so a second RViz window can show
what the camera actually reconstructed, in simulation and on the real cell alike.

Frame: model_environment.py already merges every frame into ``base_link`` (see its
module docstring, "merged, downsampled base-frame cloud"), and the PCD equivalent
of the senior's environment publisher also stamps ``base_link``.  So the cloud is
declared in ``base_link`` and is *not* silently re-labelled: if your scene.ply
were produced in another frame, pass ``--frame``.

Topic: ``/environment/point_cloud`` — the same name the senior's
``environment_point_cloud_publisher`` used, so existing RViz profiles and habits
carry over.  Nothing else in this repository publishes there.

Usage:
    source scripts/source_dev_env.sh --full

    # 只发布点云（打印话题），配合已经在跑的 RViz 或远程 RViz
    python3 scripts/publish_environment_point_cloud.py

    # 发布 + 打开专用 RViz（仿真 / 真机都可以，纯离线也能用）
    python3 scripts/publish_environment_point_cloud.py --rviz

    # 指定某一次环境模型、降采样、只发一次做话题自检
    python3 scripts/publish_environment_point_cloud.py --scene runtime/environment/20261025_070113
    python3 scripts/publish_environment_point_cloud.py --voxel 0.01
    python3 scripts/publish_environment_point_cloud.py --once

Data flow:
    runtime/environment/<ts>/scene.ply
      -> this node (read PLY once, optional voxel downsample)
      -> /environment/point_cloud   sensor_msgs/PointCloud2, frame_id=base_link
      -> RViz2 (src/cs625_bringup/config/environment_point_cloud.rviz)

The scene file is read once at start-up; a fresh environment model needs a
restart (``--watch`` is deliberately not implemented: the stored cloud is a
static measurement, not a live sensor stream).
"""

from __future__ import annotations

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

REPO = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_SCENE = REPO / "runtime" / "environment" / "latest" / "scene.ply"
DEFAULT_RVIZ_CONFIG = REPO / "src" / "cs625_bringup" / "config" / "environment_point_cloud.rviz"
DEFAULT_TOPIC = "/environment/point_cloud"
DEFAULT_FRAME = "base_link"

_PLY_SCALARS = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def resolve_scene(path: pathlib.Path) -> pathlib.Path:
    """Accept a PLY file, a <ts> directory, or 'latest'; return the PLY to read."""
    candidate = path.expanduser()
    if candidate.is_dir():
        candidate = candidate / "scene.ply"
    if candidate.is_file():
        return candidate
    hint = ""
    env_root = REPO / "runtime" / "environment"
    if env_root.is_dir():
        runs = sorted(p.name for p in env_root.iterdir() if p.is_dir() and p.name != "latest")
        if runs:
            hint = "\n  available runs: " + ", ".join(runs[-5:])
    raise SystemExit(
        f"scene point cloud not found: {candidate}\n"
        "  run scripts/model_environment.py first, or pass --scene <path/to/scene.ply>"
        + hint
    )


def read_ply_xyz(path: pathlib.Path) -> np.ndarray:
    """Return an (N, 3) float64 array from a PLY file (ascii or binary)."""
    raw = path.read_bytes()
    marker = raw.find(b"end_header")
    if marker < 0:
        raise SystemExit(f"{path} is not a PLY file (no end_header)")
    header = raw[:marker].decode("ascii", "replace").splitlines()

    fmt = None
    elements: list[dict] = []
    for line in header:
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == "format" and len(tokens) >= 2:
            fmt = tokens[1]
        elif tokens[0] == "element" and len(tokens) >= 3:
            elements.append({"name": tokens[1], "count": int(tokens[2]), "props": []})
        elif tokens[0] == "property" and elements:
            if tokens[1] == "list":
                elements[-1]["props"].append((tokens[-1], "list"))
            else:
                elements[-1]["props"].append((tokens[2], tokens[1]))

    if fmt is None:
        raise SystemExit(f"{path} has no PLY 'format' line")
    if not elements:
        raise SystemExit(f"{path} declares no elements")

    for element in elements:
        if any(kind == "list" for _, kind in element["props"]):
            raise SystemExit(
                f"{path}: element '{element['name']}' uses list properties, which this "
                "viewer does not read; export x/y/z-only geometry instead"
            )
        unknown = [kind for _, kind in element["props"] if kind not in _PLY_SCALARS]
        if unknown:
            raise SystemExit(f"{path}: unsupported PLY property type(s) {sorted(set(unknown))}")

    body_start = marker + len(b"end_header")
    while body_start < len(raw) and raw[body_start : body_start + 1] in (b"\r", b"\n"):
        body_start += 1
    body = raw[body_start:]

    values: dict[str, np.ndarray] = {}
    if fmt.startswith("ascii"):
        if elements[0]["name"] != "vertex":
            raise SystemExit(
                f"{path}: ascii PLY whose first element is not 'vertex' is not supported"
            )
        rows = [
            [float(value) for value in line.split()]
            for line in body.decode("ascii", "replace").splitlines()
            if line.strip()
        ]
        table = np.asarray(rows, dtype=np.float64)
        width = len(elements[0]["props"])
        if table.ndim != 2 or table.shape[1] != width:
            raise SystemExit(f"{path}: ascii PLY rows do not match the declared {width} properties")
        values = {name: table[:, i] for i, (name, _) in enumerate(elements[0]["props"])}
    else:
        if "binary_little_endian" in fmt:
            byteorder = "<"
        elif "binary_big_endian" in fmt:
            byteorder = ">"
        else:
            raise SystemExit(f"{path}: unknown PLY format '{fmt}'")
        offset = 0
        for element in elements:
            dtype = np.dtype(
                [(name, byteorder + _PLY_SCALARS[kind]) for name, kind in element["props"]]
            )
            needed = dtype.itemsize * element["count"]
            if offset + needed > len(body):
                raise SystemExit(
                    f"{path}: truncated PLY body for element '{element['name']}' "
                    f"(need {needed} B, have {len(body) - offset} B)"
                )
            chunk = np.frombuffer(body[offset : offset + needed], dtype=dtype)
            offset += needed
            if element["name"] == "vertex":
                values = {name: chunk[name].astype(np.float64) for name, _ in element["props"]}
        if not values:
            raise SystemExit(f"{path} has no 'vertex' element")

    for axis in ("x", "y", "z"):
        if axis not in values:
            raise SystemExit(f"{path}: vertex element has no '{axis}' property")
    points = np.column_stack([values["x"], values["y"], values["z"]])
    points = points[np.isfinite(points).all(axis=1)]
    if points.size == 0:
        raise SystemExit(f"{path}: no finite XYZ points")
    return points


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    """Keep one point per ``leaf``-sized voxel (fast, deterministic, no Open3D)."""
    if leaf <= 0.0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def build_cloud_message(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    xyz = np.ascontiguousarray(points, dtype=np.float32)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = xyz.tobytes()
    return msg


class EnvironmentCloudPublisher(Node):
    def __init__(self, topic: str, frame_id: str, rate: float, message: PointCloud2):
        super().__init__("environment_point_cloud_viewer")
        # Reliable/volatile matches the RViz PointCloud2 display in
        # src/cs625_bringup/config/environment_point_cloud.rviz.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._message = message
        self._frame_id = frame_id
        self._publisher = self.create_publisher(PointCloud2, topic, qos)
        self._period = 1.0 / rate if rate > 0.0 else 1.0
        self._timer = self.create_timer(self._period, self.publish_once)

    def publish_once(self) -> None:
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish runtime/environment/<ts>/scene.ply as /environment/point_cloud.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scene",
        type=pathlib.Path,
        default=DEFAULT_SCENE,
        help="scene.ply, a runtime/environment/<ts> directory, or 'runtime/environment/latest'",
    )
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="PointCloud2 topic to publish")
    parser.add_argument(
        "--frame",
        default=DEFAULT_FRAME,
        help="frame_id to stamp; scene.ply is merged in base_link, so change this only if yours differs",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="publish rate [Hz]")
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.0,
        help="voxel leaf [m] for an extra downsample before publishing; 0 keeps every point",
    )
    parser.add_argument("--once", action="store_true", help="publish a single message and exit")
    parser.add_argument("--rviz", action="store_true", help="also launch rviz2 with the dedicated profile")
    parser.add_argument(
        "--rviz-config",
        type=pathlib.Path,
        default=DEFAULT_RVIZ_CONFIG,
        help="RViz profile used by --rviz",
    )
    args = parser.parse_args(argv)
    if args.once and args.rviz:
        parser.error("--once and --rviz contradict each other: RViz needs republished messages")
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    return args


def start_rviz(config: pathlib.Path) -> subprocess.Popen:
    if not config.is_file():
        raise SystemExit(f"RViz profile not found: {config}")
    if not os.environ.get("DISPLAY"):
        raise SystemExit(
            "DISPLAY is not set, so rviz2 cannot open a window here; run the publisher "
            "without --rviz and use RViz on a machine that has a display"
        )
    if shutil.which("rviz2") is None:
        raise SystemExit(
            "rviz2 is not on PATH; source scripts/source_dev_env.sh --full first, or drop --rviz"
        )
    self_cmd = [
        "rviz2",
        "-d",
        str(config),
    ]
    print(f"[viewer] starting: {' '.join(self_cmd)}")
    return subprocess.Popen(self_cmd)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    scene = resolve_scene(args.scene)

    points = read_ply_xyz(scene)
    raw_count = points.shape[0]
    points = voxel_downsample(points, args.voxel)
    if args.voxel > 0.0:
        print(f"[viewer] voxel {args.voxel:.3f} m: {raw_count} -> {points.shape[0]} points")

    low = points.min(axis=0)
    high = points.max(axis=0)
    print(f"[viewer] scene    : {scene}")
    print(f"[viewer] points   : {points.shape[0]}")
    print(
        "[viewer] extent   : "
        f"x[{low[0]:+.3f},{high[0]:+.3f}] "
        f"y[{low[1]:+.3f},{high[1]:+.3f}] "
        f"z[{low[2]:+.3f},{high[2]:+.3f}] (m, {args.frame})"
    )

    rclpy.init()
    rviz_process: subprocess.Popen | None = None
    node: EnvironmentCloudPublisher | None = None
    try:
        message = build_cloud_message(points, args.frame, Time())
        node = EnvironmentCloudPublisher(args.topic, args.frame, args.rate, message)
        node.get_logger().info(
            f"publishing {points.shape[0]} points as {args.topic} "
            f"(frame_id={args.frame}, {args.rate:.2f} Hz)"
        )

        if args.once:
            # A single message can be missed while DDS discovery completes, so a
            # few repeats are sent before exiting.
            for _ in range(10):
                node.publish_once()
                rclpy.spin_once(node, timeout_sec=0.2)
            print(f"[viewer] published 10 messages on {args.topic}, exiting (--once)")
            return 0

        if args.rviz:
            rviz_process = start_rviz(args.rviz_config)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rviz_process is not None and rviz_process.poll() is None:
            rviz_process.terminate()
            try:
                rviz_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                rviz_process.kill()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
