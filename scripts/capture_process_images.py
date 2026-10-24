#!/usr/bin/env python3
"""Capture one labelled RGB-D frame at a task-phase boundary.

Why this script exists
----------------------
A phase classifier has to be trained on frames that are *labelled by the moment
they were taken*: "the arm is at the enter boundary of `grasp`", "the arm is at
the exit boundary of `insert`".  The pick-and-place driver already knows those
boundaries; this script turns one boundary into one self-describing sample.

What it does (read-only, no motion)
-----------------------------------
1. subscribes to colour, depth and the TCP pose;
2. waits for a *fresh* frame (the second frame received, never a cached one);
3. writes ``<phase>_<enter|exit>_<timestamp>_color.png``, ``_depth.npy`` and
   ``_meta.json`` under ``<out>/<run_id>/``.

No publisher is created, no controller is used and no robot command is sent.

Run aggregation
---------------
Boundaries that belong to one pick-and-place run share a ``run_id``.  The id is
the local start time (``YYYYmmdd_HHMMSS``) and is remembered in
``<out>/.current_run`` together with the last capture time.  A new run starts
when the previous capture is older than 30 min (or always with ``--run-id``).
With the default ``--out runtime/process_images`` this state file is exactly
``runtime/process_images/.current_run``.

Usage
-----
  python3 scripts/capture_process_images.py --tag "grasp:enter" \\
      --out runtime/process_images [--run-id auto] [--note "..."]

Exit codes
----------
  0  captured one colour + one depth frame and wrote the metadata;
  2  no fresh frame within the timeout (15 s by default);
  3  bad arguments (tag, boundary, run id, output path, timeout).
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import pathlib
import re
import sys
import time

import numpy as np

sys.stdout.reconfigure(line_buffering=True)

COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
TCP_TOPIC = "/tcp_pose_broadcaster/pose"

RUN_GAP_S = 30 * 60
FRAME_TIMEOUT_S = 15.0
TCP_GRACE_S = 2.0
# Colour and depth are published as a pair with the same stamp at ~0.8 fps
# (1.37 s apart).  After subscription the two streams can be one frame out of
# step, so a pair counts as aligned when the stamps differ by less than half a
# frame period.
MAX_PAIR_SKEW_S = 0.5
RUN_ID_FORMAT = "%Y%m%d_%H%M%S"
STATE_NAME = ".current_run"

VALID_BOUNDARIES = ("enter", "exit")
PHASE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

EXIT_OK = 0
EXIT_NO_FRAME = 2
EXIT_BAD_ARGS = 3

# encoding -> (numpy dtype, channels).  The Percipio driver publishes bgr8
# colour and 16UC1 depth in millimetres.
_ENCODINGS = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "rgba8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "8uc1": (np.uint8, 1),
    "8uc3": (np.uint8, 3),
    "mono16": (np.uint16, 1),
    "16uc1": (np.uint16, 1),
    "32fc1": (np.float32, 1),
}


# ---------------------------------------------------------------------------
# pure helpers (importable without ROS; covered by test/test_process_images.py)
# ---------------------------------------------------------------------------
def parse_tag(tag: str) -> tuple[str, str]:
    """Split ``"<phase>:<enter|exit>"`` into ``(phase, boundary)``."""
    text = str(tag or "").strip()
    if text.count(":") != 1:
        raise ValueError(f"tag must look like '<phase>:<enter|exit>', got {tag!r}")
    phase, boundary = (part.strip() for part in text.split(":"))
    if not PHASE_RE.match(phase):
        raise ValueError(f"invalid phase name {phase!r}")
    boundary = boundary.lower()
    if boundary not in VALID_BOUNDARIES:
        raise ValueError(
            f"boundary must be one of {VALID_BOUNDARIES}, got {boundary!r}")
    return phase, boundary


def format_run_id(now: float) -> str:
    """Local ``YYYYmmdd_HHMMSS`` run id for an epoch timestamp."""
    return time.strftime(RUN_ID_FORMAT, time.localtime(now))


def parse_run_id(run_id: str) -> float | None:
    """Epoch seconds encoded in a run id, or ``None`` if it is not one."""
    try:
        parsed = time.mktime(time.strptime(str(run_id), RUN_ID_FORMAT))
    except (ValueError, OverflowError):
        return None
    return parsed


def next_run_id(now: float, last_seen_ts: float | None) -> str | None:
    """Return a new run id, or ``None`` when the current run is still active.

    The 30-minute rule is strict: a gap of exactly 1800 s still belongs to the
    running session, only a gap ``> 1800 s`` starts a new one.  ``last_seen_ts``
    is ``None`` on the first capture, which always opens a run.
    """
    if now is None or not math.isfinite(float(now)):
        raise ValueError("now must be a finite epoch timestamp")
    if last_seen_ts is None:
        return format_run_id(now)
    last = float(last_seen_ts)
    if not math.isfinite(last):
        raise ValueError("last_seen_ts must be a finite epoch timestamp or None")
    if float(now) - last > RUN_GAP_S:
        return format_run_id(now)
    return None


def stamp_to_iso(sec: int, nanosec: int = 0) -> str:
    """ROS stamp -> ``YYYY-MM-DDTHH:MM:SS.nnnnnnnnnZ`` (UTC, fixed width)."""
    sec = int(sec)
    nanosec = int(nanosec)
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError(f"nanosec out of range: {nanosec}")
    moment = datetime.datetime.fromtimestamp(sec, tz=datetime.timezone.utc)
    return moment.strftime("%Y-%m-%dT%H:%M:%S") + f".{nanosec:09d}Z"


def file_stamp(sec: int, nanosec: int = 0) -> str:
    """File-name timestamp in the repository's ``YYYYmmdd_HHMMSS_mmm`` style."""
    sec = int(sec)
    nanosec = int(nanosec)
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError(f"nanosec out of range: {nanosec}")
    return (time.strftime(RUN_ID_FORMAT, time.localtime(sec))
            + f"_{nanosec // 1_000_000:03d}")


def _normalise_stamp(stamp) -> tuple[int, int]:
    """Accept a ROS Time, a ``{"sec","nanosec"}`` mapping or epoch seconds."""
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        return int(stamp.sec), int(stamp.nanosec)
    if isinstance(stamp, dict):
        if "sec" not in stamp:
            raise ValueError("stamp mapping needs a 'sec' field")
        return int(stamp["sec"]), int(stamp.get("nanosec", 0))
    if isinstance(stamp, (int, float)) and not isinstance(stamp, bool):
        seconds = float(stamp)
        whole = int(math.floor(seconds))
        return whole, int(round((seconds - whole) * 1e9))
    raise TypeError(f"unsupported stamp type: {type(stamp)!r}")


def _quaternion_matrix(quaternion) -> list[list[float]]:
    """Rotation matrix (3x3, nested lists) from ``(x, y, z, w)``."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def tcp_payload(tcp: dict | None) -> dict | None:
    """TCP pose dict -> ``{frame_id, xyz, quaternion, matrix}`` (4x4)."""
    if tcp is None:
        return None
    xyz = [float(value) for value in tcp["xyz"]]
    if len(xyz) != 3:
        raise ValueError("tcp xyz needs three values")
    quaternion = [float(value) for value in tcp["quaternion"]]
    if len(quaternion) != 4:
        raise ValueError("tcp quaternion needs four values (x, y, z, w)")
    # The pose broadcaster emits NaN before the robot reports a valid state; a
    # NaN pose is worse than no pose, so it must degrade to null.
    if not all(math.isfinite(value) for value in xyz + quaternion):
        return None
    rotation = _quaternion_matrix(quaternion)
    matrix = [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        "frame_id": str(tcp.get("frame_id") or "base"),
        "xyz": xyz,
        "quaternion": quaternion,
        "matrix": matrix,
    }


def build_record(phase: str, boundary: str, tcp, stamp, run_id: str,
                 note: str = "") -> dict:
    """Build the metadata record written next to one colour/depth pair.

    ``tcp`` is either ``None`` (pose topic not publishing) or a mapping with
    ``xyz``, ``quaternion`` and optional ``frame_id``.  ``stamp`` is the ROS
    capture stamp (colour frame), given as a Time message, a ``{"sec",
    "nanosec"}`` mapping or epoch seconds.
    """
    phase = str(phase or "").strip()
    if not PHASE_RE.match(phase):
        raise ValueError(f"invalid phase name {phase!r}")
    boundary = str(boundary or "").strip().lower()
    if boundary not in VALID_BOUNDARIES:
        raise ValueError(
            f"boundary must be one of {VALID_BOUNDARIES}, got {boundary!r}")
    sec, nanosec = _normalise_stamp(stamp)
    run_id = str(run_id or "").strip()
    if not RUN_ID_RE.match(run_id):
        raise ValueError(f"invalid run id {run_id!r}")
    pose = tcp_payload(tcp)
    frame_id = pose["frame_id"] if pose else "base"
    started = parse_run_id(run_id)
    return {
        "phase": phase,
        "boundary": boundary,
        "stamp": stamp_to_iso(sec, nanosec),
        "stamp_sec": sec,
        "stamp_nanosec": nanosec,
        "tcp_base": pose,
        "frame_id": frame_id,
        "run_id": run_id,
        "run_started_at": (stamp_to_iso(int(started), 0)
                           if started is not None else None),
        "note": str(note or ""),
    }


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------
def decode_image(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image into a numpy array without cv_bridge."""
    key = str(msg.encoding).strip().lower()
    if key not in _ENCODINGS:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    dtype, channels = _ENCODINGS[key]
    item = np.dtype(dtype).itemsize
    row_bytes = int(msg.width) * channels * item
    step = int(msg.step)
    if step < row_bytes:
        raise ValueError(
            f"step {step} smaller than one row ({row_bytes} bytes)")
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    needed = int(msg.height) * step
    if buffer.size < needed:
        raise ValueError(
            f"image buffer {buffer.size} bytes, expected at least {needed}")
    rows = buffer[:needed].reshape(int(msg.height), step)[:, :row_bytes]
    array = rows.copy().view(dtype)
    if channels == 1:
        return array.reshape(int(msg.height), int(msg.width))
    return array.reshape(int(msg.height), int(msg.width), channels)


def read_run_state(out_dir: pathlib.Path) -> dict:
    """Read ``<out>/.current_run``; a missing or broken file means "no run"."""
    path = out_dir / STATE_NAME
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def write_run_state(out_dir: pathlib.Path, run_id: str, now: float) -> pathlib.Path:
    """Atomically remember the active run and when it last produced a frame."""
    path = out_dir / STATE_NAME
    payload = {
        "run_id": run_id,
        "last_seen": float(now),
        "updated_at": stamp_to_iso(int(now), int(round((now % 1) * 1e9)) % 1_000_000_000),
    }
    temporary = out_dir / (STATE_NAME + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)
    return path


def last_seen_from_files(out_dir: pathlib.Path) -> float | None:
    """Fallback when the state file is gone: newest metadata under ``out``."""
    newest = None
    for meta in out_dir.glob("*/*_meta.json"):
        try:
            mtime = meta.stat().st_mtime
        except OSError:
            continue
        if newest is None or mtime > newest:
            newest = mtime
    return newest


def split_tcp(tcp_msg) -> dict:
    """geometry_msgs/PoseStamped -> the plain mapping build_record expects."""
    pose = tcp_msg.pose
    return {
        "xyz": [pose.position.x, pose.position.y, pose.position.z],
        "quaternion": [pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w],
        "frame_id": tcp_msg.header.frame_id or "base",
    }


# ---------------------------------------------------------------------------
# ROS capture node
# ---------------------------------------------------------------------------
def capture(arguments) -> int:
    try:
        import cv2
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:  # environment not sourced
        print(f"error: cannot import ROS/cv2 modules ({exc}); "
              "source scripts/source_dev_env.sh --full first", file=sys.stderr)
        return 1

    class CaptureNode(Node):
        """Collect the newest colour, depth and TCP pose; never re-publish."""

        def __init__(self) -> None:
            super().__init__("capture_process_images")
            self.color = None
            self.depth = None
            self.tcp = None
            self.color_count = 0
            self.depth_count = 0
            self.first_color = None
            self.first_depth = None
            self.create_subscription(Image, COLOR_TOPIC, self._on_color,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, DEPTH_TOPIC, self._on_depth,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseStamped, TCP_TOPIC, self._on_tcp, 10)

        @staticmethod
        def _key(message) -> tuple[int, int]:
            return (int(message.header.stamp.sec), int(message.header.stamp.nanosec))

        def _on_color(self, message) -> None:
            if self.first_color is None:
                self.first_color = self._key(message)
            self.color = message
            self.color_count += 1

        def _on_depth(self, message) -> None:
            if self.first_depth is None:
                self.first_depth = self._key(message)
            self.depth = message
            self.depth_count += 1

        def _on_tcp(self, message) -> None:
            self.tcp = message

        def fresh(self) -> bool:
            """True once both streams delivered at least one *new* frame."""
            return (self.color is not None and self.depth is not None
                    and self.color_count >= 2 and self.depth_count >= 2
                    and self._key(self.color) != self.first_color
                    and self._key(self.depth) != self.first_depth)

        def pose(self) -> dict | None:
            """Newest TCP pose as a finite payload, or None if unusable."""
            if self.tcp is None:
                return None
            try:
                return tcp_payload(split_tcp(self.tcp))
            except (KeyError, TypeError, ValueError):
                return None

        def pair_skew_s(self) -> float:
            """Stamp distance between the newest colour and depth frame."""
            if self.color is None or self.depth is None:
                return float("inf")
            color = self.color.header.stamp.sec + self.color.header.stamp.nanosec * 1e-9
            depth = self.depth.header.stamp.sec + self.depth.header.stamp.nanosec * 1e-9
            return abs(color - depth)

    rclpy.init(args=None)
    node = CaptureNode()
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.fresh():
                break
        if not node.fresh():
            missing = []
            if node.color_count < 2:
                missing.append(f"{COLOR_TOPIC} (frames={node.color_count})")
            if node.depth_count < 2:
                missing.append(f"{DEPTH_TOPIC} (frames={node.depth_count})")
            detail = ", ".join(missing) or "fresh stamps not observed"
            print(f"error: no fresh frame within {arguments.timeout:.0f} s: "
                  f"{detail}", file=sys.stderr)
            return EXIT_NO_FRAME

        # Prefer an RGB-D pair taken at the same instant.  If the driver never
        # aligns them before the deadline, keep the newest frames and say so in
        # the metadata instead of losing the boundary sample.
        while (rclpy.ok() and node.pair_skew_s() > MAX_PAIR_SKEW_S
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        # The pose topic is optional for the image itself.  Wait briefly for a
        # finite pose, then record tcp_base=null instead of failing the capture
        # or writing NaN into the metadata.
        tcp_deadline = min(deadline, time.monotonic() + TCP_GRACE_S)
        while (rclpy.ok() and node.pose() is None
               and time.monotonic() < tcp_deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        tcp = node.pose()
        if tcp is None and node.tcp is not None:
            print("warning: TCP pose contains non-finite values; "
                  "recorded as null", file=sys.stderr)

        # Recompute after the pose wait: new frames may have arrived meanwhile.
        skew = node.pair_skew_s()
        aligned = skew <= MAX_PAIR_SKEW_S
        if not aligned:
            print(f"warning: colour/depth stamps differ by {skew:.3f} s; "
                  "saving the latest pair anyway", file=sys.stderr)

        color, depth = node.color, node.depth
        color_key = CaptureNode._key(color)
        depth_key = CaptureNode._key(depth)
        seen = {"color": node.color_count, "depth": node.depth_count}
        return write_capture(arguments, cv2, color, depth, color_key, depth_key,
                             tcp, seen, skew, aligned)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def write_capture(arguments, cv2, color_msg, depth_msg, color_key, depth_key,
                  tcp, frames_seen, pair_skew_s, pair_aligned) -> int:
    """Decode, write the three files and update the run state."""
    phase, boundary = arguments.phase_boundary
    run_dir = pathlib.Path(arguments.resolved_out) / arguments.resolved_run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    color = decode_image(color_msg)
    depth = decode_image(depth_msg)
    if color.ndim == 2:
        color_bgr = cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)
    elif str(color_msg.encoding).lower() == "rgb8":
        color_bgr = color[:, :, ::-1]
    else:
        color_bgr = color

    stem = f"{phase}_{boundary}_{file_stamp(*color_key)}"
    color_path = run_dir / f"{stem}_color.png"
    depth_path = run_dir / f"{stem}_depth.npy"
    meta_path = run_dir / f"{stem}_meta.json"

    if not cv2.imwrite(str(color_path), color_bgr):
        print(f"error: failed to write {color_path}", file=sys.stderr)
        return 1
    np.save(str(depth_path), depth)

    record = build_record(phase, boundary, tcp,
                          {"sec": color_key[0], "nanosec": color_key[1]},
                          arguments.resolved_run_id, arguments.note)
    record.update({
        "captured_at": stamp_to_iso(int(time.time())),
        "color_file": color_path.name,
        "depth_file": depth_path.name,
        "color_topic": COLOR_TOPIC,
        "depth_topic": DEPTH_TOPIC,
        "tcp_topic": TCP_TOPIC,
        "color_encoding": str(color_msg.encoding),
        "depth_encoding": str(depth_msg.encoding),
        "color_shape": [int(value) for value in color.shape],
        "depth_shape": [int(value) for value in depth.shape],
        "depth_stamp": stamp_to_iso(*depth_key),
        "depth_units": ("mm" if depth.dtype == np.uint16 else "raw"),
        "tcp_available": tcp is not None,
        "frames_seen": {"color": int(frames_seen["color"]),
                        "depth": int(frames_seen["depth"])},
        "pair_skew_s": round(float(pair_skew_s), 6),
        "pair_aligned": bool(pair_aligned),
    })
    meta_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")

    write_run_state(pathlib.Path(arguments.resolved_out),
                    arguments.resolved_run_id, time.time())

    print(f"run_id      : {arguments.resolved_run_id}")
    print(f"phase       : {phase} ({boundary})")
    print(f"color       : {color_path}  "
          f"({color.shape[1]}x{color.shape[0]} {color_msg.encoding})")
    print(f"depth       : {depth_path}  ({depth.shape[1]}x{depth.shape[0]} "
          f"{depth_msg.encoding} {record['depth_units']})")
    print(f"meta        : {meta_path}")
    print(f"stamp       : {record['stamp']}")
    if tcp is not None:
        x, y, z = tcp["xyz"]
        print(f"tcp_base    : x={x:.4f} y={y:.4f} z={z:.4f} "
              f"(frame {record['frame_id']})")
    else:
        print(f"tcp_base    : unavailable ({TCP_TOPIC} not publishing); "
              "recorded as null", file=sys.stderr)
    return EXIT_OK


# ---------------------------------------------------------------------------
# command line
# ---------------------------------------------------------------------------
class _Parser(argparse.ArgumentParser):
    """argparse exits with 2 on bad input; this tool reserves 2 for "no frame"."""

    def error(self, message: str) -> None:
        self.print_usage(sys.stderr)
        print(f"{self.prog}: error: {message}", file=sys.stderr)
        raise SystemExit(EXIT_BAD_ARGS)


def main(argv: list[str] | None = None) -> int:
    parser = _Parser(description=__doc__,
                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag", required=True,
                        help="'<phase>:<enter|exit>', e.g. 'grasp:enter'")
    parser.add_argument("--out", default="runtime/process_images",
                        help="output root (default: runtime/process_images)")
    parser.add_argument("--run-id", default="auto",
                        help="'auto' (30 min rule) or an explicit run id")
    parser.add_argument("--note", default="", help="free-text note for the meta")
    parser.add_argument("--timeout", type=float, default=FRAME_TIMEOUT_S,
                        help="seconds to wait for a fresh frame (default 15)")
    arguments = parser.parse_args(argv)

    try:
        arguments.phase_boundary = parse_tag(arguments.tag)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_BAD_ARGS

    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0:
        print("error: --timeout must be a positive number of seconds",
              file=sys.stderr)
        return EXIT_BAD_ARGS

    out_dir = pathlib.Path(arguments.out).expanduser()
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"error: cannot create output directory {out_dir}: {exc}",
              file=sys.stderr)
        return EXIT_BAD_ARGS
    arguments.resolved_out = str(out_dir.resolve())

    now = time.time()
    if arguments.run_id and arguments.run_id != "auto":
        if not RUN_ID_RE.match(arguments.run_id):
            print(f"error: invalid --run-id {arguments.run_id!r} "
                  "(use letters, digits, '_', '.', '-')", file=sys.stderr)
            return EXIT_BAD_ARGS
        run_id = arguments.run_id
        is_new = True
    else:
        state = read_run_state(out_dir)
        last_seen = state.get("last_seen")
        if not isinstance(last_seen, (int, float)):
            last_seen = last_seen_from_files(out_dir)
        candidate = next_run_id(now, last_seen)
        if candidate is not None:
            run_id, is_new = candidate, True
        else:
            previous = state.get("run_id")
            if not isinstance(previous, str) or not RUN_ID_RE.match(previous):
                # A live gap with no usable id would silently merge unrelated
                # runs, so fall back to a fresh id instead.
                run_id, is_new = format_run_id(now), True
            else:
                run_id, is_new = previous, False
    arguments.resolved_run_id = run_id

    print(f"capture : {arguments.phase_boundary[0]}:{arguments.phase_boundary[1]}"
          f" -> {out_dir}/{run_id}"
          f" ({'new run' if is_new else 'continuing run'})")
    return capture(arguments)


if __name__ == "__main__":
    sys.exit(main())
