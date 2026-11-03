#!/usr/bin/env python3
"""Viewing-geometry self-check for a coarse-observation pose.

Why this exists (2026-11-03): the old coarse-observation baseline looked at the
workpiece almost horizontally (optical axis only 9 deg below horizontal) with the
part sitting 73 % of the way to the bottom edge of the image.  The usable region
was therefore razor thin -- a few centimetres of workpiece displacement dropped it
out of view, and RVS then answered NG.  This tool answers the question the
operator actually has:

    "from this pose, how far can the workpiece move before it leaves the picture?"

It projects the camera frustum onto the plane through the workpiece and reports the
margin from the workpiece to the nearest frustum edge, in millimetres, plus a
verdict.  Run it whenever a coarse-observation pose is registered or suspected.

Usage:
    # current robot pose, workpiece at its last measured position
    python3 scripts/teaching/view_geometry_check.py --from-current

    # explicit pose and part position
    python3 scripts/teaching/view_geometry_check.py \
        --pose 0.3434 0.5604 0.5960 0.2189 -0.7517 -0.2087 -0.5860 \
        --part 0.6124 0.5755 0.3524

Exit codes: 0 comfortable; 1 narrow; 2 part outside the frustum; 3 bad input.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
RVS_RUNTIME = pathlib.Path("/mnt/d/Program Files (x86)/RobotVisionSuite/runtime")
CAPTURE_DIR = REPO / "runtime" / "rvs_teaching"

# Verdict thresholds: margin from the workpiece to the nearest image edge.
COMFORTABLE_MM = 80.0
NARROW_MM = 30.0
# The workpiece is ~0.47 m across, so its centre must keep about half of that
# (plus a margin) inside the frustum.  Used when judging "does it fit at all".
PART_HALF_EXTENT_MM = 240.0


def load_intrinsics() -> tuple[float, float, float, float, int, int]:
    """Camera intrinsics as RVS writes them (resolution line, then K)."""
    candidates = sorted(RVS_RUNTIME.glob("ty_color_calib_*.txt"))
    if not candidates:
        raise SystemExit("camera intrinsics file not found under the RVS runtime")
    lines = candidates[0].read_text().splitlines()
    width, height = (int(float(v)) for v in lines[0].split())
    values = [float(v) for v in lines[1].split()]
    fx, _, cx = values[0], values[1], values[2]
    fy, cy = values[4], values[5]
    return fx, fy, cx, cy, width, height


def camera_pose(tcp_xyz, tcp_quat, t_tcp_cam):
    """Camera position and rotation in the world frame."""
    world_tcp = R.from_quat(tcp_quat).as_matrix()
    position = np.asarray(tcp_xyz, dtype=float) + world_tcp @ t_tcp_cam[:3, 3]
    rotation = world_tcp @ t_tcp_cam[:3, :3]
    return position, rotation


def project_to_image(point, cam_pos, cam_rot, fx, fy, cx, cy):
    """Pixel coordinates and depth of a world point, or None when behind the camera."""
    local = cam_rot.T @ (point - cam_pos)
    if local[2] <= 1e-6:
        return None
    u = fx * local[0] / local[2] + cx
    v = fy * local[1] / local[2] + cy
    return float(u), float(v), float(local[2])


def margin_to_edges(point_xy, polygon: list[np.ndarray]) -> float:
    """Smallest distance from a point to the polygon edges (positive = inside)."""
    best = float("inf")
    for i in range(len(polygon)):
        a = polygon[i][:2]
        b = polygon[(i + 1) % len(polygon)][:2]
        ab = b - a
        length = float(np.linalg.norm(ab))
        if length < 1e-9:
            continue
        distance = abs(np.cross(ab, point_xy - a)) / length
        best = min(best, float(distance))
    return best


def point_in_polygon(point_xy, polygon: list[np.ndarray]) -> bool:
    inside = False
    count = len(polygon)
    for i in range(count):
        a = polygon[i][:2]
        b = polygon[(i + 1) % count][:2]
        if ((a[1] > point_xy[1]) != (b[1] > point_xy[1])):
            x_cross = a[0] + (point_xy[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
            if point_xy[0] < x_cross:
                inside = not inside
    return inside


def _newest(prefix: str) -> dict | None:
    files = sorted(CAPTURE_DIR.glob(f"{prefix}_*.json"),
                   key=lambda p: p.stat().st_mtime)
    for path in reversed(files):
        try:
            return json.loads(path.read_text())
        except (OSError, ValueError):
            continue
    return None


def latest_part_position(working_distance: float = 0.52) -> tuple[np.ndarray, str] | None:
    """Where the workpiece is, best effort, with the source named.

    Order: the newest T3 (a real workpiece measurement), else the newest T2 /
    S2 capture interpreted as "RVS wants the camera there, so the part lies along
    that optical axis at the working distance".  The second path exists because
    the operator needs the check DURING the search for a usable pose, i.e. before
    any T3 has been taken.
    """
    record = _newest("T3")
    if record is not None:
        import datetime
        stamp = record.get("stamp", "")
        age_note = ""
        try:
            age_min = (datetime.datetime.now()
                       - datetime.datetime.fromisoformat(stamp)).total_seconds() / 60.0
            if age_min > 30:
                age_note = f"  [STALE {age_min:.0f} min -- the workpiece may have moved]"
        except ValueError:
            pass
        return (np.array(record["position"], dtype=float),
                f"T3 capture {stamp}{age_note}")
    record = _newest("T2")
    if record is not None:
        fx, fy, cx, cy, width, height = load_intrinsics()
        t_tcp_cam = np.array(json.loads(HAND_EYE.read_text())["T_tcp_cam"], dtype=float)
        cam_pos, cam_rot = camera_pose(record["position"], record["orientation"], t_tcp_cam)
        axis = cam_rot @ np.array([0.0, 0.0, 1.0])
        axis /= np.linalg.norm(axis)
        estimate = cam_pos + axis * working_distance
        return (estimate,
                f"S2 capture {record.get('stamp', '?')} (estimated along the S2 axis "
                f"at {working_distance * 1000:.0f} mm)")
    return None


def current_tcp(timeout: float = 8.0):
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node

    rclpy.init()
    node = Node("view_geometry_check")
    latest: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: latest.__setitem__("m", m), 10)
    end = time.time() + timeout
    while "m" not in latest and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if "m" not in latest:
        return None
    pose = latest["m"].pose
    return ([pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--from-current", action="store_true")
    parser.add_argument("--part", nargs=3, type=float,
                        help="workpiece position (default: newest T3, else the S2 axis)")
    args = parser.parse_args()

    if args.pose:
        tcp_xyz, tcp_quat = args.pose[:3], args.pose[3:]
    elif args.from_current:
        pose = current_tcp()
        if pose is None:
            print("REFUSED: cannot read /tcp_pose_broadcaster/pose")
            return 3
        tcp_xyz, tcp_quat = pose
    else:
        print("need --pose or --from-current")
        return 3

    if args.part:
        part = np.array(args.part, dtype=float)
        source = "given on the command line"
    else:
        found = latest_part_position()
        if found is None:
            print("REFUSED: no --part given and no T3/T2 capture on disk")
            return 3
        part, source = found
    print(f"workpiece source: {source}")

    fx, fy, cx, cy, width, height = load_intrinsics()
    t_tcp_cam = np.array(json.loads(HAND_EYE.read_text())["T_tcp_cam"], dtype=float)
    cam_pos, cam_rot = camera_pose(tcp_xyz, tcp_quat, t_tcp_cam)

    hfov = 2 * np.degrees(np.arctan(width / 2 / fx))
    vfov = 2 * np.degrees(np.arctan(height / 2 / fy))
    axis = cam_rot @ np.array([0.0, 0.0, 1.0])
    axis /= np.linalg.norm(axis)
    depression = float(np.degrees(np.arcsin(-axis[2])))
    distance = float(np.linalg.norm(part - cam_pos))
    look = (f"{depression:.1f} deg DOWN" if depression > 0
            else f"{-depression:.1f} deg UP")

    print(f"camera     {np.round(cam_pos, 4)}")
    print(f"optical z  {np.round(axis, 3)}   camera looks {look}")
    print(f"FOV        {hfov:.1f} x {vfov:.1f} deg")
    print(f"workpiece  {np.round(part, 4)}   distance {distance * 1000:.0f} mm")

    projection = project_to_image(part, cam_pos, cam_rot, fx, fy, cx, cy)
    if projection is None:
        print("VERDICT: OUTSIDE -- the workpiece is behind the camera")
        return 2
    u, v, depth = projection
    print(f"workpiece projects to pixel ({u:.0f}, {v:.0f}) of "
          f"({width}, {height}), depth {depth * 1000:.0f} mm")

    # Convert the pixel margin to millimetres AT THE WORKPIECE DEPTH: this is the
    # distance the workpiece centre can travel before it leaves the picture.
    margin_px = min(u, width - u, v, height - v)
    mm_per_px = depth / fx * 1000.0
    margin_mm = margin_px * mm_per_px
    print(f"margin to the nearest image edge: {margin_px:.0f} px "
          f"= {margin_mm:.0f} mm at that depth ({mm_per_px:.2f} mm/px)")

    if u < 0 or u > width or v < 0 or v > height:
        print("VERDICT: OUTSIDE -- the workpiece centre is not in the picture")
        return 2

    # The workpiece is ~0.47 m across, so half of it must also stay in frame.
    usable = margin_mm - PART_HALF_EXTENT_MM
    print(f"safe travel before the part leaves the picture: {usable:+.0f} mm "
          f"(part half-extent {PART_HALF_EXTENT_MM:.0f} mm assumed)")
    if depression < 15.0:
        print(f"VERDICT: TOO FLAT -- the camera looks only {look}.  A table-top "
              "observation pose needs a steeper view (25-45 deg down); a flat view "
              "leaves a razor-thin usable region, which is exactly why the old "
              "baseline failed (2026-11-03).")
        return 1
    if usable < NARROW_MM:
        print(f"VERDICT: NARROW -- under {NARROW_MM:.0f} mm of slack; a small "
              "displacement makes RVS answer NG. Choose a pose that looks further down.")
        return 1
    if usable < COMFORTABLE_MM:
        print(f"VERDICT: TIGHT -- {usable:.0f} mm of slack; usable but not forgiving")
        return 0
    print(f"VERDICT: COMFORTABLE -- {usable:.0f} mm of slack")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
