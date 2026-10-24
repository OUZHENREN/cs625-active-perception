#!/usr/bin/env python3
"""Build an offline environment model (point cloud + tables + collision bodies).

Why this script exists
----------------------
The real cell only has to be measured once per session: the slot fixture is
mechanically fixed, the two tables do not move, and the workpiece pose is
already known.  This script reads the RGB-D captures with their TCP matrices,
back-projects every frame into the robot base frame, merges the frames into one
cloud, fits the two table planes under the known objects, derives voxel
collision bodies and stores everything as a single JSON model plus a PLY file.

It never talks to the robot: no action goal, no planning/execution service, no
ROS node.  Inputs are files only (captures, configs); outputs are files only.

Frame chain (all numbers are base-frame metres)
-----------------------------------------------
    depth pixel (u, v) --intrinsics--> camera optical frame (X right, Y down, Z fwd)
    camera optical frame --T_tcp_cam--> TCP/flange frame
    TCP frame --tcp_matrix from metadata--> base frame

    T_base_cam = T_base_tcp @ T_tcp_cam        (hand-eye from config/handeye)

Outputs (under --out, default runtime/environment)
--------------------------------------------------
    <YYYYmmdd_HHMMSS>/scene.ply     merged, downsampled base-frame cloud
    <YYYYmmdd_HHMMSS>/model.json    tables, poses, collision bodies, deltas
    latest -> <YYYYmmdd_HHMMSS>     directory symlink to the newest model
    latest.json                    pointer record (works without symlinks)

Usage
-----
    python3 scripts/model_environment.py \
        --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw \
        --out runtime/environment

    # re-check the freshest model without touching the dataset
    python3 scripts/model_environment.py --out runtime/environment --verify-only

    # pretend "now" is later to exercise the age gate
    python3 scripts/model_environment.py --out runtime/environment \
        --verify-only --now 2026-10-25T09:00:00+08:00

The gate function `evaluate_gate(model, thresholds, now_iso)` is a pure
function: it imports nothing from ROS, so the unit tests can call it from a
plain Python shell.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import pathlib
import sys
from datetime import datetime, timezone

import numpy as np
from scipy.spatial.transform import Rotation

sys.stdout.reconfigure(line_buffering=True)

# --- repository-relative defaults -------------------------------------------
# Paths are resolved against the repository root, never against the caller's
# working directory, so the script behaves the same from any directory.
REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
INTRINSICS_PATH = REPO_ROOT / "config/real/camera_intrinsics.json"
HAND_EYE_PATH = REPO_ROOT / "config/handeye/tcp_to_camera.json"
UNLOAD_CONFIG_PATH = REPO_ROOT / "config/unload_real.yaml"
SCENE_MODEL_SCRIPT = REPO_ROOT / "scripts/build_scene_model.py"

# --- gate thresholds (defaults; the CLI may override each one) --------------
GATE_THRESHOLDS = {
    "slot_mm": 1.0,
    "slot_deg": 0.5,
    "workpiece_mm": 3.0,
    "workpiece_deg": 1.0,
    "min_points": 20000,
    "max_age_h": 2.0,
}

# --- workpiece pose ---------------------------------------------------------
# Measured 2026-10-24 10:15 at the fine-observation pose: 3 independent frames
# of ICP, consensus spread 1.0 mm.  The CAD origin is NOT the geometric centre,
# so the centre is derived with a fixed CAD offset expressed in the model frame.
# These are measurements, not configuration: override them on the CLI only when
# the part has been moved and re-measured.
WORKPIECE_POSE = {
    "xyz": [0.5029, 0.6351, 0.1254],
    "quat": [0.1910, 0.9816, 0.0019, -0.0068],  # xyzw
    "cad_centre_offset": [-0.208, 0.066, -0.011],
    "source": ("3-frame ICP consensus at the fine-observation pose, 2026-10-24T10:15 "
               "+08:00; xyz=(0.5029,0.6351,0.1254), quat xyzw=(0.1910,0.9816,0.0019,"
               "-0.0068); CAD origin is not the geometric centre"),
}

# --- cloud / table / collision defaults -------------------------------------
DEPTH_MIN_M = 0.15        # closer returns are noise on this sensor
DEPTH_MAX_M = 2.0         # far returns saturate around 16 m; they are invalid
DEFAULT_STRIDE = 4        # pixel stride before merging (4 -> 320x240 per frame)
DEFAULT_CLOUD_VOXEL = 0.01
DEFAULT_TABLE_RADIUS = 0.45   # xy search box around an object anchor
DEFAULT_VOXEL = 0.02          # collision voxel edge (per task requirement)
DEFAULT_BAND = 0.03           # keep only the table z band +- 0.03 m
DEFAULT_MIN_VOXEL_POINTS = 3
DEFAULT_MAX_BOXES = 6000
MIN_FRAME_POINTS = 1000

# --- module-level cache for the reused helper script -------------------------
_HELPER_MODULE = None


def helper_module():
    """Load scripts/build_scene_model.py by path and reuse its functions.

    The repository keeps reusable helpers inside scripts (not a package), so we
    import by file path.  load_metadata() and frame_to_base() already encode the
    intrinsics + hand-eye chain; there is no reason to re-derive them here.
    build_environment_collision.py is deliberately NOT reused: it imports rclpy
    and moveit_msgs at module level, which would break the no-ROS requirement.
    """

    global _HELPER_MODULE
    if _HELPER_MODULE is None:
        spec = importlib.util.spec_from_file_location("cs625_build_scene_model",
                                                      SCENE_MODEL_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load helper script: {SCENE_MODEL_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _HELPER_MODULE = module
    return _HELPER_MODULE


# ---------------------------------------------------------------------------
# small numeric / time helpers
# ---------------------------------------------------------------------------
def parse_iso8601(text) -> datetime | None:
    """Parse an ISO8601 stamp; naive input is treated as UTC."""

    if not text or not isinstance(text, str):
        return None
    try:
        stamp = datetime.fromisoformat(text.strip().replace("Z", "+00:00"))
    except ValueError:
        return None
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return stamp


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def as_float(value, default=0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(out):
        return default
    return out


def quat_angle_deg(quat_a, quat_b) -> float:
    """Angle of the relative rotation between two xyzw quaternions, degrees."""

    rot_a = Rotation.from_quat(np.asarray(quat_a, dtype=float))
    rot_b = Rotation.from_quat(np.asarray(quat_b, dtype=float))
    return float(np.degrees((rot_a.inv() * rot_b).magnitude()))


def pose_delta(previous: dict, current: dict) -> dict:
    """Position (mm) and orientation (deg) difference between two xyz/quat poses."""

    if not previous or not current:
        return {"mm": 0.0, "deg": 0.0}
    xyz_a = np.asarray(previous["xyz"], dtype=float)
    xyz_b = np.asarray(current["xyz"], dtype=float)
    mm = float(np.linalg.norm(xyz_b - xyz_a) * 1000.0)
    deg = quat_angle_deg(previous["quat"], current["quat"])
    return {"mm": round(mm, 4), "deg": round(deg, 4)}


def r4(value):
    return [round(float(v), 6) for v in np.asarray(value, dtype=float).ravel()]


# ---------------------------------------------------------------------------
# gate: pure function, no ROS, no file access
# ---------------------------------------------------------------------------
def evaluate_gate(model: dict, thresholds: dict, now_iso: str) -> tuple[bool, list[str]]:
    """Decide whether a stored model may be used, and why not if it may not.

    Rejected when (each check is real, not a log line):
      1. slot deviation from the previous model exceeds thresholds["slot_mm"] /
         ["slot_deg"];
      2. workpiece deviation from the previous model exceeds
         thresholds["workpiece_mm"] / ["workpiece_deg"];
      3. the merged cloud is thinner than thresholds["min_points"];
      4. the model is older than thresholds["max_age_h"] hours relative to now_iso.

    Returns (passed, reasons).  Reasons are Chinese (they are user-facing text
    stored in the model / printed by --verify-only); the rest of the script is
    English.
    """

    merged = dict(GATE_THRESHOLDS)
    if thresholds:
        merged.update({k: v for k, v in thresholds.items() if v is not None})

    if not isinstance(model, dict) or not model:
        return False, ["模型为空或不是 JSON 对象，无法评估门禁"]

    reasons: list[str] = []
    delta = model.get("baseline_delta") or {}

    slot_mm = as_float(delta.get("slot_mm"))
    slot_deg = as_float(delta.get("slot_deg"))
    work_mm = as_float(delta.get("workpiece_mm"))
    work_deg = as_float(delta.get("workpiece_deg"))

    if slot_mm > merged["slot_mm"]:
        reasons.append(
            f"插槽位置偏差 {slot_mm:.2f} mm 超过阈值 {merged['slot_mm']:.2f} mm"
            "（工装机械固定，出现这种漂移说明标定或数据有问题）")
    if slot_deg > merged["slot_deg"]:
        reasons.append(
            f"插槽姿态偏差 {slot_deg:.2f} deg 超过阈值 {merged['slot_deg']:.2f} deg"
            "（工装机械固定，姿态不应变化）")
    if work_mm > merged["workpiece_mm"]:
        reasons.append(
            f"工件位置偏差 {work_mm:.2f} mm 超过阈值 {merged['workpiece_mm']:.2f} mm"
            "（工件可能被移动，需要重新精观测）")
    if work_deg > merged["workpiece_deg"]:
        reasons.append(
            f"工件姿态偏差 {work_deg:.2f} deg 超过阈值 {merged['workpiece_deg']:.2f} deg"
            "（工件可能被转动，需要重新精观测）")

    points = as_float(model.get("valid_points"))
    if points < merged["min_points"]:
        reasons.append(
            f"有效点云只有 {int(points)} 点，少于下限 {int(merged['min_points'])} 点"
            "（视野遮挡或采集失败，不能作为环境模型）")

    created = parse_iso8601(model.get("created_at"))
    now = parse_iso8601(now_iso)
    if created is None:
        reasons.append("模型缺少可解析的 created_at 时间戳，无法判断新鲜度")
    elif now is None:
        reasons.append(f"无法解析当前时间 {now_iso!r}，无法判断模型新鲜度")
    else:
        age_h = (now - created).total_seconds() / 3600.0
        if age_h > merged["max_age_h"]:
            reasons.append(
                f"模型已过期 {age_h:.2f} 小时，超过允许的 {merged['max_age_h']:.2f} 小时"
                "（环境可能已变化，请重新建模）")

    return (len(reasons) == 0), reasons


# ---------------------------------------------------------------------------
# inputs: configs, metadata, RGB-D frames
# ---------------------------------------------------------------------------
def read_json(path: pathlib.Path) -> dict:
    return json.loads(pathlib.Path(path).read_text(encoding="utf-8"))


def read_reference_poses(path: pathlib.Path) -> dict:
    """Read the fixed slot / inserted-workpiece poses from config/unload_real.yaml."""

    import yaml  # local import: keeps `import model_environment` light

    document = yaml.safe_load(pathlib.Path(path).read_text(encoding="utf-8")) or {}
    for key in ("slot_origin", "slot_origin_q", "box_in_slot", "box_in_slot_q"):
        if key not in document:
            raise SystemExit(f"{path}: missing required key {key!r}")
    return {
        "slot_origin": [float(v) for v in document["slot_origin"]],
        "slot_origin_q": [float(v) for v in document["slot_origin_q"]],
        "box_in_slot": [float(v) for v in document["box_in_slot"]],
        "box_in_slot_q": [float(v) for v in document["box_in_slot_q"]],
    }


def collect_frames(dataset: pathlib.Path, camera: dict, hand_eye: np.ndarray,
                   stride: int, max_frames: int, log=print):
    """Back-project every (depth, pose) pair into the base frame.

    Returns (frames, skipped).  frames is a list of dicts with the batch, the
    file stem and an (N, 3) point array in the base frame; skipped maps a batch
    directory to the reason it contributed no frame.
    """

    frames: list[dict] = []
    skipped: dict[str, str] = {}

    batches = [dataset] + sorted(p for p in dataset.iterdir() if p.is_dir())
    for scene_dir in batches:
        if not scene_dir.is_dir():
            continue
        if scene_dir == dataset and not (dataset / "metadata.jsonl").is_file():
            continue  # the capture root is only a batch when it holds its own metadata
        try:
            poses = helper_module().load_metadata(scene_dir)
        except Exception as error:  # unreadable metadata is not fatal
            skipped[scene_dir.name] = f"metadata unreadable: {error}"
            continue
        depth_files = sorted(scene_dir.glob("*_depth.npy"))
        if not (scene_dir / "metadata.jsonl").is_file():
            skipped[scene_dir.name] = "no metadata.jsonl (depth frames carry no pose)"
            continue
        if not poses:
            skipped[scene_dir.name] = ("metadata.jsonl has no tcp_matrix entry "
                                       "(position-only capture, cannot back-project)")
            continue
        if not depth_files:
            skipped[scene_dir.name] = "metadata.jsonl but no *_depth.npy files"
            continue

        used_here = 0
        for depth_path in depth_files:
            if max_frames and len(frames) >= max_frames:
                break
            stem = depth_path.name[: -len("_depth.npy")]
            tcp = poses.get(stem)
            if tcp is None:
                skipped.setdefault(scene_dir.name, "depth file without a pose entry")
                continue
            depth = np.load(depth_path)
            if depth.ndim != 2:
                skipped.setdefault(scene_dir.name, "depth array is not 2-D")
                continue
            # 0 means invalid on this sensor; saturated far returns (~16 m) are
            # just as useless, so both are folded into 0 before back-projection.
            depth = np.where((depth < DEPTH_MIN_M * 1000.0) | (depth > DEPTH_MAX_M * 1000.0),
                             0, depth).astype(np.uint16)
            base_camera = np.asarray(tcp, dtype=float) @ hand_eye
            points = helper_module().frame_to_base(depth, camera, base_camera, stride=stride)
            if len(points) < MIN_FRAME_POINTS:
                skipped.setdefault(scene_dir.name, "frames with fewer than 1000 valid points")
                continue
            frames.append({
                "batch": scene_dir.name,
                "file": stem,
                "points": points.astype(np.float32),
            })
            used_here += 1
            if used_here and len(frames) % 20 == 0:
                log(f"  {len(frames)} frames back-projected")
        if used_here == 0 and scene_dir.name not in skipped:
            skipped[scene_dir.name] = "no usable frame"

    return frames, skipped


def merge_cloud(frames: list[dict], voxel: float, log=print):
    """Merge per-frame clouds, voxel-downsample and drop statistical outliers."""

    import open3d as o3d  # local import: only the modelling path needs it

    raw_points = int(sum(len(frame["points"]) for frame in frames))
    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(
        np.concatenate([frame["points"] for frame in frames]).astype(np.float64))
    log(f"merged {len(frames)} frames -> {raw_points} raw points")
    downsampled = merged.voxel_down_sample(voxel)
    log(f"voxel downsample {voxel} m -> {len(downsampled.points)} points")
    downsampled, _ = downsampled.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    points = np.asarray(downsampled.points, dtype=np.float64)
    log(f"statistical outlier removal -> {len(points)} points")
    return points, downsampled, raw_points


# ---------------------------------------------------------------------------
# table plane fitting
# ---------------------------------------------------------------------------
def _smooth(hist: np.ndarray) -> np.ndarray:
    kernel = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
    return np.convolve(hist.astype(float), kernel / kernel.sum(), mode="same")


def _connected_extent(band: np.ndarray, anchor_xy, cell: float, min_points: int,
                      max_ring: int = 40):
    """Flood fill the table surface starting at the object's xy column.

    Returns (xy_min, xy_max, cells, anchor_cell_found, anchor_cell_used).  The
    component matters: a global bounding box would also swallow the far wall and
    the ghost copies of the same table that some frames produce.
    """

    keys = np.floor(band[:, :2] / cell).astype(np.int64)
    unique_keys, counts = np.unique(keys, axis=0, return_counts=True)
    occupied = {tuple(int(v) for v in key) for key, count in zip(unique_keys, counts)
                if count >= min_points}

    start = (int(math.floor(anchor_xy[0] / cell)), int(math.floor(anchor_xy[1] / cell)))
    found = start in occupied
    if not found:  # the object itself may hide the surface right under the anchor
        best = None
        for dx in range(-max_ring, max_ring + 1):
            for dy in range(-max_ring, max_ring + 1):
                candidate = (start[0] + dx, start[1] + dy)
                if candidate in occupied:
                    distance = dx * dx + dy * dy
                    if best is None or distance < best[0]:
                        best = (distance, candidate)
        if best is None:
            return None
        start = best[1]

    seen = {start}
    queue = [start]
    while queue:
        cell_x, cell_y = queue.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour = (cell_x + dx, cell_y + dy)
            if neighbour in occupied and neighbour not in seen:
                seen.add(neighbour)
                queue.append(neighbour)

    cells = np.asarray(sorted(seen), dtype=np.float64)
    low = cells.min(axis=0) * cell
    high = (cells.max(axis=0) + 1.0) * cell
    return ([float(low[0]), float(low[1])], [float(high[0]), float(high[1])],
            len(seen), found, [round(start[0] * cell, 4), round(start[1] * cell, 4)])


def find_support_plane(points: np.ndarray, anchor_xyz, radius: float, log=print,
                       name: str = "table"):
    """Fit the horizontal surface that the given object rests on.

    Method (object-centric, so the two tables are told apart by geometry rather
    than by a hand-written box):
      1. keep the points in an xy window around the object anchor, below it;
      2. take the densest horizontal sheet in the z histogram (frame-to-frame
         pose wobble smears a table over a few cm, and stale poses can add ghost
         copies of the same table, so the peak - not a single RANSAC plane - is
         what identifies the real surface);
      3. fit a plane to that +-2 cm band with the RANSAC from
         scripts/build_scene_model.py and require it to be near-horizontal;
      4. measure the sheet's extent by flood filling its occupancy grid from the
         object's xy column inside the search window; the component is what makes
         the range meaningful (a bounding box would swallow the far wall).
    """

    anchor = np.asarray(anchor_xyz, dtype=float)
    window_mask = ((np.abs(points[:, 0] - anchor[0]) <= radius)
                   & (np.abs(points[:, 1] - anchor[1]) <= radius)
                   & (points[:, 2] <= anchor[2] - 0.02)
                   & (points[:, 2] >= anchor[2] - 0.60))
    window = points[window_mask]
    if len(window) < 500:
        raise RuntimeError(f"{name}: only {len(window)} candidate points in the "
                           f"search window around {r4(anchor)}")

    z_low = float(window[:, 2].min())
    z_high = float(anchor[2] - 0.02)
    edges = np.arange(z_low, z_high + 0.01, 0.01)
    hist, _ = np.histogram(window[:, 2], bins=edges)
    smoothed = _smooth(hist)
    peak = int(np.argmax(smoothed))
    sheet_z = float((edges[peak] + edges[peak + 1]) / 2.0)

    # Keep the runner-up sheets as well: the picked one is the densest horizontal
    # surface under the object, and the alternatives show what was rejected (e.g.
    # a fixture plate that sits closer to the object than the bench below it).
    candidates = []
    for index in np.argsort(smoothed)[::-1]:
        z_candidate = float((edges[index] + edges[index + 1]) / 2.0)
        if all(abs(z_candidate - item["z"]) >= 0.05 for item in candidates):
            candidates.append({"z": round(z_candidate, 4), "points": int(hist[index])})
        if len(candidates) == 3:
            break

    band = window[np.abs(window[:, 2] - sheet_z) <= 0.02]
    normal, offset, _ = helper_module().fit_plane(band, 0.01, 500)
    normal = np.asarray(normal, dtype=float)
    horizontal = abs(float(normal[2])) >= 0.9
    if horizontal:
        # z of the fitted plane directly under the object (n.z is not 0)
        z_anchor = float(-(normal @ np.array([anchor[0], anchor[1], 0.0]) + offset)
                         / normal[2])
        plane_z = float(np.median(band[:, 2]))
    else:
        normal = np.array([0.0, 0.0, 1.0])
        offset = -sheet_z
        z_anchor = sheet_z
        plane_z = sheet_z

    # The extent is measured on the search-window band, not on the whole cloud:
    # used globally, the same z sheet also contains the far wall and the floor of
    # the next bench, and the flood fill would merge them into one 2 x 2 m blob.
    # So the reported xy range is the visible table patch under the object, i.e. a
    # conservative lower bound on the real table size.
    extent_result = _connected_extent(band, anchor[:2], 0.02, 2)
    if extent_result is None:
        return None
    xy_min, xy_max, cells, anchor_found, anchor_cell = extent_result
    extent = [float(xy_max[0] - xy_min[0]), float(xy_max[1] - xy_min[1])]

    return {
        "z": round(plane_z, 5),
        "z_at_anchor": round(z_anchor, 5),
        "xy_min": [round(v, 4) for v in xy_min],
        "xy_max": [round(v, 4) for v in xy_max],
        "extent": [round(v, 4) for v in extent],
        "normal": [round(float(v), 5) for v in normal],
        "offset": round(float(offset), 5),
        "horizontal": bool(horizontal),
        "inliers": int(len(band)),
        "candidate_sheets": candidates,
        "sheet_cells": int(cells),
        "anchor_cell_on_sheet": bool(anchor_found),
        "anchor_cell_used": anchor_cell,
        "anchor_xyz": [round(float(v), 4) for v in anchor],
        "search_radius": radius,
    }


def count_supporting_frames(frames: list[dict], table: dict, band: float = 0.01,
                            min_points: int = 40) -> int:
    """How many independent views actually saw this sheet (robustness number)."""

    supported = 0
    for frame in frames:
        points = frame["points"]
        mask = ((np.abs(points[:, 2] - table["z"]) <= band)
                & (points[:, 0] >= table["xy_min"][0]) & (points[:, 0] <= table["xy_max"][0])
                & (points[:, 1] >= table["xy_min"][1]) & (points[:, 1] <= table["xy_max"][1]))
        if int(mask.sum()) >= min_points:
            supported += 1
    return supported


# ---------------------------------------------------------------------------
# collision bodies
# ---------------------------------------------------------------------------
def voxel_collision_bodies(points: np.ndarray, table: dict, name: str,
                           voxel: float = DEFAULT_VOXEL, band: float = DEFAULT_BAND,
                           min_points: int = DEFAULT_MIN_VOXEL_POINTS,
                           max_boxes: int = DEFAULT_MAX_BOXES) -> tuple[list, int, int]:
    """Turn the table z band into 2.5-D voxel boxes for MoveIt.

    Only points within +-band of the fitted table z are used, and only inside
    the table's own xy extent.  Each kept xy voxel becomes one box whose height
    covers the band, which is the same convention as the accepted
    scripts/build_environment_collision.py (xy voxel, fixed z band), but with the
    required 0.02 m edge instead of 0.05 m.

    Returns (boxes, band_point_count, occupied_cell_count).
    """

    selection = ((np.abs(points[:, 2] - table["z"]) <= band)
                 & (points[:, 0] >= table["xy_min"][0]) & (points[:, 0] <= table["xy_max"][0])
                 & (points[:, 1] >= table["xy_min"][1]) & (points[:, 1] <= table["xy_max"][1]))
    band_points = points[selection]
    if len(band_points) == 0:
        return [], 0, 0

    keys = np.floor(band_points[:, :2] / voxel).astype(np.int64)
    unique_keys, counts = np.unique(keys, axis=0, return_counts=True)  # sorted, deterministic
    keep = counts >= min_points
    kept_keys = unique_keys[keep]
    kept_counts = counts[keep]
    occupied = int(len(kept_keys))
    # Deterministic truncation: keep the densest cells, tie-break on the key.
    order = np.lexsort((kept_keys[:, 1], kept_keys[:, 0], -kept_counts))
    kept_keys = kept_keys[order][:max_boxes]

    boxes = []
    for index, (key_x, key_y) in enumerate(kept_keys):
        # The box hangs below the fitted surface (top face flush with the table
        # top) so the collision scene never claims free space just above the
        # table, which is exactly where a grasp or a place pose lives.
        center = [(key_x + 0.5) * voxel, (key_y + 0.5) * voxel,
                  float(table["z"]) - float(band)]
        boxes.append({
            "name": f"{name}_voxel_{index:04d}",
            "center": [round(float(v), 4) for v in center],
            "size": [round(float(voxel), 4), round(float(voxel), 4),
                     round(float(2.0 * band), 4)],
        })
    return boxes, int(len(band_points)), occupied


# ---------------------------------------------------------------------------
# model assembly
# ---------------------------------------------------------------------------
def find_previous_model(out_dir: pathlib.Path) -> tuple[pathlib.Path | None, dict | None]:
    """Newest earlier model.json under out_dir (by timestamp directory name)."""

    if not out_dir.is_dir():
        return None, None
    candidates = []
    for entry in out_dir.iterdir():
        if not entry.is_dir():
            continue
        name = entry.name
        if len(name) < 15 or name[8] != "_":
            continue
        if not (name[:8].isdigit() and name[9:15].isdigit()):
            continue
        model_path = entry / "model.json"
        if model_path.is_file():
            candidates.append((name, model_path))
    if not candidates:
        return None, None
    candidates.sort(key=lambda item: item[0])
    name, model_path = candidates[-1]
    try:
        return model_path, read_json(model_path)
    except Exception:
        return model_path, None


def write_latest(out_dir: pathlib.Path, directory: pathlib.Path, log=print):
    """Point `latest` (symlink) and `latest.json` (plain record) at the new model."""

    link = out_dir / "latest"
    try:
        if link.is_symlink() or link.exists():
            if link.is_symlink() or link.is_file():
                link.unlink()
            else:
                log(f"latest: {link} is a real directory, leaving it alone")
                link = None
        if link is not None:
            os.symlink(directory.name, link)  # relative link: survives a moved out_dir
            log(f"latest -> {directory.name}")
    except OSError as error:
        log(f"latest symlink skipped: {error}")

    pointer = {
        "latest": directory.name,
        "model": f"{directory.name}/model.json",
        "scene_ply": f"{directory.name}/scene.ply",
        "updated_at": utc_now().isoformat(timespec="seconds"),
    }
    (out_dir / "latest.json").write_text(json.dumps(pointer, indent=1) + "\n",
                                         encoding="utf-8")
    log(f"latest.json -> {pointer['model']}")


def build_model(arguments) -> tuple[int, dict | None]:
    import open3d as o3d  # noqa: F401  (kept explicit: the PLY writer needs it)

    dataset = pathlib.Path(arguments.dataset).expanduser().resolve()
    out_dir = pathlib.Path(arguments.out).expanduser().resolve()
    if not dataset.is_dir():
        print(f"dataset not found: {dataset}")
        return 2, None

    camera = read_json(pathlib.Path(arguments.intrinsics))
    hand_eye = np.array(read_json(pathlib.Path(arguments.hand_eye))["T_tcp_cam"],
                        dtype=float)
    reference = read_reference_poses(pathlib.Path(arguments.unload_config))
    print(f"camera intrinsics: fx={camera['fx']:.2f} fy={camera['fy']:.2f} "
          f"cx={camera['cx']:.2f} cy={camera['cy']:.2f} "
          f"({camera['width']}x{camera['height']})")
    print(f"hand-eye T_tcp_cam translation: "
          f"{r4(hand_eye[:3, 3])} m")

    print(f"[1/6] reading RGB-D frames from {dataset}")
    frames, skipped = collect_frames(dataset, camera, hand_eye,
                                     stride=arguments.stride,
                                     max_frames=arguments.max_frames)
    for batch, reason in sorted(skipped.items()):
        print(f"  skipped batch {batch}: {reason}")
    if not frames:
        print("no usable frame (need <file>_depth.npy plus a tcp_matrix entry)")
        return 2, None
    batches = sorted({frame['batch'] for frame in frames})
    print(f"frames used: {len(frames)} from {len(batches)} batches "
          f"({', '.join(batches)})")

    print("[2/6] merging into the base-frame cloud")
    points, cloud, raw_points = merge_cloud(frames, arguments.cloud_voxel)

    workpiece_xyz = np.array(arguments.workpiece_xyz, dtype=float)
    workpiece_quat = np.array(arguments.workpiece_quat, dtype=float)
    cad_offset = np.array(WORKPIECE_POSE["cad_centre_offset"], dtype=float)
    workpiece_centre = workpiece_xyz + Rotation.from_quat(workpiece_quat).as_matrix() @ cad_offset

    print("[3/6] fitting table planes")
    anchors = {
        "workpiece_table": (workpiece_centre,
                            "workpiece table: measured surface under the workpiece centre"),
        "slot_table": (np.array(reference["slot_origin"], dtype=float),
                       "slot table: measured surface under the fixed slot origin"),
    }
    tables: dict[str, dict] = {}
    for name, (anchor, label) in anchors.items():
        try:
            table = find_support_plane(points, anchor, arguments.table_radius, name=name)
        except RuntimeError as error:
            print(f"  {name}: {error}")
            continue
        if table is None:
            print(f"  {name}: no connected sheet found under {r4(anchor)}")
            continue
        table["label"] = label
        table["supporting_frames"] = count_supporting_frames(frames, table)
        tables[name] = table
        print(f"  {name}: z={table['z']:+.4f} m (z at anchor {table['z_at_anchor']:+.4f}) "
              f"normal={table['normal']} inliers={table['inliers']} "
              f"sheet cells={table['sheet_cells']} "
              f"anchor on sheet={table['anchor_cell_on_sheet']} "
              f"supporting frames={table['supporting_frames']}/{len(frames)}")
        print(f"    xy range x[{table['xy_min'][0]:.3f},{table['xy_max'][0]:.3f}] "
              f"y[{table['xy_min'][1]:.3f},{table['xy_max'][1]:.3f}] "
              f"extent {table['extent'][0]:.3f} x {table['extent'][1]:.3f} m")
    if not tables:
        print("no table plane could be fitted")
        return 2, None

    print("[4/6] building voxel collision bodies")
    collisions: list[dict] = []
    for name, table in tables.items():
        boxes, band_points, cells_total = voxel_collision_bodies(
            points, table, name,
            voxel=arguments.voxel, band=arguments.band,
            min_points=arguments.min_voxel_points, max_boxes=arguments.max_boxes)
        table["band_points"] = band_points
        table["voxel_boxes"] = len(boxes)
        collisions.extend(boxes)
        print(f"  {name}: {len(boxes)} voxel boxes from {band_points} points in the "
              f"z band +-{arguments.band} m (voxel {arguments.voxel} m, "
              f"{cells_total} occupied cells, box top at z={table['z']:+.4f} m)")
        if cells_total > len(boxes):
            print(f"    truncated to --max-boxes {arguments.max_boxes} "
                  f"({cells_total - len(boxes)} cells dropped)")
    total_boxes = len(collisions)

    print("[5/6] comparing with the previous model")
    previous_path, previous_model = find_previous_model(out_dir)
    slot_current = {"xyz": [round(float(v), 6) for v in reference["slot_origin"]],
                    "quat": [round(float(v), 6) for v in reference["slot_origin_q"]]}
    box_current = {"xyz": [round(float(v), 6) for v in reference["box_in_slot"]],
                   "quat": [round(float(v), 6) for v in reference["box_in_slot_q"]]}
    workpiece_current = {"xyz": [round(float(v), 6) for v in workpiece_xyz],
                         "quat": [round(float(v), 6) for v in workpiece_quat],
                         "centre": [round(float(v), 6) for v in workpiece_centre]}
    if previous_model is None:
        delta = {"slot_mm": 0.0, "slot_deg": 0.0, "workpiece_mm": 0.0, "workpiece_deg": 0.0}
        first_model = True
        print("  no previous model -> first model, baseline delta all zero")
    else:
        slot_delta = pose_delta(previous_model.get("slot"), slot_current)
        work_delta = pose_delta(previous_model.get("workpiece"), workpiece_current)
        delta = {"slot_mm": slot_delta["mm"], "slot_deg": slot_delta["deg"],
                 "workpiece_mm": work_delta["mm"], "workpiece_deg": work_delta["deg"]}
        first_model = False
        print(f"  previous model {previous_path}")
        print(f"  delta vs previous: slot {delta['slot_mm']:.2f} mm / "
              f"{delta['slot_deg']:.2f} deg, workpiece {delta['workpiece_mm']:.2f} mm / "
              f"{delta['workpiece_deg']:.2f} deg")

    created_at = (parse_iso8601(arguments.now) if arguments.now else None) or utc_now()
    stamp = created_at.strftime("%Y%m%d_%H%M%S")
    target = out_dir / stamp
    suffix = 1
    while target.exists():
        suffix += 1
        target = out_dir / f"{stamp}_{suffix}"
    target.mkdir(parents=True, exist_ok=True)

    model = {
        "created_at": created_at.isoformat(timespec="seconds"),
        "source_dataset": str(dataset),
        "frames_used": len(frames),
        "batches_used": batches,
        "batches_skipped": skipped,
        "valid_points": int(len(points)),
        "raw_points": int(raw_points),
        "pixel_stride": arguments.stride,
        "cloud_voxel_m": arguments.cloud_voxel,
        "voxel_size_m": arguments.voxel,
        "voxel_band_m": arguments.band,
        "collision_voxel_count": total_boxes,
        "slot": {**slot_current,
                 "source": "config/unload_real.yaml slot_origin / slot_origin_q "
                           "(RobotVisionSuite calibration, fixture mechanically fixed)"},
        "box_in_slot": {**box_current,
                        "source": "config/unload_real.yaml box_in_slot / box_in_slot_q"},
        "workpiece": {**workpiece_current, "source": WORKPIECE_POSE["source"]},
        "tables": tables,
        "collisions": collisions,
        "baseline_delta": delta,
        "first_model": first_model,
        "baseline_model": str(previous_path) if previous_path else None,
        "gate_thresholds": dict(GATE_THRESHOLDS),
        "scene_ply": "scene.ply",
        "notes": [
            "Offline model built from recorded RGB-D + TCP poses; nothing was "
            "commanded to the robot.",
            "scene.ply holds geometry only: scripts/build_scene_model.py "
            "frame_to_base() returns points without pixel colours.",
            "Collision bodies are 2.5-D: xy voxels of voxel_size_m, each box "
            "covering the +-voxel_band_m table band and hanging below the fitted "
            "surface (top face flush with it), so free space above a table stays "
            "free for grasping and placing.",
        ],
    }

    print("[6/6] writing outputs")
    o3d.io.write_point_cloud(str(target / "scene.ply"), cloud)
    print(f"  {target / 'scene.ply'} ({int(len(points))} points)")
    (target / "json.tmp").write_text(json.dumps(model, ensure_ascii=False, indent=1) + "\n",
                                     encoding="utf-8")
    (target / "json.tmp").replace(target / "model.json")
    print(f"  {target / 'model.json'}")
    write_latest(out_dir, target)

    passed, reasons = evaluate_gate(model, GATE_THRESHOLDS, created_at.isoformat())
    print()
    print("key numbers")
    print(f"  frames used        {model['frames_used']}")
    print(f"  valid points       {model['valid_points']} (raw {model['raw_points']})")
    for name in ("workpiece_table", "slot_table"):
        table = tables.get(name)
        if table:
            print(f"  {name:17s} z={table['z']:+.4f} m  "
                  f"x[{table['xy_min'][0]:.3f},{table['xy_max'][0]:.3f}] "
                  f"y[{table['xy_min'][1]:.3f},{table['xy_max'][1]:.3f}]  "
                  f"boxes={table['voxel_boxes']}")
    print(f"  collision voxels   {total_boxes}")
    print(f"  slot xyz           {r4(slot_current['xyz'])}")
    print(f"  slot quat          {r4(slot_current['quat'])}")
    print(f"  workpiece xyz      {r4(workpiece_current['xyz'])}")
    print(f"  workpiece quat     {r4(workpiece_current['quat'])}")
    print(f"  workpiece centre   {r4(workpiece_current['centre'])}")
    print(f"  baseline delta     slot {delta['slot_mm']:.3f} mm / {delta['slot_deg']:.3f} deg, "
          f"workpiece {delta['workpiece_mm']:.3f} mm / {delta['workpiece_deg']:.3f} deg "
          f"(first_model={first_model})")
    print()
    print(f"build gate: {'PASS' if passed else 'REJECT'}")
    for reason in reasons:
        print(f"  - {reason}")
    return (0 if passed else 1), model


# ---------------------------------------------------------------------------
# verify-only
# ---------------------------------------------------------------------------
def load_latest_model(out_dir: pathlib.Path, explicit: str | None):
    if explicit:
        path = pathlib.Path(explicit).expanduser()
        if not path.is_file():
            return None, None
        return path, read_json(path)

    pointer = out_dir / "latest.json"
    if pointer.is_file():
        try:
            record = read_json(pointer)
            path = out_dir / record.get("model", "")
            if path.is_file():
                return path, read_json(path)
        except Exception:
            pass
    path, model = find_previous_model(out_dir)
    return path, model


def print_gate_table(model: dict, thresholds: dict, now_iso: str):
    delta = model.get("baseline_delta") or {}
    created = parse_iso8601(model.get("created_at"))
    now = parse_iso8601(now_iso)
    age_h = ((now - created).total_seconds() / 3600.0
             if created and now else float("nan"))
    rows = [
        ("slot_mm", as_float(delta.get("slot_mm")), thresholds["slot_mm"], "mm"),
        ("slot_deg", as_float(delta.get("slot_deg")), thresholds["slot_deg"], "deg"),
        ("workpiece_mm", as_float(delta.get("workpiece_mm")), thresholds["workpiece_mm"], "mm"),
        ("workpiece_deg", as_float(delta.get("workpiece_deg")), thresholds["workpiece_deg"],
         "deg"),
        ("valid_points", as_float(model.get("valid_points")), thresholds["min_points"], "pt"),
        ("age_h", age_h, thresholds["max_age_h"], "h"),
    ]
    print(f"{'metric':15s}{'value':>14s}{'limit':>14s}  verdict")
    for name, value, limit, unit in rows:
        if name == "valid_points":
            ok = value >= limit
            shown = f"{int(value)}"
            lim = f">= {int(limit)}"
        else:
            ok = value <= limit
            shown = "nan" if not math.isfinite(value) else f"{value:.4f} {unit}"
            lim = f"<= {limit:g} {unit}"
        print(f"{name:15s}{shown:>14s}{lim:>14s}  {'ok' if ok else 'EXCEEDED'}")


def verify_only(arguments, out_dir: pathlib.Path) -> int:
    thresholds = dict(GATE_THRESHOLDS)
    for key in GATE_THRESHOLDS:
        value = getattr(arguments, f"threshold_{key}", None)
        if value is not None:
            thresholds[key] = value
    now_iso = arguments.now or utc_now().isoformat(timespec="seconds")

    path, model = load_latest_model(out_dir, arguments.model)
    if model is None:
        print(f"no model.json under {out_dir} (build one first)")
        return 2
    print(f"model      {path}")
    print(f"created_at {model.get('created_at')}")
    print(f"now        {now_iso}")
    print(f"frames     {model.get('frames_used')}   valid_points {model.get('valid_points')}")
    print(f"first_model {model.get('first_model')}   "
          f"baseline {model.get('baseline_model')}")
    slot = model.get("slot") or {}
    print(f"slot       xyz={r4(slot.get('xyz', []))} quat={r4(slot.get('quat', []))}")
    workpiece = model.get("workpiece") or {}
    print(f"workpiece  xyz={r4(workpiece.get('xyz', []))} "
          f"centre={r4(workpiece.get('centre', []))}")
    print()
    print_gate_table(model, thresholds, now_iso)
    print()
    passed, reasons = evaluate_gate(model, thresholds, now_iso)
    if passed:
        print("VERDICT: PASS 模型可用")
        return 0
    print("VERDICT: REJECT 模型不可用")
    for reason in reasons:
        print(f"  - {reason}")
    return 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset",
                        default=str(pathlib.Path.home() / "cs625_yolo_seg/datasets/2026-10-25_raw"),
                        help="capture root; every batch subdirectory is scanned")
    parser.add_argument("--out", default="runtime/environment",
                        help="output root (one timestamped directory per model)")
    parser.add_argument("--verify-only", action="store_true",
                        help="load the newest model.json and print the gate verdict")
    parser.add_argument("--now", default=None,
                        help="ISO8601 'now' (default: current UTC time); also stamps "
                             "the new model directory in build mode")
    parser.add_argument("--model", default=None,
                        help="with --verify-only: explicit model.json to check")
    parser.add_argument("--intrinsics", default=str(INTRINSICS_PATH))
    parser.add_argument("--hand-eye", default=str(HAND_EYE_PATH))
    parser.add_argument("--unload-config", default=str(UNLOAD_CONFIG_PATH))
    parser.add_argument("--stride", type=int, default=DEFAULT_STRIDE,
                        help="pixel stride while back-projecting (default 4)")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="0 = use every frame")
    parser.add_argument("--cloud-voxel", type=float, default=DEFAULT_CLOUD_VOXEL)
    parser.add_argument("--table-radius", type=float, default=DEFAULT_TABLE_RADIUS,
                        help="xy radius of the search window around each object")
    parser.add_argument("--voxel", type=float, default=DEFAULT_VOXEL,
                        help="collision voxel edge in metres")
    parser.add_argument("--band", type=float, default=DEFAULT_BAND,
                        help="keep only points within +-band of the table z")
    parser.add_argument("--min-voxel-points", type=int, default=DEFAULT_MIN_VOXEL_POINTS)
    parser.add_argument("--max-boxes", type=int, default=DEFAULT_MAX_BOXES)
    parser.add_argument("--workpiece-xyz", type=float, nargs=3,
                        default=WORKPIECE_POSE["xyz"])
    parser.add_argument("--workpiece-quat", type=float, nargs=4,
                        default=WORKPIECE_POSE["quat"],
                        help="xyzw")
    for key, value in GATE_THRESHOLDS.items():
        parser.add_argument(f"--threshold-{key.replace('_', '-')}", dest=f"threshold_{key}",
                            type=float, default=None,
                            help=f"override the gate threshold for {key} "
                                 f"(default {value})")
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    out_dir = pathlib.Path(arguments.out).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    if arguments.verify_only:
        return verify_only(arguments, out_dir)
    code, _model = build_model(arguments)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
