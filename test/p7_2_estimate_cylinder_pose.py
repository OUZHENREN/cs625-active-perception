#!/usr/bin/env python3
"""Estimate a conservative cylinder pose from one frozen P7.1 RGB-D window.

The estimator never opens a Gazebo pose or ground-truth file.  It uses the
camera principal point as the registered-observation prior, segments a compact
RGB/depth component, and offsets the observed front surface by the versioned
YCB cylinder radius.  Geometry alone cannot recover texture yaw, so that
degree of freedom is reported as unobservable instead of being fabricated.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_closing, label
from scipy.spatial.transform import Rotation


def read_ppm(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P6":
            raise ValueError("expected binary P6 PPM")
        width, height = map(int, stream.readline().split())
        if int(stream.readline()) != 255:
            raise ValueError("expected 8-bit PPM")
        return np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width, 3)


def transform_matrix(record: dict) -> np.ndarray:
    translation = record["translation_m"]
    orientation = record["orientation"]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [orientation[key] for key in ("x", "y", "z", "w")]
    ).as_matrix()
    matrix[:3, 3] = [translation[key] for key in ("x", "y", "z")]
    return matrix


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cylinder-radius-m", type=float, default=0.034)
    parser.add_argument("--crop-radius-px", type=int, default=12)
    parser.add_argument("--minimum-points", type=int, default=12)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite estimate: {args.output}")

    metadata = json.loads((args.window / "metadata.json").read_text(encoding="utf-8"))
    rgb = read_ppm(args.window / "color_preview.ppm")
    depth = np.load(args.window / "depth_raw.npy", allow_pickle=False)
    points = np.load(args.window / "points_xyz_m.npy", allow_pickle=False).reshape(
        rgb.shape[0], rgb.shape[1], 3
    )
    camera_info = metadata["streams"]["camera_info"]
    cx, cy = camera_info["k"][2], camera_info["k"][5]
    yy, xx = np.indices(depth.shape)
    crop = (xx - cx) ** 2 + (yy - cy) ** 2 <= args.crop_radius_px**2
    rgb_float = rgb.astype(float)
    intensity = rgb_float.mean(axis=2)
    chroma = rgb_float.max(axis=2) - rgb_float.min(axis=2)
    finite_working_range = np.isfinite(depth) & (depth >= 0.30) & (depth <= 1.00)
    usable_chroma = chroma[crop & finite_working_range]
    threshold = max(12.0, float(np.percentile(usable_chroma, 60.0)))
    chromatic_seed = (
        crop
        & finite_working_range
        & (chroma >= threshold)
        & (intensity <= 190.0)
    )
    chromatic_seed = binary_closing(
        chromatic_seed, structure=np.ones((3, 3)), iterations=2
    )
    seed_depth = depth[chromatic_seed]
    if seed_depth.size:
        bins = np.arange(0.30, 1.001, 0.010)
        histogram, edges = np.histogram(
            seed_depth, bins=bins, weights=chroma[chromatic_seed]
        )
        peak = int(np.argmax(histogram))
        colour_depth_mode = 0.5 * (edges[peak] + edges[peak + 1])
        candidate = (
            crop
            & finite_working_range
            & (np.abs(depth - colour_depth_mode) <= 0.040)
        )
    else:
        colour_depth_mode = None
        candidate = chromatic_seed

    # Split bright regions first, then retain the region nearest the principal
    # point.  A 1-D depth mode removes differently ranged background pixels.
    components, count = label(candidate)
    best_label = None
    best_distance = math.inf
    for component in range(1, count + 1):
        rows, cols = np.nonzero(components == component)
        if len(rows) < 3:
            continue
        distance = float((cols.mean() - cx) ** 2 + (rows.mean() - cy) ** 2)
        if distance < best_distance:
            best_label, best_distance = component, distance
    selected = components == best_label if best_label is not None else candidate
    selected_depth = depth[selected]
    if selected_depth.size:
        bins = np.arange(selected_depth.min(), selected_depth.max() + 0.010, 0.010)
        if bins.size >= 2:
            histogram, edges = np.histogram(selected_depth, bins=bins)
            peak = int(np.argmax(histogram))
            centre_depth = 0.5 * (edges[peak] + edges[peak + 1])
            selected &= np.abs(depth - centre_depth) <= 0.035

    cloud = points[selected]
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    failures = []
    if len(cloud) < args.minimum_points:
        failures.append("SEGMENTATION_TOO_SPARSE")
    if len(cloud):
        surface = np.median(cloud, axis=0)
        centre_camera = surface + np.array((0.0, 0.0, args.cylinder_radius_m))
        covariance_camera = np.cov(cloud.T) / max(len(cloud), 1)
    else:
        centre_camera = np.full(3, np.nan)
        covariance_camera = np.full((3, 3), np.nan)

    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    centre_world = (world_from_camera @ np.r_[centre_camera, 1.0])[:3]
    covariance_world = (
        world_from_camera[:3, :3]
        @ covariance_camera
        @ world_from_camera[:3, :3].T
    )
    covariance_6d = np.zeros((6, 6))
    covariance_6d[:3, :3] = covariance_world
    covariance_6d[3:, 3:] = np.diag([0.02**2, 0.02**2, math.pi**2 / 3.0])
    failures.append("TEXTURE_YAW_UNOBSERVABLE")

    result = {
        "schema": "p7_2_pose_estimate_v1",
        "gate": "P7.2_POSE_ESTIMATION",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "estimator": "central_rgbd_cylinder_baseline_v1",
        "input_window": str(args.window),
        "ground_truth_read": False,
        "pose_frame": "world",
        "pose": {
            "position_m": centre_world.tolist(),
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "covariance_6x6": covariance_6d.tolist(),
        "quality": {
            "segmented_point_count": int(len(cloud)),
            "rgb_chroma_threshold": threshold,
            "colour_seed_point_count": int(chromatic_seed.sum()),
            "colour_depth_mode_m": colour_depth_mode,
            "component_distance_sq_px": best_distance,
            "translation_available": bool(len(cloud) >= args.minimum_points),
            "roll_pitch_prior": "upright object on support plane",
            "texture_yaw_observable": False,
        },
        "failure_codes": failures,
        "full_se3_gate_pass": not failures,
        "claim_boundary": (
            "translation baseline only; identity orientation is an upright prior and "
            "must not be scored as measured texture yaw"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
