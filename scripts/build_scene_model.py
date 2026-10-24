#!/usr/bin/env python3
"""Build a base-frame scene model from the captured RGB-D frames.

The capture driver records colour + depth + the full TCP pose for every frame.
Back-projecting the depth through the calibrated intrinsics and applying the
hand-eye chain gives a point cloud in the robot base frame; merging many frames
yields a model of the working cell (both tables, the curtain, the slot, the
workpiece) that can be used offline for view planning and for collision-scene
checks without touching the real cell.

Outputs:
  <out>/scene.ply           merged, downsampled cloud in the base frame
  <out>/scene_model.json    fitted planes (table tops, back wall) + extents

Usage:
  python3 scripts/build_scene_model.py --dataset ~/datasets/2026-10-25_raw \
      --out ~/datasets/scene_model
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

sys.stdout.reconfigure(line_buffering=True)

HAND_EYE = "config/handeye/tcp_to_camera.json"
INTRINSICS = "config/real/camera_intrinsics.json"


def load_metadata(scene_dir: pathlib.Path) -> dict[str, np.ndarray]:
    poses = {}
    path = scene_dir / "metadata.jsonl"
    if not path.is_file():
        return poses
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            item = json.loads(line)
        except Exception:
            continue
        matrix = item.get("tcp_matrix")
        if matrix:
            poses[item["file"]] = np.array(matrix, dtype=float).reshape(4, 4)
    return poses


def frame_to_base(depth: np.ndarray, camera: dict, base_camera: np.ndarray,
                  stride: int = 4) -> np.ndarray:
    height, width = depth.shape
    ys, xs = np.mgrid[0:height:stride, 0:width:stride]
    z = depth[::stride, ::stride].astype(np.float64) / 1000.0
    valid = z > 0.05
    x = (xs - camera["cx"]) * z / camera["fx"]
    y = (ys - camera["cy"]) * z / camera["fy"]
    points = np.stack([x[valid], y[valid], z[valid]], axis=-1)
    return points @ base_camera[:3, :3].T + base_camera[:3, 3]


def fit_plane(points: np.ndarray, threshold: float, iterations: int = 500):
    """RANSAC plane fit: returns (normal, offset, inlier mask)."""

    rng = np.random.default_rng(0)
    best = (np.zeros(3), 0.0, np.zeros(len(points), dtype=bool))
    if len(points) < 100:
        return best
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        offset = -float(normal @ sample[0])
        distance = np.abs(points @ normal + offset)
        mask = distance < threshold
        if mask.sum() > best[2].sum():
            best = (normal, offset, mask)
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--hand-eye", default=HAND_EYE)
    parser.add_argument("--intrinsics", default=INTRINSICS)
    arguments = parser.parse_args()

    import open3d as o3d

    dataset = pathlib.Path(arguments.dataset).expanduser()
    output = pathlib.Path(arguments.out).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    camera = json.loads(pathlib.Path(arguments.intrinsics).read_text())
    hand_eye = np.array(json.loads(pathlib.Path(arguments.hand_eye).read_text())
                        ["T_tcp_cam"], dtype=float)

    clouds = []
    used = 0
    scene_dirs = [dataset] + sorted(d for d in dataset.glob("*") if d.is_dir())
    for scene_dir in scene_dirs:
        poses = load_metadata(scene_dir)
        if not poses:
            continue
        for depth_path in sorted(scene_dir.glob("*_depth.npy")):
            stem = depth_path.name[: -len("_depth.npy")]
            tcp = poses.get(stem)
            if tcp is None:
                continue
            depth = np.load(depth_path)
            base_camera = tcp @ hand_eye
            points = frame_to_base(depth, camera, base_camera)
            if len(points) < 1000:
                continue
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            clouds.append(cloud)
            used += 1
            if used % 20 == 0:
                print(f"  {used} frames merged", flush=True)

    if not clouds:
        print("no frames with pose metadata")
        return 2

    merged = clouds[0]
    for cloud in clouds[1:]:
        merged += cloud
    merged = merged.voxel_down_sample(0.01)
    merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"merged {used} frames -> {len(merged.points)} points")
    o3d.io.write_point_cloud(str(output / "scene.ply"), merged)

    points = np.asarray(merged.points)
    model: dict = {"frames": used, "points": int(len(points))}
    remaining = points
    planes = []
    for name in ("table_a", "table_b", "back_wall"):
        normal, offset, mask = fit_plane(remaining, 0.02)
        if mask.sum() < 5000:
            break
        inliers = remaining[mask]
        extent = inliers.max(axis=0) - inliers.min(axis=0)
        center = inliers.mean(axis=0)
        planes.append({
            "name": name,
            "normal": [round(float(v), 4) for v in normal],
            "offset": round(float(offset), 4),
            "center": [round(float(v), 4) for v in center],
            "extent": [round(float(v), 4) for v in extent],
            "inliers": int(mask.sum()),
        })
        print(f"{name}: normal {np.round(normal, 3).tolist()} "
              f"centre {np.round(center, 3).tolist()} "
              f"extent {np.round(extent, 3).tolist()} inliers {int(mask.sum())}")
        remaining = remaining[~mask]
        if len(remaining) < 5000:
            break
    model["planes"] = planes
    (output / "scene_model.json").write_text(json.dumps(model, ensure_ascii=False, indent=1))
    print(f"scene model -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
