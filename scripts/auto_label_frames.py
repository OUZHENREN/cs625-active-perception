#!/usr/bin/env python3
"""Generate YOLO segmentation labels for captured RGB-D frames.

Method
------
The workpiece is mechanically fixed, so its pose in the robot base frame is
known from the fine ICP stage.  Every captured frame records the TCP pose it was
taken from, and the hand-eye file maps TCP to the colour optical frame.  Put the
three together and the CAD silhouette can be projected into the image:

    T_cam_part = inv(T_base_tcp · T_tcp_cam) · T_base_part
    model points -> camera frame -> pixels -> outer contour -> polygon

That is a *geometric* label: it does not depend on the current YOLO model (no
self-confirmation bias) and it does not depend on fragile depth clustering (which
previously picked the wooden table instead of the workpiece).

Frames where the projection falls outside the image or collapses to a sliver
(the workpiece is truly not visible from that pose) become negative samples with
an empty label file.

Usage:
  python3 scripts/auto_label_frames.py --dataset ~/datasets/2026-10-25_raw \
      --out ~/datasets/2026-10-25_yolo
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.stdout.reconfigure(line_buffering=True)

# Fine-stage consensus pose of the workpiece in the base frame (2026-10-24).
PART_XYZ = np.array([0.6803, 0.7166, 0.1245])
PART_QUAT = np.array([0.259, 0.9659, 0.0041, -0.0035])  # xyzw

# Empty slot fixture, base frame (config/unload_real.yaml).
SLOT_XYZ = np.array([1.241629, 0.181709, 0.443245])
SLOT_HALF_EXTENT = np.array([0.16, 0.16, 0.12])

# Measured table planes in the base frame (scene model, 2026-10-24):
#   wooden table  z ~ -0.09
#   metal table   z ~ -0.03   (about 6 cm higher)
# Labelling the tables needs no CAD model: a pixel belongs to a table when its
# 3-D point lies on that plane.
TABLE_PLANES = {
    "wood": (-0.09, 0.020),
    "metal": (-0.03, 0.020),
}
# Slot fixture box in the base frame (senior's calibration).
SLOT_BOX_CENTER = np.array([1.2416, 0.1817, 0.4432])
SLOT_BOX_HALF = np.array([0.116, 0.372, 0.253])

MIN_AREA_FRACTION = 0.010
MAX_AREA_FRACTION = 0.75


def load_intrinsics(path: pathlib.Path) -> dict:
    payload = json.loads(path.read_text())
    return {key: float(payload[key]) for key in ("fx", "fy", "cx", "cy")}


def sample_model(path: pathlib.Path, count: int = 40000) -> np.ndarray:
    """Sample points on the CAD surface (STL or PLY, mm or m auto-detected)."""

    import open3d as o3d

    if path.suffix.lower() in (".stl", ".ply", ".obj"):
        mesh = o3d.io.read_triangle_mesh(str(path))
        if len(mesh.triangles) > 0:
            cloud = mesh.sample_points_uniformly(number_of_points=count)
            points = np.asarray(cloud.points)
        else:
            cloud = o3d.io.read_point_cloud(str(path))
            points = np.asarray(cloud.points)[:count]
    else:
        raise ValueError(f"unsupported model {path}")
    if np.abs(points).max() > 5.0:      # millimetres
        points = points / 1000.0
    return points


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--model",
                        default="~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL")
    parser.add_argument("--intrinsics", default="config/real/camera_intrinsics.json")
    parser.add_argument("--hand-eye", default="config/handeye/tcp_to_camera.json")
    parser.add_argument("--val-every", type=int, default=5)
    parser.add_argument("--append", action="store_true",
                        help="add to an existing dataset instead of rebuilding the folders")
    parser.add_argument("--class-mode", choices=["workpiece", "slot", "negative"],
                        default="workpiece",
                        help="workpiece = CAD projection, slot = known 3D box, negative = empty label")
    parser.add_argument("--part-xyz", type=float, nargs=3, default=PART_XYZ.tolist())
    parser.add_argument("--part-quat", type=float, nargs=4, default=PART_QUAT.tolist())
    arguments = parser.parse_args()

    source = pathlib.Path(arguments.dataset).expanduser()
    target = pathlib.Path(arguments.out).expanduser()
    camera = load_intrinsics(pathlib.Path(arguments.intrinsics))
    hand_eye = np.array(json.loads(
        pathlib.Path(arguments.hand_eye).read_text())["T_tcp_cam"], dtype=float)
    model = sample_model(pathlib.Path(arguments.model).expanduser())

    part = np.eye(4)
    part[:3, :3] = Rotation.from_quat(arguments.part_quat).as_matrix()
    part[:3, 3] = arguments.part_xyz

    images_dir = target / "images"
    labels_dir = target / "labels"
    for split in ("train", "val"):
        (images_dir / split).mkdir(parents=True, exist_ok=True)
        (labels_dir / split).mkdir(parents=True, exist_ok=True)

    positives = negatives = skipped = counter = 0
    scene_dirs = [source] + sorted(d for d in source.glob("*") if d.is_dir())
    for scene_dir in scene_dirs:
        metadata_path = scene_dir / "metadata.jsonl"
        poses: dict[str, np.ndarray] = {}
        if metadata_path.is_file():
            for line in metadata_path.read_text(encoding="utf-8").splitlines():
                try:
                    item = json.loads(line)
                except Exception:
                    continue
                matrix = item.get("tcp_matrix")
                if matrix:
                    poses[item["file"]] = np.array(matrix, dtype=float).reshape(4, 4)
        for color_path in sorted(scene_dir.glob("*_color.png")):
            stem = color_path.name[: -len("_color.png")]
            tcp = poses.get(stem)
            if tcp is None:
                skipped += 1
                continue
            base_camera = tcp @ hand_eye
            camera_part = np.linalg.inv(base_camera) @ part
            if arguments.class_mode == "negative":
                points = np.empty((0, 3))
            elif arguments.class_mode == "slot":
                depth_path = color_path.with_name(stem + "_depth.npy")
                if not depth_path.is_file():
                    skipped += 1
                    continue
                depth_image = np.load(depth_path).astype(np.float64) / 1000.0
                height_d, width_d = depth_image.shape
                ys, xs = np.mgrid[0:height_d, 0:width_d]
                valid = depth_image > 0.05
                zz = depth_image[valid]
                xx = (xs[valid] - camera["cx"]) * zz / camera["fx"]
                yy = (ys[valid] - camera["cy"]) * zz / camera["fy"]
                cloud = np.stack([xx, yy, zz], axis=-1)
                world = cloud @ base_camera[:3, :3].T + base_camera[:3, 3]
                inside = np.all(np.abs(world - SLOT_XYZ) < SLOT_HALF_EXTENT, axis=1)
                points = cloud[inside]
            else:
                points = model @ camera_part[:3, :3].T + camera_part[:3, 3]
                front = points[:, 2] > 0.05
                points = points[front]
            image = cv2.imread(str(color_path))
            height, width = image.shape[:2]
            polygon = []
            if len(points) > 50:
                u = points[:, 0] * camera["fx"] / points[:, 2] + camera["cx"]
                v = points[:, 1] * camera["fy"] / points[:, 2] + camera["cy"]
                inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
                if inside.sum() > 50:
                    pixels = np.stack([u[inside], v[inside]], axis=-1).astype(np.int32)
                    hull = cv2.convexHull(pixels)
                    area = cv2.contourArea(hull)
                    fraction = area / float(width * height)
                    if MIN_AREA_FRACTION < fraction < MAX_AREA_FRACTION:
                        polygon = cv2.approxPolyDP(hull, 2.0, True).reshape(-1, 2)
            counter += 1
            split = "val" if counter % arguments.val_every == 0 else "train"
            (images_dir / split / color_path.name).write_bytes(color_path.read_bytes())
            label_path = labels_dir / split / (stem + ".txt")
            if len(polygon) >= 3:
                vertices = " ".join(f"{p[0] / width:.6f} {p[1] / height:.6f}"
                                    for p in polygon)
                class_id = {"workpiece": 0, "slot": 1, "negative": 0}[arguments.class_mode]
                label_path.write_text(f"{class_id} {vertices}\n")
                positives += 1
            else:
                label_path.write_text("")
                negatives += 1
            if counter % 50 == 0:
                print(f"  {counter} frames ({positives}+/{negatives}-)", flush=True)

    (target / "data.yaml").write_text(
        f"path: {target.resolve()}\ntrain: images/train\nval: images/val\n"
        "names:\n  0: workpiece\n  1: slot\n")
    print(f"done: {positives} positive, {negatives} negative, {skipped} without pose")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
