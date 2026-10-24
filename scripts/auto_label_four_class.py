#!/usr/bin/env python3
"""Four-class automatic labelling for the CS625 capture set.

Classes
-------
0  workpiece   CAD projection (needs the workpiece pose of that capture batch)
1  slot        fixed slot box projection (senior's calibration)
2  wood table  pixels whose 3-D point lies on the wooden table plane
3  metal table pixels whose 3-D point lies on the metal table plane

Only the workpiece has a CAD model; the slot is a fixed-size box and the two
tables are planes, so every class can be produced without hand drawing.  Frames
where nothing projects become negative samples (empty label file).

Usage:
  python3 scripts/auto_label_four_class.py --dataset ~/datasets/2026-10-25_raw \
      --out ~/datasets/2026-10-25_yolo4
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

# Workpiece pose per capture batch (measured by ICP during the session).
BATCH_POSES = {
    "workpiece_d065": ([0.6803, 0.7166, 0.1245], [0.259, 0.9659, 0.0041, -0.0035]),
    "workpiece_d080": ([0.6803, 0.7166, 0.1245], [0.259, 0.9659, 0.0041, -0.0035]),
    "workpiece_v2":   ([0.6803, 0.7166, 0.1245], [0.259, 0.9659, 0.0041, -0.0035]),
    "workpiece_v3":   ([1.0382, 0.8691, 0.1219], [0.5344, 0.8452, -0.001, -0.0035]),
    "workpiece_v4":   ([1.0382, 0.8691, 0.1219], [0.5344, 0.8452, -0.001, -0.0035]),
    "workpiece_v5":   ([0.8618, 0.825, -0.0586], [0.0502, 0.71, -0.0519, 0.7005]),
    "workpiece_v6":   ([0.8114, 0.7115, 0.1234], [0.3849, 0.9229, 0.0073, -0.0045]),
    "workpiece_v7":   ([0.7894, 1.106, 0.1271], [0.9469, 0.3214, -0.0033, -0.0045]),
    "workpiece_v9":   ([0.5875, 0.771, 0.1258], [0.1896, 0.9818, 0.006, -0.0029]),
    "workpiece_v10":  ([0.5875, 0.771, 0.1258], [0.1896, 0.9818, 0.006, -0.0029]),
    "workpiece_v11":  ([0.5875, 0.771, 0.1258], [0.1896, 0.9818, 0.006, -0.0029]),
    "box_in_slot":    ([1.1635, 0.2032, 0.4423], [-0.0016, 0.0043, 0.0034, 1.0]),
    "box_in_slot2":   ([1.1635, 0.2032, 0.4423], [-0.0016, 0.0043, 0.0034, 1.0]),
}
PART_CENTROID = np.array([-0.208, 0.066, -0.011])
PART_HALF = np.array([0.2275, 0.0655, 0.2325])
SLOT_CENTER = np.array([1.2416, 0.1817, 0.4432])
SLOT_HALF = np.array([0.116, 0.372, 0.253])
TABLE_PLANES = {"wood": (-0.09, 0.022), "metal": (-0.03, 0.022)}
MIN_POLY_AREA = 1500.0


def project_points(points_base, base_camera, camera,
                   depth_image=None, tolerance=0.06):
    """Project base-frame points into the image and return their convex hull.

    Using the sampled SURFACE points instead of the eight bounding-box corners
    matters: under strong perspective the corner hull is much wider than the real
    silhouette (measured: 1120 px hull vs 650 px object).
    """

    inverse = np.linalg.inv(base_camera)
    local = points_base @ inverse[:3, :3].T + inverse[:3, 3]
    front = local[:, 2] > 0.05
    if front.sum() < 30:
        return []
    local = local[front]
    u = local[:, 0] * camera["fx"] / local[:, 2] + camera["cx"]
    v = local[:, 1] * camera["fy"] / local[:, 2] + camera["cy"]
    inside = (u > 0) & (u < 1280) & (v > 0) & (v < 960)
    if depth_image is not None:
        # occlusion test: drop model points that project where the measured depth
        # is much closer/further -- those are the hidden back faces that used to
        # stretch the hull far outside the object
        ui = np.clip(u.astype(int), 0, depth_image.shape[1] - 1)
        vi = np.clip(v.astype(int), 0, depth_image.shape[0] - 1)
        measured = depth_image[vi, ui]
        consistent = (measured > 0.05) & (np.abs(measured - local[:, 2]) < tolerance)
        inside = inside & consistent
    if inside.sum() < 30:
        return []
    hull = cv2.convexHull(np.stack([u[inside], v[inside]], axis=-1).astype(np.int32))
    if cv2.contourArea(hull) < MIN_POLY_AREA:
        return []
    return cv2.approxPolyDP(hull, 2.0, True).reshape(-1, 2)


def project_box(center, half, rotation, base_camera, camera):
    corners = center + np.array(
        [[sx * half[0], sy * half[1], sz * half[2]]
         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) @ rotation.T
    inverse = np.linalg.inv(base_camera)
    points = corners @ inverse[:3, :3].T + inverse[:3, 3]
    if np.any(points[:, 2] <= 0.05):
        return []
    u = points[:, 0] * camera["fx"] / points[:, 2] + camera["cx"]
    v = points[:, 1] * camera["fy"] / points[:, 2] + camera["cy"]
    hull = cv2.convexHull(np.stack([u, v], axis=-1).astype(np.int32))
    if cv2.contourArea(hull) < MIN_POLY_AREA:
        return []
    return cv2.approxPolyDP(hull, 2.0, True).reshape(-1, 2)


def table_polygon(depth_m, base_camera, camera, height, tolerance, step: int = 2):
    """Mask every pixel whose 3-D point lies on the given horizontal plane."""

    z = depth_m[::step, ::step]
    valid = z > 0.05
    rows, cols = np.mgrid[0:z.shape[0], 0:z.shape[1]]
    zz = z[valid]
    xx = (cols[valid] * step - camera["cx"]) * zz / camera["fx"]
    yy = (rows[valid] * step - camera["cy"]) * zz / camera["fy"]
    world = np.stack([xx, yy, zz], axis=-1) @ base_camera[:3, :3].T + base_camera[:3, 3]
    # Height alone is not enough: the curtain also has points at table height.
    # Restrict to the working region in front of the curtain (y < 1.08).
    region = (world[:, 1] < 1.08) & (world[:, 0] > 0.05) & (world[:, 0] < 1.60)
    on_plane = (np.abs(world[:, 2] - height) < tolerance) & region
    if on_plane.sum() < 200:
        return []
    small = np.zeros(z.shape, dtype=np.uint8)
    small[rows[valid][on_plane], cols[valid][on_plane]] = 1
    mask = cv2.resize(small, (depth_m.shape[1], depth_m.shape[0]),
                      interpolation=cv2.INTER_NEAREST)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    ys, xs = np.nonzero(mask)
    if len(xs) < 500:
        return []
    # the table is one large planar patch; a convex hull is the honest shape for
    # a plane (contour extraction produced thin slivers on sparse depth)
    hull = cv2.convexHull(np.stack([xs, ys], axis=-1).astype(np.int32))
    if cv2.contourArea(hull) < MIN_POLY_AREA:
        return []
    return cv2.approxPolyDP(hull, 4.0, True).reshape(-1, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--model",
                        default="~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL")
    parser.add_argument("--intrinsics", default="config/real/camera_intrinsics.json")
    parser.add_argument("--hand-eye", default="config/handeye/tcp_to_camera.json")
    parser.add_argument("--val-every", type=int, default=6)
    parser.add_argument("--classes", choices=["two", "four"], default="two",
                        help="two = workpiece+slot (tables are not labelled)")
    arguments = parser.parse_args()

    import open3d as o3d

    source = pathlib.Path(arguments.dataset).expanduser()
    target = pathlib.Path(arguments.out).expanduser()
    camera = json.loads(pathlib.Path(arguments.intrinsics).read_text())
    hand_eye = np.array(json.loads(
        pathlib.Path(arguments.hand_eye).read_text())["T_tcp_cam"], dtype=float)
    mesh = o3d.io.read_triangle_mesh(str(pathlib.Path(arguments.model).expanduser()))
    sampled = mesh.sample_points_uniformly(number_of_points=30000)
    sampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    sampled.orient_normals_towards_camera_location(np.zeros(3))
    model = np.asarray(sampled.points)
    model_normals = np.asarray(sampled.normals)
    if np.abs(model).max() > 5.0:
        model = model / 1000.0

    slot_ply = pathlib.Path("/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/"
                            "slot_pointcloud20260127.ply")
    slot_points = np.empty((0, 3))
    if slot_ply.is_file():
        slot_cloud = o3d.io.read_point_cloud(str(slot_ply))
        slot_points = np.asarray(slot_cloud.points)
        if np.abs(slot_points).max() > 5.0:
            slot_points = slot_points / 1000.0
        # The senior's slot scan is stored in the SLOT's local frame (its bbox is
        # roughly -0.14..0.01 x -0.43..0.04 x -0.435..0), so the world points are
        # just the scan shifted by the slot pose -- no re-centring.
        slot_points = slot_points + SLOT_CENTER
    for split in ("train", "val"):
        (target/"images"/split).mkdir(parents=True, exist_ok=True)
        (target/"labels"/split).mkdir(parents=True, exist_ok=True)

    counts = {"workpiece": 0, "slot": 0, "wood": 0, "metal": 0}
    negative = 0
    index = 0
    scene_dirs = [source] + sorted(d for d in source.glob("*") if d.is_dir())
    for scene_dir in scene_dirs:
        if scene_dir.name.startswith("_"):
            continue
        meta = scene_dir/"metadata.jsonl"
        if not meta.is_file():
            continue
        by_file = {}
        for line in meta.read_text(encoding="utf-8").splitlines():
            if line.strip():
                item = json.loads(line)
                by_file[item["file"]] = item
        batch_pose = BATCH_POSES.get(scene_dir.name)
        for color_path in sorted(scene_dir.glob("*_color.png")):
            stem = color_path.name[: -len("_color.png")]
            item = by_file.get(stem)
            depth_path = color_path.with_name(stem + "_depth.npy")
            if item is None or not item.get("tcp_matrix") or not depth_path.is_file():
                continue
            tcp = np.array(item["tcp_matrix"], dtype=float).reshape(4, 4)
            base_camera = tcp @ hand_eye
            depth_m = np.load(depth_path).astype(np.float64) / 1000.0
            height, width = depth_m.shape
            lines = []

            if batch_pose is not None:
                rotation = Rotation.from_quat(batch_pose[1]).as_matrix()
                centre = np.array(batch_pose[0]) + rotation @ PART_CENTROID
                world_points = (model - PART_CENTROID) @ rotation.T + centre
                polygon = project_points(
                    world_points, base_camera, camera,
                    depth_image=depth_m, tolerance=0.12)
                if len(polygon) >= 3:
                    vertices = " ".join(f"{p[0]/width:.6f} {p[1]/height:.6f}" for p in polygon)
                    lines.append(f"0 {vertices}")
                    counts["workpiece"] += 1

            polygon = project_points(slot_points, base_camera, camera)
            if len(polygon) >= 3:
                vertices = " ".join(f"{p[0]/width:.6f} {p[1]/height:.6f}" for p in polygon)
                lines.append(f"1 {vertices}")
                counts["slot"] += 1

            for class_id, name in (((2, "wood"), (3, "metal"))
                                   if arguments.classes == "four" else ()):
                height_value, tolerance = TABLE_PLANES[name]
                polygon = table_polygon(depth_m, base_camera, camera,
                                        height_value, tolerance)
                if len(polygon) >= 3:
                    vertices = " ".join(f"{p[0]/width:.6f} {p[1]/height:.6f}"
                                        for p in polygon)
                    lines.append(f"{class_id} {vertices}")
                    counts[name] += 1

            index += 1
            split = "val" if index % arguments.val_every == 0 else "train"
            (target/"images"/split/color_path.name).write_bytes(color_path.read_bytes())
            # YOLO matches labels to images by file stem, so the label must carry
            # the same "_color" suffix the image has.
            label_path = target/"labels"/split/(color_path.stem + ".txt")
            label_path.write_text("\n".join(lines) + ("\n" if lines else ""))
            if not lines:
                negative += 1
            if index % 40 == 0:
                print(f"  {index} frames {counts} negative={negative}", flush=True)

    (target/"data.yaml").write_text(
        f"path: {target.resolve()}\ntrain: images/train\nval: images/val\n"
        "names:\n  0: workpiece\n  1: slot\n  2: wood_table\n  3: metal_table\n")
    print(f"done {index} frames: {counts}, {negative} negative")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
