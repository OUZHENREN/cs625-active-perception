#!/usr/bin/env python3
"""Fit the simulated fixture mesh to the fixture the camera actually sees.

Why this exists: the simulation places the slot fixture at the calibrated
``slot_origin``.  That calibration is a hand measurement, and the collision mesh is
an export of the senior's CAD.  Before trusting a simulated assembly run we need to
know whether those two describe the same physical object, and by how much they are
off.

The check is a registration, not a bounding-box comparison:

    camera frames of the slot batches -> base-frame cloud (crop to the fixture)
    + collision mesh sampled at slot_origin
    -> point-to-plane ICP
    -> fitness / inlier RMSE / pose correction

A bounding-box comparison is misleading here: a single view of the fixture is
occluded, so its bbox is smaller than the real part.  The earlier version of this
analysis fell into exactly that trap and wrongly rejected the mesh.

Usage:
    source scripts/source_dev_env.sh --full
    python3 scripts/fit_fixture_to_cloud.py [--apply] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
ANALYSIS = REPO / "scripts" / "analyze_fixture_geometry.py"
MESH = (REPO / "src" / "cs625_simulation" / "assets" / "cs625_real_cell"
        / "slot_fixture" / "meshes" / "slot_fixture_collision.stl")
DATASET = pathlib.Path.home() / "cs625_yolo_seg" / "datasets" / "2026-10-25_raw"
COMMON = REPO / "config" / "tasks" / "common.yaml"


def load_analysis_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location("analyze_fixture_geometry", ANALYSIS)
    module = importlib.util.module_from_spec(spec)
    sys.modules["analyze_fixture_geometry"] = module
    spec.loader.exec_module(module)
    return module


def crop_to_fixture(cloud: np.ndarray, centre: np.ndarray,
                    radius: float = 0.60, z_low: float = -0.06,
                    z_high: float = 0.50) -> np.ndarray:
    """Keep the points that can belong to the fixture (and nothing else)."""
    horizontal = np.linalg.norm(cloud[:, :2] - centre[:2], axis=1)
    keep = (horizontal <= radius) & (cloud[:, 2] >= z_low) & (cloud[:, 2] <= z_high)
    return cloud[keep]


def sample_mesh(module, mesh_path: pathlib.Path, pose_xyz, yaw: float,
                count: int = 120000, seed: int = 11) -> np.ndarray:
    triangles, _fmt = module.read_stl_triangles(mesh_path)
    points = module.sample_triangles(triangles, count, seed=seed)
    rotation = module.rotation_yaw(yaw)
    return (rotation @ points.T).T + np.asarray(pose_xyz, dtype=float)


def register(source: np.ndarray, target: np.ndarray) -> dict:
    """Point-to-plane ICP; returns fitness, inlier RMSE and the pose correction."""
    import open3d as o3d

    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source))
    dst = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target))
    src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
    dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
    initial = np.eye(4)
    result = o3d.pipelines.registration.registration_icp(
        src, dst, max_correspondence_distance=0.02, init=initial,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-7, relative_rmse=1e-7, max_iteration=80))
    return {
        "fitness": float(result.fitness),
        "inlier_rmse_mm": float(result.inlier_rmse * 1000.0),
        "translation_mm": [float(v * 1000.0) for v in result.transformation[:3, 3]],
        "translation_norm_mm": float(np.linalg.norm(result.transformation[:3, 3]) * 1000.0),
        "rotation_deg": float(np.degrees(
            np.linalg.norm(_rotation_vector(result.transformation[:3, :3])))),
        "transformation": result.transformation.tolist(),
    }


def _rotation_vector(matrix: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(np.array(matrix, dtype=float, copy=True)).as_rotvec()


def coverage(source: np.ndarray, target: np.ndarray) -> dict:
    """How much of the mesh surface is explained by the camera cloud."""
    from scipy.spatial import cKDTree

    distances, _ = cKDTree(target).query(source)
    return {f"within_{int(thr * 1000)}mm_pct":
            float((distances < thr).mean() * 100.0)
            for thr in (0.005, 0.010, 0.020, 0.050)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--json", default="")
    parser.add_argument("--dataset", default=str(DATASET))
    parser.add_argument("--mesh", default=str(MESH))
    parser.add_argument("--apply", action="store_true",
                        help="write the refined fixture pose into the scene config")
    args = parser.parse_args()

    module = load_analysis_module()
    common = module.load_common()
    slot = np.asarray(common["slot_origin"], dtype=float)
    yaw = module.yaw_from_quat(common["slot_origin_q"])

    print(f"slot_origin      : {np.round(slot, 4).tolist()}  yaw {np.degrees(yaw):.3f} deg")
    print(f"collision mesh   : {args.mesh}")
    mesh_points = sample_mesh(module, pathlib.Path(args.mesh), slot, yaw)
    print(f"mesh samples     : {len(mesh_points)}  bbox "
          f"{np.round(module.bbox_size(mesh_points), 3).tolist()}")

    raw = module.reconstruct_fixture_cloud(pathlib.Path(args.dataset))
    cloud = module.voxel_downsample(crop_to_fixture(raw, slot), 0.004)
    print(f"cloud (cropped)  : {len(cloud)} points from {len(raw)}")
    if len(cloud) < 2000:
        print("NOT ENOUGH CLOUD POINTS — cannot fit")
        return 2
    print(f"cloud bbox       : {np.round(module.bbox_size(cloud), 3).tolist()}")

    # Baseline: how well the CALIBRATED placement already explains the cloud.
    # This is the number that matters.  ICP comes second and only as a cross
    # check, because the fixture is a near-symmetric wedge and ICP happily slides
    # ~10 cm along its long axis while keeping the same residual — a wrong answer
    # that looks right.
    from scipy.spatial import cKDTree

    baseline_distance, _ = cKDTree(cloud).query(mesh_points)
    baseline = {
        "median_mm": float(np.median(baseline_distance) * 1000.0),
        "p90_mm": float(np.percentile(baseline_distance, 90) * 1000.0),
        "within_5mm_pct": float((baseline_distance < 0.005).mean() * 100.0),
        "within_10mm_pct": float((baseline_distance < 0.010).mean() * 100.0),
        "within_20mm_pct": float((baseline_distance < 0.020).mean() * 100.0),
    }
    print("\nBASELINE (mesh at slot_origin vs camera cloud) — this is the verdict")
    print(f"  median         : {baseline['median_mm']:.2f} mm")
    print(f"  p90            : {baseline['p90_mm']:.2f} mm")
    print(f"  within 5 mm    : {baseline['within_5mm_pct']:.1f}%")
    print(f"  within 10 mm   : {baseline['within_10mm_pct']:.1f}%")
    print(f"  within 20 mm   : {baseline['within_20mm_pct']:.1f}%")

    result = register(mesh_points, cloud)
    print("\nICP (mesh -> camera cloud, cross check only)")
    print(f"  fitness        : {result['fitness']:.3f}")
    print(f"  inlier RMSE    : {result['inlier_rmse_mm']:.2f} mm")
    print(f"  correction     : {result['translation_norm_mm']:.2f} mm / "
          f"{result['rotation_deg']:.3f} deg")
    print(f"  translation    : {np.round(result['translation_mm'], 2).tolist()} mm")
    cover = coverage(mesh_points, cloud)
    print("mesh surface explained by the camera cloud: "
          + "  ".join(f"{k} {v:.1f}%" for k, v in cover.items()))

    report = {"slot_origin": slot.tolist(), "yaw_deg": float(np.degrees(yaw)),
              "mesh": args.mesh, "cloud_points": int(len(cloud)),
              "baseline": baseline,
              "icp": {k: v for k, v in result.items() if k != "transformation"},
              "coverage": cover}
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(report, indent=2),
                                           encoding="utf-8")
        print(f"report           : {args.json}")

    if result["translation_norm_mm"] > 30.0 and baseline["median_mm"] < 15.0:
        print("  NOTE: ICP moved the mesh "
              f"{result['translation_norm_mm']:.0f} mm while the calibrated placement "
              "already fits — that is a symmetry slip on a near-symmetric wedge, "
              "not a real correction.  Ignored.")
    verdict = ("PASS" if baseline["median_mm"] <= 10.0
               and baseline["within_10mm_pct"] >= 60.0
               else "PARTIAL" if baseline["median_mm"] <= 25.0 else "FAIL")
    print(f"verdict          : {verdict}  "
          "(thresholds: calibrated-placement median <=10 mm and >=60% of the mesh "
          "within 10 mm)")

    if args.apply and result["translation_norm_mm"] > 2.0:
        print("refinement > 2 mm: not applied automatically — change the generator "
              "input (slot_origin) instead of hand-editing the generated scene")
    return 0 if verdict != "FAIL" else 1


if __name__ == "__main__":
    sys.exit(main())
