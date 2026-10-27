#!/usr/bin/env python3
"""Localize the workpiece from the multi-view environment capture.

Why this exists
---------------
The task's grasp poses must follow the workpiece, and the workpiece is the one
thing that changes between runs.  Until 2026-10-27 the pose was a constant
(``model_environment.WORKPIECE_POSE``) and the "gate" compared that constant with
itself; a single-view ICP at the fine observation pose was used to justify a
change and produced a 117 mm wrong answer on 2026-10-27 (proved wrong by this
multi-view method: the part was 3.4 mm from its 2026-10-24 position).

Single-view ICP on this part is degenerate: the front face is almost square, so
the rotation has a four-fold ambiguity, and only 30-50 % of the surface is seen
(RVS's own criterion, fitness > 0.5 / RMSE < 5 mm, is a multi-view criterion).

This script does what RVS did: merge the views captured during environment
modelling, register the whole CAD model against them, and additionally constrain
the rotation to stay close to the known placement (the part is always stood
upright), which removes the quarter-turn branches.

Output: the measured workpiece pose in the ``workpiece.xyz / workpiece.quat``
convention (CAD origin), plus quality numbers.  No motion.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation

REPO = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_TEMPLATE = pathlib.Path.home() / (
    "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL")
# CAD origin -> geometric centre, same constant the environment model uses.
CAD_CENTRE_OFFSET = np.array([-0.208, 0.066, -0.011])
DEFAULT_PRIOR_XYZ = np.array([0.5029, 0.6351, 0.1254])
DEFAULT_PRIOR_QUAT = np.array([0.1910, 0.9816, 0.0019, -0.0068])


def latest_scene(out_dir: pathlib.Path) -> pathlib.Path:
    pointer = json.loads((out_dir / "latest.json").read_text(encoding="utf-8"))
    return out_dir / str(pointer["latest"]) / "scene.ply"


def read_ply(path: pathlib.Path) -> np.ndarray:
    data = pathlib.Path(path).read_bytes()
    header, body = data.split(b"end_header\n", 1)
    header_text = header.decode("ascii", errors="replace")
    if "binary_little_endian" not in header_text:
        raise SystemExit(f"unsupported PLY format in {path}")
    count = int([line for line in header_text.splitlines()
                 if line.startswith("element vertex")][0].split()[-1])
    names = [line.split()[-1] for line in header_text.splitlines()
             if line.startswith("property")]
    kinds = [line.split()[1] for line in header_text.splitlines()
             if line.startswith("property")]
    dtype = np.dtype([(name, "<f8" if kind == "double" else "<f4")
                      for name, kind in zip(names, kinds)])
    array = np.frombuffer(body, dtype=dtype, count=count)
    return np.stack([array["x"], array["y"], array["z"]], axis=1)


def crop_around(points: np.ndarray, centre, radius: float,
                z_min: float, z_max: float) -> np.ndarray:
    centre = np.asarray(centre, dtype=float)
    return points[(np.linalg.norm(points[:, :2] - centre[:2], axis=1) < radius)
                  & (points[:, 2] > z_min) & (points[:, 2] < z_max)]


def rotation_difference_deg(rotation: np.ndarray, prior: np.ndarray) -> float:
    delta = Rotation.from_matrix(rotation) * Rotation.from_quat(prior).inv()
    return float(np.degrees(np.linalg.norm(delta.as_rotvec())))


def centre_from_origin(origin, rotation, cad_offset=None) -> np.ndarray:
    """Geometric centre from the CAD-origin pose (model convention)."""
    offset = CAD_CENTRE_OFFSET if cad_offset is None else np.asarray(cad_offset, float)
    return np.asarray(origin, float) + np.asarray(rotation, float) @ offset


def quality_ok(fitness: float, rmse_m: float, rotation_deg: float,
               min_fitness: float, max_rmse_m: float, max_rotation_deg: float) -> bool:
    return (fitness >= min_fitness and rmse_m <= max_rmse_m
            and rotation_deg <= max_rotation_deg)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--environment-dir", default=str(REPO / "runtime/environment"))
    parser.add_argument("--scene", default=None, help="default: the latest scene.ply")
    parser.add_argument("--template", default=str(DEFAULT_TEMPLATE))
    parser.add_argument("--prior-xyz", type=float, nargs=3, default=None)
    parser.add_argument("--prior-quat", type=float, nargs=4, default=None)
    parser.add_argument("--radius", type=float, default=0.42,
                        help="crop radius around the prior centre in the xy plane")
    parser.add_argument("--voxel", type=float, default=0.004)
    parser.add_argument("--json-out", default="")
    parser.add_argument("--max-rotation-deg", type=float, default=15.0)
    parser.add_argument("--min-fitness", type=float, default=0.30)
    parser.add_argument("--max-rmse-mm", type=float, default=10.0)
    args = parser.parse_args()

    import open3d as o3d

    prior_xyz = np.array(args.prior_xyz if args.prior_xyz else DEFAULT_PRIOR_XYZ)
    prior_quat = np.array(args.prior_quat if args.prior_quat else DEFAULT_PRIOR_QUAT)
    rotation_prior = Rotation.from_quat(prior_quat).as_matrix()
    prior_centre = centre_from_origin(prior_xyz, rotation_prior)

    scene = pathlib.Path(args.scene) if args.scene else latest_scene(
        pathlib.Path(args.environment_dir))
    points = read_ply(scene)
    table_z = prior_centre[2] - 0.23  # roughly the measured table top
    cropped = crop_around(points, prior_centre, args.radius, table_z, prior_centre[2] + 0.35)
    print(f"scene {scene.name}: {len(points)} points, {len(cropped)} after crop "
          f"(centre {np.round(prior_centre, 3).tolist()})")
    if len(cropped) < 5000:
        print("too few points to localize")
        return 2

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cropped)).voxel_down_sample(
        args.voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

    mesh = o3d.io.read_triangle_mesh(args.template)
    if mesh.is_empty():
        print(f"template not readable: {args.template}")
        return 2
    mesh.compute_vertex_normals()
    template = mesh.sample_points_uniformly(number_of_points=20000).voxel_down_sample(
        args.voxel)
    # The STL is expressed in the same CAD frame as workpiece.xyz/quat, so the
    # transform maps CAD frame -> base frame directly.  (Centring the template on
    # its point centroid and then treating that centroid as the geometric centre
    # was wrong: the two differ by tens of millimetres and drove ICP into a local
    # minimum 50 mm away.)
    template.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

    feature_radius = 0.045
    features_template = o3d.pipelines.registration.compute_fpfh_feature(
        template, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100))
    features_cloud = o3d.pipelines.registration.compute_fpfh_feature(
        cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100))
    def refine(initial):
        transform = np.array(initial, copy=True)
        for threshold in (0.05, 0.02, 0.008):
            transform = np.array(o3d.pipelines.registration.registration_icp(
                template, cloud, threshold, transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            ).transformation, copy=True)
        result = o3d.pipelines.registration.registration_icp(
            template, cloud, 0.008, transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return np.array(result.transformation, copy=True), float(result.fitness), float(
            result.inlier_rmse)

    # 1) The part is expected near its previous placement: ICP from the prior pose
    #    is deterministic and cannot land on a quarter-turn branch.
    prior_transform = np.eye(4)
    prior_transform[:3, :3] = rotation_prior
    prior_transform[:3, 3] = prior_xyz
    transform, fitness, rmse_m = refine(prior_transform)
    rotation = transform[:3, :3].copy()
    rotation_deg = rotation_difference_deg(rotation, prior_quat)
    branch = "prior-icp"

    # 2) Only when that fails (the part really moved) use global registration, and
    #    keep only candidates whose rotation matches the placement prior.
    if not quality_ok(fitness, rmse_m, rotation_deg, args.min_fitness,
                      args.max_rmse_mm / 1000.0, args.max_rotation_deg):
        best = None
        for _ in range(3):
            ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                template, cloud, features_template, features_cloud, True, 0.0225,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.0225)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))
            candidate, candidate_fitness, candidate_rmse = refine(ransac.transformation)
            candidate_deg = rotation_difference_deg(candidate[:3, :3], prior_quat)
            print(f"  global candidate: fitness {candidate_fitness:.3f} rmse "
                  f"{candidate_rmse * 1000:.1f} mm, rotation vs prior {candidate_deg:.1f} deg")
            if candidate_deg > args.max_rotation_deg:
                continue
            if best is None or candidate_fitness > best[1]:
                best = (candidate, candidate_fitness, candidate_rmse, candidate_deg)
        if best is None:
            print("global registration found no candidate consistent with the "
                  "upright placement prior")
        else:
            transform, fitness, rmse_m, rotation_deg = best
            rotation = transform[:3, :3].copy()
            branch = "global"

    origin_xyz = transform[:3, 3].copy()
    measured_centre = centre_from_origin(origin_xyz, rotation)
    ok = quality_ok(fitness, rmse_m, rotation_deg, args.min_fitness,
                    args.max_rmse_mm / 1000.0, args.max_rotation_deg)

    print(f"measured centre       {np.round(measured_centre, 4).tolist()}")
    print(f"measured CAD origin   {np.round(origin_xyz, 4).tolist()}")
    print(f"fitness {fitness:.3f}  rmse {rmse_m * 1000:.1f} mm  "
          f"rotation vs prior {rotation_deg:.1f} deg  branch {branch}")
    print(f"prior centre          {np.round(prior_centre, 4).tolist()}")
    print(f"centre shift vs prior {1000 * np.linalg.norm(measured_centre - prior_centre):.1f} mm")
    print("LOCALIZE: " + ("PASS" if ok else "FAIL"))
    if not ok:
        print("  the measurement did not meet the quality gate; do not use it to "
              "change the grasp poses (add views / check the template / lighting)")

    if args.json_out:
        pathlib.Path(args.json_out).write_text(json.dumps({
            "workpiece": {"xyz": [float(v) for v in origin_xyz],
                          "quat": [float(v) for v in prior_quat]},
            "centre": [float(v) for v in measured_centre],
            "fitness": fitness, "rmse_mm": rmse_m * 1000.0,
            "rotation_vs_prior_deg": rotation_deg,
            "ok": ok, "scene": str(scene), "template": args.template,
        }, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        print(f"wrote {args.json_out}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
