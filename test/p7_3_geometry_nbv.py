#!/usr/bin/env python3
"""Rank P7.3 NBV candidates from CAD geometry and P7.2 uncertainty.

This is deliberately an offline gate tool.  It consumes only a P7.2 pose
estimate, its covariance, and a frozen RGB-D observation.  It does *not* read
Gazebo ground truth, call MoveIt, or send a trajectory.  ``coverage_proxy`` is
not an input to the algorithm.

The score has two separately reported physical terms:

* geometry coverage: CAD triangle area that was not confirmed visible in the
  current depth image and is front-facing/in-frame from a prospective view;
* pose information: expected Fisher information of textured CAD samples,
  propagated into the current yaw covariance.

Thus a severe P7.2 rejection can select a re-observation, while P7.4 remains
the only gate allowed to say that the selected eye-in-hand pose is executable.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from p7_2_estimate_cylinder_pose import transform_matrix


def normalize(value: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if norm <= 1.0e-12:
        raise ValueError("zero-length vector")
    return value / norm


def quaternion_matrix(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(item) for item in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
         (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
         (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y))),
        dtype=float,
    )


def load_obj_samples(path: Path, texture_path: Path, stride: int) -> dict[str, np.ndarray]:
    """Load area-weighted triangle samples and their real texture gradients."""
    vertices: list[list[float]] = []
    texcoords: list[list[float]] = []
    triangles: list[tuple[tuple[int, int], tuple[int, int], tuple[int, int]]] = []
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        fields = raw.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append([float(value) for value in fields[1:4]])
        elif fields[0] == "vt":
            texcoords.append([float(value) for value in fields[1:3]])
        elif fields[0] == "f":
            face = []
            for token in fields[1:]:
                pair = token.split("/")
                if len(pair) >= 2 and pair[0] and pair[1]:
                    face.append((int(pair[0]) - 1, int(pair[1]) - 1))
            for offset in range(1, len(face) - 1):
                triangles.append((face[0], face[offset], face[offset + 1]))
    if not triangles:
        raise ValueError(f"no textured triangles in {path}")
    xyz = np.asarray(vertices, dtype=float)
    uv = np.asarray(texcoords, dtype=float)
    selected = triangles[::max(1, stride)]
    tri_xyz = np.asarray([[xyz[index] for index, _ in triangle] for triangle in selected])
    tri_uv = np.asarray([[uv[index] for _, index in triangle] for triangle in selected])
    cross = np.cross(tri_xyz[:, 1] - tri_xyz[:, 0], tri_xyz[:, 2] - tri_xyz[:, 0])
    double_area = np.linalg.norm(cross, axis=1)
    keep = double_area > 1.0e-12
    tri_xyz, tri_uv, cross, double_area = (
        tri_xyz[keep], tri_uv[keep], cross[keep], double_area[keep]
    )
    normals = cross / double_area[:, None]
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise ValueError(f"cannot read texture {texture_path}")
    gradient_x = cv2.Sobel(texture, cv2.CV_64F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(texture, cv2.CV_64F, 0, 1, ksize=3)
    uv_center = tri_uv.mean(axis=1)
    cols = np.clip(np.rint(uv_center[:, 0] * (texture.shape[1] - 1)).astype(int), 0, texture.shape[1] - 1)
    rows = np.clip(np.rint((1.0 - uv_center[:, 1]) * (texture.shape[0] - 1)).astype(int), 0, texture.shape[0] - 1)
    texture_gradient = np.hypot(gradient_x[rows, cols], gradient_y[rows, cols])
    # The 10th percentile floor prevents uniformly coloured but geometrically
    # useful CAD patches from collapsing the numerical information matrix.
    texture_gradient = np.maximum(texture_gradient, np.percentile(texture_gradient, 10))
    return {
        "centroids": tri_xyz.mean(axis=1),
        "normals": normals,
        "areas": double_area * 0.5,
        "texture_gradient": texture_gradient / max(float(texture_gradient.max()), 1.0),
    }


def rotation_from_to(source: np.ndarray, destination: np.ndarray) -> np.ndarray:
    """Return the proper rotation mapping one non-zero unit vector to another."""
    source, destination = normalize(source), normalize(destination)
    cross = np.cross(source, destination)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.dot(source, destination))
    if sine <= 1.0e-12:
        if cosine > 0.0:
            return np.eye(3)
        axis = normalize(np.cross(source, np.array((1.0, 0.0, 0.0))))
        if not np.isfinite(axis).all():
            axis = np.array((0.0, 1.0, 0.0))
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    skew = np.array(((0.0, -cross[2], cross[1]), (cross[2], 0.0, -cross[0]), (-cross[1], cross[0], 0.0)))
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / (sine * sine))


def camera_rotation(
    position: np.ndarray,
    target: np.ndarray,
    *,
    k: np.ndarray | None = None,
    target_pixel_offset_px: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    """World-from-camera rotation with an optional off-axis target projection.

    The fixed eye-in-hand gripper obscures the lower central image region in
    the real Gazebo observation.  This function permits a candidate to place
    the target at a measured safe image offset while preserving the same
    camera position and optical range.
    """
    forward = normalize(target - position)
    up = np.array((0.0, 0.0, 1.0), dtype=float)
    if abs(float(np.dot(forward, up))) > 0.98:
        up = np.array((0.0, 1.0, 0.0), dtype=float)
    right = normalize(np.cross(forward, up))
    down = np.cross(forward, right)
    direct = np.column_stack((right, down, forward))
    if k is None or target_pixel_offset_px == (0.0, 0.0):
        return direct
    target_ray = normalize(
        np.array(
            (target_pixel_offset_px[0] / k[0, 0], target_pixel_offset_px[1] / k[1, 1], 1.0),
            dtype=float,
        )
    )
    # Q maps the desired off-axis camera ray to the direct optical +Z ray;
    # therefore direct @ Q projects target at exactly the requested pixels.
    return direct @ rotation_from_to(target_ray, np.array((0.0, 0.0, 1.0)))


def project(points_world: np.ndarray, world_from_camera: np.ndarray, k: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    camera_from_world = np.linalg.inv(world_from_camera)
    camera = points_world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    uv = np.full((len(points_world), 2), np.nan, dtype=float)
    valid = camera[:, 2] > 1.0e-9
    uv[valid, 0] = k[0, 0] * camera[valid, 0] / camera[valid, 2] + k[0, 2]
    uv[valid, 1] = k[1, 1] * camera[valid, 1] / camera[valid, 2] + k[1, 2]
    return camera, uv


def current_observation_mask(
    samples: dict[str, np.ndarray], pose: dict, metadata: dict, depth: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Determine which CAD surfaces are actually supported by frozen depth."""
    local_rotation = quaternion_matrix(pose["orientation_xyzw"])
    local_center = samples["centroids"].mean(axis=0)
    world_points = (samples["centroids"] - local_center) @ local_rotation.T + np.asarray(pose["position_m"], dtype=float)
    world_normals = samples["normals"] @ local_rotation.T
    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    k = np.asarray(metadata["streams"]["camera_info"]["k"], dtype=float).reshape(3, 3)
    camera_points, uv = project(world_points, world_from_camera, k)
    camera_origin = world_from_camera[:3, 3]
    front_facing = np.einsum("ij,ij->i", world_normals, camera_origin - world_points) > 0.0
    pixels = np.rint(uv).astype(int)
    inside = (
        np.isfinite(uv).all(axis=1) & (pixels[:, 0] >= 0) & (pixels[:, 0] < depth.shape[1])
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < depth.shape[0])
    )
    observed = np.zeros(len(world_points), dtype=bool)
    indices = np.nonzero(inside & front_facing)[0]
    measured = depth[pixels[indices, 1], pixels[indices, 0]]
    # A CAD point is confirmed only when the frozen range image has matching
    # depth.  A nearer occluder therefore never becomes target coverage.
    observed[indices] = np.isfinite(measured) & (np.abs(measured - camera_points[indices, 2]) <= 0.012)
    return observed, local_center


def obstacle_cloud_from_frozen_points(
    points_camera: np.ndarray, metadata: dict, target_position: np.ndarray
) -> np.ndarray:
    """Extract a compact, measured non-target obstacle cloud near the object.

    These are not scene/SDF oracle boxes: each retained point was measured in
    the frozen P7.1 PointCloud and transformed by its timestamped TF.  A
    15-mm voxel representative is adequate for prospective occlusion tests.
    """
    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    points = np.asarray(points_camera, dtype=float).reshape(-1, 3)
    finite = np.all(np.isfinite(points), axis=1)
    world = points[finite] @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    horizontal_distance = np.linalg.norm(world[:, :2] - target_position[:2], axis=1)
    object_distance = np.linalg.norm(world - target_position, axis=1)
    # Ground-plane samples and the estimated object volume cannot become
    # external occluders.  The retained band contains the observed blocker.
    keep = (
        (world[:, 2] >= 0.015)
        & (world[:, 2] <= 0.25)
        & (horizontal_distance <= 0.20)
        & (object_distance >= 0.060)
    )
    world = world[keep]
    if len(world) == 0:
        return np.empty((0, 3), dtype=float)
    voxels = np.floor(world / 0.015).astype(np.int64)
    _, representatives = np.unique(voxels, axis=0, return_index=True)
    return world[np.sort(representatives)]


def externally_occluded(
    camera_position: np.ndarray, points_world: np.ndarray, obstacle_points: np.ndarray
) -> np.ndarray:
    """Ray-test CAD samples against measured external obstacle points."""
    if len(obstacle_points) == 0:
        return np.zeros(len(points_world), dtype=bool)
    rays = points_world - camera_position
    ranges = np.linalg.norm(rays, axis=1)
    directions = rays / np.maximum(ranges[:, None], 1.0e-12)
    relative = obstacle_points[None, :, :] - camera_position[None, None, :]
    along = np.einsum("nmk,nk->nm", relative, directions)
    perpendicular = np.linalg.norm(relative - along[:, :, None] * directions[:, None, :], axis=2)
    return np.any(
        (along > 0.02)
        & (along < ranges[:, None] - 0.010)
        & (perpendicular <= 0.018),
        axis=1,
    )


def candidate_metrics(
    samples: dict[str, np.ndarray], pose: dict, covariance: np.ndarray, observed: np.ndarray,
    local_center: np.ndarray, position: np.ndarray, k: np.ndarray, width: int, height: int,
    texture_samples_per_m2: float, target_pixel_offset_px: tuple[float, float],
    obstacle_points: np.ndarray | None = None,
) -> dict:
    local_rotation = quaternion_matrix(pose["orientation_xyzw"])
    target = np.asarray(pose["position_m"], dtype=float)
    points = (samples["centroids"] - local_center) @ local_rotation.T + target
    normals = samples["normals"] @ local_rotation.T
    world_from_camera = np.eye(4, dtype=float)
    world_from_camera[:3, :3] = camera_rotation(
        position, target, k=k, target_pixel_offset_px=target_pixel_offset_px
    )
    world_from_camera[:3, 3] = position
    camera, uv = project(points, world_from_camera, k)
    facing = np.einsum("ij,ij->i", normals, position - points) > 0.0
    inside = (
        np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0.0) & (uv[:, 0] < width)
        & (uv[:, 1] >= 0.0) & (uv[:, 1] < height) & (camera[:, 2] > 0.30) & (camera[:, 2] < 1.00)
    )
    blocked = externally_occluded(
        position, points, obstacle_points if obstacle_points is not None else np.empty((0, 3))
    )
    visible = facing & inside & ~blocked
    areas = samples["areas"]
    total_area = float(areas.sum())
    new_area = float(areas[visible & ~observed].sum())
    coverage = new_area / total_area

    # Pinhole yaw Jacobian for each textured CAD sample.  This is the actual
    # sensitivity d(u,v)/d(yaw) at the candidate pose, not an elevation proxy.
    local = samples["centroids"] - local_center
    derivative_world = np.column_stack((-local[:, 1], local[:, 0], np.zeros(len(local)))) @ local_rotation.T
    rotation_cw = world_from_camera[:3, :3].T
    derivative_camera = derivative_world @ rotation_cw.T
    z = camera[:, 2]
    du = k[0, 0] * (derivative_camera[:, 0] * z - camera[:, 0] * derivative_camera[:, 2]) / np.maximum(z*z, 1.0e-12)
    dv = k[1, 1] * (derivative_camera[:, 1] * z - camera[:, 1] * derivative_camera[:, 2]) / np.maximum(z*z, 1.0e-12)
    pixel_variance = 2.0**2
    # Surface area alone has m² units and cannot directly be added as a count
    # of independent image residuals.  The versioned sampling density converts
    # visible textured CAD area into a conservative count of independent
    # texture residuals (not an arbitrary score normalizer).
    weights = (
        samples["areas"] * float(texture_samples_per_m2) * samples["texture_gradient"]
    )
    yaw_information = float(np.sum(weights[visible] * (du[visible]**2 + dv[visible]**2) / pixel_variance))
    prior_yaw_variance = float(covariance[5, 5])
    posterior_yaw_variance = 1.0 / (1.0 / prior_yaw_variance + yaw_information)
    information_gain_nats = 0.5 * math.log(prior_yaw_variance / posterior_yaw_variance)
    return {
        "camera_position_world_m": position.tolist(),
        "candidate_optical_range_m": float(np.linalg.norm(target - position)),
        "target_pixel_offset_px": list(target_pixel_offset_px),
        "visible_surface_fraction": float(areas[visible].sum() / total_area),
        "external_occluded_surface_fraction": float(areas[blocked].sum() / total_area),
        "new_geometry_coverage_fraction": coverage,
        "new_geometry_area_m2": new_area,
        "yaw_fisher_information_rad_minus2": yaw_information,
        "prior_yaw_variance_rad2": prior_yaw_variance,
        "expected_posterior_yaw_variance_rad2": posterior_yaw_variance,
        "expected_yaw_information_gain_nats": information_gain_nats,
        "expected_yaw_standard_deviation_deg": float(math.degrees(math.sqrt(posterior_yaw_variance))),
        "candidate_pose_world_from_camera": {
            "rotation_matrix": world_from_camera[:3, :3].tolist(), "translation_m": position.tolist()
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--window", type=Path, required=True, help="Frozen P7.1/P7.2 raw window")
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--texture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--candidate-radius-m", type=float, default=0.70)
    parser.add_argument(
        "--candidate-elevation-deg", type=float, action="append", default=None,
        help="Repeat for multiple candidate elevation bands; default is 42, 56, 70 degrees.",
    )
    parser.add_argument("--candidate-count", type=int, default=12)
    parser.add_argument(
        "--target-v-offset-px", type=float, action="append", default=None,
        help="Candidate target vertical offsets; default -110 and 0 px. Negative places the target above the image centre.",
    )
    parser.add_argument(
        "--maximum-target-v-px", type=float, default=None,
        help="Reject target projections below this image row; use the calibrated gripper self-occlusion boundary.",
    )
    parser.add_argument("--mesh-stride", type=int, default=16)
    parser.add_argument(
        "--texture-samples-per-m2", type=float, default=15000.0,
        help="Independent textured residual density for the 640x480 frozen-camera model.",
    )
    parser.add_argument("--minimum-information-gain-nats", type=float, default=0.20)
    parser.add_argument("--stop-yaw-standard-deviation-deg", type=float, default=5.0)
    args = parser.parse_args()
    if not 0.30 <= args.candidate_radius_m <= 1.00:
        raise ValueError("candidate radius must remain in the PS800-E1 0.30--1.00 m range")
    if args.candidate_count < 4:
        raise ValueError("at least four geometric candidates are required")
    if args.texture_samples_per_m2 <= 0.0:
        raise ValueError("texture sample density must be positive")

    estimate = json.loads(args.estimate.read_text(encoding="utf-8"))
    metadata = json.loads((args.window / "metadata.json").read_text(encoding="utf-8"))
    depth = np.load(args.window / "depth_raw.npy")
    frozen_points = np.load(args.window / "points_xyz_m.npy")
    covariance = np.asarray(estimate["covariance_6x6"], dtype=float)
    samples = load_obj_samples(args.mesh, args.texture, args.mesh_stride)
    observed, local_center = current_observation_mask(samples, estimate["pose"], metadata, depth)
    k = np.asarray(metadata["streams"]["camera_info"]["k"], dtype=float).reshape(3, 3)
    target = np.asarray(estimate["pose"]["position_m"], dtype=float)
    obstacle_points = obstacle_cloud_from_frozen_points(frozen_points, metadata, target)
    elevations_deg = args.candidate_elevation_deg or [42.0, 56.0, 70.0]
    target_v_offsets = args.target_v_offset_px or [-110.0, 0.0]
    if any(not 5.0 <= elevation <= 85.0 for elevation in elevations_deg):
        raise ValueError("candidate elevations must be within 5--85 degrees")
    results = []
    for elevation_deg in elevations_deg:
        elevation = math.radians(elevation_deg)
        for target_v_offset in target_v_offsets:
            for index in range(args.candidate_count):
                azimuth = index * 2.0 * math.pi / args.candidate_count
                direction = np.array(
                    (
                        math.cos(elevation) * math.cos(azimuth),
                        math.cos(elevation) * math.sin(azimuth),
                        math.sin(elevation),
                    )
                )
                record = candidate_metrics(
                    samples,
                    estimate["pose"],
                    covariance,
                    observed,
                    local_center,
                    target + args.candidate_radius_m * direction,
                    k,
                    int(metadata["streams"]["camera_info"]["width"]),
                    int(metadata["streams"]["camera_info"]["height"]),
                    args.texture_samples_per_m2, (0.0, float(target_v_offset)),
                    obstacle_points,
                )
                record["candidate_id"] = f"geometry_nbv_e{int(elevation_deg):02d}_v{int(target_v_offset):+04d}_a{index:02d}"
                record["azimuth_deg"] = float(math.degrees(azimuth))
                record["elevation_deg"] = float(elevation_deg)
                record["target_projection_v_px"] = float(k[1, 2] + target_v_offset)
                # The weighted sum is only a deterministic tie breaker between the two
                # exported physical terms; both terms are reported unmodified.
                record["selection_score"] = record["new_geometry_coverage_fraction"] + record["expected_yaw_information_gain_nats"]
                record["self_occlusion_safe"] = (
                    args.maximum_target_v_px is None
                    or record["target_projection_v_px"] <= args.maximum_target_v_px
                )
                results.append(record)
    self_occlusion_safe = [item for item in results if item["self_occlusion_safe"]]
    if not self_occlusion_safe:
        raise RuntimeError("no candidate remains outside the calibrated gripper self-occlusion region")
    selected = max(self_occlusion_safe, key=lambda item: (item["selection_score"], item["candidate_id"]))
    current_yaw_standard_deviation_deg = float(math.degrees(math.sqrt(covariance[5, 5])))
    # Stop decisions use the *measured current* P7.2 covariance.  The
    # prospective Fisher projection is informative for ranking only and can
    # never manufacture a stop before a re-observation has been estimated.
    current_stop = current_yaw_standard_deviation_deg <= args.stop_yaw_standard_deviation_deg
    result = {
        "schema": "p7_3_geometry_information_nbv_v1",
        "gate": "P7.3_NBV",
        "ground_truth_read": False,
        "legacy_coverage_proxy_used": False,
        "input": {"estimate": str(args.estimate), "window": str(args.window), "mesh": str(args.mesh)},
        "current_observed_cad_surface_fraction": float(samples["areas"][observed].sum() / samples["areas"].sum()),
        "measured_external_obstacle_point_count": int(len(obstacle_points)),
        "current_yaw_standard_deviation_deg": current_yaw_standard_deviation_deg,
        "candidate_count": len(results),
        "self_occlusion_constraint": {
            "maximum_target_v_px": args.maximum_target_v_px,
            "safe_candidate_count": len(self_occlusion_safe),
            "definition": "image-plane protected region calibrated from the frozen eye-in-hand gripper self-occlusion diagnostic",
        },
        "information_model": {
            "pixel_measurement_standard_deviation_px": 2.0,
            "texture_samples_per_m2": args.texture_samples_per_m2,
            "definition": "CAD surface area times texture-gradient weight times independent residual density; propagated by the pinhole yaw Jacobian.",
        },
        "candidates": results,
        "selected_candidate": selected,
        "reobservation_required": not current_stop,
        "stop_criterion": {
            "criterion": "measured P7.2 yaw standard deviation <= configured threshold; prospective Fisher projections rank candidates only and cannot terminate the gate",
            "configured_yaw_standard_deviation_deg": args.stop_yaw_standard_deviation_deg,
            "measured_current_covariance_met": current_stop,
            "selected_candidate_expected_yaw_standard_deviation_deg": selected["expected_yaw_standard_deviation_deg"],
        },
        "gate_decision": "P7.3_STOP_CRITERION_MET" if current_stop else "P7.4_REQUIRED_FOR_SELECTED_VIEW_EXECUTION",
        "claim_boundary": "CAD/depth geometry and expected information only; no IK, collision, planning, trajectory, gripper, or grasp claim.",
    }
    if not current_stop and selected["expected_yaw_information_gain_nats"] < args.minimum_information_gain_nats:
        result["gate_decision"] = "NO_INFORMATIVE_CANDIDATE"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"gate_decision": result["gate_decision"], "selected_candidate": selected["candidate_id"], "information_gain_nats": selected["expected_yaw_information_gain_nats"], "coverage": selected["new_geometry_coverage_fraction"]}, sort_keys=True))


if __name__ == "__main__":
    main()
