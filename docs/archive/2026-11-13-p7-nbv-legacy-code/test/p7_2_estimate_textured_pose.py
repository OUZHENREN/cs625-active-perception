#!/usr/bin/env python3
"""Estimate textured YCB 6D pose with OBJ UV correspondences and SIFT PnP."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from p7_2_estimate_cylinder_pose import read_ppm, transform_matrix


LOCAL_CENTER = np.array((-0.0091685, 0.0840170, 0.0510065), dtype=float)


def load_obj_uv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[list[float]] = []
    texcoords: list[list[float]] = []
    face_indices: list[tuple[list[int], list[int]]] = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append([float(value) for value in fields[1:4]])
        elif fields[0] == "vt":
            texcoords.append([float(value) for value in fields[1:3]])
        elif fields[0] == "f":
            vertex_indices = []
            texture_indices = []
            for token in fields[1:]:
                indices = token.split("/")
                if len(indices) >= 2 and indices[1]:
                    vertex_indices.append(int(indices[0]) - 1)
                    texture_indices.append(int(indices[1]) - 1)
            if len(vertex_indices) >= 3:
                face_indices.append((vertex_indices, texture_indices))
    vertex_array = np.asarray(vertices, dtype=np.float64)
    texture_array = np.asarray(texcoords, dtype=np.float64)
    uv_triangles = []
    xyz_triangles = []
    for vertex_indices, texture_indices in face_indices:
        for offset in range(1, len(vertex_indices) - 1):
            selection = [0, offset, offset + 1]
            uv_triangles.append(texture_array[[texture_indices[i] for i in selection]])
            xyz_triangles.append(vertex_array[[vertex_indices[i] for i in selection]])
    return np.asarray(uv_triangles), np.asarray(xyz_triangles)


def texture_point_to_model(
    point_px: tuple[float, float],
    uv_triangles_px: np.ndarray,
    xyz_triangles: np.ndarray,
) -> tuple[np.ndarray | None, float | None]:
    point = np.asarray(point_px, dtype=float)
    minimum = uv_triangles_px.min(axis=1)
    maximum = uv_triangles_px.max(axis=1)
    candidates = np.nonzero(
        np.all(point >= minimum - 0.5, axis=1)
        & np.all(point <= maximum + 0.5, axis=1)
    )[0]
    best = None
    best_margin = -np.inf
    for index in candidates:
        triangle = uv_triangles_px[index]
        a, b, c = triangle
        matrix = np.column_stack((b - a, c - a))
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) < 1.0e-9:
            continue
        uv = np.linalg.solve(matrix, point - a)
        weights = np.array((1.0 - uv.sum(), uv[0], uv[1]))
        margin = float(weights.min())
        if margin >= -0.03 and margin > best_margin:
            best = weights @ xyz_triangles[index]
            best_margin = margin
    return best, best_margin if best is not None else None


def target_search_roi(
    depth: np.ndarray, rgb: np.ndarray, centre_uv: tuple[float, float] | None = None
) -> tuple[int, int, int, int]:
    height, width = depth.shape
    cx, cy = centre_uv if centre_uv is not None else (width // 2, height // 2)
    radius = max(48, min(width, height) // 5)
    x0, x1 = max(0, cx - radius), min(width, cx + radius + 1)
    y0, y1 = max(0, cy - radius), min(height, cy + radius + 1)
    patch_depth = depth[y0:y1, x0:x1]
    patch_rgb = rgb[y0:y1, x0:x1].astype(float)
    chroma = patch_rgb.max(axis=2) - patch_rgb.min(axis=2)
    valid = (
        np.isfinite(patch_depth)
        & (patch_depth >= 0.30)
        & (patch_depth <= 1.00)
        & (chroma >= 12.0)
    )
    if valid.sum() < 8:
        return x0, y0, x1, y1
    bins = np.arange(0.30, 1.001, 0.010)
    histogram, edges = np.histogram(
        patch_depth[valid], bins=bins, weights=chroma[valid]
    )
    peak = int(np.argmax(histogram))
    depth_mode = 0.5 * (edges[peak] + edges[peak + 1])
    mask = valid & (np.abs(patch_depth - depth_mode) <= 0.045)
    rows, cols = np.nonzero(mask)
    if len(rows) < 8:
        return x0, y0, x1, y1
    margin = 18
    return (
        max(0, x0 + int(cols.min()) - margin),
        max(0, y0 + int(rows.min()) - margin),
        min(width, x0 + int(cols.max()) + margin + 1),
        min(height, y0 + int(rows.max()) + margin + 1),
    )


def rgbd_center_world(
    depth: np.ndarray,
    rgb: np.ndarray,
    points: np.ndarray,
    metadata: dict,
    model_min_z_m: float,
    model_max_z_m: float,
    support_plane_z_m: float = 0.0,
) -> tuple[
    np.ndarray,
    np.ndarray,
    int,
    float,
    tuple[float, float],
    str,
    float | None,
    list[list[float]],
    float,
]:
    """Estimate a cylinder centre from colour-seeded depth without GT."""
    height, width = depth.shape
    yy, xx = np.indices(depth.shape)
    cx = float(metadata["streams"]["camera_info"]["k"][2])
    cy = float(metadata["streams"]["camera_info"]["k"][5])
    crop_radius = max(36, min(width, height) // 8)
    rgb_float = rgb.astype(float)
    chroma = rgb_float.max(axis=2) - rgb_float.min(axis=2)
    intensity = rgb_float.mean(axis=2)
    working = np.isfinite(depth) & (depth >= 0.30) & (depth <= 1.00)
    central_crop = (xx - cx) ** 2 + (yy - cy) ** 2 <= crop_radius**2
    colour_depth_seed = working & (chroma >= 12.0) & (intensity <= 190.0)
    seed = central_crop & colour_depth_seed
    search_mode = "central_rgbd_seed"
    if seed.sum() < 8:
        # NBV is allowed to frame a target away from the image centre to avoid
        # eye-in-hand self-occlusion.  The estimator must therefore fall back
        # to a full-frame RGB-D search rather than silently treating central
        # framing as an algorithmic precondition.
        seed = colour_depth_seed
        search_mode = "full_frame_rgbd_seed_fallback"
    if seed.sum() < 8:
        raise RuntimeError("RGBD_COLOUR_SEED_TOO_SPARSE")
    bins = np.arange(0.30, 1.001, 0.010)
    histogram, edges = np.histogram(depth[seed], bins=bins, weights=chroma[seed])
    peak = int(np.argmax(histogram))
    depth_mode = float(0.5 * (edges[peak] + edges[peak + 1]))
    selected = working & (np.abs(depth - depth_mode) <= 0.040)
    component_count, labels, statistics, centroids = cv2.connectedComponentsWithStats(
        selected.astype(np.uint8), connectivity=8
    )
    candidates = []
    for label in range(1, component_count):
        area = int(statistics[label, cv2.CC_STAT_AREA])
        if area < 30:
            continue
        chroma_sum = float(chroma[labels == label].sum())
        distance = float(np.linalg.norm(centroids[label] - np.array((cx, cy))))
        if search_mode == "central_rgbd_seed":
            candidates.append((0.0, distance, -area, label))
        else:
            # In fallback mode, textured RGB-D support is primary; distance
            # is retained only as a deterministic secondary tie breaker.
            candidates.append((-chroma_sum, distance, -area, label))
    if not candidates:
        raise RuntimeError("RGBD_TARGET_COMPONENT_UNAVAILABLE")
    target_label = min(candidates)[3]
    selected = labels == target_label
    cloud = points[selected]
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    if len(cloud) < 30:
        raise RuntimeError("RGBD_SEGMENTATION_TOO_SPARSE")
    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    contours, _ = cv2.findContours(
        selected.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    contour = max(contours, key=cv2.contourArea)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    component_centre = centroids[target_label]
    crop_x0, crop_x1 = (
        max(0, int(component_centre[0] - crop_radius)),
        min(width, int(component_centre[0] + crop_radius)),
    )
    crop_y0, crop_y1 = (
        max(0, int(component_centre[1] - crop_radius)),
        min(height, int(component_centre[1] + crop_radius)),
    )
    circle_roi = cv2.GaussianBlur(gray[crop_y0:crop_y1, crop_x0:crop_x1], (5, 5), 1.0)
    expected_radius_px = float(
        metadata["streams"]["camera_info"]["k"][0] * 0.034 / depth_mode
    )
    component_coverage = float(
        len(cloud) / max(1.0, np.pi * expected_radius_px**2)
    )
    if len(contour) >= 5:
        (ellipse_u, ellipse_v), _, _ = cv2.fitEllipse(contour)
        ellipse_radius = None
    else:
        (ellipse_u, ellipse_v), ellipse_radius = cv2.minEnclosingCircle(contour)
    requires_hough = bool(
        component_coverage < 0.45
        or component_coverage > 0.90
        or ellipse_v > cy
    )
    circles = cv2.HoughCircles(
        circle_roi,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=max(12.0, expected_radius_px * 0.7),
        param1=60.0,
        param2=8.0 if (component_coverage > 0.90 or ellipse_v > cy) else 14.0,
        minRadius=max(8, int(expected_radius_px * 0.60)),
        maxRadius=max(12, int(expected_radius_px * 1.35)),
    )
    center_method = "depth_component_ellipse"
    detected_radius_px = None
    circle_candidate_records: list[list[float]] = []
    if (
        requires_hough
        and circles is not None
        and circles.shape[1] > 0
    ):
        circle_candidates = circles[0].astype(float)
        circle_candidates[:, 0] += crop_x0
        circle_candidates[:, 1] += crop_y0
        circle_candidate_records = circle_candidates.tolist()
        target_circle = select_hough_candidate(
            circle_candidates,
            chroma=chroma,
            target_seed_mask=selected & colour_depth_seed,
            expected_radius_px=expected_radius_px,
            component_centre=component_centre,
        )
        target_u, target_v, detected_radius_px = target_circle
        center_method = "occlusion_robust_hough_circle"
    else:
        target_u, target_v = ellipse_u, ellipse_v
        detected_radius_px = ellipse_radius
    ray_camera = np.array(
        ((target_u - cx) / metadata["streams"]["camera_info"]["k"][0],
         (target_v - cy) / metadata["streams"]["camera_info"]["k"][4],
         1.0),
        dtype=float,
    )
    ray_world = world_from_camera[:3, :3] @ ray_camera
    camera_world = world_from_camera[:3, 3]
    centre_plane_z = support_plane_z_m + LOCAL_CENTER[2] - model_min_z_m
    intersection_plane_z = (
        support_plane_z_m + model_max_z_m - model_min_z_m
        if center_method == "occlusion_robust_hough_circle"
        else centre_plane_z
    )
    ray_scale = (intersection_plane_z - camera_world[2]) / ray_world[2]
    centre_world = camera_world + ray_scale * ray_world
    centre_world[2] = centre_plane_z
    cloud_world = cloud @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    covariance_world = np.cov(cloud_world.T) / len(cloud_world)
    return (
        centre_world,
        covariance_world,
        int(len(cloud)),
        depth_mode,
        (float(target_u), float(target_v)),
        center_method,
        None if detected_radius_px is None else float(detected_radius_px),
        circle_candidate_records,
        component_coverage,
    )


def select_hough_candidate(
    circle_candidates: np.ndarray,
    *,
    chroma: np.ndarray,
    target_seed_mask: np.ndarray,
    expected_radius_px: float,
    component_centre: np.ndarray,
) -> np.ndarray:
    """Select the target circle from measured colour/depth support.

    Under a severe partial occlusion the target and blocker can form one
    connected depth component.  Its geometric centroid then lies inside the
    blocker, so ranking circles by centroid proximity selects the blocker.
    The colour/depth seed is independent of Gazebo ground truth and remains
    concentrated on the visible YCB texture; the supported chroma mass is
    therefore the primary discriminator.
    """
    if len(circle_candidates) == 0:
        raise ValueError("circle_candidates must not be empty")
    rows, cols = np.indices(target_seed_mask.shape)

    def ranking_key(circle: np.ndarray) -> tuple[float, float, float, float]:
        u, v, radius = (float(value) for value in circle)
        inside = (cols - u) ** 2 + (rows - v) ** 2 <= radius**2
        supported = target_seed_mask & inside
        chroma_mass = float(chroma[supported].sum())
        seed_count = int(supported.sum())
        radius_error = abs(radius - expected_radius_px)
        centre_distance = float(np.linalg.norm(np.array((u, v)) - component_centre))
        return (-chroma_mass, -seed_count, radius_error, centre_distance)

    return min(circle_candidates, key=ranking_key)


def canonical_top_template(
    texture: np.ndarray,
    uv_triangles: np.ndarray,
    xyz_triangles: np.ndarray,
    *,
    size: int = 256,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rasterize CAD top-face texture once in an orthographic XY canvas."""
    xy = xyz_triangles[:, :, :2]
    xy_min = xy.reshape(-1, 2).min(axis=0)
    xy_max = xy.reshape(-1, 2).max(axis=0)
    extent = np.maximum(xy_max - xy_min, 1.0e-9)
    destination = np.empty_like(xy)
    destination[:, :, 0] = (xy[:, :, 0] - xy_min[0]) / extent[0] * (size - 1)
    destination[:, :, 1] = (xy_max[1] - xy[:, :, 1]) / extent[1] * (size - 1)
    source = uv_triangles.copy()
    source[:, :, 0] *= texture.shape[1]
    source[:, :, 1] = (1.0 - source[:, :, 1]) * texture.shape[0]
    canvas = np.zeros((size, size), dtype=np.uint8)
    canvas_mask = np.zeros((size, size), dtype=np.uint8)
    for source_triangle, destination_triangle in zip(source, destination):
        sx0, sy0 = np.floor(source_triangle.min(axis=0)).astype(int)
        sx1, sy1 = np.ceil(source_triangle.max(axis=0)).astype(int) + 1
        dx0, dy0 = np.floor(destination_triangle.min(axis=0)).astype(int)
        dx1, dy1 = np.ceil(destination_triangle.max(axis=0)).astype(int) + 1
        sx0, sy0 = max(0, sx0), max(0, sy0)
        sx1, sy1 = min(texture.shape[1], sx1), min(texture.shape[0], sy1)
        dx0, dy0 = max(0, dx0), max(0, dy0)
        dx1, dy1 = min(size, dx1), min(size, dy1)
        if sx1 <= sx0 or sy1 <= sy0 or dx1 <= dx0 or dy1 <= dy0:
            continue
        source_local = source_triangle - np.array((sx0, sy0), dtype=float)
        destination_local = destination_triangle - np.array((dx0, dy0), dtype=float)
        affine = cv2.getAffineTransform(
            source_local.astype(np.float32), destination_local.astype(np.float32)
        )
        warped = cv2.warpAffine(
            texture[sy0:sy1, sx0:sx1],
            affine,
            (dx1 - dx0, dy1 - dy0),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        triangle_mask = np.zeros((dy1 - dy0, dx1 - dx0), dtype=np.uint8)
        cv2.fillConvexPoly(
            triangle_mask, np.rint(destination_local).astype(np.int32), 255
        )
        target = canvas[dy0:dy1, dx0:dx1]
        target_mask = canvas_mask[dy0:dy1, dx0:dx1]
        target[triangle_mask > 0] = warped[triangle_mask > 0]
        target_mask[triangle_mask > 0] = 255
    corners_model = np.array(
        [
            [xy_min[0], xy_max[1], xyz_triangles[:, :, 2].mean()],
            [xy_max[0], xy_max[1], xyz_triangles[:, :, 2].mean()],
            [xy_max[0], xy_min[1], xyz_triangles[:, :, 2].mean()],
            [xy_min[0], xy_min[1], xyz_triangles[:, :, 2].mean()],
        ],
        dtype=float,
    )
    return canvas, canvas_mask, corners_model


def project_world_points(points_world: np.ndarray, camera_from_world: np.ndarray, k: np.ndarray) -> np.ndarray:
    camera = points_world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    result = np.empty((len(camera), 2), dtype=float)
    result[:, 0] = k[0, 0] * camera[:, 0] / camera[:, 2] + k[0, 2]
    result[:, 1] = k[1, 1] * camera[:, 1] / camera[:, 2] + k[1, 2]
    return result


def template_yaw_search(
    observed_gray: np.ndarray,
    depth: np.ndarray,
    depth_mode: float,
    centre_world: np.ndarray,
    camera_from_world: np.ndarray,
    k: np.ndarray,
    template: np.ndarray,
    template_mask: np.ndarray,
    corners_model: np.ndarray,
    maximum_shift_px: int = 8,
) -> tuple[float, float, float, int, int, int]:
    source_corners = np.array(
        [[0, 0], [template.shape[1] - 1, 0], [template.shape[1] - 1, template.shape[0] - 1], [0, template.shape[0] - 1]],
        dtype=np.float32,
    )
    observed_blur = cv2.GaussianBlur(observed_gray, (3, 3), 0).astype(np.float32)
    observed_gx = cv2.Sobel(observed_blur, cv2.CV_32F, 1, 0, ksize=3)
    observed_gy = cv2.Sobel(observed_blur, cv2.CV_32F, 0, 1, ksize=3)
    observed_gradient = cv2.magnitude(observed_gx, observed_gy)
    target_depth_mask = np.isfinite(depth) & (np.abs(depth - depth_mode) <= 0.015)

    def normalized_correlation(
        first: np.ndarray, second: np.ndarray, mask: np.ndarray
    ) -> float:
        first_values = first[mask].astype(float)
        second_values = second[mask].astype(float)
        first_values -= first_values.mean()
        second_values -= second_values.mean()
        denominator = np.linalg.norm(first_values) * np.linalg.norm(second_values)
        return (
            float(first_values @ second_values / denominator)
            if denominator > 0
            else -1.0
        )

    def render_yaw(yaw_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        rotation = Rotation.from_euler("z", np.radians(yaw_deg)).as_matrix()
        corners_world = (corners_model - LOCAL_CENTER) @ rotation.T + centre_world
        destination = project_world_points(
            corners_world, camera_from_world, k
        ).astype(np.float32)
        homography = cv2.getPerspectiveTransform(source_corners, destination)
        rendered = cv2.warpPerspective(
            template, homography, (observed_gray.shape[1], observed_gray.shape[0])
        )
        rendered_mask = cv2.warpPerspective(
            template_mask,
            homography,
            (observed_gray.shape[1], observed_gray.shape[0]),
            flags=cv2.INTER_NEAREST,
        ) > 0
        rendered_blur = cv2.GaussianBlur(rendered, (3, 3), 0).astype(np.float32)
        rendered_gx = cv2.Sobel(rendered_blur, cv2.CV_32F, 1, 0, ksize=3)
        rendered_gy = cv2.Sobel(rendered_blur, cv2.CV_32F, 0, 1, ksize=3)
        rendered_gradient = cv2.magnitude(rendered_gx, rendered_gy)
        return rendered_mask, rendered_blur, rendered_gradient

    scores = []
    for yaw_deg in np.arange(0.0, 360.0, 1.0):
        rendered_mask, rendered_blur, rendered_gradient = render_yaw(yaw_deg)
        comparison_mask = rendered_mask & target_depth_mask
        comparison_mask = cv2.erode(
            comparison_mask.astype(np.uint8), np.ones((3, 3), np.uint8)
        ) > 0
        count = int(comparison_mask.sum())
        if count < 80:
            scores.append((-1.0, yaw_deg, count))
            continue
        intensity_correlation = normalized_correlation(
            observed_blur, rendered_blur, comparison_mask
        )
        gradient_correlation = normalized_correlation(
            observed_gradient, rendered_gradient, comparison_mask
        )
        correlation = 0.7 * intensity_correlation + 0.3 * gradient_correlation
        scores.append((correlation, yaw_deg, count))
    scores.sort(reverse=True)
    coarse_yaw = scores[0][1]
    refined_scores = []
    for yaw_deg in np.arange(coarse_yaw - 25.0, coarse_yaw + 25.001, 1.0):
        yaw_deg = float(yaw_deg % 360.0)
        rendered_mask, rendered_blur, rendered_gradient = render_yaw(yaw_deg)
        rows, cols = np.nonzero(rendered_mask)
        if len(rows) == 0:
            continue
        x0, x1 = int(cols.min()), int(cols.max()) + 1
        y0, y1 = int(rows.min()), int(rows.max()) + 1
        rendered_mask_patch = rendered_mask[y0:y1, x0:x1]
        rendered_blur_patch = rendered_blur[y0:y1, x0:x1]
        rendered_gradient_patch = rendered_gradient[y0:y1, x0:x1]
        for shift_y in range(-maximum_shift_px, maximum_shift_px + 1):
            for shift_x in range(-maximum_shift_px, maximum_shift_px + 1):
                ox0, ox1 = x0 + shift_x, x1 + shift_x
                oy0, oy1 = y0 + shift_y, y1 + shift_y
                if (
                    ox0 < 0
                    or oy0 < 0
                    or ox1 > observed_gray.shape[1]
                    or oy1 > observed_gray.shape[0]
                ):
                    continue
                comparison_mask = (
                    rendered_mask_patch & target_depth_mask[oy0:oy1, ox0:ox1]
                )
                comparison_mask = cv2.erode(
                    comparison_mask.astype(np.uint8), np.ones((3, 3), np.uint8)
                ) > 0
                count = int(comparison_mask.sum())
                if count < 80:
                    continue
                intensity_correlation = normalized_correlation(
                    observed_blur[oy0:oy1, ox0:ox1],
                    rendered_blur_patch,
                    comparison_mask,
                )
                gradient_correlation = normalized_correlation(
                    observed_gradient[oy0:oy1, ox0:ox1],
                    rendered_gradient_patch,
                    comparison_mask,
                )
                correlation = (
                    0.7 * intensity_correlation + 0.3 * gradient_correlation
                )
                refined_scores.append(
                    (correlation, yaw_deg, count, shift_x, shift_y)
                )
    refined_scores.sort(reverse=True)
    best_score, best_yaw, best_count, best_shift_x, best_shift_y = refined_scores[0]
    separated = [
        item
        for item in refined_scores[1:]
        if abs(((item[1] - best_yaw + 180) % 360) - 180) >= 10.0
    ]
    second_score = separated[0][0] if separated else -1.0
    return (
        float(best_yaw),
        float(best_score),
        float(best_score - second_score),
        int(best_count),
        int(best_shift_x),
        int(best_shift_y),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", required=True, type=Path)
    parser.add_argument("--model-obj", required=True, type=Path)
    parser.add_argument("--texture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ratio-threshold", type=float, default=0.78)
    parser.add_argument("--minimum-inliers", type=int, default=6)
    parser.add_argument("--minimum-template-score", type=float, default=0.05)
    parser.add_argument("--minimum-template-margin", type=float, default=0.003)
    parser.add_argument("--minimum-template-pixels", type=int, default=200)
    parser.add_argument(
        "--sift-surface",
        choices=("top", "all"),
        default="top",
        help=(
            "CAD UV faces available to the SIFT/PnP diagnostic.  The default "
            "preserves the top-template estimator; all is diagnostic until its "
            "independent rejection and scoring contract are validated."
        ),
    )
    parser.add_argument("--diagnostic-dir", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite estimate: {args.output}")

    metadata = json.loads((args.window / "metadata.json").read_text(encoding="utf-8"))
    rgb = read_ppm(args.window / "color_preview.ppm")
    depth = np.load(args.window / "depth_raw.npy", allow_pickle=False)
    points = np.load(args.window / "points_xyz_m.npy", allow_pickle=False).reshape(
        rgb.shape[0], rgb.shape[1], 3
    )
    uv_triangles, xyz_triangles = load_obj_uv(args.model_obj)
    model_min_z_m = float(xyz_triangles[:, :, 2].min())
    model_max_z_m = float(xyz_triangles[:, :, 2].max())
    (
        centre_world_rgbd,
        covariance_world_rgbd,
        rgbd_point_count,
        depth_mode,
        target_image_center,
        target_center_method,
        target_circle_radius_px,
        target_circle_candidates,
        target_component_coverage,
    ) = (
        rgbd_center_world(
            depth,
            rgb,
            points,
            metadata,
            model_min_z_m=model_min_z_m,
            model_max_z_m=model_max_z_m,
        )
    )
    x0, y0, x1, y1 = target_search_roi(
        depth,
        rgb,
        (int(round(target_image_center[0])), int(round(target_image_center[1]))),
    )
    observed = rgb[y0:y1, x0:x1]
    observed_scale = 4.0
    observed_large = cv2.resize(
        cv2.cvtColor(observed, cv2.COLOR_RGB2GRAY),
        None,
        fx=observed_scale,
        fy=observed_scale,
        interpolation=cv2.INTER_CUBIC,
    )
    texture = cv2.imread(str(args.texture), cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise ValueError(f"failed to read texture: {args.texture}")
    texture_scale = 0.5
    texture_small = cv2.resize(
        texture, None, fx=texture_scale, fy=texture_scale, interpolation=cv2.INTER_AREA
    )
    texture_height, texture_width = texture.shape
    uv_triangles_px = uv_triangles.copy()
    uv_triangles_px[:, :, 0] *= texture_width * texture_scale
    uv_triangles_px[:, :, 1] = (
        1.0 - uv_triangles_px[:, :, 1]
    ) * texture_height * texture_scale
    top_threshold = float(xyz_triangles[:, :, 2].max() - 0.006)
    top_triangle_mask = np.all(xyz_triangles[:, :, 2] >= top_threshold, axis=1)
    top_uv_triangles_px = uv_triangles_px[top_triangle_mask]
    top_xyz_triangles = xyz_triangles[top_triangle_mask]
    top_template, top_template_mask, top_corners_model = canonical_top_template(
        texture,
        uv_triangles[top_triangle_mask],
        top_xyz_triangles,
    )
    if args.diagnostic_dir is not None:
        args.diagnostic_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.diagnostic_dir / "canonical_top_template.png"), top_template)
        cv2.imwrite(str(args.diagnostic_dir / "canonical_top_mask.png"), top_template_mask)
    texture_feature_mask = np.zeros(texture_small.shape, dtype=np.uint8)
    sift_uv_triangles_px = (
        top_uv_triangles_px if args.sift_surface == "top" else uv_triangles_px
    )
    sift_xyz_triangles = (
        top_xyz_triangles if args.sift_surface == "top" else xyz_triangles
    )
    for triangle in sift_uv_triangles_px:
        cv2.fillConvexPoly(
            texture_feature_mask, np.rint(triangle).astype(np.int32), 255
        )
    sift = cv2.SIFT_create(nfeatures=8000, contrastThreshold=0.015, edgeThreshold=12)
    observed_keypoints, observed_descriptors = sift.detectAndCompute(observed_large, None)
    texture_keypoints, texture_descriptors = sift.detectAndCompute(
        texture_small, texture_feature_mask
    )
    failures: list[str] = []
    good_matches = []
    if observed_descriptors is None or texture_descriptors is None:
        failures.append("TEXTURE_DESCRIPTORS_UNAVAILABLE")
    else:
        matcher = cv2.BFMatcher(cv2.NORM_L2)
        for pair in matcher.knnMatch(observed_descriptors, texture_descriptors, k=2):
            if len(pair) == 2 and pair[0].distance < args.ratio_threshold * pair[1].distance:
                good_matches.append(pair[0])

    object_points = []
    image_points = []
    barycentric_margins = []
    for match in good_matches:
        observed_point = observed_keypoints[match.queryIdx].pt
        texture_point = texture_keypoints[match.trainIdx].pt
        model_point, margin = texture_point_to_model(
            texture_point, sift_uv_triangles_px, sift_xyz_triangles
        )
        if model_point is None:
            continue
        object_points.append(model_point)
        image_points.append(
            [x0 + observed_point[0] / observed_scale, y0 + observed_point[1] / observed_scale]
        )
        barycentric_margins.append(float(margin))

    object_array = np.asarray(object_points, dtype=np.float64)
    image_array = np.asarray(image_points, dtype=np.float64)
    k = np.asarray(metadata["streams"]["camera_info"]["k"], dtype=np.float64).reshape(3, 3)
    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    camera_from_world = np.linalg.inv(world_from_camera)

    def score_yaw(yaw_rad: float) -> tuple[int, float, np.ndarray]:
        rotation = Rotation.from_euler("z", yaw_rad).as_matrix()
        world_points = (
            (object_array - LOCAL_CENTER) @ rotation.T + centre_world_rgbd
        )
        camera_points = (
            world_points @ camera_from_world[:3, :3].T
            + camera_from_world[:3, 3]
        )
        projected = np.full_like(image_array, np.nan)
        valid = camera_points[:, 2] > 0.0
        projected[valid, 0] = (
            k[0, 0] * camera_points[valid, 0] / camera_points[valid, 2] + k[0, 2]
        )
        projected[valid, 1] = (
            k[1, 1] * camera_points[valid, 1] / camera_points[valid, 2] + k[1, 2]
        )
        residuals = np.linalg.norm(projected - image_array, axis=1)
        inlier_mask = np.isfinite(residuals) & (residuals <= 5.0)
        count = int(inlier_mask.sum())
        rmse = (
            float(np.sqrt(np.mean(residuals[inlier_mask] ** 2)))
            if count
            else float("inf")
        )
        return count, rmse, residuals

    coarse = []
    if len(object_array) >= args.minimum_inliers:
        for degrees in range(360):
            count, rmse, _ = score_yaw(np.radians(degrees))
            coarse.append((count, rmse, float(degrees)))
    best_count, best_rmse, best_degrees = max(
        coarse or [(0, float("inf"), 0.0)], key=lambda item: (item[0], -item[1])
    )
    fine = []
    if coarse:
        for degrees in np.arange(best_degrees - 2.0, best_degrees + 2.001, 0.25):
            count, rmse, residuals = score_yaw(np.radians(degrees % 360.0))
            fine.append((count, rmse, float(degrees % 360.0), residuals))
    inlier_count, reprojection_rmse, sift_yaw_degrees, _ = max(
        fine or [(0, float("inf"), 0.0, np.empty(0))],
        key=lambda item: (item[0], -item[1]),
    )
    observed_gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    top_centre_world = centre_world_rgbd.copy()
    top_centre_world[2] = model_max_z_m - model_min_z_m
    top_centre_camera = (
        top_centre_world @ camera_from_world[:3, :3].T
        + camera_from_world[:3, 3]
    )
    template_depth_mode = float(top_centre_camera[2])
    image_centre = np.array((k[0, 2], k[1, 2]), dtype=float)
    off_axis_target = bool(
        np.linalg.norm(np.asarray(target_image_center, dtype=float) - image_centre)
        > 0.12 * min(depth.shape)
    )
    (
        yaw_degrees,
        template_score,
        template_margin,
        template_pixel_count,
        template_shift_x,
        template_shift_y,
    ) = (
        template_yaw_search(
            observed_gray,
            depth,
            template_depth_mode,
            centre_world_rgbd,
            camera_from_world,
            k,
            top_template,
            top_template_mask,
            top_corners_model,
            maximum_shift_px=(
                32
                if off_axis_target
                else (14 if target_component_coverage > 0.90 else 8)
            ),
        )
    )
    initial_target_image_center = target_image_center
    target_image_center = (
        target_image_center[0] + template_shift_x,
        target_image_center[1] + template_shift_y,
    )
    refined_ray_camera = np.array(
        (
            (target_image_center[0] - k[0, 2]) / k[0, 0],
            (target_image_center[1] - k[1, 2]) / k[1, 1],
            1.0,
        ),
        dtype=float,
    )
    refined_ray_world = world_from_camera[:3, :3] @ refined_ray_camera
    camera_world = world_from_camera[:3, 3]
    centre_plane_z = LOCAL_CENTER[2] - model_min_z_m
    intersection_plane_z = (
        model_max_z_m - model_min_z_m
        if target_center_method == "occlusion_robust_hough_circle"
        else centre_plane_z
    )
    refined_scale = (
        intersection_plane_z - camera_world[2]
    ) / refined_ray_world[2]
    centre_world_rgbd = camera_world + refined_scale * refined_ray_world
    centre_world_rgbd[2] = centre_plane_z
    post_registration_scores = []
    if len(object_array) >= args.minimum_inliers:
        for diagnostic_degrees in range(360):
            diagnostic_count, diagnostic_rmse, _ = score_yaw(
                np.radians(diagnostic_degrees)
            )
            post_registration_scores.append(
                (diagnostic_count, diagnostic_rmse, float(diagnostic_degrees))
            )
    (
        post_registration_sift_inliers,
        post_registration_sift_rmse,
        post_registration_sift_yaw,
    ) = max(
        post_registration_scores or [(0, float("inf"), 0.0)],
        key=lambda item: (item[0], -item[1]),
    )
    if args.diagnostic_dir is not None:
        source_corners = np.array(
            [
                [0, 0],
                [top_template.shape[1] - 1, 0],
                [top_template.shape[1] - 1, top_template.shape[0] - 1],
                [0, top_template.shape[0] - 1],
            ],
            dtype=np.float32,
        )
        cv2.imwrite(str(args.diagnostic_dir / "observed_gray.png"), observed_gray)
        for label, diagnostic_yaw in (("zero", 0.0), ("best", yaw_degrees)):
            diagnostic_rotation = Rotation.from_euler(
                "z", np.radians(diagnostic_yaw)
            ).as_matrix()
            diagnostic_corners_world = (
                (top_corners_model - LOCAL_CENTER) @ diagnostic_rotation.T
                + centre_world_rgbd
            )
            diagnostic_destination = project_world_points(
                diagnostic_corners_world, camera_from_world, k
            ).astype(np.float32)
            diagnostic_homography = cv2.getPerspectiveTransform(
                source_corners, diagnostic_destination
            )
            diagnostic_render = cv2.warpPerspective(
                top_template,
                diagnostic_homography,
                (observed_gray.shape[1], observed_gray.shape[0]),
            )
            diagnostic_overlay = cv2.addWeighted(
                observed_gray, 0.55, diagnostic_render, 0.45, 0.0
            )
            cv2.imwrite(
                str(args.diagnostic_dir / f"render_{label}.png"), diagnostic_render
            )
            cv2.imwrite(
                str(args.diagnostic_dir / f"overlay_{label}.png"), diagnostic_overlay
            )
    if template_pixel_count < args.minimum_template_pixels:
        failures.append("CAD_TEMPLATE_SUPPORT_TOO_SPARSE")
    if template_score < args.minimum_template_score:
        failures.append("CAD_TEMPLATE_SCORE_TOO_LOW")
    if template_margin < args.minimum_template_margin:
        failures.append("CAD_TEMPLATE_YAW_AMBIGUOUS")
    yaw_observed = not any(
        code in failures
        for code in (
            "CAD_TEMPLATE_SUPPORT_TOO_SPARSE",
            "CAD_TEMPLATE_SCORE_TOO_LOW",
            "CAD_TEMPLATE_YAW_AMBIGUOUS",
        )
    )
    if not yaw_observed:
        failures.append("POSE_YAW_UNOBSERVABLE")
    yaw_rad = float(np.radians(yaw_degrees))
    quaternion = Rotation.from_euler("z", yaw_rad).as_quat()
    rotation_sigma = (
        np.radians(max(1.0, 5.0 / np.sqrt(max(inlier_count, 1))))
        if yaw_observed
        else np.pi / np.sqrt(3.0)
    )
    covariance = np.zeros((6, 6), dtype=float)
    covariance[:3, :3] = covariance_world_rgbd
    if not yaw_observed:
        covariance[:3, :3] += np.diag([0.040**2, 0.040**2, 0.020**2])
    covariance[3:, 3:] = np.diag(
        [np.radians(0.5) ** 2, np.radians(0.5) ** 2, rotation_sigma**2]
    )
    result = {
        "schema": "p7_2_pose_estimate_v1",
        "gate": "P7.2_POSE_ESTIMATION",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "estimator": "ycb_rgbd_cad_top_template_v1",
        "input_window": str(args.window),
        "ground_truth_read": False,
        "pose_frame": "world",
        "pose": {
            "position_m": centre_world_rgbd.tolist(),
            "orientation_xyzw": quaternion.tolist(),
        },
        "covariance_6x6": covariance.tolist(),
        "quality": {
            "search_roi_xyxy_px": [x0, y0, x1, y1],
            "observed_keypoint_count": len(observed_keypoints),
            "texture_keypoint_count": len(texture_keypoints),
            "top_uv_triangle_count": int(top_triangle_mask.sum()),
            "sift_surface_mode": args.sift_surface,
            "sift_uv_triangle_count": int(len(sift_uv_triangles_px)),
            "ratio_match_count": len(good_matches),
            "uv_correspondence_count": len(object_array),
            "yaw_inlier_count": inlier_count,
            "yaw_inlier_ratio": inlier_count / max(1, len(object_array)),
            "yaw_reprojection_rmse_px": reprojection_rmse,
            "sift_diagnostic_yaw_deg": sift_yaw_degrees,
            "post_registration_sift_yaw_deg": post_registration_sift_yaw,
            "post_registration_sift_inlier_count": post_registration_sift_inliers,
            "post_registration_sift_rmse_px": post_registration_sift_rmse,
            "estimated_yaw_deg": yaw_degrees,
            "template_ncc_score": template_score,
            "template_score_margin": template_margin,
            "template_comparison_pixel_count": template_pixel_count,
            "minimum_template_score": args.minimum_template_score,
            "minimum_template_margin": args.minimum_template_margin,
            "minimum_template_pixels": args.minimum_template_pixels,
            "rgbd_segmented_point_count": rgbd_point_count,
            "rgbd_depth_mode_m": depth_mode,
            "cad_predicted_top_depth_m": template_depth_mode,
            "target_image_center_uv_px": list(target_image_center),
            "initial_target_image_center_uv_px": list(
                initial_target_image_center
            ),
            "template_registration_shift_xy_px": [
                template_shift_x,
                template_shift_y,
            ],
            "target_center_method": target_center_method,
            "target_circle_radius_px": target_circle_radius_px,
            "target_circle_candidates_uvr_px": target_circle_candidates,
            "target_component_coverage": target_component_coverage,
            "off_axis_target": off_axis_target,
            "translation_source": "depth_component_top_ellipse_and_cad_support_geometry",
            "roll_pitch_source": "upright_support_plane_constraint",
            "yaw_source": "direct_cad_top_texture_gradient_correlation",
            "minimum_uv_barycentric_margin": min(barycentric_margins) if barycentric_margins else None,
            "texture_yaw_observable": bool(
                yaw_observed
            ),
            "pose_valid_for_grasp": bool(yaw_observed),
            "yaw_standard_deviation_deg": float(np.degrees(rotation_sigma)),
        },
        "failure_codes": sorted(set(failures)),
        "full_se3_gate_pass": bool(yaw_observed and not failures),
        "claim_boundary": (
            "single-frame RGB-D centre plus support-plane-constrained CAD-template yaw; "
            "no Gazebo pose was read"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
