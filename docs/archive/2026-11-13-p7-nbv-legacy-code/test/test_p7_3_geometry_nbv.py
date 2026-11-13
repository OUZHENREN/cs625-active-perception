"""Unit checks for the P7.3 physical geometry/information primitives."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_3_geometry_nbv import (  # noqa: E402
    camera_rotation,
    candidate_metrics,
    externally_occluded,
    project,
)


def test_camera_rotation_optical_axis_looks_at_target():
    position = np.array((0.4, -0.3, 0.5))
    target = np.array((0.1, 0.2, 0.1))
    rotation = camera_rotation(position, target)
    assert np.allclose(rotation[:, 2], (target - position) / np.linalg.norm(target - position))
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-12)


def test_off_axis_camera_rotation_places_target_in_requested_image_row():
    position = np.array((0.0, -0.7, 0.35))
    target = np.array((0.0, 0.0, 0.0))
    k = np.array(((500.0, 0.0, 320.0), (0.0, 500.0, 240.0), (0.0, 0.0, 1.0)))
    world_from_camera = np.eye(4)
    world_from_camera[:3, :3] = camera_rotation(
        position, target, k=k, target_pixel_offset_px=(0.0, -90.0)
    )
    world_from_camera[:3, 3] = position
    _, uv = project(target.reshape(1, 3), world_from_camera, k)
    assert np.allclose(uv[0], (320.0, 150.0), atol=1.0e-9)


def test_measured_obstacle_point_blocks_candidate_ray():
    target_samples = np.array(((0.0, 0.0, 0.0), (0.08, 0.10, 0.0)))
    blockers = np.array(((-0.35, 0.0, 0.0),))
    blocked = externally_occluded(np.array((-0.70, 0.0, 0.0)), target_samples, blockers)
    assert blocked.tolist() == [True, False]


def test_visible_textured_geometry_reduces_yaw_variance():
    samples = {
        "centroids": np.array(((0.02, 0.0, 0.02), (-0.02, 0.01, 0.01), (0.0, -0.02, 0.03))),
        "normals": np.array(((0.0, -1.0, 0.0), (0.0, -1.0, 0.0), (0.0, -1.0, 0.0))),
        "areas": np.array((1.0e-3, 1.0e-3, 1.0e-3)),
        "texture_gradient": np.array((1.0, 0.8, 0.9)),
    }
    pose = {"position_m": [0.0, 0.0, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
    covariance = np.diag((0.001, 0.001, 0.001, 0.001, 0.001, 1.0))
    result = candidate_metrics(
        samples, pose, covariance, np.zeros(3, dtype=bool), np.zeros(3),
        np.array((0.0, -0.7, 0.35)), np.array(((500.0, 0.0, 320.0), (0.0, 500.0, 240.0), (0.0, 0.0, 1.0))),
        640, 480, 15000.0, (0.0, 0.0),
    )
    assert result["new_geometry_coverage_fraction"] > 0.0
    assert result["yaw_fisher_information_rad_minus2"] > 0.0
    assert result["expected_posterior_yaw_variance_rad2"] < result["prior_yaw_variance_rad2"]
