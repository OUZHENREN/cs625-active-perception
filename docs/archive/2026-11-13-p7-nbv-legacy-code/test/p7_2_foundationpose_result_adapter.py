#!/usr/bin/env python3
"""Transform a FoundationPose camera-frame result into the P7.2 world-frame contract."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def transform_matrix(record: dict) -> np.ndarray:
    translation = record["translation_m"]
    orientation = record["orientation"]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [orientation[key] for key in ("x", "y", "z", "w")]
    ).as_matrix()
    matrix[:3, 3] = [translation[key] for key in ("x", "y", "z")]
    return matrix


def validate_se3(matrix: np.ndarray, label: str) -> None:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{label} has invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise ValueError(f"{label} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4):
        raise ValueError(f"{label} rotation determinant is not +1")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--foundationpose-result", required=True, type=Path)
    parser.add_argument("--source-metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite result: {args.output}")
    inference = json.loads(args.foundationpose_result.read_text(encoding="utf-8"))
    metadata = json.loads(args.source_metadata.read_text(encoding="utf-8"))
    object_to_camera = np.asarray(inference["object_model_to_camera_optical"], dtype=float)
    world_from_camera = transform_matrix(metadata["tf"]["world_to_points"])
    validate_se3(object_to_camera, "FoundationPose result")
    validate_se3(world_from_camera, "P7.1 exact-time TF")
    object_to_world = world_from_camera @ object_to_camera
    validate_se3(object_to_world, "world result")

    covariance_camera = np.asarray(inference["covariance_6x6_camera"], dtype=float)
    if covariance_camera.shape != (6, 6) or not np.all(np.isfinite(covariance_camera)):
        raise ValueError("camera covariance must be a finite 6x6 matrix")
    eigenvalues = np.linalg.eigvalsh(0.5 * (covariance_camera + covariance_camera.T))
    if eigenvalues.min() < -1e-10:
        raise ValueError("camera covariance must be positive semidefinite")
    # Block-diagonal SE(3) adjoint is sufficient here because covariance is
    # defined at the reported object origin, not transported to a new origin.
    rotation = world_from_camera[:3, :3]
    adjoint_rotation = np.zeros((6, 6))
    adjoint_rotation[:3, :3] = rotation
    adjoint_rotation[3:, 3:] = rotation
    covariance_world = adjoint_rotation @ covariance_camera @ adjoint_rotation.T
    quaternion = Rotation.from_matrix(object_to_world[:3, :3]).as_quat()
    failures = list(inference.get("failure_codes", []))
    result = {
        "schema": "p7_2_pose_estimate_v2",
        "gate": "P7.2_POSE_ESTIMATION",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "estimator": "FoundationPose",
        "ground_truth_read": False,
        "pose_frame": "world",
        "pose": {
            "position_m": object_to_world[:3, 3].tolist(),
            "orientation_xyzw": quaternion.tolist(),
        },
        "object_model_to_world": object_to_world.tolist(),
        "covariance_6x6": covariance_world.tolist(),
        "quality": inference["quality"],
        "failure_codes": failures,
        "full_se3_gate_pass": not failures,
        "coordinate_contract": "T_world_object = T_world_camera_depth_optical @ T_camera_object",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
