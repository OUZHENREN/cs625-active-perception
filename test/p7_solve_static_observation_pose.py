#!/usr/bin/env python3
"""Solve a static target-looking camera pose offline; never call MoveIt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from p7_static_camera_projection import (
    TARGET_LOCAL_CENTER_M,
    camera_transform_from_urdf,
    link_transforms_from_urdf,
    project_target,
    quaternion_matrix,
)


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def target_center_from_pose_record(source: dict) -> np.ndarray:
    """Return the target centre from either P7.2 or Gazebo pose schema.

    P7.2 publishes the estimated CAD centre directly as ``position_m``.
    Gazebo's diagnostic pose is the model origin and therefore requires the
    fixed YCB local-centre transform.  Supporting both schemas avoids a
    hand-written adapter between the P7.2 and P7.3 interfaces.
    """
    pose = source.get("pose", source)
    if "position_m" in pose:
        centre = np.asarray(pose["position_m"], dtype=float)
        if centre.shape != (3,) or not np.isfinite(centre).all():
            raise ValueError("P7.2 position_m must contain three finite values")
        return centre
    if "position" not in pose or "orientation" not in pose:
        raise ValueError("unsupported target pose schema")
    target_origin = np.array(
        [pose["position"][axis] for axis in ("x", "y", "z")], dtype=float
    )
    return target_origin + quaternion_matrix(pose["orientation"]) @ TARGET_LOCAL_CENTER_M


def look_at_rotation(camera_position: np.ndarray, target_position: np.ndarray) -> np.ndarray:
    forward = target_position - camera_position
    forward /= np.linalg.norm(forward)
    world_up = np.array((0.0, 0.0, 1.0), dtype=float)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack((right, down, forward))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--seed-joints", required=True, type=Path)
    parser.add_argument("--target-pose", required=True, type=Path)
    parser.add_argument(
        "--desired-range-m",
        type=float,
        default=None,
        help="Move the camera along the seed target ray to this optical range.",
    )
    parser.add_argument(
        "--desired-camera-position-m",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Explicit world-frame camera position for an elevated viewpoint.",
    )
    parser.add_argument(
        "--desired-camera-rotation-matrix",
        nargs=9,
        type=float,
        metavar=("R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22"),
        help="Explicit world-from-optical-camera rotation emitted by P7.3 for an off-axis target framing.",
    )
    parser.add_argument(
        "--lock-proximal",
        action="store_true",
        help="Keep shoulder pan/lift and elbow at the seed; solve wrist look-at only.",
    )
    parser.add_argument(
        "--minimum-link-origin-z-m",
        type=float,
        default=0.12,
        help="Conservative floor-clearance proxy for articulated link origins.",
    )
    arguments = parser.parse_args()
    seed_map = yaml.safe_load(arguments.seed_joints.read_text(encoding="utf-8"))
    seed = np.array([float(seed_map[name]) for name in JOINT_NAMES], dtype=float)
    source = json.loads(arguments.target_pose.read_text(encoding="utf-8"))
    target_center = target_center_from_pose_record(source)
    seed_transform = camera_transform_from_urdf(
        arguments.urdf, dict(zip(JOINT_NAMES, seed))
    )
    desired_position = seed_transform[:3, 3]
    if arguments.desired_range_m is not None and arguments.desired_camera_position_m is not None:
        raise ValueError("choose desired range or explicit camera position, not both")
    if arguments.desired_range_m is not None:
        if not 0.30 <= arguments.desired_range_m <= 1.00:
            raise ValueError("PS800-E1 desired range must remain within 0.30--1.00 m")
        seed_ray = target_center - desired_position
        desired_position = target_center - (
            seed_ray / np.linalg.norm(seed_ray) * arguments.desired_range_m
        )
    elif arguments.desired_camera_position_m is not None:
        desired_position = np.asarray(arguments.desired_camera_position_m, dtype=float)
        requested_range = float(np.linalg.norm(target_center - desired_position))
        if not 0.30 <= requested_range <= 1.00:
            raise ValueError("explicit camera position is outside PS800-E1 0.30--1.00 m range")
    desired_rotation = look_at_rotation(desired_position, target_center)
    if arguments.desired_camera_rotation_matrix is not None:
        desired_rotation = np.asarray(
            arguments.desired_camera_rotation_matrix, dtype=float
        ).reshape(3, 3)
        if not np.allclose(desired_rotation.T @ desired_rotation, np.eye(3), atol=1.0e-6):
            raise ValueError("requested camera rotation must be orthonormal")
        if np.linalg.det(desired_rotation) <= 0.0:
            raise ValueError("requested camera rotation must be proper")
    seed_range = float(np.linalg.norm(target_center - seed_transform[:3, 3]))

    active_slice = slice(3, 6) if arguments.lock_proximal else slice(0, 6)

    def expand(active_joints: np.ndarray) -> np.ndarray:
        joints = seed.copy()
        joints[active_slice] = active_joints
        return joints

    def residual(active_joints: np.ndarray) -> np.ndarray:
        joints = expand(active_joints)
        transform = camera_transform_from_urdf(
            arguments.urdf, dict(zip(JOINT_NAMES, joints))
        )
        if arguments.lock_proximal:
            optical_range = float(np.linalg.norm(target_center - transform[:3, 3]))
            range_error = (optical_range - seed_range) / 0.05
            low_camera_penalty = max(0.0, 0.25 - float(transform[2, 3])) / 0.03
            position_error = np.array((range_error, low_camera_penalty))
            target_rotation = look_at_rotation(transform[:3, 3], target_center)
        else:
            position_error = (transform[:3, 3] - desired_position) / 0.03
            target_rotation = desired_rotation
        orientation_error = Rotation.from_matrix(
            target_rotation.T @ transform[:3, :3]
        ).as_rotvec() / 0.08
        regularization = (active_joints - seed[active_slice]) * 0.005
        link_transforms = link_transforms_from_urdf(
            arguments.urdf, dict(zip(JOINT_NAMES, joints))
        )
        articulated_links = (
            "forearm_link", "wrist_1_link",
            "wrist_2_link", "wrist_3_link", "my_end_effector_link",
            "gripper_base_link", "camera_link",
        )
        floor_penalties = np.array(
            [
                max(
                    0.0,
                    arguments.minimum_link_origin_z_m
                    - float(link_transforms[name][2, 3]),
                )
                / 0.02
                for name in articulated_links
            ]
        )
        return np.concatenate(
            (position_error, orientation_error, regularization, floor_penalties)
        )

    lower = np.full(seed[active_slice].shape, -2.0 * np.pi, dtype=float)
    upper = np.full(seed[active_slice].shape, 2.0 * np.pi, dtype=float)
    if not arguments.lock_proximal:
        lower[2] = -np.pi
        upper[2] = np.pi
    solution = least_squares(
        residual,
        seed[active_slice],
        bounds=(lower, upper),
        max_nfev=1200,
        xtol=1.0e-12,
        ftol=1.0e-12,
        gtol=1.0e-12,
    )
    solved = expand(solution.x)
    joints = dict(zip(JOINT_NAMES, (float(value) for value in solved)))
    transform = camera_transform_from_urdf(arguments.urdf, joints)
    link_transforms = link_transforms_from_urdf(arguments.urdf, joints)
    result = {
        "solver_converged": bool(solution.success),
        "solver_message": solution.message,
        "cost": float(solution.cost),
        "max_abs_scaled_residual": float(np.max(np.abs(residual(solution.x)))),
        "joint_positions_rad": joints,
        "max_seed_delta_rad": float(np.max(np.abs(solved - seed))),
        "proximal_joints_locked": arguments.lock_proximal,
        "camera_position_world_m": transform[:3, 3].tolist(),
        "requested_optical_range_m": arguments.desired_range_m,
        "requested_camera_position_world_m": arguments.desired_camera_position_m,
        "requested_camera_rotation_matrix": (
            None
            if arguments.desired_camera_rotation_matrix is None
            else desired_rotation.tolist()
        ),
        "projection": project_target(
            transform,
            target_center,
            width=320,
            height=240,
            horizontal_fov_rad=1.047,
        ),
        "articulated_link_origin_z_m": {
            name: float(link_transforms[name][2, 3])
            for name in (
                "shoulder_link", "upperarm_link", "forearm_link", "wrist_1_link",
                "wrist_2_link", "wrist_3_link", "my_end_effector_link",
                "gripper_base_link", "camera_link",
            )
        },
        "minimum_distal_link_origin_z_m": min(
            float(link_transforms[name][2, 3])
            for name in (
                "forearm_link", "wrist_1_link",
                "wrist_2_link", "wrist_3_link", "my_end_effector_link",
                "gripper_base_link", "camera_link",
            )
        ),
        "claim_boundary": (
            "offline kinematic seed only; collision, Gazebo stability and P7.1 "
            "runtime acceptance remain unverified"
        ),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
