"""Pure geometry checks for an observed P7 target and grasp-centre pose."""

from __future__ import annotations

import math
from typing import Any


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def decode_pose(value: dict[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Decode a position/quaternion mapping and normalize its quaternion."""

    if not isinstance(value, dict):
        raise ValueError("pose must be an object")
    position = value.get("position")
    orientation = value.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError("pose needs position and orientation objects")
    xyz = tuple(_finite(position.get(axis), f"position.{axis}") for axis in "xyz")
    quaternion = tuple(
        _finite(orientation.get(axis), f"orientation.{axis}")
        for axis in ("x", "y", "z", "w")
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < math.sqrt(0.5):
        raise ValueError("pose quaternion norm is invalid")
    return xyz, tuple(component / norm for component in quaternion)


def rotate_point(
    point: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    """Rotate one point by an xyzw unit quaternion."""

    x, y, z, w = quaternion
    px, py, pz = point
    # R(q) p, expanded to avoid a runtime ROS dependency in the evaluator.
    return (
        (1 - 2 * (y * y + z * z)) * px + 2 * (x * y - z * w) * py + 2 * (x * z + y * w) * pz,
        2 * (x * y + z * w) * px + (1 - 2 * (x * x + z * z)) * py + 2 * (y * z - x * w) * pz,
        2 * (x * z - y * w) * px + 2 * (y * z + x * w) * py + (1 - 2 * (x * x + y * y)) * pz,
    )


def multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def target_pose_relative_to_grasp(
    model_pose: dict[str, Any],
    grasp_pose: dict[str, Any],
    target_local_center_m: tuple[float, float, float],
) -> dict[str, Any]:
    """Express the target collision centre/orientation in the grasp frame."""

    model_xyz, model_q = decode_pose(model_pose)
    grasp_xyz, grasp_q = decode_pose(grasp_pose)
    target_center = target_center_world(model_pose, target_local_center_m)
    inverse_grasp = (-grasp_q[0], -grasp_q[1], -grasp_q[2], grasp_q[3])
    relative_position = rotate_point(
        tuple(target - grasp for target, grasp in zip(target_center, grasp_xyz)),
        inverse_grasp,
    )
    relative_q = multiply_quaternions(inverse_grasp, model_q)
    return {
        "position": dict(zip("xyz", relative_position)),
        "orientation": dict(zip(("x", "y", "z", "w"), relative_q)),
    }


def evaluate_grasp_geometry(
    model_pose: dict[str, Any],
    grasp_pose: dict[str, Any],
    target_local_center_m: tuple[float, float, float],
    threshold_m: float = 0.015,
) -> dict[str, Any]:
    """Measure target-centre to grasp-centre translation without inference."""

    threshold = _finite(threshold_m, "threshold_m")
    if threshold <= 0.0:
        raise ValueError("threshold_m must be positive")
    local = tuple(
        _finite(value, f"target_local_center_m[{index}]")
        for index, value in enumerate(target_local_center_m)
    )
    model_xyz, _ = decode_pose(model_pose)
    grasp_xyz, _ = decode_pose(grasp_pose)
    target_center = target_center_world(model_pose, local)
    error = math.sqrt(sum((a - b) ** 2 for a, b in zip(target_center, grasp_xyz)))
    return {
        "geometry_measurement_available": True,
        "target_model_origin_world_m": list(model_xyz),
        "target_local_center_m": list(local),
        "target_center_world_m": list(target_center),
        "grasp_center_world_m": list(grasp_xyz),
        "target_to_grasp_center_error_m": error,
        "geometry_threshold_m": threshold,
        "geometry_pass": error <= threshold,
    }


def target_center_world(
    model_pose: dict[str, Any], target_local_center_m: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Transform the model-local target centre into the world/base frame."""

    local = tuple(
        _finite(value, f"target_local_center_m[{index}]")
        for index, value in enumerate(target_local_center_m)
    )
    model_xyz, model_q = decode_pose(model_pose)
    rotated = rotate_point(local, model_q)
    return tuple(a + b for a, b in zip(model_xyz, rotated))
