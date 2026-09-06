#!/usr/bin/env python3
"""Audit a static P7.1 camera pose from URDF without MoveIt or simulation."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import numpy as np
import yaml


TARGET_LOCAL_CENTER_M = np.array((-0.0091685, 0.0840170, 0.0510065), dtype=float)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def quaternion_matrix(orientation: dict[str, float]) -> np.ndarray:
    x, y, z, w = (float(orientation[key]) for key in ("x", "y", "z", "w"))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    one = 1.0 - c
    return np.array(
        [
            [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
            [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
            [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
        ],
        dtype=float,
    )


def parse_vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected three values, got: {text}")
    return np.array(values, dtype=float)


def camera_transform_from_urdf(
    urdf_path: Path,
    joint_positions: dict[str, float],
    *,
    root_link: str = "world",
    camera_link: str = "camera_depth_optical_frame",
) -> np.ndarray:
    root = element_tree.parse(urdf_path).getroot()
    by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        if child is not None and parent is not None:
            by_child[child.attrib["link"]] = joint
    chain = []
    current = camera_link
    while current != root_link:
        if current not in by_child:
            raise ValueError(
                f"no URDF chain from {root_link} to {camera_link}: stopped at {current}"
            )
        joint = by_child[current]
        chain.append(joint)
        current = joint.find("parent").attrib["link"]
    transform = np.eye(4, dtype=float)
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = parse_vector(
            origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)
        )
        rpy = parse_vector(
            origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)
        )
        transform = transform @ homogeneous(rpy_matrix(*rpy), xyz)
        if joint.attrib.get("type") in {"revolute", "continuous"}:
            axis_element = joint.find("axis")
            axis = parse_vector(
                axis_element.attrib.get("xyz") if axis_element is not None else None,
                (1, 0, 0),
            )
            angle = float(joint_positions.get(joint.attrib["name"], 0.0))
            transform = transform @ homogeneous(
                axis_angle_matrix(axis, angle), np.zeros(3)
            )
    return transform


def link_transforms_from_urdf(
    urdf_path: Path,
    joint_positions: dict[str, float],
    *,
    root_link: str = "world",
) -> dict[str, np.ndarray]:
    """Return root-to-link transforms for the connected URDF tree."""
    root = element_tree.parse(urdf_path).getroot()
    by_parent: dict[str, list] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        if parent is not None:
            by_parent.setdefault(parent.attrib["link"], []).append(joint)
    transforms = {root_link: np.eye(4, dtype=float)}
    pending = [root_link]
    while pending:
        parent_name = pending.pop()
        for joint in by_parent.get(parent_name, []):
            child_name = joint.find("child").attrib["link"]
            origin = joint.find("origin")
            xyz = parse_vector(
                origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)
            )
            rpy = parse_vector(
                origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)
            )
            local = homogeneous(rpy_matrix(*rpy), xyz)
            if joint.attrib.get("type") in {"revolute", "continuous"}:
                axis_element = joint.find("axis")
                axis = parse_vector(
                    axis_element.attrib.get("xyz") if axis_element is not None else None,
                    (1, 0, 0),
                )
                local = local @ homogeneous(
                    axis_angle_matrix(
                        axis, float(joint_positions.get(joint.attrib["name"], 0.0))
                    ),
                    np.zeros(3),
                )
            transforms[child_name] = transforms[parent_name] @ local
            pending.append(child_name)
    return transforms


def project_target(
    world_from_camera: np.ndarray,
    target_center_world: np.ndarray,
    *,
    width: int,
    height: int,
    horizontal_fov_rad: float,
) -> dict:
    camera = world_from_camera[:3, :3].T @ (
        target_center_world - world_from_camera[:3, 3]
    )
    fx = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    fy = fx
    cx, cy = width / 2.0, height / 2.0
    z = float(camera[2])
    if z <= 0.0:
        return {
            "camera_xyz_m": camera.tolist(),
            "inside_image": False,
            "u_px": None,
            "v_px": None,
        }
    u = float(fx * camera[0] / z + cx)
    v = float(fy * camera[1] / z + cy)
    return {
        "camera_xyz_m": camera.tolist(),
        "inside_image": bool(0.0 <= u < width and 0.0 <= v < height),
        "u_px": u,
        "v_px": v,
        "width": width,
        "height": height,
        "horizontal_fov_rad": horizontal_fov_rad,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--joint-positions", required=True, type=Path)
    parser.add_argument("--target-pose", required=True, type=Path)
    arguments = parser.parse_args()
    joints = {
        str(name): float(value)
        for name, value in yaml.safe_load(
            arguments.joint_positions.read_text(encoding="utf-8")
        ).items()
    }
    source = json.loads(arguments.target_pose.read_text(encoding="utf-8"))
    pose = source.get("pose", source)
    target_position = np.array(
        [pose["position"][axis] for axis in ("x", "y", "z")], dtype=float
    )
    target_center = (
        target_position + quaternion_matrix(pose["orientation"]) @ TARGET_LOCAL_CENTER_M
    )
    world_from_camera = camera_transform_from_urdf(arguments.urdf, joints)
    result = {
        "camera_position_world_m": world_from_camera[:3, 3].tolist(),
        "projection": project_target(
            world_from_camera,
            target_center,
            width=320,
            height=240,
            horizontal_fov_rad=1.047,
        ),
        "claim_boundary": "offline static-FK pre-screen only; not runtime P7.1 evidence",
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
