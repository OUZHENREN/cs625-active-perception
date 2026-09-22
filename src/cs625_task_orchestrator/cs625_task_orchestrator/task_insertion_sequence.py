"""World-frame waypoints for the shielding module insertion task.

Pure geometry: no ROS, no mesh files, no configuration loading.  Everything it
needs arrives as arguments, which is what makes the sequence testable and what lets
a ROS node import it without dragging a solver along.

The task is grasp -> transfer -> insert.  The important structural decision is that
the grasp is expressed in the MODULE frame, so the flange pose needed to hold the
module at ANY module pose is one composition:

    flange_world = module_world . grasp_in_module

Nothing here re-derives a pose per waypoint; the waypoints are the same transform
evaluated at the module poses the task visits.  That is also why a pose observed by
P7.2 can be grasped without touching this module.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def compose(parent: list[float], child: list[float]) -> list[float]:
    """World pose of a child given in the parent's frame; both are SDF poses."""

    rotation = _rotation(parent[3:6])
    child_rotation = _rotation(child[3:6])
    product = rotation @ child_rotation
    position = rotation @ np.asarray(child[:3], dtype=float) + np.asarray(
        parent[:3], dtype=float
    )
    return [float(v) for v in position] + _rpy(product)


def _rotation(rpy: list[float]) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler("xyz", rpy).as_matrix()


def _rpy(matrix: np.ndarray) -> list[float]:
    return [
        math.atan2(float(matrix[2, 1]), float(matrix[2, 2])),
        -math.asin(max(-1.0, min(1.0, float(matrix[2, 0])))),
        math.atan2(float(matrix[1, 0]), float(matrix[0, 0])),
    ]


def grasp_pose_in_module(grasp_template: dict[str, Any]) -> list[float]:
    """The template's module-frame grasp transform as an SDF pose."""

    from scipy.spatial.transform import Rotation

    rotation = np.asarray(grasp_template["rotation_gripper_to_module"], dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("rotation_gripper_to_module must be 3x3")
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-9:
        raise ValueError("rotation_gripper_to_module is not a proper rotation")
    translation = [float(v) for v in grasp_template["translation_module_m"]]
    if len(translation) != 3:
        raise ValueError("translation_module_m must have three components")
    # as_euler hands back numpy scalars, which PyYAML refuses to serialise, so every
    # value that leaves this module goes through float().
    return translation + [
        float(v) for v in Rotation.from_matrix(rotation).as_euler("xyz")
    ]


def flange_for_module_pose(
    module_pose_world: list[float], grasp_pose_module: list[float]
) -> list[float]:
    """Flange pose that holds the module at the given world pose."""

    return compose(module_pose_world, grasp_pose_module)


def insertion_pose(
    seated_module_world: list[float],
    axis_world: list[float],
    travel_m: float,
) -> list[float]:
    """Module pose part-way along the insertion axis, away from the seated pose.

    A positive travel retreats out of the fixture, which is the direction the module
    is withdrawn; the insertion itself is the same line traversed the other way.
    """

    axis = np.asarray(axis_world, dtype=float)
    if axis.shape != (3,):
        raise ValueError("axis_world must have three components")
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        raise ValueError("axis_world must not be zero")
    unit = axis / norm
    position = np.asarray(seated_module_world[:3], dtype=float) + unit * travel_m
    return [float(v) for v in position] + [float(v) for v in seated_module_world[3:6]]


def build_sequence(
    *,
    module_home_world: list[float],
    seated_module_world: list[float],
    grasp_template: dict[str, Any],
    insertion_axis_world: list[float],
    withdrawal_travel_m: float,
    approach_margin_m: float,
) -> dict[str, Any]:
    """Named waypoints for grasp, transfer, insert and release.

    The transfer leg between the pick and the insertion entry is deliberately NOT
    interpolated here.  It is a free-space motion that reorients the module from the
    pose it was picked at to the pose the slot requires, and inventing a straight
    line for it would be wrong; the planner owns that leg.
    """

    grasp_in_module = grasp_pose_in_module(grasp_template)
    approach = grasp_template["approach"]
    if approach["axis_module"] != [0.0, -1.0, 0.0]:
        raise ValueError("this sequence assumes the measured module -Y approach")
    standoff = float(approach["standoff_m"])

    module_pregrasp = insertion_pose(
        module_home_world, [-v for v in approach["axis_module"]], -standoff
    )
    module_lift = list(module_home_world)
    module_lift[2] += float(grasp_template["lift"]["lift_m"])

    entry_travel = float(withdrawal_travel_m) + float(approach_margin_m)
    module_entry = insertion_pose(seated_module_world, insertion_axis_world, entry_travel)

    waypoints = {
        "pregrasp_module": flange_for_module_pose(module_pregrasp, grasp_in_module),
        "grasp_module": flange_for_module_pose(module_home_world, grasp_in_module),
        "lift_module": flange_for_module_pose(module_lift, grasp_in_module),
        # Free-space transfer leg lives between these two.
        "insert_entry": flange_for_module_pose(module_entry, grasp_in_module),
        "insert_seated": flange_for_module_pose(seated_module_world, grasp_in_module),
        "release_retreat": flange_for_module_pose(module_entry, grasp_in_module),
    }
    arms = {
        "pregrasp_module": float(grasp_template["arm"]["retracted_command_m"]),
        "grasp_module": float(grasp_template["arm"]["retracted_command_m"]),
        "close_arms": float(grasp_template["arm"]["contact_command_m"]),
        "lift_module": float(grasp_template["arm"]["contact_command_m"]),
        "insert_entry": float(grasp_template["arm"]["contact_command_m"]),
        "insert_seated": float(grasp_template["arm"]["contact_command_m"]),
        "open_arms": float(grasp_template["arm"]["retracted_command_m"]),
        "release_retreat": float(grasp_template["arm"]["retracted_command_m"]),
    }
    return {
        "grasp_pose_in_module": grasp_in_module,
        "waypoints_world": waypoints,
        "arm_command_m": arms,
        "module_poses_world": {
            "home": list(module_home_world),
            "pregrasp": module_pregrasp,
            "lift": module_lift,
            "insert_entry": module_entry,
            "seated": list(seated_module_world),
        },
        "order": [
            "pregrasp_module",
            "grasp_module",
            "close_arms",
            "lift_module",
            "TRANSFER_FREE_SPACE",
            "insert_entry",
            "insert_seated",
            "open_arms",
            "release_retreat",
        ],
        "insertion_axis_world": [float(v) for v in insertion_axis_world],
        "insertion_travel_m": entry_travel,
    }


def verify_insertion_line(
    seated_module_world: list[float],
    axis_world: list[float],
    travel_m: float,
    module_points: np.ndarray,
    fixture_points: np.ndarray,
    fixture_tolerance_m: float = 0.0005,
) -> dict[str, Any]:
    """Sample the insertion line and report the worst module-to-fixture clearance.

    A straight descent is the whole point of the insertion leg, so it is checked
    directly: the module is walked from the entry pose to the seated pose and the
    minimum distance to the fixture is recorded at every step.  The meshes are passed
    in because this module deliberately owns no files.
    """

    from scipy.spatial import cKDTree

    tree = cKDTree(fixture_points)
    samples = []
    # A raised module is not in contact, so only the entry pose is required to clear;
    # the travel is sampled so a graze part-way down cannot hide.
    for index in range(41):
        offset = travel_m * (1.0 - index / 40.0)
        pose = insertion_pose(seated_module_world, axis_world, offset)
        rotation = _rotation(pose[3:6])
        world = (rotation @ module_points.T).T + np.asarray(pose[:3], dtype=float)
        distance, _ = tree.query(world)
        samples.append(
            {
                "offset_from_seated_m": round(float(offset), 6),
                "minimum_clearance_m": round(float(distance.min()), 9),
            }
        )
    worst = min(samples, key=lambda entry: entry["minimum_clearance_m"])
    return {
        "samples": samples,
        "minimum_clearance_m": worst["minimum_clearance_m"],
        "minimum_at_offset_m": worst["offset_from_seated_m"],
        "seated_clearance_m": samples[-1]["minimum_clearance_m"],
        "entry_clearance_m": samples[0]["minimum_clearance_m"],
        "passes": worst["minimum_clearance_m"] > fixture_tolerance_m,
    }
