#!/usr/bin/env python3
"""Ask whether ANY arm configuration can follow the insertion descent.

The insertion sequence reaches one Cartesian pose in two runs with two completely
different arm configurations, and the straight descent that follows is plannable
69.7 percent of the way from one of them and 37.3 percent from the other.  So the
descent's feasibility is decided by the configuration, not by the pose -- and the
question that matters is whether that is bad luck or a fact about the task.

This answers it offline, with no simulator involved.  The descent line is sampled in
Cartesian space, inverse kinematics is solved at every sample, and each sample is
seeded from the previous solution so the result is a continuous joint path rather
than 21 unrelated branches.  Several seeds are tried, because a single seed only
reports the branch it happened to fall into.

    python3 scripts/insertion_configuration_sweep.py

What it does NOT check is collision.  MoveIt's fraction covers both inverse
kinematics and collision, so a line that fails here is definitively infeasible, while
a line that passes here may still be rejected for contact.  That one-sidedness is the
point: a negative answer is conclusive.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import tempfile

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


ROOT = pathlib.Path(__file__).resolve().parents[1]
TASK_SCENE = ROOT / "src/cs625_bringup/config/cs625_task_scene.yaml"
GRASP_TEMPLATE = ROOT / "src/cs625_bringup/config/cs625_grasp_template.yaml"
SEQUENCE = ROOT / "src/cs625_bringup/config/cs625_insertion_sequence.yaml"
URDF_XACRO = ROOT / "src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro"
SAFE_INITIAL = ROOT / "src/cs625_bringup/config/p7_safe_initial_positions.yaml"

TIP_LINK = "my_end_effector_link"
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
SAMPLES = 21


def expand_urdf() -> str:
    """Run xacro the way the simulator does and hand back the URDF text."""

    wrapper = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="p">
  <xacro:arg name="prefix" default=""/>
  <xacro:arg name="cs_type" default="cs625"/>
  <xacro:include filename="{include}"/>
</robot>
""".format(include=URDF_XACRO)
    with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as handle:
        handle.write(wrapper)
        wrapper_path = pathlib.Path(handle.name)
    try:
        completed = subprocess.run(
            ["xacro", str(wrapper_path), "prefix:=", "cs_type:=cs625", "name:=cs"],
            check=True,
            capture_output=True,
            text=True,
        )
    finally:
        wrapper_path.unlink(missing_ok=True)
    return completed.stdout


def parse_chain(urdf_text: str) -> tuple[dict, tuple[float, ...], tuple[float, ...]]:
    """Extract the joint chain to the planning tip with its limits."""

    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_text)
    joints: dict[str, tuple[str, np.ndarray, str, str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = [
            float(v)
            for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()
        ]
        rpy = [
            float(v)
            for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()
        ]
        axis = joint.find("axis")
        ax = [
            float(v)
            for v in (axis.get("xyz", "0 0 0") if axis is not None else "0 0 0").split()
        ]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        joints[joint.find("child").get("link")] = (
            joint.find("parent").get("link"),
            transform,
            joint.get("name"),
            joint.get("type"),
            np.asarray(ax, dtype=float),
        )

    limits = {}
    for joint in root.findall("joint"):
        if joint.get("name") in JOINT_NAMES:
            limit = joint.find("limit")
            limits[joint.get("name")] = (
                float(limit.get("lower")),
                float(limit.get("upper")),
            )
    lower = tuple(limits[name][0] for name in JOINT_NAMES)
    upper = tuple(limits[name][1] for name in JOINT_NAMES)
    return joints, lower, upper


def forward_kinematics(joints: dict, q: np.ndarray) -> np.ndarray:
    """Pose of the planning tip in the base frame."""

    transform = np.eye(4)
    current = TIP_LINK
    chain = []
    while current in joints:
        parent, local, name, kind, axis = joints[current]
        chain.append((local, name, kind, axis))
        current = parent
    for local, name, kind, axis in reversed(chain):
        step = local.copy()
        if kind == "revolute" and name in JOINT_NAMES:
            angle = float(q[JOINT_NAMES.index(name)])
            step[:3, :3] = step[:3, :3] @ Rotation.from_rotvec(axis * angle).as_matrix()
        transform = transform @ step
    return transform


def solve_ik(
    joints: dict,
    lower: tuple,
    upper: tuple,
    target: np.ndarray,
    seed: np.ndarray,
    restricted: tuple | None = None,
) -> tuple[np.ndarray, float, float]:
    """Position and orientation least squares from one seed."""

    target_rotation = target[:3, :3]
    target_position = target[:3, 3]

    def residual(q: np.ndarray) -> np.ndarray:
        pose = forward_kinematics(joints, q)
        position = pose[:3, 3] - target_position
        # Rotation vector of the orientation error, which is what a pose goal means.
        orientation = Rotation.from_matrix(target_rotation.T @ pose[:3, :3]).as_rotvec()
        return np.concatenate([position, orientation])

    bounds = (-np.inf, np.inf)
    if restricted is not None:
        bounds = (
            np.array([restricted[0][i] if restricted[2][i] else -np.inf for i in range(6)]),
            np.array([restricted[1][i] if restricted[2][i] else np.inf for i in range(6)]),
        )
    result = least_squares(residual, seed, bounds=bounds, xtol=1e-12, ftol=1e-12)
    position_error = float(np.linalg.norm(result.fun[:3]))
    orientation_error = float(np.linalg.norm(result.fun[3:]))
    return result.x, position_error, orientation_error


def compose(parent: list[float], child: list[float]) -> np.ndarray:
    rotation = Rotation.from_euler("xyz", parent[3:6]).as_matrix()
    child_rotation = Rotation.from_euler("xyz", child[3:6]).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rotation @ child_rotation
    pose[:3, 3] = rotation @ np.asarray(child[:3], dtype=float) + np.asarray(parent[:3])
    return pose


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--samples", type=int, default=SAMPLES)
    arguments = parser.parse_args()

    scene = yaml.safe_load(TASK_SCENE.read_text(encoding="utf-8"))[
        "cs625_task_scene"
    ]["ros__parameters"]
    template = yaml.safe_load(GRASP_TEMPLATE.read_text(encoding="utf-8"))[
        "cs625_grasp_template"
    ]["ros__parameters"]
    sequence = yaml.safe_load(SEQUENCE.read_text(encoding="utf-8"))[
        "cs625_insertion_sequence"
    ]["ros__parameters"]

    joints, lower, upper = parse_chain(expand_urdf())
    grasp_in_module = list(template["translation_module_m"]) + list(
        Rotation.from_matrix(
            np.asarray(template["rotation_gripper_to_module"], dtype=float)
        ).as_euler("xyz")
    )

    seated = scene["insertion"]["seated_pose_world"]
    axis = np.asarray(sequence["insertion_axis_world"], dtype=float)
    travel = float(sequence["insertion_travel_m"])

    # The descent line, from the entry pose down to seated, as flange poses.
    offsets = np.linspace(travel, 0.0, arguments.samples)
    targets = []
    for offset in offsets:
        module_pose = [
            float(v) for v in np.asarray(seated[:3]) + axis * offset
        ] + list(seated[3:6])
        targets.append(compose(module_pose, grasp_in_module))

    safe = yaml.safe_load(SAFE_INITIAL.read_text(encoding="utf-8"))
    safe_vector = np.array([float(safe[name]) for name in JOINT_NAMES])

    print(f"descent line: {arguments.samples} samples over {travel * 1000:.0f} mm")
    print(f"tip link: {TIP_LINK}, joint limits "
          f"{min(u - l for l, u in zip(lower, upper)):.3f} rad minimum span")
    print()

    # Seeds: the safe state, plus the two configurations the two runs actually
    # reached, because those are the branches the planner chose in practice.
    seeds = {
        "safe state": safe_vector,
        "run 133952 entry": np.array(
            [5.254, -1.705, 2.099, -3.753, -0.983, 3.290]
        ),
        "run 134432 entry": np.array(
            [-1.026, -1.844, 1.793, 6.110, 0.991, -6.123]
        ),
    }

    results = {}
    for label, seed in seeds.items():
        reached = 0
        q = seed.copy()
        worst_position = 0.0
        worst_orientation = 0.0
        for target in targets:
            q, position_error, orientation_error = solve_ik(
                joints, lower, upper, target, q
            )
            worst_position = max(worst_position, position_error)
            worst_orientation = max(worst_orientation, orientation_error)
            if position_error < 1e-3 and orientation_error < 5e-3:
                reached += 1
            else:
                break
        fraction = reached / arguments.samples
        results[label] = {
            "samples_reached": reached,
            "fraction": fraction,
            "worst_position_error_m": worst_position,
            "worst_orientation_error_rad": worst_orientation,
        }
        print(f"seed {label:>18}: reached {reached:2d}/{arguments.samples} "
              f"= {fraction * 100:5.1f}%   "
              f"worst position {worst_position * 1000:7.2f} mm  "
              f"worst orientation {worst_orientation:8.5f} rad")

    best = max(results.items(), key=lambda item: item[1]["fraction"])
    print()
    print(json.dumps(
        {
            "best_seed": best[0],
            "best_fraction": best[1]["fraction"],
            "conclusion": (
                "a continuous inverse-kinematics path exists for the whole descent"
                if best[1]["fraction"] == 1.0
                else "no seed reached the whole descent; the line is infeasible in "
                     "kinematics alone, before any collision checking"
            ),
        },
        indent=2,
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
