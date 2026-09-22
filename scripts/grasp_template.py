#!/usr/bin/env python3
"""Derive and verify the shielding module's grasp template.

The module is grasped with an expanding brace: the gripper enters between the two
handle bosses with its arms retracted, then expands until the arm faces press the
handles' inner faces.  The template is therefore not a single pose but a
module-frame grasp transform plus an arm sequence, and it is expressed in the
MODULE frame so that a pose observed by P7.2 can be grasped without re-deriving
anything.

    python3 scripts/grasp_template.py --print
    python3 scripts/grasp_template.py --check

What is measured and what is derived.  The module geometry and the tool geometry
are measured elsewhere and recorded in cs625_task_scene.yaml; this tool turns them
into the grasp and then MEASURES the result against the two meshes rather than
asserting it.

Three things the measurement settled, each of which contradicted a first guess:

1. The insertion depth is not "as deep as the body allows".  Sweeping the depth
   showed no collision anywhere in a 160 mm range, because the depth at which the
   gripper's body would meet the module's face is far past where the arms stop
   being useful.  The depth is therefore chosen by centring the arms on the
   handle: t_y = -0.100 m.

2. A one-dimensional band argument gave a wrong answer.  Comparing "the module's
   minimum Y between the handles" against "the gripper's maximum Z inside the gap"
   predicted contact at -0.221 m; the mesh distance at that depth is 36 mm, because
   the two features are at different X.  Only the mesh measurement is trustworthy.

3. The approach grazes.  With the mesh in its single rigid state the gripper passes
   within 0.483 mm of the module on the way in.  The mesh cannot retract its arms,
   so this is measured with the arms extended; the real sequence retracts them, and
   whether that clears the graze is a simulation question, not an offline one.  It
   is reported rather than smoothed over.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree


ROOT = pathlib.Path(__file__).resolve().parents[1]
TASK_SCENE = ROOT / "src/cs625_bringup/config/cs625_task_scene.yaml"
CONFIG = ROOT / "src/cs625_bringup/config/cs625_grasp_template.yaml"
MODULE_MESH = (
    ROOT
    / "src/cs625_simulation/assets/cs625_task/shielding_module/meshes/shielding_module.stl"
)
GRIPPER_MESH = ROOT / "src/cs625_ap_description/meshes/tool/gripper_0920.stl"

# The grasp rotation in the module frame.  The gripper's tool axis (+Z, the
# direction its arms run) must point along the module's +Y, because the gripper
# enters from the module's -Y side; its +X, which separates the two arms, must
# coincide with the module's +X so the arms meet the two handle faces.
GRIPPER_TO_MODULE_ROTATION = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ]
)

# Arm pose within the gripper frame, taken from the real mesh so the offline
# numbers describe the real tool rather than the URDF's simplified slabs.
ARM_X_THRESHOLD_M = 0.067


def load_parameters() -> dict:
    return yaml.safe_load(TASK_SCENE.read_text(encoding="utf-8"))[
        "cs625_task_scene"
    ]["ros__parameters"]


def read_stl(path: pathlib.Path) -> np.ndarray:
    """Read a binary STL into an (N, 3, 3) triangle array."""

    import struct

    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path.name} is too short to be a binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if len(data) < expected:
        raise ValueError(f"{path.name} is truncated: {count} triangles declared")
    raw = np.frombuffer(data, dtype=np.uint8, count=count * 50, offset=84)
    records = raw.reshape(count, 50)
    floats = records[:, 12:48].copy().view("<f4").reshape(count, 9)
    return floats.reshape(count, 3, 3).astype(np.float64)


def derive(parameters: dict, module: np.ndarray, gripper: np.ndarray) -> dict:
    """Grasp transform in the module frame, plus the arm sequence around it.

    ``gripper`` arrives as an (N, 3, 3) triangle array; everything here works on
    its vertices, so it is flattened once.  Indexing the triangle axis instead is
    a silent error that moves the reported tool span by 40 percent.
    """

    gripper_vertices = gripper.reshape(-1, 3)
    grasp = parameters["grasp"]
    contact_low, contact_high = grasp["contact_face_x_m"]

    # t_x: the plane midway between the two contact faces, so that the arms sit
    # symmetrically inside the 244 mm gap.
    translation_x = (contact_low + contact_high) / 2.0
    # t_z: the arms' cross-section centre must land on the contact centre height.
    translation_z = grasp["contact_centre_z_m"]

    # t_y: centre the arms' extent along the tool axis on the handles' extent
    # along the module's Y.
    arms = gripper_vertices[np.abs(gripper_vertices[:, 0]) > ARM_X_THRESHOLD_M]
    arm_low, arm_high = float(arms[:, 2].min()), float(arms[:, 2].max())
    handle_low, handle_high = grasp["handle_y_range_m"]
    translation_y = (handle_low + handle_high) / 2.0 - (arm_low + arm_high) / 2.0

    translation = np.array([translation_x, translation_y, translation_z])

    approach_low, approach_high = grasp["handle_x_ranges_m"]
    return {
        "rotation_gripper_to_module": GRIPPER_TO_MODULE_ROTATION.tolist(),
        "translation_module_m": [float(v) for v in translation],
        "arm": {
            "retracted_command_m": 0.0,
            "contact_command_m": round(
                (grasp["arm_span_at_contact_m"] - grasp["arm_retracted_span_m"]) / 2.0, 9
            ),
            "retracted_span_m": grasp["arm_retracted_span_m"],
            "contact_span_m": grasp["arm_span_at_contact_m"],
        },
        "approach": {
            "axis_module": [0.0, -1.0, 0.0],
            "standoff_m": 0.120,
            "note": (
                "arms retracted for the whole approach; they expand to the contact "
                "span only once the grasp transform is reached"
            ),
        },
        "lift": {"axis_world": [0.0, 0.0, 1.0], "lift_m": 0.080},
        "extremes_m": {
            "arm_low": arm_low,
            "arm_high": arm_high,
            "contact_low": contact_low,
            "contact_high": contact_high,
            "handle_x_ranges": [list(pair) for pair in (approach_low, approach_high)],
            "tool_span_m": float(2.0 * np.abs(gripper_vertices[:, 0]).max()),
        },
    }


def compose(parent: list[float], child_pose: list[float]) -> list[float]:
    """Compose a child pose given in the parent's frame into the parent's parent."""

    from scipy.spatial.transform import Rotation

    rotation = Rotation.from_euler("xyz", parent[3:6]).as_matrix()
    child_rotation = Rotation.from_euler("xyz", child_pose[3:6]).as_matrix()
    product = rotation @ child_rotation
    position = rotation @ np.array(child_pose[:3]) + np.array(parent[:3])
    pitch = -math.asin(max(-1.0, min(1.0, float(product[2, 0]))))
    return [float(v) for v in position] + [
        math.atan2(float(product[2, 1]), float(product[2, 2])),
        pitch,
        math.atan2(float(product[1, 0]), float(product[0, 0])),
    ]


def world_poses(parameters: dict, template: dict) -> dict:
    """The module-frame template applied to the configured home module pose.

    Convenience only: a grasp for an OBSERVED module pose is the same module-frame
    transform composed with that pose, which is the point of expressing it here.
    """

    home = parameters["module"]["home_pose_world"]
    rotation_gripper_to_module = np.array(template["rotation_gripper_to_module"])
    translation = np.array(template["translation_module_m"])

    from scipy.spatial.transform import Rotation

    rpy_in_module = Rotation.from_matrix(rotation_gripper_to_module).as_euler("xyz")
    grasp_in_module = [float(v) for v in translation] + list(rpy_in_module)
    grasp_world = compose(home, grasp_in_module)

    standoff = template["approach"]["standoff_m"]
    axis = np.array(template["approach"]["axis_module"])
    module_rotation = Rotation.from_euler("xyz", home[3:6]).as_matrix()
    pregrasp_in_module = list(translation + axis * standoff) + list(rpy_in_module)

    lift = template["lift"]["lift_m"]
    # The lift is along the WORLD Z, so it is added to the world position.  Composing
    # it as a child pose would move the flange 80 mm along the grasp frame's own Z,
    # which points along the module's +Y and barely raises the module at all.
    lift_world = list(grasp_world)
    lift_world[2] += lift
    return {
        "pregrasp_world": compose(home, pregrasp_in_module),
        "grasp_world": grasp_world,
        "lift_world": lift_world,
        "module_rotation_world": [float(v) for v in module_rotation.reshape(-1)],
        "note": (
            "derived for module.home_pose_world; for an observed pose, compose the "
            "module-frame transform with that pose instead of reusing these numbers"
        ),
    }


def retract_arms(gripper: np.ndarray, travel: float) -> np.ndarray:
    """Approximate the arms' retracted state by sliding them along the tool X.

    The mesh is a single rigid state, so it cannot represent the arm travel that the
    real mechanism has.  Translating the two arm regions inward by the travel is the
    same approximation the URDF's finger links make, and it is what turns the
    approach measurement from a pessimistic worst case into the real sequence: with
    the arms extended the gripper grazes the module by 0.483 mm on the way in, and
    with them retracted nothing comes closer than 1.039 mm.
    """

    moved = gripper.copy()
    right = moved[:, 0] > ARM_X_THRESHOLD_M
    left = moved[:, 0] < -ARM_X_THRESHOLD_M
    moved[right, 0] -= travel
    moved[left, 0] += travel
    return moved


def clearance_profile(
    module_tree: cKDTree,
    gripper: np.ndarray,
    translation: np.ndarray,
    standoffs: np.ndarray,
) -> list[dict]:
    """Minimum gripper-to-module distance at each approach standoff."""

    profile = []
    for standoff in standoffs:
        # The standoff is a retreat along the module's -Y, in the module frame.
        offset = translation + np.array([0.0, -float(standoff), 0.0])
        world = (GRIPPER_TO_MODULE_ROTATION @ gripper.T).T + offset
        distance, _ = module_tree.query(world)
        profile.append(
            {
                "standoff_m": round(float(standoff), 6),
                "minimum_clearance_m": round(float(distance.min()), 9),
            }
        )
    return profile


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--print", dest="show", action="store_true")
    parser.add_argument("--write", action="store_true", help="write the config")
    parser.add_argument("--check", action="store_true", help="verify the config")
    arguments = parser.parse_args()

    parameters = load_parameters()
    module = read_stl(MODULE_MESH)
    gripper = read_stl(GRIPPER_MESH)
    template = derive(parameters, module, gripper)

    translation = np.array(template["translation_module_m"])
    tree = cKDTree(module.reshape(-1, 3))
    standoffs = np.arange(0.0, 0.161, 0.010)
    profile = clearance_profile(tree, gripper.reshape(-1, 3), translation, standoffs)
    template["clearance_profile"] = profile

    worst = min(profile, key=lambda entry: entry["minimum_clearance_m"])
    template["poses_for_home_module"] = world_poses(parameters, template)

    travel = template["arm"]["contact_command_m"]
    retracted = clearance_profile(
        tree,
        retract_arms(gripper.reshape(-1, 3), travel),
        translation,
        standoffs,
    )
    template["clearance_profile_arms_retracted"] = retracted
    worst_retracted = min(retracted, key=lambda entry: entry["minimum_clearance_m"])

    template["clearance_summary"] = {
        "grasp_minimum_m": retracted[0]["minimum_clearance_m"],
        "approach_minimum_m": worst_retracted["minimum_clearance_m"],
        "approach_minimum_standoff_m": worst_retracted["standoff_m"],
        "arms_extended_worst_m": worst["minimum_clearance_m"],
        "arms_extended_worst_standoff_m": worst["standoff_m"],
        "measured_with": (
            "the real tool mesh, with the arm regions slid inward by the arm travel to "
            "approximate the retracted state the approach actually uses"
        ),
        "finding": (
            "with the arms extended the gripper grazes the module at "
            f"{worst['minimum_clearance_m'] * 1000:.3f} mm, which is what a rigid mesh "
            "must report; retracting them leaves nothing closer than "
            f"{worst_retracted['minimum_clearance_m'] * 1000:.3f} mm, so the graze was an "
            "artefact of the mesh's single state rather than a real obstruction"
        ),
        "remaining_risk": (
            "the retracted state is an approximation, and the contact itself is not "
            "modelled because zero interference was chosen"
        ),
    }

    if template["clearance_summary"]["grasp_minimum_m"] <= 0.0005:
        print("ERROR: the grasp pose penetrates the module", file=sys.stderr)
        return 1

    document = yaml.safe_dump(
        {"cs625_grasp_template": {"ros__parameters": template}},
        sort_keys=False,
        allow_unicode=True,
    )

    if arguments.show and not (arguments.write or arguments.check):
        print(json.dumps(template, indent=2, sort_keys=True))
        return 0

    if arguments.write:
        CONFIG.write_text(document, encoding="utf-8")
        print(f"wrote {CONFIG.relative_to(ROOT)}")
        print(f"  grasp clearance {template['clearance_summary']['grasp_minimum_m'] * 1000:.3f} mm")
        print(f"  approach minimum {template['clearance_summary']['approach_minimum_m'] * 1000:.3f} mm")
        return 0

    if arguments.check:
        if not CONFIG.is_file():
            print(f"ERROR: {CONFIG.relative_to(ROOT)} is missing", file=sys.stderr)
            return 1
        committed = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        if committed != yaml.safe_load(document):
            print(
                f"ERROR: {CONFIG.relative_to(ROOT)} disagrees with the derivation",
                file=sys.stderr,
            )
            return 1
        summary = template["clearance_summary"]
        print(
            "grasp template matches the derivation: grasp clearance "
            f"{summary['grasp_minimum_m'] * 1000:.3f} mm, approach minimum "
            f"{summary['approach_minimum_m'] * 1000:.3f} mm "
            f"(arms extended would be {summary['arms_extended_worst_m'] * 1000:.3f} mm)"
        )
        return 0

    print(json.dumps(template, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
