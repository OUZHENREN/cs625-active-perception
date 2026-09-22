"""Regression tests for the shielding module grasp template.

The template is derived from measured geometry and then verified against the two
real meshes, so the tests pin the derivation rules and the measurement plumbing
rather than the numbers.  Two of them exist because a wrong version of this code
looked plausible: indexing the triangle axis instead of the vertex axis moved the
reported tool span from 295 mm to 414 mm, and a one-dimensional band argument
predicted contact at a depth where the meshes are 36 mm apart.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import struct

import numpy as np
import pytest
from scipy.spatial import cKDTree


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts/grasp_template.py"
spec = importlib.util.spec_from_file_location("grasp_template", SCRIPT)
grasp_template = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp_template)


@pytest.fixture(scope="module")
def assets():
    parameters = grasp_template.load_parameters()
    module = grasp_template.read_stl(grasp_template.MODULE_MESH)
    gripper = grasp_template.read_stl(grasp_template.GRIPPER_MESH)
    return parameters, module, gripper


def test_grasp_rotation_aligns_the_tool_axis_with_the_approach(assets):
    rotation = grasp_template.GRIPPER_TO_MODULE_ROTATION
    assert np.linalg.det(rotation) == pytest.approx(1.0)
    # The arms run along the gripper's +Z and must point along the module's +Y,
    # because the gripper enters from the module's -Y side.
    assert (rotation @ [0.0, 0.0, 1.0]) == pytest.approx([0.0, 1.0, 0.0])
    # The arm separation axis must coincide with the module's X so that the two
    # arms meet the two handle faces.
    assert (rotation @ [1.0, 0.0, 0.0]) == pytest.approx([1.0, 0.0, 0.0])


def test_translation_is_set_by_the_three_measured_invariants(assets):
    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    grasp = parameters["grasp"]
    low, high = grasp["contact_face_x_m"]
    handle_low, handle_high = grasp["handle_y_range_m"]
    extremes = template["extremes_m"]

    # x: midway between the two contact faces.
    assert template["translation_module_m"][0] == pytest.approx((low + high) / 2.0)
    # z: on the contact centre height.
    assert template["translation_module_m"][2] == pytest.approx(
        grasp["contact_centre_z_m"]
    )
    # y: the arms' extent along the tool axis is centred on the handles' extent.
    arm_centre = (extremes["arm_low"] + extremes["arm_high"]) / 2.0
    assert template["translation_module_m"][1] == pytest.approx(
        (handle_low + handle_high) / 2.0 - arm_centre
    )


def test_arm_extents_come_from_every_vertex_not_one_per_triangle(assets):
    """The tool span is the check that caught the triangle-axis indexing bug."""

    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    vertices = gripper.reshape(-1, 3)
    assert template["extremes_m"]["tool_span_m"] == pytest.approx(
        2.0 * np.abs(vertices[:, 0]).max(), abs=1e-9
    )
    # 295 mm is the real tool; one vertex per triangle reports 414 mm.
    assert 0.29 < template["extremes_m"]["tool_span_m"] < 0.30
    assert template["extremes_m"]["arm_low"] == pytest.approx(
        vertices[np.abs(vertices[:, 0]) > grasp_template.ARM_X_THRESHOLD_M][:, 2].min()
    )


def test_arm_sequence_spans_the_handle_gap(assets):
    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    arm = template["arm"]
    assert arm["retracted_span_m"] < parameters["grasp"]["handle_inner_span_m"]
    assert arm["contact_span_m"] == pytest.approx(
        parameters["grasp"]["handle_inner_span_m"]
    )
    # Zero interference was chosen deliberately, so the contact span must equal the
    # handle span exactly and the command must be the half-travel.
    assert parameters["grasp"]["interference_fit"] == "none_zero_preload"
    assert arm["contact_command_m"] == pytest.approx(
        (arm["contact_span_m"] - arm["retracted_span_m"]) / 2.0
    )


def test_grasp_pose_does_not_penetrate_and_the_approach_is_reported(assets):
    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    translation = np.array(template["translation_module_m"])
    tree = cKDTree(module.reshape(-1, 3))
    profile = grasp_template.clearance_profile(
        tree, gripper.reshape(-1, 3), translation, np.arange(0.0, 0.161, 0.010)
    )

    assert profile[0]["minimum_clearance_m"] > 0.0005, "the grasp pose penetrates"
    # Every standoff is reported, including the one that grazes, so the pessimistic
    # approach figure cannot be silently dropped.
    assert len(profile) == 17
    worst = min(entry["minimum_clearance_m"] for entry in profile)
    assert worst < 0.001, "the recorded graze should still be present to be reported"

    # Retracting the arms along the tool X is what the real approach does, and it is
    # what removes the graze.  If this ever stops holding, the approach is unsafe.
    retracted = grasp_template.clearance_profile(
        tree,
        grasp_template.retract_arms(
            gripper.reshape(-1, 3), template["arm"]["contact_command_m"]
        ),
        translation,
        np.arange(0.0, 0.161, 0.010),
    )
    worst_retracted = min(entry["minimum_clearance_m"] for entry in retracted)
    assert worst_retracted > 0.001, "the retracted approach still grazes"
    assert worst_retracted > worst


def test_read_stl_rejects_a_truncated_file(tmp_path):
    path = tmp_path / "truncated.stl"
    path.write_bytes(b"\x00" * 80 + struct.pack("<I", 1000) + b"\x00" * 50)
    with pytest.raises(ValueError):
        grasp_template.read_stl(path)


def test_world_poses_lift_along_world_z(assets):
    """The lift is along the WORLD Z, not along the grasp frame.

    Composing it as a child pose moves the flange along the grasp frame's Z, which
    points along the module's +Y, so the module would translate sideways and barely
    rise at all.  That version looked plausible in the config.
    """

    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    poses = grasp_template.world_poses(parameters, template)
    grasp, lift = poses["grasp_world"], poses["lift_world"]
    assert lift[2] - grasp[2] == pytest.approx(template["lift"]["lift_m"])
    assert lift[0] == pytest.approx(grasp[0])
    assert lift[1] == pytest.approx(grasp[1])


def test_pre_grasp_retreats_along_the_module_approach_axis(assets):
    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    poses = grasp_template.world_poses(parameters, template)
    grasp = np.array(poses["grasp_world"][:3])
    pregrasp = np.array(poses["pregrasp_world"][:3])
    home = parameters["module"]["home_pose_world"]

    from scipy.spatial.transform import Rotation

    module_rotation = Rotation.from_euler("xyz", home[3:6]).as_matrix()
    # The pre-grasp sits a standoff BACK along the approach axis, so travelling from
    # the pre-grasp to the grasp is the opposite direction to that axis.
    advance = -module_rotation @ np.array(template["approach"]["axis_module"])
    displacement = grasp - pregrasp
    assert np.linalg.norm(displacement) == pytest.approx(
        template["approach"]["standoff_m"], abs=1e-6
    )
    assert displacement / np.linalg.norm(displacement) == pytest.approx(advance, abs=1e-6)


def test_world_poses_orientation_matches_the_module_frame_transform(assets):
    parameters, module, gripper = assets
    template = grasp_template.derive(parameters, module, gripper)
    poses = grasp_template.world_poses(parameters, template)
    # The grasp and lift share the grasp orientation; the pre-grasp too, because
    # the approach is a pure translation.
    for key in ("pregrasp_world", "lift_world"):
        assert poses[key][3:6] == pytest.approx(poses["grasp_world"][3:6])
