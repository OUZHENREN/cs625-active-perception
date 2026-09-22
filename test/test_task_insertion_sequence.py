"""Regression tests for the insertion task waypoints.

The sequence is pure geometry, so it is tested with hand-built poses.  Two of these
exist because a wrong version looked right: using the module's seated pose as the
assembly-to-world rotation reports the opening facing the wrong way, and leaving the
fixture mesh in the assembly frame while placing the module in the world reports a
clearance of hundreds of millimetres, which is the distance to a fixture that is not
there.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/cs625_task_orchestrator"))
from cs625_task_orchestrator import task_insertion_sequence as sequence  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    "insertion_sequence", ROOT / "scripts/insertion_sequence.py"
)
insertion_sequence = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(insertion_sequence)


IDENTITY_GRASP = {
    "rotation_gripper_to_module": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "translation_module_m": [0.1, 0.2, 0.3],
    "approach": {"axis_module": [0.0, -1.0, 0.0], "standoff_m": 0.12},
    "lift": {"lift_m": 0.08},
    "arm": {"retracted_command_m": 0.0, "contact_command_m": 0.02},
}


def test_compose_matches_a_hand_built_transform():
    parent = [1.0, 2.0, 3.0, 0.0, 0.0, math.pi / 2.0]
    child = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    # A +90 degree yaw maps x to y, so the child's 0.5 m along x becomes 0.5 m along y.
    assert sequence.compose(parent, child)[:3] == pytest.approx([1.0, 2.5, 3.0])


def test_grasp_pose_in_module_rejects_a_improper_rotation():
    broken = dict(IDENTITY_GRASP)
    broken["rotation_gripper_to_module"] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]
    with pytest.raises(ValueError):
        sequence.grasp_pose_in_module(broken)


def test_insertion_pose_moves_along_the_axis_and_keeps_orientation():
    seated = [0.0, 0.0, 0.5, 0.1, 0.2, 0.3]
    moved = sequence.insertion_pose(seated, [0.0, 0.0, 1.0], 0.25)
    assert moved[:3] == pytest.approx([0.0, 0.0, 0.75])
    assert moved[3:6] == pytest.approx(seated[3:6])
    with pytest.raises(ValueError):
        sequence.insertion_pose(seated, [0.0, 0.0, 0.0], 0.25)


def test_sequence_orders_the_steps_and_holds_the_grasp_offset():
    home = [1.0, 0.0, 0.2, 0.0, 0.0, 0.0]
    seated = [-0.5, 0.0, 0.3, 0.0, 0.0, 0.0]
    result = sequence.build_sequence(
        module_home_world=home,
        seated_module_world=seated,
        grasp_template=IDENTITY_GRASP,
        insertion_axis_world=[0.0, 0.0, 1.0],
        withdrawal_travel_m=0.44,
        approach_margin_m=0.02,
    )
    assert result["order"][0] == "pregrasp_module"
    assert "TRANSFER_FREE_SPACE" in result["order"]
    assert result["order"].index("insert_seated") > result["order"].index("insert_entry")
    # Every flange waypoint is the module pose plus the same module-frame offset.
    for name, flange in result["waypoints_world"].items():
        module = result["module_poses_world"][
            {"pregrasp_module": "pregrasp", "grasp_module": "home",
             "lift_module": "lift", "insert_entry": "insert_entry",
             "insert_seated": "seated", "release_retreat": "insert_entry"}[name]
        ]
        expected = np.asarray(module[:3]) + np.asarray(IDENTITY_GRASP["translation_module_m"])
        assert np.asarray(flange[:3]) == pytest.approx(expected)
    # The insertion line is vertical and the entry sits a margin above the travel.
    assert result["insertion_travel_m"] == pytest.approx(0.46)
    assert result["waypoints_world"]["insert_seated"][2] == pytest.approx(
        seated[2] + IDENTITY_GRASP["translation_module_m"][2]
    )
    # Arms retract while approaching and after release, and close around the grasp.
    arms = result["arm_command_m"]
    assert arms["pregrasp_module"] == arms["grasp_module"] == 0.0
    assert arms["close_arms"] == arms["lift_module"] == 0.02
    assert arms["open_arms"] == arms["release_retreat"] == 0.0


def test_sequence_refuses_an_approach_axis_it_does_not_implement():
    broken = dict(IDENTITY_GRASP)
    broken["approach"] = {"axis_module": [1.0, 0.0, 0.0], "standoff_m": 0.12}
    with pytest.raises(ValueError):
        sequence.build_sequence(
            module_home_world=[1.0, 0.0, 0.2, 0.0, 0.0, 0.0],
            seated_module_world=[-0.5, 0.0, 0.3, 0.0, 0.0, 0.0],
            grasp_template=broken,
            insertion_axis_world=[0.0, 0.0, 1.0],
            withdrawal_travel_m=0.44,
            approach_margin_m=0.02,
        )


def test_measured_insertion_axis_points_out_of_the_fixture():
    """The opening is where the module protrudes, and that is worth pinning.

    The fixture's stored pose is rotated by pi about X, so the fixture frame's +Z is
    the world's -Z.  Getting that backwards previously reported the axis as
    (0.10, 0.0, -0.99) -- pointing down into the floor -- and the check below caught
    it only because the recorded clearance was absurd rather than because the sign
    was checked.
    """

    module = insertion_sequence.read_stl(insertion_sequence.MODULE_MESH).reshape(-1, 3)
    fixture = insertion_sequence.read_stl(insertion_sequence.FIXTURE_MESH).reshape(-1, 3)
    import yaml

    scene = yaml.safe_load(
        insertion_sequence.TASK_SCENE.read_text(encoding="utf-8")
    )["cs625_task_scene"]["ros__parameters"]
    measured = insertion_sequence.measure_insertion(
        module, fixture, scene["fixture"]["pose_world"]
    )
    # The module is lowered in from above, so the axis that extracts it points up.
    assert measured["axis_world"][2] > 0.9, "the insertion axis does not point upward"
    # It is the fixture's own Z, and the module protrudes at that end.
    assert measured["axis_assembly"] == pytest.approx([0.0, 0.0, -1.0])
    assert 0.0 < measured["proud_m"] < 0.05
    assert 0.40 < measured["travel_m"] < 0.60


def test_insertion_line_separates_penetration_from_precision():
    """No penetration and being executable are different questions.

    The real fixture clears the module by 0.040 mm on the full-resolution export.  An
    earlier version answered only \"does it pass a 0.5 mm threshold\" and reported
    failure, which read as \"the geometry is wrong\" when the geometry is right and the
    motion is simply not a position-controlled one.
    """

    import numpy as np

    seated = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    axis = [0.0, 0.0, 1.0]
    # A wall at x = 0.5 mm so the module's slab clears it by 0.5 mm less than its width.
    fixture = np.array([[0.0005, -0.5, 0.0], [0.0005, 0.5, 0.0], [0.0005, 0.0, 1.0]])
    module = np.array([[-0.05, -0.5, -0.01], [0.05, -0.5, 0.01], [0.0, 0.5, 0.0]])
    wide = sequence.verify_insertion_line(
        seated, axis, 0.1, module, fixture, precision_fit_threshold_m=0.001
    )
    assert not wide["penetrates"]
    assert wide["precision_fit"]
    assert wide["advisory"]

    # Move the wall far away and the same line becomes an ordinary one.
    far = fixture.copy()
    far[:, 0] = 0.5
    roomy = sequence.verify_insertion_line(
        seated, axis, 0.1, module, far, precision_fit_threshold_m=0.001
    )
    assert not roomy["penetrates"]
    assert not roomy["precision_fit"]
    assert roomy["advisory"] == ""

    # A wall the module overlaps is a penetration, which is a different verdict.
    overlapping = fixture.copy()
    overlapping[:, 0] = 0.0
    hit = sequence.verify_insertion_line(
        seated, axis, 0.1, module, overlapping, precision_fit_threshold_m=0.001
    )
    assert hit["penetrates"]
