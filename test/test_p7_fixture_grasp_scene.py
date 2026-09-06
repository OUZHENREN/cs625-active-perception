"""Minimal regression for the perception-conditioned grasp scene transition."""
from pathlib import Path
import sys

import pytest
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_apply_fixture_scene import (
    estimated_center_pose, fixture_geometry_record, set_finger_target_policy,
)


def test_approach_retains_target_collision_body():
    objects = fixture_geometry_record("severe_v5", "approach")["world_collision_objects"]
    assert next(o for o in objects if o["id"] == "p7_target_contact_proxy")["operation"] == "ADD"


def test_only_finger_pairs_change_and_existing_acm_is_preserved():
    matrix = AllowedCollisionMatrix()
    matrix.entry_names = ["base_link", "p7_ground"]
    matrix.entry_values = [AllowedCollisionEntry(enabled=[False, True]),
                           AllowedCollisionEntry(enabled=[True, False])]
    matrix = set_finger_target_policy(matrix, True)
    assert matrix.entry_values[0].enabled[1] is True
    target = matrix.entry_names.index("p7_target_contact_proxy")
    assert matrix.entry_values[0].enabled[target] is False
    assert sum(matrix.entry_values[target].enabled) == 2
    matrix = set_finger_target_policy(matrix, False)
    assert not any(matrix.entry_values[target].enabled)
    assert matrix.entry_values[0].enabled[1] is True


def test_estimated_center_not_shifted_again():
    estimate = {
        "schema": "p7_2_pose_estimate_v1", "pose_frame": "world",
        "ground_truth_read": False, "full_se3_gate_pass": True,
        "quality": {"pose_valid_for_grasp": True},
        "pose": {"position_m": [0.71, 0.084, 0.051], "orientation_xyzw": [0, 0, 0, 1]},
    }
    frame, pose = estimated_center_pose(estimate)
    assert frame == "world"
    assert list(pose["position"].values()) == pytest.approx([0.71, 0.084, 0.051])
