"""Minimal geometry and estimator-input checks for offline grasp generation."""

from copy import deepcopy
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_generate_grasp_candidates import (  # noqa: E402
    HISTORICAL_PREGRASP_OFFSET_M,
    TEMPLATE_ORIENTATION_XYZW,
    generate_candidates,
    rotate,
)


def estimate(frame="base_link"):
    return {
        "schema": "p7_2_pose_estimate_v1", "pose_frame": frame,
        "ground_truth_read": False, "full_se3_gate_pass": True,
        "quality": {"pose_valid_for_grasp": True},
        "pose": {"position_m": [0.71, 0.083, 0.051], "orientation_xyzw": [0, 0, 0, 1]},
    }


def xyz(pose):
    return [pose["position"][a] for a in "xyz"]


def test_cad_center_is_not_offset_twice_and_historical_template_is_reused():
    result = generate_candidates(estimate())
    candidate = result["candidates"][0]
    assert xyz(candidate["grasp"]) == pytest.approx([0.71, 0.083, 0.051])
    assert xyz(candidate["pregrasp"]) == pytest.approx([
        c + d for c, d in zip([0.71, 0.083, 0.051], HISTORICAL_PREGRASP_OFFSET_M)
    ])
    assert xyz(candidate["lift"]) == pytest.approx([0.71, 0.083, 0.171])
    assert result["candidate_count"] == 3
    assert all(c["feasibility_status"] == "NOT_EVALUATED" for c in result["candidates"])


def test_object_rotation_rotates_template_offset_and_tool_orientation():
    record = estimate()
    record["pose"]["orientation_xyzw"] = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    candidate = generate_candidates(record)["candidates"][0]
    dx, dy, dz = HISTORICAL_PREGRASP_OFFSET_M
    assert xyz(candidate["pregrasp"]) == pytest.approx([0.71 - dy, 0.083 + dx, 0.051 + dz])
    q = [candidate["grasp"]["orientation"][a] for a in "xyzw"]
    old_axis = rotate((1, 0, 0), TEMPLATE_ORIENTATION_XYZW)
    assert rotate((1, 0, 0), q) == pytest.approx([-old_axis[1], old_axis[0], old_axis[2]], abs=1e-6)
    assert sum(v*v for v in q) == pytest.approx(1.0)


def test_template_yaw_changes_approach_but_not_estimated_grasp_centre():
    candidates = generate_candidates(estimate(), template_yaws_deg=(0.0, 90.0))["candidates"]
    assert len(candidates) == 6
    assert candidates[3]["candidate_id"] == "tomato_yaw90_historical_r5"
    assert xyz(candidates[3]["grasp"]) == pytest.approx(xyz(candidates[0]["grasp"]))
    dx, dy, dz = HISTORICAL_PREGRASP_OFFSET_M
    assert xyz(candidates[3]["pregrasp"]) == pytest.approx([0.71-dy, 0.083+dx, 0.051+dz])


def test_world_requires_correct_explicit_transform_direction():
    record = estimate("world")
    with pytest.raises(ValueError, match="explicit"):
        generate_candidates(record)
    tf = {
        "capture_schema": "p7_tf_pose_v1", "parent_frame": "base_link", "child_frame": "world",
        "pose": {"position": {"x": 1, "y": 2, "z": 3},
                 "orientation": {"x": 0, "y": 0, "z": math.sqrt(0.5), "w": math.sqrt(0.5)}},
    }
    result = generate_candidates(record, tf)
    assert xyz(result["candidates"][0]["grasp"]) == pytest.approx([1 - 0.083, 2 + 0.71, 3 + 0.051])
    wrong = deepcopy(tf)
    wrong["parent_frame"], wrong["child_frame"] = "world", "base_link"
    with pytest.raises(ValueError, match="direction"):
        generate_candidates(record, wrong)


def test_horizontal_side_grasp_keeps_camera_up_and_approaches_from_positive_x():
    result = generate_candidates(estimate(), level_side_yaws_deg=(180.0,))
    assert result["candidate_count"] == 5
    candidate = result["candidates"][3]
    assert candidate["candidate_id"] == "tomato_level_yaw180_standoff_1"
    assert xyz(candidate["pregrasp"]) == pytest.approx([0.86, 0.083, 0.051])
    assert xyz(candidate["grasp"]) == pytest.approx([0.71, 0.083, 0.051])
    q = [candidate["grasp"]["orientation"][a] for a in "xyzw"]
    assert rotate((1, 0, 0), q) == pytest.approx([-1, 0, 0])
    assert rotate((0, 0, 1), q) == pytest.approx([0, 0, 1])
    assert candidate["feasibility_status"] == "NOT_EVALUATED"


def test_top_grasp_uses_estimated_upper_wall_and_downward_tool_x():
    candidate = generate_candidates(estimate(), top_grasp_yaws_deg=(180.0,))["candidates"][3]
    assert candidate["candidate_id"] == "tomato_top_yaw180_standoff_1"
    assert xyz(candidate["grasp"]) == pytest.approx([0.71, 0.083, 0.081])
    assert xyz(candidate["pregrasp"]) == pytest.approx([0.71, 0.083, 0.231])
    q = [candidate["grasp"]["orientation"][a] for a in "xyzw"]
    assert rotate((1, 0, 0), q) == pytest.approx([0, 0, -1])
    assert xyz(candidate["lift"])[2] - xyz(candidate["grasp"])[2] == pytest.approx(0.12)


@pytest.mark.parametrize("change", [
    {"ground_truth_read": True}, {"ground_truth_read": None},
    {"full_se3_gate_pass": False}, {"quality": {"pose_valid_for_grasp": False}},
    {"schema": "external_ground_truth_evaluation"},
    {"pose": {"position_m": [0, float("nan"), 0], "orientation_xyzw": [0, 0, 0, 1]}},
    {"pose": {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 0]}},
])
def test_rejects_invalid_or_unaccepted_estimator_input(change):
    record = estimate()
    record.update(change)
    with pytest.raises(ValueError):
        generate_candidates(record)
