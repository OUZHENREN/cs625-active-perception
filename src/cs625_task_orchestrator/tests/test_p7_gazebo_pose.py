import math

import pytest

from cs625_task_orchestrator.p7_gazebo_pose import parse_gz_model_pose


SAMPLE = """Requesting state for world [p7_ycb_tomato_light]...\n\nModel: [8]\n\n- Name: target_object\n\n- Pose [ XYZ (m) ] [ RPY (rad) ]:\n\n[0.719173 | 0.001108 | 0.120095]\n\n[-0.001445 | -0.000335 | -0.006715]\n"""

HARMONIC_8_SAMPLE = """Requesting state for world [p7_ycb_tomato_light]...\n\nModel: [12]\n  - Name: target_object\n  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [0.720159 0.001238 0.000108]\n    [-0.001501 -0.000621 0.001488]\n"""


def test_documented_gz_model_pose_format_is_parsed():
    pose = parse_gz_model_pose(SAMPLE, "target_object")
    assert pose["position"] == pytest.approx(
        {"x": 0.719173, "y": 0.001108, "z": 0.120095}
    )
    quaternion = pose["orientation"]
    assert math.sqrt(sum(value * value for value in quaternion.values())) == pytest.approx(1.0)


def test_wrong_model_identity_is_rejected():
    with pytest.raises(ValueError, match="does not identify"):
        parse_gz_model_pose(SAMPLE, "other_object")


def test_harmonic_8_space_separated_pose_format_is_parsed():
    pose = parse_gz_model_pose(HARMONIC_8_SAMPLE, "target_object")
    assert pose["position"] == pytest.approx(
        {"x": 0.720159, "y": 0.001238, "z": 0.000108}
    )
