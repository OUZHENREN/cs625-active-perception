from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_pregrasp_path_search import normalise_solution


def test_nearest_equivalent_must_stay_inside_active_soft_limit():
    joint = JointState(name=["shoulder_pan_joint"], position=[-0.23])
    normalise_solution(joint, {"shoulder_pan_joint": -3.459},
                       {"shoulder_pan_joint": (-6.1331853, 6.1331853)})
    assert joint.position[0] == pytest.approx(-0.23)


def test_nearby_equivalent_is_kept_when_it_is_within_bounds():
    joint = JointState(name=["shoulder_pan_joint"], position=[2.9])
    normalise_solution(joint, {"shoulder_pan_joint": -3.459},
                       {"shoulder_pan_joint": (-6.1331853, 6.1331853)})
    assert joint.position[0] == pytest.approx(2.9 - 2 * 3.141592653589793)
