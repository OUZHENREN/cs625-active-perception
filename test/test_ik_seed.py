"""IK seed must be name-matched, not taken from /joint_states order.

The bug this guards: /joint_states arrives in whatever order the publisher chose
(Gazebo sorts alphabetically, the real driver uses its own order, both include the
gripper fingers).  Pairing that raw array with JOINTS makes the IK seed point at a
different arm posture, the solver returns a flipped branch, and the branch-jump
guard refuses a reachable pose.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SPEC = REPO / "scripts" / "run_pick_and_place.py"


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location("rpp_seed", SPEC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rpp_seed"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_alphabetical_state_is_reordered_to_the_arm_order(module):
    names = ["elbow_joint", "gripper_left_finger_joint", "gripper_right_finger_joint",
             "shoulder_lift_joint", "shoulder_pan_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    positions = [-2.6, 0.021285, 0.021285, -0.9, 0.56, 0.0049, 1.88, 1.45]
    seed = module.arm_seed_from_state(names, positions)
    assert seed == pytest.approx([0.56, -0.9, -2.6, 0.0049, 1.88, 1.45])
    assert len(seed) == 6


def test_real_driver_order_is_also_handled(module):
    names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert module.arm_seed_from_state(names, positions) == pytest.approx(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_missing_joint_is_an_error_not_a_silent_shift(module):
    with pytest.raises(RuntimeError) as excinfo:
        module.arm_seed_from_state(
            ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
            [0.1, 0.2, 0.3])
    assert "missing arm joints" in str(excinfo.value)


def test_gripper_joints_never_enter_the_seed(module):
    names = list(module.JOINTS) + ["gripper_left_finger_joint"]
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 9.9]
    seed = module.arm_seed_from_state(names, positions)
    assert len(seed) == 6
    assert 9.9 not in seed


def test_extra_unknown_joints_are_ignored(module):
    names = list(module.JOINTS) + ["some_other_joint"]
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 7.7]
    assert module.arm_seed_from_state(names, positions) == pytest.approx(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
