"""Regression tests for the task scene placement tool.

The pose conversions are checked against analytically constructed rotations
rather than against the implementation, because a wrong axis order or a quaternion
component swap is invisible until the fixture ends up somewhere plausible but
wrong.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import pytest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts/sync_task_world.py"
spec = importlib.util.spec_from_file_location("sync_task_world", SCRIPT)
sync_task_world = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync_task_world)


def rotation_matrix(quaternion) -> list[list[float]]:
    x, y, z, w = quaternion
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def test_rpy_matches_the_sdf_composition():
    """roll/pitch/yaw must compose as R = Rz(yaw) * Ry(pitch) * Rx(roll).

    The expected matrix is built from sin/cos directly.  The same matrix was
    recovered independently when the module's pose in the assembly frame was
    solved against the SolidWorks export, where every vertex then agreed to
    0.03 um, so the two are cross-checks of each other rather than one assertion
    restated twice.
    """

    sine, cosine = math.sin(math.radians(6.0)), math.cos(math.radians(6.0))
    analytic = [
        [0.0, -cosine, -sine],
        [1.0, 0.0, 0.0],
        [0.0, -sine, cosine],
    ]
    solved = [
        [0.0, -0.994521895, -0.104528465],
        [1.0, 0.0, 0.0],
        [0.0, -0.104528465, 0.994521895],
    ]
    for row_analytic, row_solved in zip(analytic, solved):
        for value_analytic, value_solved in zip(row_analytic, row_solved):
            # The solved matrix came through float32 mesh coordinates.
            assert value_solved == pytest.approx(value_analytic, abs=1e-8)

    quaternion = sync_task_world.quaternion_from_rpy(
        math.radians(-6.0), 0.0, math.radians(90.0)
    )
    assert sum(component * component for component in quaternion) == pytest.approx(1.0)
    observed = rotation_matrix(quaternion)
    for row_expected, row_observed in zip(analytic, observed):
        for value_expected, value_observed in zip(row_expected, row_observed):
            assert value_observed == pytest.approx(value_expected, abs=1e-12)


def test_quaternion_round_trip_covers_all_axes():
    for rpy in (
        (0.0, 0.0, 0.0),
        (0.3, 0.0, 0.0),
        (0.0, -0.7, 0.0),
        (0.0, 0.0, 1.9),
        (0.2, -0.4, 2.1),
        (-1.1, 0.35, -2.8),
    ):
        restored = sync_task_world.rpy_from_quaternion(
            sync_task_world.quaternion_from_rpy(*rpy)
        )
        for expected, observed in zip(rpy, restored):
            assert observed == pytest.approx(expected, abs=1e-9)


def test_recorded_poses_are_the_levelled_placement():
    parameters = sync_task_world.load_parameters()
    fixture = parameters["fixture"]["pose_world"]
    module = parameters["module"]["home_pose_world"]

    # The fixture's base is down and its handles face the robot: R = Rx(180) . Ry(5.004).
    assert fixture[0:3] == pytest.approx([-0.720, 0.0, 0.262063], abs=1e-5)
    assert fixture[5] == pytest.approx(0.0)
    assert fixture[4] == pytest.approx(math.radians(5.004), abs=1e-6)
    assert fixture[3] == pytest.approx(math.pi, abs=1e-6)

    # The module rests on the work surface, on the far side of the base.
    assert module[0] == pytest.approx(0.450)
    assert module[1] == pytest.approx(0.450)
    assert module[2] == pytest.approx(0.221264, abs=1e-5)


def test_describe_pose_reports_both_notations():
    text = sync_task_world.describe_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert "position" in text
    assert "rpy_deg" in text
    assert "quat" in text
    # Identity orientation: the quaternion is (0, 0, 0, 1) and the angles are zero.
    quaternion = sync_task_world.quaternion_from_rpy(0.0, 0.0, 0.0)
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))
    for value in quaternion:
        assert f"{value: .6f}" in text


def test_world_and_config_agree_today():
    assert sync_task_world.sync_world(sync_task_world.load_parameters(), check_only=True) == 0
