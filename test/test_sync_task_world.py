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


HERE = pathlib.Path(__file__).resolve().parent
SCRIPT = HERE.parent / "scripts/sync_task_world.py"
sys.path.insert(0, str(HERE))
from inspect_stl_mass_properties import read_binary_stl  # noqa: E402
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


def world_bounds(model: str) -> tuple:
    """Transformed AABB of a task model's mesh, from the recorded pose."""

    import numpy as np
    from scipy.spatial.transform import Rotation

    parameters = sync_task_world.load_parameters()
    section, key = ("fixture", "pose_world") if model == "slot_fixture" else (
        "module", "home_pose_world"
    )
    pose = parameters[section][key]
    root = pathlib.Path(__file__).resolve().parents[1]
    mesh = (
        root / "src/cs625_simulation/assets/cs625_task" / model / "meshes" / f"{model}.stl"
    )
    triangles = read_binary_stl(mesh).reshape(-1, 3)
    rotation = Rotation.from_euler("xyz", pose[3:6]).as_matrix()
    points = (rotation @ triangles.T).T + np.array(pose[:3])
    return points.min(0), points.max(0), rotation


def test_recorded_placement_is_geometrically_sane():
    """The placement is data the operator chooses, so pin invariants, not numbers.

    Whatever pose is recorded, the two bodies must rest on the work surface, the
    module must stand along its own axis, and the fixture and module must not
    intersect.  Pinning the exact coordinates instead would fail every time the
    scene is moved, which is a normal thing to do.
    """

    fixture_low, fixture_high, _ = world_bounds("slot_fixture")
    module_low, module_high, module_rotation = world_bounds("shielding_module")

    # On the work surface, or a few millimetres above it.
    assert -0.005 <= fixture_low[2] <= 0.050
    assert -0.005 <= module_low[2] <= 0.050

    # The module stands upright: its own +Z within 2 degrees of world +Z.
    upright = module_rotation @ [0.0, 0.0, 1.0]
    assert math.degrees(math.acos(min(1.0, abs(upright[2])))) < 2.0

    # The two bodies do not overlap.
    overlap = [
        min(fixture_high[axis], module_high[axis])
        - max(fixture_low[axis], module_low[axis])
        for axis in range(3)
    ]
    assert min(overlap) <= 0.0, f"fixture and module intersect: {overlap}"


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
