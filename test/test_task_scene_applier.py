"""Regression tests for the CS625 insertion task scene mirror.

These run without a live move_group: they exercise the pure construction of the
collision objects and the pose conversion, which is where a mistake would
silently place the fixture somewhere plausible but wrong.
"""

from __future__ import annotations

import math
import pathlib
import sys

import pytest

sys.path.insert(
    0, str(pathlib.Path(__file__).resolve().parents[1] / "src/cs625_bringup/scripts")
)
from apply_task_scene import (  # noqa: E402
    FIXTURE_OBJECT_ID,
    derive_seated_pose,
    seated_pose_is_stale,
    MODULE_SEATED_OBJECT_ID,
    PLANNING_FRAME,
    describe,
    fixture_collision_object,
    load_task_parameters,
    rpy_to_quaternion,
    seated_module_collision_object,
)


def quaternion_to_matrix(quaternion) -> list[list[float]]:
    x, y, z, w = quaternion
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def test_roll_pitch_yaw_matches_the_sdf_composition():
    """The seated module orientation must survive the RPY -> quaternion step.

    The expectation is built analytically as Rz(90 deg) * Rx(-6 deg), which is
    the definition of the SDF pose rather than a restatement of the conversion.
    The same matrix was recovered independently when the module's pose in the
    assembly frame was solved against the exported STL, where every vertex then
    agreed to 0.03 um, so the two are cross-checks of each other.
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
            # The solved matrix came through float32 STL coordinates.
            assert value_solved == pytest.approx(value_analytic, abs=1e-8)

    quaternion = rpy_to_quaternion(math.radians(-6.0), 0.0, math.radians(90.0))
    assert sum(component * component for component in quaternion) == pytest.approx(1.0)
    observed = quaternion_to_matrix(quaternion)
    for row_expected, row_observed in zip(analytic, observed):
        for value_expected, value_observed in zip(row_expected, row_observed):
            assert value_observed == pytest.approx(value_expected, abs=1e-12)


def test_planning_frame_is_the_srdf_parent_frame():
    """The SRDF pins base_link to world, and MoveIt's octomap frame is world."""

    assert PLANNING_FRAME == "world"
    srdf = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src/cs625_bringup/config/cs625_active_perception.srdf"
    ).read_text(encoding="utf-8")
    assert 'parent_frame="world"' in srdf


def test_fixture_object_uses_the_configured_pose_and_mesh():
    parameters = load_task_parameters()
    collision_object = fixture_collision_object(parameters)

    assert collision_object.id == FIXTURE_OBJECT_ID
    assert collision_object.header.frame_id == PLANNING_FRAME
    expected = [float(value) for value in parameters["fixture"]["pose_world"]]
    pose = collision_object.mesh_poses[0]
    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(expected[:3])
    assert (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) == pytest.approx(
        rpy_to_quaternion(*expected[3:6])
    )

    mesh = collision_object.meshes[0]
    # The decimated asset, not the 150114-triangle CAD export.
    assert len(mesh.triangles) == 9006
    assert len(mesh.vertices) < len(mesh.triangles) * 3
    xs = [vertex.x for vertex in mesh.vertices]
    ys = [vertex.y for vertex in mesh.vertices]
    zs = [vertex.z for vertex in mesh.vertices]
    assert (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) == pytest.approx(
        (0.23207, 0.74500, 0.50583), abs=2e-3
    )


def test_seated_module_object_uses_the_insertion_section_not_the_home_pose():
    parameters = load_task_parameters()
    collision_object = seated_module_collision_object(parameters)

    assert collision_object.id == MODULE_SEATED_OBJECT_ID
    seated = [float(value) for value in parameters["insertion"]["seated_pose_world"]]
    home = [float(value) for value in parameters["module"]["home_pose_world"]]
    pose = collision_object.mesh_poses[0]
    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(seated[:3])
    assert [pose.position.x, pose.position.y, pose.position.z] != pytest.approx(home[:3])
    assert (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) == pytest.approx(
        rpy_to_quaternion(*seated[3:6])
    )


def test_describe_reports_one_line_per_collision_object():
    parameters = load_task_parameters()
    assert len(describe(parameters, include_seated_module=False)) == 1
    lines = describe(parameters, include_seated_module=True)
    assert len(lines) == 2
    assert any(FIXTURE_OBJECT_ID in line for line in lines)
    assert any(MODULE_SEATED_OBJECT_ID in line for line in lines)


def test_derived_seated_pose_matches_the_sync_tool():
    """The applier and sync_task_world.py must compose poses identically.

    The two derive the world seated pose independently -- the applier cannot
    import from the repository's scripts/ directory once installed -- so the
    duplication is asserted rather than trusted.
    """

    import importlib.util

    script = pathlib.Path(__file__).resolve().parents[1] / "scripts/sync_task_world.py"
    spec = importlib.util.spec_from_file_location("sync_task_world", script)
    sync_task_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sync_task_world)

    parameters = load_task_parameters()
    assert derive_seated_pose(parameters) == pytest.approx(
        sync_task_world.derive_seated_pose(parameters), abs=1e-12
    )
    assert not seated_pose_is_stale(parameters)[0]
