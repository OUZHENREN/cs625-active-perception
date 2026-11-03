"""Pure-logic tests for scripts/attach_workpiece.py (专项 2, 2026-11-02).

No ROS, no motion: attach-pose and release-pose composition only, cross-checked
against scripts/teaching/pose_math.py (which was validated against the senior's
tcp_server output to 1 mm).
"""

from __future__ import annotations

import importlib.util
import pathlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location(
        "attach_workpiece", REPO / "scripts" / "attach_workpiece.py")
    module = importlib.util.module_from_spec(spec)
    sys_mods = __import__("sys")
    sys_mods.modules["attach_workpiece"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def pose_math():
    spec = importlib.util.spec_from_file_location(
        "pose_math", REPO / "scripts" / "teaching" / "pose_math.py")
    module = importlib.util.module_from_spec(spec)
    __import__("sys").modules["pose_math"] = module
    spec.loader.exec_module(module)
    return module


def test_attach_pose_matches_inverse_T_b_tcp(module, pose_math):
    xyz, quat = module.attach_pose()
    expected = np.linalg.inv(pose_math.T_B_TCP)
    np.testing.assert_allclose(xyz, expected[:3, 3], atol=1e-9)
    np.testing.assert_allclose(
        R.from_quat(quat).as_matrix(), expected[:3, :3], atol=1e-9)
    # sanity numbers: diag(-1,1,-1) translation inversion
    np.testing.assert_allclose(xyz, [-0.004, 0.215, -0.411], atol=1e-9)


def test_release_world_pose_identity_tcp(module):
    xyz, quat = module.release_world_pose([1.0, 2.0, 3.0], [0, 0, 0, 1.0])
    np.testing.assert_allclose(xyz, [0.996, 2.215, 2.589], atol=1e-9)
    np.testing.assert_allclose(
        R.from_quat(quat).as_matrix(), np.diag([-1, 1, -1]), atol=1e-9)


def test_round_trip_grasp_attach(module, pose_math):
    """box -> tcp via T_b_tcp must invert the attach pose exactly."""
    box_xyz = [0.6146, 0.5763, 0.3542]
    box_quat = [0.0062, -0.0026, 0.7346, 0.6785]
    grasp, _ = pose_math.grasp_from_box(box_xyz, box_quat)
    attach_xyz, attach_quat = module.attach_pose()
    # box_world = tcp_world @ T_tcp_box
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(grasp[1]).as_matrix()
    matrix[:3, 3] = grasp[0]
    t_tcp_box = np.eye(4)
    t_tcp_box[:3, :3] = R.from_quat(attach_quat).as_matrix()
    t_tcp_box[:3, 3] = attach_xyz
    world_box = matrix @ t_tcp_box
    np.testing.assert_allclose(world_box[:3, 3], box_xyz, atol=1e-9)
    np.testing.assert_allclose(
        R.from_matrix(world_box[:3, :3]).as_matrix(),
        R.from_quat(box_quat).as_matrix(), atol=1e-6)
