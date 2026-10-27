"""Grasp pose derivation: the collision of 2026-10-26 must not be repeatable.

The arm drove into the workpiece during ``grasp`` because the stored pre_grasp /
grasp are constants derived from the 2026-10-24 workpiece pose, and nothing checks
whether today's workpiece is still there.  These tests pin the derivation formula
so a corrected pose can be produced from a measured one.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"


@pytest.fixture(scope="module")
def graspmod():
    spec = importlib.util.spec_from_file_location(
        "grasp_from_measurement", SCRIPTS / "grasp_from_measurement.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["grasp_from_measurement"] = module
    spec.loader.exec_module(module)
    return module


def test_formula_reproduces_the_proven_config_values(graspmod):
    pre, grasp = graspmod.poses_from_workpiece(
        graspmod.REFERENCE_WORKPIECE_XYZ, graspmod.REFERENCE_WORKPIECE_QUAT)
    # The part-frame constants are stored to 0.1 mm, so allow that rounding.
    assert np.allclose(grasp, graspmod.REFERENCE_GRASP, atol=3e-4)
    assert np.allclose(pre, graspmod.REFERENCE_PRE_GRASP, atol=3e-4)


def test_part_point_is_the_documented_cad_grasp_point(graspmod):
    # The handoff records the part-frame grasp point as (-0.3235, 0.0829, 0.1793).
    assert np.allclose(graspmod.part_point(), [-0.3235, 0.0829, 0.1793], atol=5e-5)
    assert np.linalg.norm(graspmod.approach_offset_part()) == pytest.approx(0.120, abs=2e-4)


def test_translating_the_workpiece_translates_the_grasp_point(graspmod):
    shift = np.array([0.0, 0.020, 0.0])  # 20 mm "to the right"
    pre, grasp = graspmod.poses_from_workpiece(
        graspmod.REFERENCE_WORKPIECE_XYZ + shift, graspmod.REFERENCE_WORKPIECE_QUAT)
    assert np.allclose(grasp - graspmod.REFERENCE_GRASP, shift, atol=3e-4)
    assert np.allclose(pre - graspmod.REFERENCE_PRE_GRASP, shift, atol=3e-4)


def test_rotating_the_workpiece_rotates_the_approach_offset(graspmod):
    from scipy.spatial.transform import Rotation
    turn = Rotation.from_euler("z", 10.0, degrees=True)
    quat = (turn * Rotation.from_quat(graspmod.REFERENCE_WORKPIECE_QUAT)).as_quat()
    pre, grasp = graspmod.poses_from_workpiece(
        graspmod.REFERENCE_WORKPIECE_XYZ, quat)
    offset = grasp - pre
    assert np.linalg.norm(offset) == pytest.approx(0.120, abs=2e-4)
    expected = turn.as_matrix() @ (graspmod.REFERENCE_GRASP - graspmod.REFERENCE_PRE_GRASP)
    assert np.allclose(offset, expected, atol=3e-4)


def test_drift_is_zero_at_the_configured_reference(graspmod):
    # The reference lives in config/tasks/common.yaml (workpiece_reference); it is
    # updated whenever the part is re-placed and the grasp poses re-derived.
    reference_xyz, reference_quat = graspmod.reference_pose()
    mm, deg = graspmod.drift(reference_xyz, reference_quat)
    assert mm == pytest.approx(0.0, abs=1e-6)
    assert deg == pytest.approx(0.0, abs=1e-6)
    mm, _ = graspmod.drift(reference_xyz + np.array([0, 0.02, 0]), reference_quat)
    assert mm == pytest.approx(20.0, abs=1e-3)


def test_check_mode_refuses_a_moved_workpiece(graspmod, capsys):
    argv = ["--workpiece-xyz", "0.5029", "0.6551", "0.1254",
            "--workpiece-quat", "0.1910", "0.9816", "0.0019", "-0.0068",
            "--check", "--tol-mm", "5"]
    old = sys.argv
    sys.argv = ["grasp_from_measurement.py", *argv]
    try:
        assert graspmod.main() == 1
    finally:
        sys.argv = old
    assert "REFUSE" in capsys.readouterr().out
