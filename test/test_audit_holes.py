"""Regression tests for the 7 audit holes (2026-11-02 review).

Covers the two holes that previously had code but no evidence:

  hole 2  capture validity / freshness  -> teaching_sequence.validate_capture
  hole 6  frame-spread judgement        -> rvs_pose_client.spread_violation

plus the configuration shape of holes 1 and 4 (locked planner wiring and the
segmented/carried sets).  No ROS runtime is needed: the modules are imported
directly, and no node is created.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import time

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent


def _load(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def teach():
    sys.path.insert(0, str(REPO / "scripts" / "teaching"))
    return _load("teaching_sequence_test", REPO / "scripts" / "teaching" / "teaching_sequence.py")


@pytest.fixture(scope="module")
def rvs():
    sys.path.insert(0, str(REPO / "scripts" / "teaching"))
    return _load("rvs_pose_client_test", REPO / "scripts" / "teaching" / "rvs_pose_client.py")


def _record(position, orientation=(0.0, 0.0, 0.0, 1.0), stamp=None):
    return {
        "command": "T3",
        "stamp": stamp or time.strftime("%Y-%m-%dT%H:%M:%S"),
        "position": list(position),
        "orientation": list(orientation),
    }


# --- hole 2: capture validation -------------------------------------------------

def test_valid_capture_passes(teach):
    assert teach.validate_capture(_record([0.6146, 0.5763, 0.3542]), "T3") is None


def test_out_of_workspace_capture_rejected(teach):
    # the 2026-10-30 garbage S0 answer: x = 4.41 m
    reason = teach.validate_capture(_record([4.41, 0.14, 0.34]), "T0")
    assert reason and "outside the workspace" in reason


def test_implausible_z_for_target_rejected(teach):
    # the 2026-10-30 bad S2 answers sat below the table, z = -0.077
    reason = teach.validate_capture(_record([0.7844, 0.2928, -0.077]), "T3")
    assert reason and "implausible for T3" in reason


def test_stale_capture_rejected(teach):
    old = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(time.time() - 8 * 3600))
    reason = teach.validate_capture(_record([0.6146, 0.5763, 0.3542], stamp=old), "T3")
    assert reason and "old" in reason


def test_bad_quaternion_rejected(teach):
    # zero quaternion: norm 0, cannot be a rotation
    reason = teach.validate_capture(
        _record([0.6146, 0.5763, 0.3542], orientation=(0.0, 0.0, 0.0, 0.0)), "T3")
    assert reason and "quaternion" in reason


def test_missing_stamp_rejected(teach):
    record = _record([0.6146, 0.5763, 0.3542])
    record.pop("stamp")
    reason = teach.validate_capture(record, "T3")
    assert reason is not None


# --- hole 6: spread judgement ---------------------------------------------------

def test_spread_within_limit_passes(rvs):
    assert rvs.spread_violation("T3", 5.0, 0.5) is None


def test_spread_beyond_limit_rejected(rvs):
    reason = rvs.spread_violation("T3", 45.0, 0.5)
    assert reason and "SPREAD_TOO_LARGE" in reason


def test_spread_rotation_limit_rejected(rvs):
    reason = rvs.spread_violation("T1", 2.0, 5.0)
    assert reason and "SPREAD_TOO_LARGE" in reason


def test_view_poses_have_no_spread_limit(rvs):
    # S0/S2 are recomputed per trigger, so a large spread is legitimate there
    assert rvs.spread_violation("T0", 500.0, 90.0) is None
    assert rvs.spread_violation("T2", 500.0, 90.0) is None


# --- holes 1 and 4: wiring shape ------------------------------------------------

def test_move_uses_the_locked_planner(teach):
    assert teach.PLAN.name == "plan_joint_locked.py", \
        "move() must plan through the collision-checked, branch-locked planner"


def test_descent_and_carried_sets(teach):
    for step in ("pre_grasp", "grasp", "lift", "transit", "staging"):
        assert teach.SEGMENTED.get(step, 1) > 1, f"{step} must be segmented"
    assert teach.CARRIED == {"transit", "staging"}, \
        "the carried box applies to lateral transport, not to the vertical lift"


def test_envelope_check_is_wired(teach):
    assert teach.ENVELOPE_CHECK.is_file()


# --- hole 1 completion (2026-11-03): wrapped-branch sanity rules -----------------

def test_sanity_rules_reject_wrapped_branch():
    import argparse
    import numpy as np
    mod = _load("plan_joint_locked_test",
                REPO / "scripts" / "teaching" / "plan_joint_locked.py")
    args = argparse.Namespace(max_abs_joint_deg=180.0, max_joint_delta_deg=120.0)
    seed = [0.6, -2.1, -1.9, -0.7, 1.6, 0.8]
    # the RVS fine-view solution measured 2026-11-03: J1=-204, wrist_3=+332
    wrapped = np.radians([-204.4, -0.5, -107.5, 34.2, -34.4, 332.0]).tolist()
    assert mod.sanity_violation(wrapped, seed, args) is not None
    # a legitimate move must pass
    sane = np.radians([43.0, -123.0, -101.7, -45.0, 90.7, 38.4]).tolist()
    assert mod.sanity_violation(sane, seed, args) is None
