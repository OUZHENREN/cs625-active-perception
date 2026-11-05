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


def test_view_pose_spread_limits(rvs):
    # S0 is a slot coarse view and stays unlimited, but S2 (T2) must be self-consistent:
    # 2026-11-05 three frames taken from a STATIC camera disagreed by 920 mm / 180 deg and
    # the client still reported one of them as the answer.
    assert rvs.spread_violation("T0", 500.0, 90.0) is None
    assert rvs.spread_violation("T2", 30.0, 4.0) is None
    assert "SPREAD_TOO_LARGE" in rvs.spread_violation("T2", 500.0, 90.0)


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


# --- frame convention lock (2026-11-03 flange/TCP confusion) --------------------

def test_view_pose_frame_is_declared_and_valid(teach):
    """The RVS view-pose frame lives in ONE place and is a known value.

    Confusing RVS's flange-referenced calibration with its TCP-referenced output
    cost a 353 mm error that took two days to notice.  This test fails the moment
    the config drifts or a new value appears.
    """
    frame = teach.view_pose_frame()
    assert frame in teach.VALID_VIEW_FRAMES
    assert frame == "tcp", (
        "2026-11-03 evidence: standing at S0-as-TCP made T1 match in 3.5-4.2 s "
        "(3 frames, 0.2 mm); treating it as a flange pose puts the camera 0.23 m "
        "from the slot, outside the 0.5-0.7 m working distance.")


def test_view_pose_to_tcp_is_identity_for_tcp(teach):
    xyz, quat = teach.view_pose_to_tcp([1.0785, 0.0101, 0.6172],
                                      [-0.0404, -0.9275, 0.0572, -0.3671])
    assert xyz == [1.0785, 0.0101, 0.6172]
    assert quat == [-0.0404, -0.9275, 0.0572, -0.3671]


def test_flange_conversion_shifts_353mm(teach):
    """Even when not in force, the conversion must be the documented 353 mm."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    rotation = R.from_quat([0.0, 0.0, 0.0, 1.0]).as_matrix()
    shifted = np.array([0.0, 0.0, 0.0]) + rotation @ np.array(teach.FLANGE_TO_TCP)
    np.testing.assert_allclose(np.linalg.norm(shifted), 0.356, atol=0.005)


# --- slot attitude ambiguity (2026-11-03) ---------------------------------------

def test_slot_ambiguity_picks_the_reference_aligned_candidate(teach):
    """A 180-deg-flipped R1 must be corrected, an aligned R1 must be kept.

    The senior's own RVS projects contain both variants for the same fixed slot
    (rpy ~180 and rpy ~0), i.e. the template match is ambiguous.  Seating the part
    needs exactly one; the proven six-mode insert attitude decides which.
    """
    reference_quat = teach.REFERENCE_INSERT_QUAT
    aligned_quat, label = teach.resolve_slot_ambiguity([1.2466, 0.1430, 0.4468],
                                                       [0.0, 0.0, 0.0, 1.0])
    assert label == "as-is", (
        "an identity-attitude R1 is 1.3 deg from the reference and must be kept")
    flipped_q = __import__("scipy.spatial.transform", fromlist=["Rotation"]).Rotation.from_quat(
        [0.0, 0.0, 0.0, 1.0]) * __import__("scipy.spatial.transform", fromlist=["Rotation"]).Rotation.from_euler(
        "z", 180.0, degrees=True)
    corrected_quat, label2 = teach.resolve_slot_ambiguity([1.2466, 0.1430, 0.4468],
                                                          list(flipped_q.as_quat()))
    assert label2 == "flipped+180"
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    assert np.degrees((R.from_quat(corrected_quat).inv()
                       * R.from_quat(aligned_quat)).magnitude()) < 1e-6
    assert teach.REFERENCE_INSERT_QUAT == reference_quat


# --- load-adaptive effort thresholds (2026-11-03, owner corrections) ------------

def test_effort_thresholds_are_margins_over_the_live_baseline():
    """Absolute |effort| cutoffs trip falsely as soon as the payload changes.

    Owner data: empty pre-grasp baseline 16.001 -> confirmed cutoffs
    16.68/17.63 (margins 0.68/1.63); carrying the 6.8 kg workpiece the baseline
    measured 19.7, where the old absolute cutoffs would fire at motion start.
    """
    mod = _load("compliant_insert_test", REPO / "scripts" / "compliant_insert.py")
    monitor = object.__new__(mod.Monitor)          # no ROS in this test
    monitor.contact_margin = 0.68
    monitor.safety_margin = 1.63
    monitor.slow = monitor.fast = 0.0
    monitor.seeded = False

    monitor._apply_baseline(16.001)
    assert monitor.contact == pytest.approx(16.681, abs=1e-9)
    assert monitor.safety == pytest.approx(17.631, abs=1e-9)

    monitor._apply_baseline(19.7)
    assert monitor.contact == pytest.approx(20.38, abs=1e-9)
    assert monitor.safety == pytest.approx(21.33, abs=1e-9)
    assert monitor.seeded


# --- coarse-match drift gate (2026-11-03 RVS forensics) -------------------------

def test_view_drift_matches_session_bias(teach):
    """The same fixed slot's coarse output drifted 39 mm between sessions.

    Historical replay: 10-30's T0 sits 0.5 mm from the taught S0 (pass);
    2026-11-03's T0 sits 39.4 mm (must trip the >20 mm refusal).  The refusal
    itself lives at the S0 call site in the sequence.
    """
    ok = teach.view_drift_mm("S0", [1.0841, 0.0490, 0.6145],
                             [-0.0266, -0.9304, 0.0535, -0.3617])
    bad = teach.view_drift_mm("S0", [1.0785, 0.0101, 0.6172],
                              [-0.0404, -0.9275, 0.0572, -0.3671])
    assert ok < 1.0
    assert bad > 20.0
    # unknown command -> no reference -> None (not a crash)
    assert teach.view_drift_mm("S9", [0, 0, 0], [0, 0, 0, 1]) is None
