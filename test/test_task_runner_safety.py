"""Regression tests for the safety holes found on the real cell (2026-10-26).

Pure Python only: no ROS, no motion, no hardware.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"


def load_module(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def runner():
    return load_module("rpp_safety", SCRIPTS / "run_pick_and_place.py")


@pytest.fixture(scope="module")
def health():
    return load_module("control_health_under_test", SCRIPTS / "control_health.py")


@pytest.fixture(scope="module")
def run_task():
    return load_module("run_task_safety", SCRIPTS / "run_task.py")


# --------------------------------------------------------------- chunk count
def test_chunk_count_never_repeats_a_segment(runner):
    # The old formula with 3 points and 6 chunks produced [1, 2, 1, 1, 2, 1].
    assert runner.chunk_count(3, 6) == 2
    assert runner.chunk_count(2, 7) == 1
    assert runner.chunk_count(30, 6) == 6


def test_chunk_count_cannot_execute_a_point(runner):
    assert runner.chunk_count(1, 5) == 0
    assert runner.chunk_count(0, 5) == 0
    assert runner.chunk_count(30, 0) == 0


# ----------------------------------------------------- controller state parse
def test_controller_state_does_not_confuse_inactive_with_active(health):
    # This is the exact line the driver printed while arm_controller was inactive
    # and the old substring test still reported "healthy".
    text = "arm_controller joint_trajectory_controller/JointTrajectoryController inactive\n"
    assert health.controller_state(text) == "inactive"
    text = "arm_controller joint_trajectory_controller/JointTrajectoryController  active  \n"
    assert health.controller_state(text) == "active"
    assert health.controller_state("") is None
    assert health.controller_state("other_controller x active\n") is None


# ------------------------------------------------------------- config checks
def test_phase_speeds_reject_zero_and_oversized_chunks(runner):
    problems = runner.phase_speed_problems(
        {"insert": {"velocity": 0.005, "chunks": 0},
         "transit": {"velocity": 0.05, "chunks": runner.MAX_CHUNKS + 1}},
        "safe")
    assert any("insert.chunks 0" in item for item in problems)
    assert any("transit.chunks" in item for item in problems)


def test_validate_rejects_a_loose_joint_goal_tolerance(runner, run_task):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    mode_cfg = yaml.safe_load(
        (REPO / "config/tasks/load_efficient.yaml").read_text(encoding="utf-8"))
    merged = run_task.merge_configs(common, mode_cfg)
    merged["goal_joint_tolerance"] = 0.5
    path = REPO / "runtime" / "test_loose_tolerance.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(merged), encoding="utf-8")
    try:
        with pytest.raises(SystemExit):
            runner.load_config(str(path))
    finally:
        path.unlink(missing_ok=True)


def test_grip_current_comes_from_the_config(runner, run_task, tmp_path):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    mode_cfg = yaml.safe_load(
        (REPO / "config/tasks/load_efficient.yaml").read_text(encoding="utf-8"))
    merged = run_task.merge_configs(common, mode_cfg)
    merged["grip_current"] = 0.35
    path = tmp_path / "effective.yaml"
    path.write_text(yaml.safe_dump(merged), encoding="utf-8")
    assert runner.load_config(str(path)).grip_current == pytest.approx(0.35)


def test_grip_current_has_a_documented_default(runner, run_task, tmp_path):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    mode_cfg = yaml.safe_load(
        (REPO / "config/tasks/load_efficient.yaml").read_text(encoding="utf-8"))
    merged = run_task.merge_configs(common, mode_cfg)
    merged.pop("grip_current", None)
    path = tmp_path / "effective.yaml"
    path.write_text(yaml.safe_dump(merged), encoding="utf-8")
    assert runner.load_config(str(path)).grip_current == pytest.approx(0.1)


# ------------------------------------------------------------ dry-run verdict
def test_dry_run_reports_the_gate_difference():
    """``--dry-run`` must not return 0 when the gate rejects (runbook step 3)."""
    source = (SCRIPTS / "run_task.py").read_text(encoding="utf-8")
    assert "return 0 if ok else 2" in source
