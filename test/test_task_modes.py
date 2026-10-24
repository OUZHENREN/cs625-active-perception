"""Unit tests for the six-mode entry point and the environment gate.

These run without ROS: ``scripts/run_task.py`` and the gate part of
``scripts/model_environment.py`` import only standard library plus PyYAML.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
from datetime import datetime, timedelta, timezone

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
def run_task():
    return load_module("run_task_under_test", SCRIPTS / "run_task.py")


@pytest.fixture(scope="module")
def model_env():
    return load_module("model_environment_under_test", SCRIPTS / "model_environment.py")


@pytest.fixture(scope="module")
def merged(run_task):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    mode_cfg = yaml.safe_load(
        (REPO / "config/tasks/load_efficient.yaml").read_text(encoding="utf-8"))
    return run_task.merge_configs(common, mode_cfg)


# ------------------------------------------------------------------ scopes
def test_six_mode_paths_exist(run_task):
    for scope in run_task.SCOPES:
        for mode in run_task.MODES:
            assert run_task.mode_path(scope, mode).exists(), f"{scope}_{mode} missing"
    assert run_task.common_path().exists()


def test_scope_steps(run_task):
    assert run_task.scope_steps("load") == ["load"]
    assert run_task.scope_steps("unload") == ["unload"]
    # The full flow is load followed by unload, never merged into one runner call.
    assert run_task.scope_steps("full") == ["load", "unload"]
    assert run_task.flow_of("load") == "pick_place"
    assert run_task.flow_of("unload") == "unload"


def test_merge_keeps_world_constants(run_task, merged):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    # World constants and the gate block survive the merge untouched.
    for key in ("slot_origin", "slot_origin_q", "box_in_slot", "baseline", "baseline_q"):
        assert key in merged
        assert merged[key] == common[key]
    assert "environment" in merged and "thresholds" in merged["environment"]


def test_merge_mode_wins(run_task, merged):
    assert merged["mode"] == "efficient"
    assert float(merged["dwell_sec"]) == 0.0
    assert str(merged["scope"]) == "load"


# ------------------------------------------------------------ runner contract
def test_runner_accepts_every_effective_config(run_task, tmp_path):
    """The merge output must load through the runner's own validator."""
    spec = importlib.util.spec_from_file_location(
        "rpp_under_test", SCRIPTS / "run_pick_and_place.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["rpp_under_test"] = module
    spec.loader.exec_module(module)

    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    for scope in run_task.SCOPES:
        for mode in run_task.MODES:
            mode_cfg = yaml.safe_load(
                (REPO / f"config/tasks/{scope}_{mode}.yaml").read_text(encoding="utf-8"))
            path = tmp_path / f"{scope}_{mode}.yaml"
            path.write_text(yaml.safe_dump(run_task.merge_configs(common, mode_cfg),
                                           sort_keys=False))
            loader = (module.load_config if scope == "load"
                      else module.load_unload_config)
            loader(str(path))  # raises SystemExit when a speed cap is violated


def test_plan_only_never_passes_confirm(run_task, tmp_path, merged):
    class Args:
        plan_only = True
        confirm = True          # a plan-only request must win over --confirm
        phases = ""
        allow_release = False

    command = run_task.runner_command(tmp_path / "eff.yaml", "load", Args())
    assert "--plan-only" in command
    assert "--confirm" not in command


def test_confirm_is_passed_when_requested(run_task, tmp_path):
    class Args:
        plan_only = False
        confirm = True
        phases = ""
        allow_release = False

    command = run_task.runner_command(tmp_path / "eff.yaml", "load", Args())
    assert "--confirm" in command
    assert "--plan-only" not in command


def test_plan_only_reaches_the_unload_flow_too(run_task, tmp_path):
    class Args:
        plan_only = True
        confirm = False
        phases = ""
        allow_release = False

    command = run_task.runner_command(tmp_path / "eff.yaml", "unload", Args())
    assert "--flow" in command and "unload" in command
    assert "--plan-only" in command


# --------------------------------------------------------------- env gate
def _model(**overrides):
    model = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "frames_used": 93,
        "valid_points": 612121,
        "slot": {"xyz": [1.24, 0.18, 0.44], "quat": [0, 0, 0, 1]},
        "workpiece": {"xyz": [0.5, 0.63, 0.12], "quat": [0, 0, 0, 1]},
        "baseline_delta": {"slot_mm": 0.0, "slot_deg": 0.0,
                           "workpiece_mm": 0.0, "workpiece_deg": 0.0},
    }
    model.update(overrides)
    return model


THRESHOLDS = {"slot_mm": 1.0, "slot_deg": 0.5, "workpiece_mm": 3.0,
              "workpiece_deg": 1.0, "min_points": 20000, "max_age_h": 2.0}


def test_gate_accepts_a_fresh_model(model_env):
    now = datetime.now(timezone.utc).isoformat()
    ok, reasons = model_env.evaluate_gate(_model(), THRESHOLDS, now)
    assert ok, reasons


def test_gate_rejects_slot_shift(model_env):
    now = datetime.now(timezone.utc).isoformat()
    model = _model(baseline_delta={"slot_mm": 4.5, "slot_deg": 1.2,
                                   "workpiece_mm": 0.0, "workpiece_deg": 0.0})
    ok, reasons = model_env.evaluate_gate(model, THRESHOLDS, now)
    assert not ok
    assert any("插槽" in reason for reason in reasons)


def test_gate_rejects_workpiece_shift(model_env):
    now = datetime.now(timezone.utc).isoformat()
    model = _model(baseline_delta={"slot_mm": 0.0, "slot_deg": 0.0,
                                   "workpiece_mm": 8.0, "workpiece_deg": 2.5})
    ok, reasons = model_env.evaluate_gate(model, THRESHOLDS, now)
    assert not ok
    assert any("工件" in reason for reason in reasons)


def test_gate_rejects_thin_cloud(model_env):
    now = datetime.now(timezone.utc).isoformat()
    ok, reasons = model_env.evaluate_gate(_model(valid_points=1200), THRESHOLDS, now)
    assert not ok
    assert any("点云" in reason for reason in reasons)


def test_gate_rejects_stale_model(model_env):
    old = (datetime.now(timezone.utc) - timedelta(hours=9)).isoformat()
    ok, reasons = model_env.evaluate_gate(_model(created_at=old), THRESHOLDS,
                                          datetime.now(timezone.utc).isoformat())
    assert not ok
    assert any("过期" in reason for reason in reasons)


def test_gate_rejects_a_missing_model_file(run_task, merged, tmp_path):
    ok, reasons, model = run_task.check_environment(merged, tmp_path / "nope.json")
    assert not ok
    assert model == {}
    assert any("缺失" in reason for reason in reasons)


def test_check_environment_loads_a_real_model(run_task, merged):
    latest = run_task.latest_model_path()
    if latest is None:
        pytest.skip("no environment model built yet")
    ok, reasons, model = run_task.check_environment(merged, latest)
    # A model that is older than the 2 h window is a legitimate rejection; what
    # must hold is that the file parsed and the gate produced a verdict.
    assert isinstance(model, dict) and model
    assert isinstance(ok, bool)
    assert isinstance(reasons, list)
