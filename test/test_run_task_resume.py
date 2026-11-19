"""离线测试：效率入口的续跑状态解析（2026-11-19）。

背景：示教链有 resume_teaching.sh + --resume，效率入口 run_task.py 没有；面板效率页
用的正是它，所以失败只能从头重来。补上时必须按 **index** 重试，因为
load_efficient 流程里 observe:slot_coarse 出现两次，按名字会定位错阶段。
"""

from __future__ import annotations

import importlib.util
import json
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("run_task", ROOT / "scripts" / "run_task.py")
run_task = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(run_task)


def test_resume_state_matches_and_returns_the_failed_index():
    state = {"scope": "load", "mode": "efficient", "world": "real",
             "step": "load", "failed_phase": "measure:workpiece_fine",
             "start_index": 3, "phases_total": 21}
    index, note = run_task.resolve_resume(state, "load", "efficient", "real")
    assert index == 3
    assert "measure:workpiece_fine" in note


def test_resume_refuses_a_different_scope():
    state = {"scope": "unload", "mode": "efficient", "world": "real", "start_index": 2}
    with pytest.raises(ValueError, match="scope"):
        run_task.resolve_resume(state, "load", "efficient", "real")


def test_resume_refuses_a_different_mode_or_world():
    base = {"scope": "load", "mode": "efficient", "world": "real", "start_index": 2}
    with pytest.raises(ValueError):
        run_task.resolve_resume(base, "load", "safe", "real")
    with pytest.raises(ValueError):
        run_task.resolve_resume(base, "load", "efficient", "sim")


def test_resume_refuses_an_empty_state():
    with pytest.raises(ValueError, match="没有可续跑的状态"):
        run_task.resolve_resume({}, "load", "efficient", "real")


def test_resume_refuses_a_bad_index():
    state = {"scope": "load", "mode": "efficient", "world": "real", "start_index": -1}
    with pytest.raises(ValueError, match="start_index"):
        run_task.resolve_resume(state, "load", "efficient", "real")


def test_failed_phase_from_log_uses_the_recorded_index(tmp_path):
    """日志里带 index 时必须用它，而不是按名字猜（重名阶段会错位）。"""
    log = tmp_path / "phase_log_load.json"
    log.write_text(json.dumps([
        {"phase": "observe:workpiece_coarse_efficient", "ok": True, "index": 0},
        {"phase": "observe:slot_coarse", "ok": True, "index": 5},
        {"phase": "measure:workpiece_fine", "ok": False, "index": 3},
    ]), encoding="utf-8")
    name, index, total = run_task.failed_phase_from_log(log)
    assert name == "measure:workpiece_fine"
    assert index == 3          # 记录值，而不是"最后一个 observe:slot_coarse"的 5
    assert total == 3


def test_failed_phase_from_log_without_a_failure_returns_zero(tmp_path):
    log = tmp_path / "phase_log_load.json"
    log.write_text(json.dumps([{"phase": "pre_grasp", "ok": True, "index": 0}]),
                   encoding="utf-8")
    name, index, total = run_task.failed_phase_from_log(log)
    assert (name, index, total) == ("?", 0, 1)


def test_failed_phase_from_log_handles_a_missing_file(tmp_path):
    name, index, total = run_task.failed_phase_from_log(tmp_path / "nope.json")
    assert (name, index, total) == ("?", 0, 0)


def test_runner_command_passes_start_index_only_for_the_resumed_step():
    import argparse

    args = argparse.Namespace(plan_only=True, confirm=False, phases="", until="",
                              start_joints="", allow_release=False)
    args._start_index_for_step = {"load": 3}
    cmd = run_task.runner_command(pathlib.Path("cfg.yaml"), "load", args)
    assert "--start-index" in cmd
    assert cmd[cmd.index("--start-index") + 1] == "3"

    # 别的 step 不受影响（full scope 时只有失败的那个 step 需要续跑）
    cmd2 = run_task.runner_command(pathlib.Path("cfg.yaml"), "unload", args)
    assert "--start-index" not in cmd2
