"""Checks for the six-mode dashboard helpers in ``scripts/cs625_ui.py``.

Two things are covered without starting the task entry point:

* ``build_task_command`` — the pure button -> ``run_task.py`` mapping;
* ``ZoomableImageView`` — aspect-preserving fit and the independent zoom state.

The UI module treats the ROS 2 imports as optional, so this test runs without
sourcing the workspace.  Run from the repository root:

    python3 -m pytest test/test_ui_command.py -q
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

pytest.importorskip("PyQt5")
QtCore = pytest.importorskip("PyQt5.QtCore")
QtGui = pytest.importorskip("PyQt5.QtGui")
QtWidgets = pytest.importorskip("PyQt5.QtWidgets")

import cs625_ui  # noqa: E402


# --------------------------------------------------------------- pure mapping
def test_acceptance_load_efficient_plan_only_real():
    command = cs625_ui.build_task_command("load", "efficient", "plan-only", "real")
    assert command[0] == sys.executable
    assert command[1:3] == ["-u", "scripts/run_task.py"]
    assert command[3:] == [
        "--scope", "load", "--mode", "efficient", "--plan-only", "--world", "real",
    ]
    assert "run_task.py --scope load --mode efficient --plan-only --world real" in (
        cs625_ui.format_task_command(command)
    )


def test_acceptance_full_safe_dry_run_sim():
    command = cs625_ui.build_task_command("full", "safe", "dry-run", "sim")
    assert command[3:] == [
        "--scope", "full", "--mode", "safe", "--dry-run", "--world", "sim",
    ]
    assert "run_task.py --scope full --mode safe --dry-run --world sim" in (
        cs625_ui.format_task_command(command)
    )


def test_every_mode_has_exactly_one_action_flag():
    for scope in cs625_ui.SCOPES:
        for mode in cs625_ui.MODES:
            for action in cs625_ui.ACTIONS:
                for world in cs625_ui.WORLDS:
                    command = cs625_ui.build_task_command(scope, mode, action, world)
                    action_flags = [
                        part for part in command
                        if part in ("--dry-run", "--plan-only", "--confirm")
                    ]
                    assert action_flags == [cs625_ui.ACTION_FLAGS[action]]
                    assert command[command.index("--scope") + 1] == scope
                    assert command[command.index("--mode") + 1] == mode
                    assert command[command.index("--world") + 1] == world
                    assert command[-2:] == ["--world", world]
                    assert command.count("--world") == 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"scope": "grasp", "mode": "safe", "action": "dry-run", "world": "real"},
        {"scope": "load", "mode": "fast", "action": "dry-run", "world": "real"},
        {"scope": "load", "mode": "safe", "action": "run", "world": "real"},
        {"scope": "load", "mode": "safe", "action": "dry-run", "world": "gazebo"},
    ],
)
def test_unknown_mode_values_raise(kwargs):
    with pytest.raises(ValueError):
        cs625_ui.build_task_command(**kwargs)


# --------------------------------------------------------------- image widget
@pytest.fixture(scope="module")
def qt_app():
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
    return app


def _source_pixmap() -> QtGui.QPixmap:
    pixmap = QtGui.QPixmap(640, 480)
    pixmap.fill(QtGui.QColor("#204060"))
    return pixmap


def test_image_view_fits_without_distorting(qt_app):
    view = cs625_ui.ZoomableImageView()
    view.resize(800, 600)
    view.set_pixmap(_source_pixmap())
    margin = cs625_ui.ZoomableImageView.MARGIN
    expected = min((800 - 2 * margin) / 640.0, (600 - 2 * margin) / 480.0)
    assert view.display_scale() == pytest.approx(expected)
    # The displayed size keeps the sensor aspect ratio (no window-driven stretch).
    assert (640 * view.display_scale()) / (480 * view.display_scale()) == pytest.approx(
        640 / 480
    )
    view.deleteLater()


def test_image_view_zoom_is_independent_and_resettable(qt_app):
    view = cs625_ui.ZoomableImageView()
    view.resize(800, 600)
    view.set_pixmap(_source_pixmap())
    fit = view.display_scale()

    wheel = QtGui.QWheelEvent(
        QtCore.QPointF(400.0, 300.0), QtCore.QPointF(400.0, 300.0),
        QtCore.QPoint(0, 0), QtCore.QPoint(0, 120),
        QtCore.Qt.NoButton, QtCore.Qt.NoModifier,
        QtCore.Qt.NoScrollPhase, False,
    )
    view.wheelEvent(wheel)
    assert view.display_scale() > fit

    view.reset_view()
    assert view.display_scale() == pytest.approx(fit)

    # Window resizes only change the fit scale; the zoom factor survives.
    view.wheelEvent(wheel)
    zoomed = view.display_scale() / fit
    view.resize(1000, 700)
    new_fit = min((1000 - 8) / 640.0, (700 - 8) / 480.0)
    assert view.display_scale() / new_fit == pytest.approx(zoomed)
    view.deleteLater()


# ------------------------------------------------------------ status helpers
def test_log_level_classification():
    assert cs625_ui.classify_log_line("environment gate : PASS") == "ok"
    assert cs625_ui.classify_log_line("ERROR: could not connect") == "error"
    assert cs625_ui.classify_log_line("refusing to run: gate rejected") == "error"
    assert cs625_ui.classify_log_line("(WARNING) driver slow") == "warn"
    assert cs625_ui.classify_log_line("已停止：退出码 143") == "warn"
    assert cs625_ui.classify_log_line("phase pre_grasp start") == "info"
    # An explicit level (the UI knows it printed a command) always wins.
    assert cs625_ui.classify_log_line("anything", "cmd") == "cmd"


def test_elapsed_formatting():
    assert cs625_ui.format_elapsed(0) == "00:00"
    assert cs625_ui.format_elapsed(75.9) == "01:15"
    assert cs625_ui.format_elapsed(3675) == "1:01:15"


def test_robot_status_summary():
    text, level, detail = cs625_ui.describe_robot_status(1, 7, True)
    assert level == "ok"
    assert "1 / 7" in text and "NORMAL" in detail
    assert cs625_ui.describe_robot_status(3, 7, True)[1] == "bad"
    assert cs625_ui.describe_robot_status(1, 7, False)[1] == "warn"
    assert cs625_ui.describe_robot_status(None, None, False)[0].startswith("未连接")


# --------------------------------------------------------- environment model
def _write_environment_model(root: Path, created_at: str,
                             valid_points: int = 612121) -> Path:
    directory = root / "runtime" / "environment" / "20261024_122523"
    directory.mkdir(parents=True, exist_ok=True)
    (root / "runtime" / "environment" / "latest.json").write_text(
        json.dumps({"latest": "20261024_122523",
                    "model": "20261024_122523/model.json"}),
        encoding="utf-8")
    (directory / "model.json").write_text(json.dumps({
        "created_at": created_at,
        "valid_points": valid_points,
        "frames_used": 93,
        "baseline_delta": {"slot_mm": 0.1, "slot_deg": 0.01,
                           "workpiece_mm": 0.2, "workpiece_deg": 0.02},
    }), encoding="utf-8")
    return directory


def test_environment_status_pass_and_reject(tmp_path):
    now = datetime(2026, 10, 24, 13, 0, tzinfo=timezone.utc)
    _write_environment_model(tmp_path, "2026-10-24T12:30:00+00:00")
    status = cs625_ui.read_environment_status(tmp_path, now=now)
    assert status["available"] is True
    assert status["gate_ok"] is True
    assert status["age_h"] == pytest.approx(0.5, abs=0.01)

    _write_environment_model(tmp_path, "2026-10-24T02:00:00+00:00")
    status = cs625_ui.read_environment_status(tmp_path, now=now)
    assert status["available"] is True
    assert status["gate_ok"] is False
    assert any("过期" in reason for reason in status["reasons"])


def test_environment_status_missing_pointer(tmp_path):
    status = cs625_ui.read_environment_status(tmp_path)
    assert status["available"] is False
    assert status["gate_ok"] is None
    assert status["reasons"]


# ---------------------------------------------------------------- animations
def test_status_dot_breathing_starts_and_stops(qt_app):
    from PyQt5.QtTest import QTest

    dot = cs625_ui.StatusDot()
    dot.set_active(True)
    assert dot.is_animating()
    QTest.qWait(260)
    first = dot.get_breath()
    QTest.qWait(260)
    second = dot.get_breath()
    assert first != pytest.approx(second)
    assert 0.4 <= second <= 1.0
    dot.set_active(False)
    assert not dot.is_animating()
    assert dot.get_breath() == pytest.approx(1.0)
    dot.deleteLater()


def test_button_hover_transition(qt_app):
    from PyQt5.QtTest import QTest

    button = cs625_ui.FlatButton("开始任务", role="start")
    button.enterEvent(QtCore.QEvent(QtCore.QEvent.Enter))
    QTest.qWait(180)
    assert button.hover_value() == pytest.approx(1.0, abs=0.02)
    button.leaveEvent(QtCore.QEvent(QtCore.QEvent.Leave))
    QTest.qWait(180)
    assert button.hover_value() == pytest.approx(0.0, abs=0.02)
    button.deleteLater()


def test_log_view_levels_and_clear(qt_app):
    log = cs625_ui.LogView()
    log.append_line("gate PASS", "ok")
    log.append_line("boom", "error")
    assert log.blockCount() == 2
    assert "gate PASS" in log.toPlainText()
    log.clear_log()
    assert log.toPlainText() == ""
    assert log.blockCount() == 1
    log.deleteLater()
