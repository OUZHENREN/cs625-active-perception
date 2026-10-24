#!/usr/bin/env python3
"""CS625 six-mode task dashboard (compact panel + independent camera window).

The panel is the main operator surface for the six task modes:

    scope  load / unload / full      x
    mode   safe / efficient          x
    action dry-run / plan-only / confirm   x
    world  real / sim

The live colour image no longer sits inside the main window.  It lives in its
own :class:`CameraWindow` (title "CS625 Camera View", about 1280x960, freely
resizable), which reuses :class:`ZoomableImageView` -- aspect preserving fit,
wheel zoom around the cursor, drag pan and double-click reset all survive.
Closing that window only hides it; the main panel can reopen it at any time and
closes it together with itself.  The camera window owns no timer: it is fed by
the single 500 ms state timer of the main panel.

Main window layout (about 470x840 at 1K, control column stays 460-520 px):

    header        title + "图像窗口" toggle (open/closed + live fps)
    status card   robot / camera / environment model / task / run state, one row each
    task modes    scope x mode x action x world, start/stop
    monitors      tabs: joints/controller, camera chain, gripper (confirm)
    task log      collapsible QPlainTextEdit, 200-ish px, last 500 lines

The panel owns no motion logic.  "开始任务" only assembles the command line of
``scripts/run_task.py`` (pure function :func:`build_task_command`), prints it to
the log and then runs it asynchronously through ``QProcess``.  The arm is moved
only by ``run_task.py`` itself and only when the operator picked 执行 and passed
both confirmation dialogs.

Run (the repository ROS environment must be sourced in the same shell):

    source scripts/source_dev_env.sh --full
    python3 scripts/cs625_ui.py

Offscreen self-test (writes the panel + camera window screenshots and exits,
never spawns the task):

    QT_QPA_PLATFORM=offscreen python3 scripts/cs625_ui.py --smoke /tmp/ui.png

Print the assembled commands without any ROS/PyQt runtime (pure function):

    python3 scripts/cs625_ui.py --print-command-only
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import importlib.util
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

# The panel needs the ROS 2 Python stack, but the pure command builder and
# --print-command-only must stay usable without sourcing the workspace.  Keep
# the import failure as a value instead of a traceback and report it in main().
try:
    from ui_plots import RealtimePlot
    PLOTS_AVAILABLE = True
except Exception as _plots_exc:  # noqa: BLE001 - the panel starts without it
    RealtimePlot = None  # type: ignore[assignment]
    PLOTS_AVAILABLE = False
    _PLOTS_ERROR = f"{type(_plots_exc).__name__}: {_plots_exc}"

ROS_AVAILABLE = True
ROS_IMPORT_ERROR = ""
try:
    import rclpy
    import tf2_ros
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from scipy.spatial.transform import Rotation
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import String as RosString
    from geometry_msgs.msg import WrenchStamped
except Exception as exc:  # noqa: BLE001 - reported to the operator
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


REPO = Path(__file__).resolve().parent.parent
TASK_ENTRY = "scripts/run_task.py"

SCOPES = ("load", "unload", "full")
MODES = ("safe", "efficient")
ACTIONS = ("dry-run", "plan-only", "confirm")
WORLDS = ("real", "sim")

SCOPE_LABELS = {"load": "装入", "unload": "取出", "full": "全流程"}
MODE_LABELS = {"safe": "安全模式", "efficient": "效率模式"}
ACTION_LABELS = {"dry-run": "预演", "plan-only": "只规划", "confirm": "执行"}
WORLD_LABELS = {"real": "真机", "sim": "仿真"}
ACTION_FLAGS = {"dry-run": "--dry-run", "plan-only": "--plan-only", "confirm": "--confirm"}

# Compact 1K layout: one narrow control column.  The camera lives in its own
# 1280x960 window (4:3, matching the sensor), so the panel does not reserve any
# horizontal space for the image any more.
MAIN_WINDOW_SIZE = (468, 960)
MAIN_MIN_WIDTH = 460
MAIN_MAX_WIDTH = 520
IMAGE_WINDOW_SIZE = (1280, 960)
LOG_VIEW_HEIGHT = 200
STATE_REFRESH_MS = 500

#: Top-level window titles must stay ASCII under WSLg.
#:
#: WSLg runs its Weston compositor in a *separate* system distro and draws the
#: window frame there: ``xprop`` on the panel reports
#: ``_NET_FRAME_EXTENTS(CARDINAL) = 38, 38, 59, 38``, i.e. the 59 px top frame
#: with the title text is painted by Weston, not by Qt.  Weston renders that
#: text with the system distro's own fonts, and that font set is DejaVu only
#: (WSLGd logs ``FontMonitor: monitoring /mnt/wslg/distro/usr/share/fonts``).
#: DejaVu has no CJK glyph at all, so every Chinese character in a title bar
#: shows up as an empty box, while the ``·`` (U+00B7) in the same string still
#: renders -- exactly the reported ``CS625 · 图□□□□...`` symptom.
#:
#: This is *not* a Qt encoding bug, so the obvious workarounds do not work:
#:
#: * the panel already publishes ``_NET_WM_NAME(UTF8_STRING)`` with the correct
#:   text (verified with ``xprop -id <win> WM_NAME _NET_WM_NAME``), therefore
#:   re-setting ``_NET_WM_NAME`` as UTF-8 through ``xdotool``/``xprop`` -- the
#:   usual "UTF-8 title" recipe -- is a no-op; the bytes are identical;
#: * installing WenQuanYi inside Ubuntu cannot help either: the compositor only
#:   reads the system distro's font directory, never Ubuntu's fontconfig.
#:
#: Only ASCII titles are safe.  The Chinese product name stays *inside* the
#: windows (header label "CS625 · 六类任务主控", camera status labels, ...),
#: where the fonts come from Ubuntu and render correctly.
WINDOW_TITLE_MAIN = "CS625 Task Console"
WINDOW_TITLE_CAMERA = "CS625 Camera View"

# One status row per subsystem, in the order the operator checks them.
STATUS_ROWS = (
    ("robot", "机器人"),
    ("camera", "相机"),
    ("env", "环境模型"),
    ("task", "任务"),
    ("run", "运行状态"),
)
# Draining a few camera callbacks per tick keeps the displayed fps reasonable
# even though the panel only wakes up twice per second.
SPIN_CALLS_PER_TICK = 12

JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

CAMERA_FIELDS = (
    ("color", "彩色"),
    ("depth", "深度"),
    ("resolution", "分辨率"),
    ("fps", "帧率"),
    ("valid", "深度有效"),
    ("depth_median", "深度中位"),
    ("intrinsics", "内参 fx/fy"),
    ("stamp", "最近帧"),
)

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 7500
GRIPPER_CMD_PS1 = r"C:\Users\Public\hitbot_cmd.ps1"


# --------------------------------------------------------------- pure helpers
def build_task_command(
    scope: str,
    mode: str,
    action: str,
    world: str,
    python: str | None = None,
) -> list[str]:
    """Assemble the ``run_task.py`` command for one of the six task modes.

    Pure function: no subprocess, no ROS, no filesystem access, so the UI, the
    smoke test and test/test_ui_command.py all share the same mapping.

    ``action`` selects exactly one of --dry-run / --plan-only / --confirm;
    ``--world`` is passed through so the task layer can pick real vs sim.
    """
    if scope not in SCOPES:
        raise ValueError(f"unknown scope {scope!r}, expected one of {SCOPES}")
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}, expected one of {MODES}")
    if action not in ACTIONS:
        raise ValueError(f"unknown action {action!r}, expected one of {ACTIONS}")
    if world not in WORLDS:
        raise ValueError(f"unknown world {world!r}, expected one of {WORLDS}")
    return [
        str(python or sys.executable),
        "-u",
        TASK_ENTRY,
        "--scope", scope,
        "--mode", mode,
        ACTION_FLAGS[action],
        "--world", world,
    ]


def format_task_command(command: Sequence[str]) -> str:
    """Render a command line for the log/terminal without losing quoting."""
    return shlex.join([str(part) for part in command])


def _descendant_pids(pid: int) -> list[int]:
    """Every descendant of ``pid``, read from /proc (Linux / WSL).

    ``run_task.py`` starts ``run_pick_and_place.py`` as a child, so stopping the
    panel must stop the whole process tree, not only the direct child.
    """
    children: dict[int, list[int]] = {}
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return []
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().split()
        except OSError:
            continue
        if len(fields) < 4:
            continue
        try:
            parent = int(fields[3])
            child = int(entry.name)
        except ValueError:
            continue
        children.setdefault(parent, []).append(child)
    result: list[int] = []
    stack = [pid]
    while stack:
        for child in children.get(stack.pop(), []):
            result.append(child)
            stack.append(child)
    return result


def _print_command_demo(args) -> int:
    """--print-command-only: prove the button -> command mapping, start nothing."""
    fields = (args.scope, args.mode, args.action, args.world)
    if all(value is not None for value in fields):
        print(format_task_command(build_task_command(*fields)))
        return 0
    demo = (
        ("装入+效率+只规划+真机", ("load", "efficient", "plan-only", "real")),
        ("全流程+安全+预演+仿真", ("full", "safe", "dry-run", "sim")),
    )
    for label, values in demo:
        print(f"{label} → {format_task_command(build_task_command(*values))}")
    return 0


# ------------------------------------------------------------- image widget

# --- shared status helpers -------------------------------------------
# Caches for the environment-model gate: importing scripts/model_environment.py
# once is enough, and the thresholds only change when common.yaml changes.
_GATE_MODULE = None
_GATE_IMPORT_ERROR = ""
_THRESHOLD_CACHE: dict[str, dict] = {}

SAFETY_MODE_NAMES = {
    1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY",
    5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION", 9: "FAULT", 10: "VALIDATE_JOINT_ID", 11: "UNDEFINED",
    12: "AUTOMATIC_MODE_SAFEGUARD_STOP", 13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
    14: "TP_THREE_POSITION_ENABLING_STOP",
}


ROBOT_MODE_NAMES = {
    -2: "UNKNOWN", -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY",
    2: "BOOTING", 3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE",
    7: "RUNNING", 8: "UPDATING_FIRMWARE", 9: "WAITING_CALIBRATION",
}


def format_elapsed(seconds: float) -> str:
    """``mm:ss`` (``h:mm:ss`` past one hour) for the elapsed-time card."""
    total = max(0, int(seconds))
    hours, rest = divmod(total, 3600)
    minutes, secs = divmod(rest, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


def classify_log_line(text: str, level: str | None = None) -> str:
    """Map one ``run_task.py`` / UI log line to a colour level.

    Explicit levels win (the panel knows when it prints a command).  Otherwise
    the same words are used as the task layer: ERROR/FAIL/REJECT are red,
    WARN/timeout/stop are amber, PASS/OK/success are green.
    """
    if level:
        return level
    upper = str(text).upper()
    if re.search(r"\b(ERROR|FAILED|FAILURE|FAIL|TRACEBACK|EXCEPTION|REJECT|REFUSING)\b",
                 upper) or "错误" in text or "失败" in text:
        return "error"
    if re.search(r"\b(WARN|WARNING|TIMEOUT|STOPPED|CANCEL{1,2}ED)\b", upper) \
            or "超时" in text or "已停止" in text or "取消" in text:
        return "warn"
    if re.search(r"\b(OK|PASS|PASSED|SUCCESS|SUCCEEDED|DONE)\b", upper) \
            or "完成" in text or "正常" in text:
        return "ok"
    return "info"


def describe_robot_status(
    safety: int | None,
    robot: int | None,
    sockets: bool,
) -> tuple[str, str, str]:
    """(one-line value, level, tooltip) for the robot-connection card.

    "Can the arm move" is exactly three facts in this repository
    (docs/source_authority.md): safety_mode=1, robot_mode=7 and the three
    reverse-control sockets established.
    """
    socket_text = "sockets ✓" if sockets else "sockets ✗"
    if safety is None and robot is None:
        return (
            f"未连接 · {socket_text}",
            "warn",
            "没有收到 /io_and_status_controller/safety_mode 或 robot_mode",
        )
    safety_text = "—" if safety is None else str(safety)
    robot_text = "—" if robot is None else str(robot)
    text = f"{safety_text} / {robot_text} · {socket_text}"
    healthy = safety == 1 and robot == 7 and sockets
    if healthy:
        level = "ok"
    elif (safety is not None and safety not in (1, 2)) or \
            (robot is not None and robot in (0, 1, 2, 3)):
        level = "bad"
    else:
        level = "warn"
    safety_name = SAFETY_MODE_NAMES.get(safety, "?") if safety is not None else "?"
    robot_name = ROBOT_MODE_NAMES.get(robot, "?") if robot is not None else "?"
    detail = (f"safety_mode={safety_text}({safety_name}) "
              f"robot_mode={robot_text}({robot_name}) · {socket_text}")
    return text, level, detail


def _gate_module(repo: Path = REPO):
    """Lazily import the pure gate function from scripts/model_environment.py."""
    global _GATE_MODULE, _GATE_IMPORT_ERROR
    if _GATE_MODULE is not None:
        return _GATE_MODULE
    if _GATE_IMPORT_ERROR:
        raise RuntimeError(_GATE_IMPORT_ERROR)
    path = Path(repo) / "scripts" / "model_environment.py"
    if not path.exists():
        path = REPO / "scripts" / "model_environment.py"
    try:
        spec = importlib.util.spec_from_file_location("cs625_model_environment", path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except Exception as exc:  # noqa: BLE001 - surfaces in the status card tooltip
        _GATE_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
        raise RuntimeError(_GATE_IMPORT_ERROR) from exc
    _GATE_MODULE = module
    return module


def _environment_thresholds(repo: Path = REPO) -> dict:
    """Gate thresholds from config/tasks/common.yaml (defaults when absent)."""
    path = repo / "config" / "tasks" / "common.yaml"
    key = str(path)
    if key in _THRESHOLD_CACHE:
        return _THRESHOLD_CACHE[key]
    thresholds: dict = {}
    try:
        import yaml  # noqa: PLC0415 - only needed when the file exists

        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        thresholds = dict((document.get("environment") or {}).get("thresholds") or {})
    except Exception:  # noqa: BLE001 - defaults from the gate module are fine
        thresholds = {}
    _THRESHOLD_CACHE[key] = thresholds
    return thresholds


def _parse_iso8601(text) -> datetime | None:
    if not text or not isinstance(text, str):
        return None
    try:
        stamp = datetime.fromisoformat(text.strip().replace("Z", "+00:00"))
    except ValueError:
        return None
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return stamp


def resolve_environment_model(repo: Path = REPO) -> Path | None:
    """Resolve runtime/environment/latest.json -> the concrete model.json."""
    pointer = repo / "runtime" / "environment" / "latest.json"
    if pointer.exists():
        try:
            payload = json.loads(pointer.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            payload = None
        if isinstance(payload, (dict, str)):
            values = [payload] if isinstance(payload, str) else [
                payload.get("model"), payload.get("path"), payload.get("latest")]
            for value in values:
                if not value:
                    continue
                candidate = Path(str(value))
                if not candidate.is_absolute():
                    candidate = pointer.parent / candidate
                if candidate.is_dir() or candidate.suffix.lower() != ".json":
                    candidate = candidate / "model.json"
                if candidate.exists():
                    return candidate
    fallback = repo / "runtime" / "environment" / "latest" / "model.json"
    return fallback if fallback.exists() else None


def read_environment_status(
    repo: Path = REPO,
    now: datetime | None = None,
) -> dict:
    """Read the environment model + gate result for the status card.

    Same inputs as ``run_task.py``: runtime/environment/latest.json, the
    pointed model.json, and the thresholds in config/tasks/common.yaml.  The
    gate call is the shared pure ``evaluate_gate`` so the panel can never
    disagree with the task layer about PASS/REJECT.
    """
    now = now or datetime.now(timezone.utc)
    path = resolve_environment_model(repo)
    if path is None or not path.exists():
        return {
            "available": False,
            "model_path": "",
            "created_at": "",
            "age_h": None,
            "gate_ok": None,
            "reasons": ["环境模型缺失：runtime/environment/latest.json 未指向 model.json"],
            "frames_used": None,
            "valid_points": None,
        }
    try:
        model = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {
            "available": False,
            "model_path": str(path),
            "created_at": "",
            "age_h": None,
            "gate_ok": None,
            "reasons": [f"model.json 读取失败：{exc}"],
            "frames_used": None,
            "valid_points": None,
        }
    created = _parse_iso8601(model.get("created_at"))
    age_h = max(0.0, (now - created).total_seconds() / 3600.0) if created else None
    gate_ok: bool | None = None
    reasons: list[str] = []
    try:
        module = _gate_module(repo)
        gate_ok, reasons = module.evaluate_gate(
            model, _environment_thresholds(repo), now.isoformat())
    except Exception as exc:  # noqa: BLE001 - card shows the reason instead
        reasons = [f"门禁评估不可用：{type(exc).__name__}: {exc}"]
    return {
        "available": True,
        "model_path": str(path),
        "created_at": str(model.get("created_at") or ""),
        "age_h": age_h,
        "gate_ok": gate_ok,
        "reasons": [str(reason) for reason in reasons],
        "frames_used": model.get("frames_used"),
        "valid_points": model.get("valid_points"),
    }

# --------------------------------------------------------------------------- #
# Small self-contained widgets: breathing status dot, hover-fading button and a
# level-coloured log view.  Kept dependency-free (only PyQt5) so the panel can be
# smoke-tested offscreen without the rest of the UI stack.
# --------------------------------------------------------------------------- #
LEVEL_COLORS = {
    "info": "#8aa0b4",
    "ok": "#46c17b",
    "warn": "#f2b04a",
    "error": "#ef6b6b",
    "bad": "#ef6b6b",
}


class StatusDot(QtWidgets.QWidget):
    """14 px dot that breathes while a task runs (one QPropertyAnimation)."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._breath = 1.0
        self._active = False
        self.setFixedSize(14, 14)
        self._anim = QtCore.QPropertyAnimation(self, b"breath", self)
        self._anim.setStartValue(1.0)
        self._anim.setEndValue(0.45)
        self._anim.setDuration(1600)
        self._anim.setEasingCurve(QtCore.QEasingCurve.InOutSine)
        self._anim.setLoopCount(-1)

    def get_breath(self) -> float:  # noqa: D401 - Qt property getter
        return self._breath

    def set_breath(self, value: float) -> None:
        self._breath = float(value)
        self.update()

    breath = QtCore.pyqtProperty(float, fget=get_breath, fset=set_breath)

    def set_active(self, active: bool) -> None:
        self._active = bool(active)
        if self._active:
            if self._anim.state() != QtCore.QAbstractAnimation.Running:
                self._anim.start()
        else:
            self._anim.stop()
            self._breath = 1.0
        self.update()

    def is_animating(self) -> bool:
        return self._anim.state() == QtCore.QAbstractAnimation.Running

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt override
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        color = QtGui.QColor(LEVEL_COLORS["ok"] if self._active
                             else LEVEL_COLORS["info"])
        color.setAlphaF(max(0.25, min(1.0, self._breath)))
        painter.setBrush(color)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(1, 1, 12, 12)


class FlatButton(QtWidgets.QPushButton):
    """Push button whose hover state fades in over 120 ms (one animation each)."""

    def __init__(self, text: str = "", role: str = "", parent=None) -> None:
        super().__init__(text, parent)
        if role:
            self.setProperty("role", role)
        self._hover = 0.0
        self._anim = QtCore.QVariantAnimation(self)
        self._anim.setDuration(120)
        self._anim.valueChanged.connect(self._on_hover_value)

    def hover_value(self) -> float:
        return self._hover

    def _on_hover_value(self, value) -> None:
        self._hover = float(value)
        self.update()

    def enterEvent(self, event) -> None:  # noqa: N802 - Qt override
        self._animate_to(1.0)
        super().enterEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt override
        self._animate_to(0.0)
        super().leaveEvent(event)

    def _animate_to(self, target: float) -> None:
        self._anim.stop()
        self._anim.setStartValue(self._hover)
        self._anim.setEndValue(target)
        self._anim.start()


class LogView(QtWidgets.QPlainTextEdit):
    """Read-only log with per-level colouring and a clear() alias."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setUndoRedoEnabled(False)
        self.setMaximumBlockCount(500)
        self.setObjectName("Log")

    def append_line(self, text: str, level: str | None = None) -> None:
        level = classify_log_line(text, level)
        color = LEVEL_COLORS.get(level, LEVEL_COLORS["info"])
        cursor = QtGui.QTextCursor(self.document())
        cursor.movePosition(QtGui.QTextCursor.End)
        if not self.document().isEmpty():
            cursor.insertBlock()
        fmt = QtGui.QTextCharFormat()
        fmt.setForeground(QtGui.QColor(color))
        cursor.insertText(time.strftime("%H:%M:%S") + "  ", fmt)
        cursor.insertText(text, fmt)
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())

    def clear_log(self) -> None:
        self.clear()


class ZoomableImageView(QtWidgets.QWidget):
    """Aspect-preserving colour image with an independent pan/zoom state.

    The widget keeps its own zoom factor and pixel offset.  Window resizing
    only changes the fit scale, never the image aspect ratio, and the operator's
    zoom/pan survives new frames.
    """

    viewChanged = QtCore.pyqtSignal(float)  # displayed pixels per source pixel

    MIN_ZOOM = 0.25
    MAX_ZOOM = 16.0
    MARGIN = 4

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._pixmap: QtGui.QPixmap | None = None
        self._zoom = 1.0
        self._offset = QtCore.QPointF(0.0, 0.0)
        self._dragging = False
        self._drag_pos = QtCore.QPoint()
        self.setMinimumSize(360, 300)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.setToolTip("拖拽平移 · 滚轮缩放 · 双击复位")

    # -- state ---------------------------------------------------------
    def has_image(self) -> bool:
        return self._pixmap is not None and not self._pixmap.isNull()

    def set_pixmap(self, pixmap: QtGui.QPixmap) -> None:
        self._pixmap = pixmap
        self.update()
        self._emit_scale()

    def reset_view(self) -> None:
        self._zoom = 1.0
        self._offset = QtCore.QPointF(0.0, 0.0)
        self.update()
        self._emit_scale()

    def display_scale(self) -> float:
        return self._fit_scale() * self._zoom

    def _fit_scale(self) -> float:
        if not self.has_image():
            return 1.0
        area = self.rect().adjusted(self.MARGIN, self.MARGIN,
                                    -self.MARGIN, -self.MARGIN)
        if area.width() <= 0 or area.height() <= 0:
            return 1.0
        return min(area.width() / self._pixmap.width(),
                   area.height() / self._pixmap.height())

    def _emit_scale(self) -> None:
        self.viewChanged.emit(self.display_scale())

    def _clamp_offset(self) -> None:
        """Keep at least a sliver of the image reachable after a wild drag."""
        if not self.has_image():
            return
        scale = self.display_scale()
        limit_x = self._pixmap.width() * scale / 2.0 + self.width() / 2.0 - 40.0
        limit_y = self._pixmap.height() * scale / 2.0 + self.height() / 2.0 - 40.0
        self._offset.setX(min(max(self._offset.x(), -limit_x), limit_x))
        self._offset.setY(min(max(self._offset.y(), -limit_y), limit_y))

    # -- Qt events -----------------------------------------------------
    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0d12"))
        border = self.rect().adjusted(1, 1, -1, -1)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setPen(QtGui.QPen(QtGui.QColor("#263241")))
        painter.drawRoundedRect(border, 8, 8)
        if not self.has_image():
            font = painter.font()
            font.setPointSizeF(12.0)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QColor("#6f7f90"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter,
                             "等待 /camera/color/image_raw ...")
            return
        scale = self.display_scale()
        width = self._pixmap.width() * scale
        height = self._pixmap.height() * scale
        center = QtCore.QPointF(self.width() / 2.0, self.height() / 2.0) + self._offset
        target = QtCore.QRectF(center.x() - width / 2.0,
                               center.y() - height / 2.0, width, height)
        painter.save()
        painter.setClipRect(border)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.drawPixmap(target, self._pixmap,
                           QtCore.QRectF(self._pixmap.rect()))
        painter.restore()

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:  # noqa: N802
        if not self.has_image():
            return
        delta = event.angleDelta().y()
        if delta == 0:
            return
        old_scale = self.display_scale()
        new_zoom = min(max(self._zoom * (1.0015 ** delta),
                           self.MIN_ZOOM), self.MAX_ZOOM)
        new_scale = self._fit_scale() * new_zoom
        if abs(new_scale - old_scale) < 1.0e-9:
            return
        # Keep the image point under the cursor pinned while zooming.
        cursor = QtCore.QPointF(event.pos())
        center = QtCore.QPointF(self.width() / 2.0, self.height() / 2.0)
        image_point = (cursor - center - self._offset) / old_scale
        self._zoom = new_zoom
        self._offset = cursor - center - image_point * new_scale
        self._clamp_offset()
        self.update()
        self._emit_scale()
        event.accept()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() == QtCore.Qt.LeftButton and self.has_image():
            self._dragging = True
            self._drag_pos = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if self._dragging:
            delta = event.pos() - self._drag_pos
            self._drag_pos = event.pos()
            self._offset = self._offset + QtCore.QPointF(delta)
            self._clamp_offset()
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() == QtCore.Qt.LeftButton:
            self._dragging = False
            self.setCursor(QtCore.Qt.OpenHandCursor)
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() == QtCore.Qt.LeftButton:
            self.reset_view()
        super().mouseDoubleClickEvent(event)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._clamp_offset()
        self._emit_scale()


# --------------------------------------------------------------- appearance
# One dark sheet for the panel and its camera window; the accent is cyan, used
# for selected modes, the image-window toggle and key values.  There are no
# property animations on purpose (owner limit: <= 3), the only motion is the
# live camera image itself.
STYLE_SHEET = """
QMainWindow, QWidget, QDialog { background: #11151c; color: #dbe4ee;
    font-family: 'Noto Sans CJK SC', 'Microsoft YaHei', sans-serif;
    font-size: 10pt; }
QLabel { font-size: 10pt; }
QLabel#Title { font-size: 14pt; font-weight: 700; color: #f4f8fb; }
QLabel#Mono { font-family: 'DejaVu Sans Mono', 'Consolas', monospace;
    font-size: 9.5pt; color: #c9d7e5; }
QLabel#Muted { color: #8496a8; font-size: 9.5pt; }
QLabel#StatusName { color: #8496a8; font-size: 9.5pt; }
QLabel#StatusValue { font-family: 'DejaVu Sans Mono', 'Consolas', monospace;
    font-size: 9.5pt; color: #c9d7e5; }
QLabel#StatusValue[level="ok"] { color: #4fe0b4; }
QLabel#StatusValue[level="warn"] { color: #f0c36a; }
QLabel#StatusValue[level="bad"] { color: #ff8d95; }
QLabel#StatusValue[level="idle"] { color: #8496a8; }
QGroupBox { border: 1px solid #263241; border-radius: 6px;
    margin-top: 15px; padding-top: 6px; font-size: 11pt;
    font-weight: 700; color: #9fc4cc; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
QTableWidget { background: #0f141b; alternate-background-color: #131b24;
    gridline-color: #1e2a37; border: none; font-size: 9.5pt; }
QTableWidget::item { padding: 1px 3px; }
QHeaderView::section { background: #18222e; color: #9fb3c8;
    border: none; padding: 3px 3px; font-size: 9.5pt; }
QPlainTextEdit#Log { background: #0b1016; border: none;
    color: #b7c6d4; font-family: 'DejaVu Sans Mono', 'Consolas', monospace;
    font-size: 9pt; }
QPushButton { background: #1b2a3a; border: 1px solid #2d4055;
    border-radius: 6px; padding: 4px 8px; color: #e6eef7; font-size: 10pt; }
QPushButton:hover { background: #24384d; }
QPushButton:pressed { background: #152230; }
QPushButton:disabled { background: #141b23; color: #4d5a68;
    border-color: #1e2a37; }
QPushButton[role="choice"] { padding: 2px 4px; font-size: 9.5pt; }
QPushButton[role="choice"]:checked { background: #0f3f46;
    border: 2px solid #2fd6c9; color: #eaffff; font-weight: 700; }
QPushButton[role="choice"]:checked:hover { background: #14525b; }
QPushButton[role="start"] { background: #14524d; border: 1px solid #2fd6c9;
    color: #eaffff; font-size: 13pt; font-weight: 700; }
QPushButton[role="start"]:hover { background: #186860; }
QPushButton[role="start"]:disabled { background: #142321;
    border-color: #24453f; color: #5d7a72; }
QPushButton[role="stop"] { background: #5a1d22; border: 1px solid #a83c44;
    color: #ffe1e3; font-size: 13pt; font-weight: 700; }
QPushButton[role="stop"]:hover { background: #70262c; }
QPushButton[role="stop"]:disabled { background: #241719;
    border-color: #3a2225; color: #7d5b5e; }
QPushButton#ImageToggle { background: #102c33; border: 1px solid #2fd6c9;
    color: #c8fbff; font-size: 9.5pt; font-weight: 700; }
QPushButton#ImageToggle:hover { background: #16404a; }
QPushButton#ImageToggle[open="false"] { background: #1b2a3a;
    border: 1px solid #2d4055; color: #9fb3c8; font-weight: 400; }
QToolButton#LogToggle { background: transparent; border: none;
    color: #9fc4cc; font-size: 9.5pt; padding: 0 2px; }
QToolButton#LogToggle:hover { color: #2fd6c9; }
QTabWidget::pane { border: 1px solid #1e2a37; border-radius: 4px;
    background: #0f141b; top: -1px; }
QTabBar::tab { background: #18222e; color: #9fb3c8; padding: 4px 14px;
    border: none; border-top-left-radius: 4px;
    border-top-right-radius: 4px; margin-right: 2px; font-size: 9.5pt; }
QTabBar::tab:selected { background: #0f3f46; color: #7ff3ea; }
QTabBar::tab:hover { background: #22303f; }
QSpinBox { background: #0f141b; border: 1px solid #2d4055;
    border-radius: 5px; color: #dbe4ee; font-size: 9.5pt; padding: 2px 5px; }
QLabel[level="ok"] { background: #123b2a; color: #7fe0b0;
    border: 1px solid #1f6f4d; border-radius: 6px;
    padding: 2px 8px; font-size: 9.5pt; }
QLabel[level="warn"] { background: #3a2e12; color: #f0c36a;
    border: 1px solid #6f561f; border-radius: 6px;
    padding: 2px 8px; font-size: 9.5pt; }
QLabel[level="bad"] { background: #3a1518; color: #ff8d95;
    border: 1px solid #6f252b; border-radius: 6px;
    padding: 2px 8px; font-size: 9.5pt; }
"""


class CameraWindow(QtWidgets.QDialog):
    """Independent, resizable camera window.

    It owns no timer and no ROS subscription: the dashboard pushes the latest
    frame into :attr:`image_view` from its own 500 ms tick.  Closing the window
    hides it (``hiddenByUser``) instead of quitting the application, and the
    dashboard closes it for real via :meth:`shutdown` when the panel exits.
    """

    hiddenByUser = QtCore.pyqtSignal()

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("CameraWindow")
        # ASCII on purpose: see WINDOW_TITLE_MAIN/WINDOW_TITLE_CAMERA.  The
        # Chinese name of the window lives in its status labels instead.
        self.setWindowTitle(WINDOW_TITLE_CAMERA)
        self.setWindowFlags(
            QtCore.Qt.Window
            | QtCore.Qt.WindowCloseButtonHint
            | QtCore.Qt.WindowMinMaxButtonsHint
        )
        self.setModal(False)
        self.setSizeGripEnabled(True)
        self.setStyleSheet(STYLE_SHEET)
        self.setMinimumSize(640, 480)
        self.resize(*IMAGE_WINDOW_SIZE)
        self._allow_close = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        self.image_view = ZoomableImageView(self)
        layout.addWidget(self.image_view, 1)

        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(8)
        self.source_label = QtWidgets.QLabel(
            "等待 /camera/color/image_raw ...")
        self.source_label.setObjectName("Muted")
        self.zoom_label = QtWidgets.QLabel("视图 —")
        self.zoom_label.setObjectName("Muted")
        bottom.addWidget(self.source_label, 1)
        bottom.addWidget(self.zoom_label)
        layout.addLayout(bottom)

        self.image_view.viewChanged.connect(self._on_view_changed)
        self._center_on_screen()

    def _center_on_screen(self) -> None:
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            return
        available = screen.availableGeometry()
        self.move(available.center() - self.rect().center())

    def _on_view_changed(self, scale: float) -> None:
        self.zoom_label.setText(f"视图 {scale * 100:.0f}% · 双击复位")

    def set_source_status(self, text: str) -> None:
        self.source_label.setText(text)

    def reject(self) -> None:  # noqa: N802 - Esc must hide, not destroy
        self.close()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        if self._allow_close:
            event.accept()
            return
        event.ignore()
        self.hide()
        self.hiddenByUser.emit()

    def shutdown(self) -> None:
        """Really close the window when the main panel is shutting down."""
        self._allow_close = True
        self.close()


# ---------------------------------------------------------------- dashboard
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, screenshot: str | None = None,
                 allow_spawn: bool = True) -> None:
        super().__init__()
        if not ROS_AVAILABLE:
            raise RuntimeError(
                "ROS 2 Python 依赖不可用，请先 source scripts/source_dev_env.sh --full："
                + ROS_IMPORT_ERROR
            )

        self._screenshot = screenshot
        self._allow_spawn = allow_spawn

        # Task selection (dry-run / real are the safe defaults).
        self._scope = "load"
        self._mode = "safe"
        self._action = "dry-run"
        self._world = "real"
        self._pending_output = b""

        self._ros = rclpy.create_node("cs625_ui")
        self._joints: dict[str, float] = {}
        self._controller: dict[str, list[float]] = {}
        self._tcp: tuple[float, float, float] | None = None
        self._wrench: dict[str, float] = {}
        self._plot = None
        self._plot_group = "joints"
        self._task_state: dict | None = None
        self._task_state_at = 0.0
        self._tcp_quat: tuple[float, float, float, float] | None = None
        self._tcp_stamp = 0.0
        self._poses: dict[str, tuple[list[float], list[float], float]] = {}
        self._last_gripper: dict = {}
        self._last_gripper_poll = 0.0
        self._color_frame: np.ndarray | None = None
        self._depth_stats: tuple[float, float, float] | None = None
        self._cam_info: CameraInfo | None = None
        self._frame_stamps: list[float] = []
        self._frame_counter = 0
        self._shown_frame = -1
        self._last_cam_pose: tuple[float, float, float] | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._ros)

        # Live ROS camera resources (replaces the RVS ImageSave_* snapshots).
        self._ros.create_subscription(
            Image, "/camera/color/image_raw", self._on_color_image, qos_profile_sensor_data)
        self._ros.create_subscription(
            Image, "/camera/depth/image_raw", self._on_depth_image, qos_profile_sensor_data)
        self._ros.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_camera_info,
            qos_profile_sensor_data)
        self._ros.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose", self._on_tcp_pose, 10)
        self._ros.create_subscription(
            WrenchStamped, "/force_torque_sensor_broadcaster/wrench",
            self._on_wrench, 10)
        self._ros.create_subscription(RosString, "/task_state", self._on_task_state, 10)
        self._ros.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._ros.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._on_controller,
            10,
        )
        for topic, key in (
            ("/target_pose", "target"),
            ("/slot_precision_view_tcp_pose", "S0"),
            ("/slot_target_pose", "S1"),
            ("/box_precision_view_tcp_pose", "S2"),
            ("/box_target_pose", "S3"),
            ("/box_pre_grasp_tcp_pose", "pre_grasp"),
            ("/box_grasp_tcp_pose", "grasp"),
            ("/slot_insert_tcp_pose", "slot_insert"),
            ("/slot_pre_insert_rotated_tcp_pose", "slot_pre_insert_rot"),
            ("/slot_pre_insert_tcp_pose", "slot_pre_insert"),
        ):
            self._ros.create_subscription(
                PoseStamped, topic, lambda msg, k=key: self._on_pose(k, msg), 10
            )
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ros.create_subscription(
            PoseStamped, "/box_target_pose",
            lambda msg: self._on_pose("S3", msg), latched
        )

        self._image_window = CameraWindow(self)
        self._image_window.hiddenByUser.connect(self._on_image_window_hidden)

        self._build_ui()
        self._setup_task_process()
        self._sync_task_summary()
        self._task_started_at: float | None = None
        self._stop_requested = False

        # One timer for the whole panel (owner limit): it drains the ROS
        # callbacks and refreshes the status card, monitors, camera window and
        # image-window button together.  The camera window has no timer.
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(STATE_REFRESH_MS)
        self._update_image_button()
        self._refresh_gripper(force=True)
        self._refresh_image()

        if screenshot:
            # Offscreen check: never spawn a task, just show and shoot.
            self._append_log("smoke 模式：只拼命令，不启动任务子进程。")
            print("[smoke] 命令预览: " + format_task_command(
                build_task_command(self._scope, self._mode, self._action, self._world)))
            # The colour stream can be slow to deliver its first frame, so give
            # it one extra window instead of shooting a blank pane.
            QtCore.QTimer.singleShot(3000, self._smoke_maybe_finish)

    def show_image_window(self) -> None:
        """Open (or bring back) the independent camera window."""
        if not self._image_window.isVisible():
            self._image_window.show()
        self._image_window.raise_()
        self._image_window.activateWindow()
        self._update_image_button()

    def _on_image_window_hidden(self) -> None:
        self._update_image_button()
        self._append_log("图像窗口已隐藏（采集不停止，可随时重新打开）。")

    def _toggle_image_window(self) -> None:
        if self._image_window.isVisible():
            self._image_window.close()  # closeEvent hides, never quits
        else:
            self.show_image_window()

    def _smoke_maybe_finish(self) -> None:
        if self._screenshot is None:
            return
        if self._color_frame is None:
            self._append_log("smoke：3 s 内没有相机帧，再等 3 s 以便截到实时画面。")
            QtCore.QTimer.singleShot(3000, self._save_smoke)
        else:
            self._save_smoke()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        # ASCII on purpose: WSLg's Weston draws the title bar with a CJK-less
        # font, so a Chinese title would only show boxes.  The Chinese name of
        # the panel is the header label "CS625 · 六类任务主控" below.
        self.setWindowTitle(WINDOW_TITLE_MAIN)
        self.setMinimumSize(MAIN_MIN_WIDTH, 700)
        self.resize(*MAIN_WINDOW_SIZE)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        self._header_widget = self._build_header()
        self._status_card = self._build_status_card()
        self._mode_card = self._build_task_mode_group()
        self._flow_card = self._build_flow_card()
        self._monitor_card = self._build_monitor_card()
        self._log_card = self._build_log_card()

        root.addWidget(self._header_widget)
        root.addWidget(self._status_card)
        root.addWidget(self._mode_card)
        root.addWidget(self._flow_card)
        root.addWidget(self._monitor_card, 1)
        root.addWidget(self._log_card)

        self._style()

    # ----------------------------------------------------------------- header
    def _build_header(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        title = QtWidgets.QLabel("CS625 · 六类任务主控")
        title.setObjectName("Title")
        row.addWidget(title)
        row.addStretch(1)
        self.image_window_button = QtWidgets.QPushButton("图像窗口 · 已关闭")
        self.image_window_button.setObjectName("ImageToggle")
        self.image_window_button.setMinimumHeight(30)
        self.image_window_button.setToolTip(
            "显示 / 隐藏独立相机窗口：关闭图像窗口只隐藏，不会退出程序")
        self.image_window_button.clicked.connect(self._toggle_image_window)
        row.addWidget(self.image_window_button)
        return bar

    # ------------------------------------------------------------- status card
    def _build_status_card(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("状态")
        grid = QtWidgets.QGridLayout(box)
        grid.setContentsMargins(8, 16, 8, 6)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(1)
        grid.setColumnStretch(1, 1)
        self._status_values: dict[str, QtWidgets.QLabel] = {}
        for row, (key, name) in enumerate(STATUS_ROWS):
            name_label = QtWidgets.QLabel(name)
            name_label.setObjectName("StatusName")
            name_label.setFixedWidth(64)
            value_label = QtWidgets.QLabel("—")
            value_label.setObjectName("StatusValue")
            value_label.setMinimumHeight(20)
            value_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                      QtWidgets.QSizePolicy.Preferred)
            grid.addWidget(name_label, row, 0)
            grid.addWidget(value_label, row, 1)
            self._status_values[key] = value_label
        self._set_state("robot", "等待 /joint_states ...", "warn")
        self._set_state("camera", "等待 /camera/color/image_raw ...", "warn")
        self._set_state("env", "—", "idle")
        self._set_state("run", "空闲", "idle")
        return box

    # ---------------------------------------------------------- monitor card
    def _on_task_state(self, msg) -> None:  # noqa: ANN001 - ROS callback
        """Live flow/phase feed from scripts/run_pick_and_place.py."""
        try:
            self._task_state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._task_state_at = time.time()

    def _on_wrench(self, msg) -> None:  # noqa: ANN001 - ROS callback
        force, torque = msg.wrench.force, msg.wrench.torque
        self._wrench = {"fx": force.x, "fy": force.y, "fz": force.z,
                        "tx": torque.x, "ty": torque.y, "tz": torque.z}

    def _read_task_state_file(self):
        """Fallback when the panel started before the phase runner did."""
        path = REPO / "runtime" / "task_state.json"
        try:
            if path.exists():
                return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        return None

    def _build_flow_card(self):
        """Where am I in the task: flow, phase, progress."""
        box = QtWidgets.QGroupBox("当前流程")
        layout = QtWidgets.QVBoxLayout(box)
        layout.setContentsMargins(8, 16, 8, 6)
        layout.setSpacing(3)
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(6)
        self.flow_dot = StatusDot()
        head.addWidget(self.flow_dot)
        self.flow_label = QtWidgets.QLabel("空闲 · 等待任务开始")
        self.flow_label.setObjectName("StatusValue")
        head.addWidget(self.flow_label, 1)
        layout.addLayout(head)
        self.phase_bar = QtWidgets.QProgressBar()
        self.phase_bar.setRange(0, 1)
        self.phase_bar.setValue(0)
        self.phase_bar.setTextVisible(False)
        self.phase_bar.setFixedHeight(10)
        layout.addWidget(self.phase_bar)
        self.phase_label = QtWidgets.QLabel("阶段 —")
        self.phase_label.setObjectName("Mono")
        layout.addWidget(self.phase_label)
        return box

    def _refresh_flow_card(self) -> None:
        state = self._task_state
        if state is None or time.time() - self._task_state_at > 5.0:
            state = self._read_task_state_file()
        if not state:
            if getattr(self, "flow_dot", None) is not None:
                self.flow_dot.set_active(False)
            self.flow_label.setText("空闲 · 等待任务开始")
            self.phase_label.setText("阶段 —")
            self.phase_bar.setRange(0, 1)
            self.phase_bar.setValue(0)
            return
        flow = str(state.get("flow") or "—")
        mode = str(state.get("mode") or "")
        world = str(state.get("world") or "")
        status = str(state.get("status") or "")
        index = int(state.get("phase_index") or 0)
        total = int(state.get("phase_total") or 0)
        phase = str(state.get("phase") or "—")
        parts = [SCOPE_LABELS.get(flow, flow)]
        if mode:
            parts.append(MODE_LABELS.get(mode, mode))
        if world:
            parts.append(WORLD_LABELS.get(world, world))
        level = "idle"
        if status == "running":
            parts.append("运行中")
            level = "ok"
        elif status == "failed":
            parts.append("失败")
            level = "bad"
        self.flow_label.setText(" · ".join(parts))
        if getattr(self, "flow_dot", None) is not None:
            self.flow_dot.set_active(level == "ok")
        self.flow_label.setProperty("level", level)
        style = self.flow_label.style()
        style.unpolish(self.flow_label)
        style.polish(self.flow_label)
        if total > 0:
            self.phase_bar.setRange(0, total)
            self.phase_bar.setValue(max(0, min(index, total)))
        self.phase_label.setText("阶段 {}/{} · {}".format(index, total, phase))

    # Three channel groups for the oscilloscope tab.  The right-hand value is the
    # key the source publishes under, so the mapping stays obvious.
    PLOT_GROUPS = {
        "joints": ("关节位置 (rad)",
                   [(name.replace("_joint", ""), name) for name in JOINT_ORDER]),
        "tcp": ("TCP 位置 (m)", [("x", "x"), ("y", "y"), ("z", "z")]),
        "force": ("力/力矩 (N, N·m)",
                  [("fx", "fx"), ("fy", "fy"), ("fz", "fz"),
                   ("tx", "tx"), ("ty", "ty"), ("tz", "tz")]),
    }

    def _build_plot_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(3)
        if not PLOTS_AVAILABLE:
            label = QtWidgets.QLabel("曲线控件不可用（{}）".format(_PLOTS_ERROR))
            label.setWordWrap(True)
            layout.addWidget(label)
            return tab
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        self._plot_buttons = {}
        for key, text in (("joints", "关节"), ("tcp", "TCP"), ("force", "力/力矩")):
            button = QtWidgets.QPushButton(text)
            button.setCheckable(True)
            button.setFixedHeight(22)
            button.clicked.connect(
                lambda _checked=False, k=key: self._select_plot_group(k))
            row.addWidget(button)
            self._plot_buttons[key] = button
        row.addStretch(1)
        pause = QtWidgets.QPushButton("暂停")
        pause.setCheckable(True)
        pause.setFixedHeight(22)
        pause.toggled.connect(
            lambda on: self._plot.set_paused(bool(on)) if self._plot else None)
        row.addWidget(pause)
        clear = QtWidgets.QPushButton("清空")
        clear.setFixedHeight(22)
        clear.clicked.connect(lambda: self._plot.clear() if self._plot else None)
        row.addWidget(clear)
        export = QtWidgets.QPushButton("导出")
        export.setFixedHeight(22)
        export.clicked.connect(self._export_plot_csv)
        row.addWidget(export)
        layout.addLayout(row)
        self._plot = RealtimePlot(("j1",), window_seconds=20.0)
        self._plot.setMinimumHeight(168)
        layout.addWidget(self._plot, 1)
        self._select_plot_group("joints")
        return tab

    def _select_plot_group(self, key: str) -> None:
        self._plot_group = key
        for name, button in getattr(self, "_plot_buttons", {}).items():
            button.setChecked(name == key)
        if self._plot is None:
            return
        label, pairs = self.PLOT_GROUPS[key]
        self._plot.set_channels([name for name, _source in pairs])
        self._plot.setToolTip(label)

    def _feed_plots(self) -> None:
        """Push one sample per tick; the widget coalesces its own repaints."""
        if self._plot is None:
            return
        _label, pairs = self.PLOT_GROUPS[self._plot_group]
        values = {}
        for name, source in pairs:
            if source in self._joints:
                values[name] = float(self._joints[source])
            elif source in self._wrench:
                values[name] = float(self._wrench[source])
            elif source in ("x", "y", "z") and self._tcp is not None:
                values[name] = float(self._tcp[{"x": 0, "y": 1, "z": 2}[source]])
        if values:
            self._plot.push_sample(time.time(), values)

    def _export_plot_csv(self) -> None:
        if self._plot is None:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出曲线数据", str(Path.home() / "cs625_plot.csv"), "CSV (*.csv)")
        if not path:
            return
        try:
            rows = self._plot.export_csv(path)
            self._append_log("曲线已导出：{}（{} 行）".format(path, rows))
        except Exception as exc:  # noqa: BLE001
            self._append_log("曲线导出失败：{}".format(exc))

    def _build_monitor_card(self) -> QtWidgets.QGroupBox:
        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("MonitorTabs")
        tabs.addTab(self._build_joint_tab(), "关节")
        tabs.addTab(self._build_camera_tab(), "相机")
        tabs.addTab(self._build_gripper_tab(), "夹爪")
        tabs.addTab(self._build_plot_tab(), "曲线")
        tabs.setMinimumHeight(168)
        return self._group("监视", tabs)

    def _build_joint_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(3)

        self.joint_table = QtWidgets.QTableWidget(len(JOINT_ORDER), 5)
        self.joint_table.setHorizontalHeaderLabels(
            ["关节", "实际", "参考", "反馈", "误差"]
        )
        self.joint_table.verticalHeader().setVisible(False)
        header = self.joint_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        for column in range(1, 5):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Stretch)
        self.joint_table.setColumnWidth(0, 104)
        for column, tip in ((1, "实测关节角 (rad)"), (2, "控制器参考 (rad)"),
                            (3, "控制器反馈 (rad)"), (4, "跟踪误差 (rad)")):
            item = self.joint_table.horizontalHeaderItem(column)
            if item is not None:
                item.setToolTip(tip)
        self.joint_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.joint_table.setObjectName("JointTable")
        # The compact panel leaves room for ~4 rows; let the operator scroll to
        # the remaining joints instead of silently clipping them.
        self.joint_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.joint_table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.joint_table.verticalHeader().setDefaultSectionSize(24)
        self.joint_table.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                       QtWidgets.QSizePolicy.Expanding)
        self.joint_table.setToolTip("单位 rad；数据来自 /joint_states 与 "
                                    "/arm_controller/controller_state")
        layout.addWidget(self.joint_table, 1)

        self.tcp_label = QtWidgets.QLabel("TCP: 等待 /tcp_pose_broadcaster/pose ...")
        self.tcp_label.setObjectName("Mono")
        self.tcp_label.setWordWrap(True)
        layout.addWidget(self.tcp_label)
        return tab

    def _build_camera_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)
        layout.addWidget(self._build_camera_grid())
        self._camera_note = QtWidgets.QLabel(
            "原始帧不做处理；滚轮/拖拽只影响独立图像窗口的显示。")
        self._camera_note.setObjectName("Muted")
        self._camera_note.setWordWrap(True)
        layout.addWidget(self._camera_note)
        layout.addStretch(1)
        return tab

    def _build_gripper_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        layout.addWidget(self._build_gripper_group())
        return tab

    def _build_camera_grid(self) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(container)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        self._camera_value_labels: dict[str, QtWidgets.QLabel] = {}
        for index, (key, name) in enumerate(CAMERA_FIELDS):
            row, column = divmod(index, 2)
            name_label = QtWidgets.QLabel(name)
            name_label.setObjectName("Muted")
            value_label = QtWidgets.QLabel("—")
            value_label.setObjectName("Mono")
            value_label.setMinimumHeight(20)
            grid.addWidget(name_label, row, column * 2)
            grid.addWidget(value_label, row, column * 2 + 1)
            self._camera_value_labels[key] = value_label
        pose_name = QtWidgets.QLabel("相机在基座系")
        pose_name.setObjectName("Muted")
        self._camera_pose_label = QtWidgets.QLabel("—")
        self._camera_pose_label.setObjectName("Mono")
        self._camera_pose_label.setMinimumHeight(20)
        grid.addWidget(pose_name, 4, 0)
        grid.addWidget(self._camera_pose_label, 4, 1, 1, 3)
        return container

    def _build_gripper_group(self) -> QtWidgets.QWidget:
        self.gripper_position = QtWidgets.QLabel("位置: -- mm")
        self.gripper_state = QtWidgets.QLabel("状态: --")
        self.gripper_init = QtWidgets.QLabel("初始化: --")
        for label in (self.gripper_position, self.gripper_state, self.gripper_init):
            label.setObjectName("Mono")

        self.open_button = QtWidgets.QPushButton("张开到 50 mm")
        self.close_button = QtWidgets.QPushButton("收拢/闭合")
        self.init_button = QtWidgets.QPushButton("初始化 / 回零")
        self.refresh_button = QtWidgets.QPushButton("刷新状态")
        self.grip_value = QtWidgets.QSpinBox()
        self.grip_value.setRange(0, 80)
        self.grip_value.setValue(50)
        self.grip_value.setSuffix(" mm")
        self.grip_button = QtWidgets.QPushButton("夹紧 50 mm")
        self.open_button.clicked.connect(lambda: self._command_gripper("open", 50))
        self.close_button.clicked.connect(lambda: self._command_gripper("close", 0))
        self.init_button.clicked.connect(lambda: self._command_gripper("init", 0))
        self.grip_button.clicked.connect(
            lambda: self._command_gripper("grip", self.grip_value.value())
        )
        self.refresh_button.clicked.connect(lambda: self._refresh_gripper(force=True))
        self.grip_value.valueChanged.connect(
            lambda value: self.grip_button.setText(f"夹紧 {value} mm")
        )

        inner = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(inner)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        line = QtWidgets.QHBoxLayout()
        line.setSpacing(10)
        for label in (self.gripper_position, self.gripper_state, self.gripper_init):
            line.addWidget(label)
        line.addStretch(1)
        self.gripper_badge = self._badge("夹爪未查询", "warn")
        line.addWidget(self.gripper_badge)
        layout.addLayout(line)

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)
        grid.addWidget(self.refresh_button, 0, 0)
        grid.addWidget(self.init_button, 0, 1)
        grid.addWidget(self.close_button, 1, 0)
        grid.addWidget(self.open_button, 1, 1)
        grid.addWidget(self.grip_value, 2, 0)
        grid.addWidget(self.grip_button, 2, 1)
        layout.addLayout(grid)
        for widget in (self.refresh_button, self.init_button, self.close_button,
                       self.open_button, self.grip_button, self.grip_value):
            widget.setMinimumHeight(30)
        self.gripper_note = QtWidgets.QLabel(
            "抓取前先“初始化 / 回零”再收拢；直接闭合到 0 会留间隙。"
        )
        self.gripper_note.setObjectName("Muted")
        self.gripper_note.setWordWrap(True)
        layout.addWidget(self.gripper_note)
        return inner

    # ----------------------------------------------------------- task modes
    def _build_task_mode_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("任务模式")
        layout = QtWidgets.QVBoxLayout(box)
        layout.setContentsMargins(8, 16, 8, 6)
        layout.setSpacing(4)

        self._choice_keys: dict[str, list[str]] = {}
        self._choice_buttons: dict[str, dict[str, QtWidgets.QPushButton]] = {}
        self._choice_groups: dict[str, QtWidgets.QButtonGroup] = {}

        self._add_choice_row(layout, "范围", "scope", (
            ("load", "装入"), ("unload", "取出"), ("full", "全流程")))
        self._add_choice_row(layout, "模式", "mode", (
            ("safe", "安全模式"), ("efficient", "效率模式")))
        self._add_choice_row(layout, "动作", "action", (
            ("dry-run", "预演"), ("plan-only", "只规划"), ("confirm", "执行")))
        self._add_choice_row(layout, "环境", "world", (
            ("real", "真机"), ("sim", "仿真")))

        self._choice_buttons["scope"][self._scope].setChecked(True)
        self._choice_buttons["mode"][self._mode].setChecked(True)
        self._choice_buttons["action"][self._action].setChecked(True)
        self._choice_buttons["world"][self._world].setChecked(True)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(6)
        self.start_button = FlatButton("开始任务", role="start")
        self.start_button.setProperty("role", "start")
        self.start_button.setMinimumHeight(44)
        self.start_button.setToolTip(
            "拼出 run_task.py 命令行，先写日志，再异步执行。\n"
            "预演只打印阶段表，只规划只做规划，执行才会驱动机械臂。")
        self.stop_button = FlatButton("停止", role="stop")
        self.stop_button.setProperty("role", "stop")
        self.stop_button.setMinimumHeight(44)
        self.stop_button.setEnabled(False)
        self.stop_button.setToolTip("终止 run_task.py 子进程")
        self.start_button.clicked.connect(self._start_task)
        self.stop_button.clicked.connect(self._stop_task)
        buttons.addWidget(self.start_button, 5)
        buttons.addWidget(self.stop_button, 3)
        layout.addLayout(buttons)
        return box

    def _add_choice_row(self, layout: QtWidgets.QVBoxLayout, title: str, kind: str,
                        options: Sequence[tuple[str, str]]) -> None:
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        label = QtWidgets.QLabel(title)
        label.setObjectName("Muted")
        label.setFixedWidth(30)
        row.addWidget(label)
        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)
        buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (key, text) in enumerate(options):
            button = QtWidgets.QPushButton(text)
            button.setCheckable(True)
            button.setProperty("role", "choice")
            button.setProperty("kind", kind)
            button.setMinimumHeight(30)
            group.addButton(button, index)
            row.addWidget(button)
            buttons[key] = button
        group.buttonClicked[int].connect(
            lambda index, kind=kind: self._on_choice_clicked(kind, index)
        )
        layout.addLayout(row)
        self._choice_keys[kind] = [key for key, _text in options]
        self._choice_buttons[kind] = buttons
        self._choice_groups[kind] = group

    # ------------------------------------------------------------- task log
    def _build_log_card(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("任务日志")
        layout = QtWidgets.QVBoxLayout(box)
        layout.setContentsMargins(8, 16, 8, 6)
        layout.setSpacing(2)

        head = QtWidgets.QHBoxLayout()
        head.setSpacing(6)
        self._log_hint = QtWidgets.QLabel("最近 500 行 · run_task.py 实时输出")
        self._log_hint.setObjectName("Muted")
        head.addWidget(self._log_hint, 1)
        self._log_toggle = QtWidgets.QToolButton()
        self._log_toggle.setObjectName("LogToggle")
        self._log_toggle.setText("收起")
        self._log_toggle.setCheckable(True)
        self._log_toggle.setChecked(True)
        self._log_toggle.setToolTip("折叠 / 展开任务日志")
        self._log_toggle.toggled.connect(self._toggle_log)
        head.addWidget(self._log_toggle)
        layout.addLayout(head)

        self.log = self._build_log()
        layout.addWidget(self.log)
        box.setToolTip("run_task.py 的 stdout/stderr 实时输出，保留最近 500 行")
        return box

    def _build_log(self) -> "LogView":
        log = LogView()
        log.setReadOnly(True)
        log.setMaximumBlockCount(500)
        log.setObjectName("Log")
        log.setFixedHeight(LOG_VIEW_HEIGHT)
        log.setPlaceholderText("任务命令与 run_task.py 输出会实时出现在这里。")
        return log

    def _toggle_log(self, expanded: bool) -> None:
        self.log.setVisible(expanded)
        self._log_toggle.setText("收起" if expanded else "展开")
        self._log_hint.setText(
            "最近 500 行 · run_task.py 实时输出" if expanded else "日志已折叠")
        # Reflow the card and give the reclaimed height back to the window
        # instead of leaving a blank log block behind.
        self._log_card.layout().activate()
        self.centralWidget().layout().activate()
        if expanded:
            self.resize(self.width(),
                        getattr(self, "_log_expanded_height", self.height()))
        else:
            self._log_expanded_height = self.height()
            self.resize(self.width(),
                        max(self.minimumHeight(), self.minimumSizeHint().height()))

    @staticmethod
    def _group(title: str, widget) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout(box)
        layout.setContentsMargins(8, 14, 8, 8)
        if isinstance(widget, QtWidgets.QLayout):
            layout.addLayout(widget)
        else:
            layout.addWidget(widget)
        return box

    def _style(self) -> None:
        self.setStyleSheet(STYLE_SHEET)

    @staticmethod
    def _badge(text: str, level: str) -> QtWidgets.QLabel:
        badge = QtWidgets.QLabel(text)
        badge.setProperty("level", level)
        badge.setAlignment(QtCore.Qt.AlignCenter)
        badge.setMinimumHeight(30)
        return badge

    # ------------------------------------------------------------- task mode
    def _on_choice_clicked(self, kind: str, index: int) -> None:
        key = self._choice_keys[kind][index]
        current = getattr(self, f"_{kind}")
        if kind == "action" and key == "confirm" and current != "confirm":
            if not self._confirm_execute_mode():
                self._choice_buttons["action"][current].setChecked(True)
                self._append_log("已取消“执行”，保持“{}”。".format(ACTION_LABELS[current]))
                return
        setattr(self, f"_{kind}", key)
        self._sync_task_summary()

    def _confirm_execute_mode(self) -> bool:
        answer = QtWidgets.QMessageBox.warning(
            self,
            "Confirm: switch to Execute",
            "“执行”模式会让 run_task.py 真正驱动机械臂做运动。\n\n"
            "确认切换吗？点击“开始任务”时还会再确认一次。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        return answer == QtWidgets.QMessageBox.Yes

    def _sync_task_summary(self) -> None:
        if not hasattr(self, "_status_values"):
            return
        text = "{} · {} · {} · {}".format(
            SCOPE_LABELS[self._scope], MODE_LABELS[self._mode],
            ACTION_LABELS[self._action], WORLD_LABELS[self._world])
        self._set_state("task", text, "ok")
        self._update_image_button()

    # ------------------------------------------------------------ subprocess
    def _setup_task_process(self) -> None:
        self._process = QtCore.QProcess(self)
        self._process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        # The log pane decodes child output as UTF-8.  Python chooses its stdout
        # encoding from the locale, so a child started from a shell with
        # LANG=C / LANG=zh_CN.GBK would emit non-UTF-8 bytes and turn into
        # mojibake (or U+FFFD) in the log.  Pin the child to UTF-8 so the text
        # is correct at the source instead of patching the decode side.
        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONIOENCODING", "utf-8")
        self._process.setProcessEnvironment(env)
        self._process.setWorkingDirectory(str(REPO))
        self._process.started.connect(self._on_task_started)
        self._process.readyReadStandardOutput.connect(self._drain_task_output)
        self._process.finished.connect(self._on_task_finished)
        self._process.errorOccurred.connect(self._on_task_error)

    def _start_task(self) -> None:
        if self._process.state() != QtCore.QProcess.NotRunning:
            self._append_log("已有任务在运行，请先停止。")
            return
        try:
            command = build_task_command(
                self._scope, self._mode, self._action, self._world)
        except ValueError as exc:
            self._append_log(f"命令拼装失败: {exc}")
            return

        # Print the exact command before anything is executed.
        self._append_log("")
        self._append_log("=" * 46)
        self._append_log(
            "任务: {} / {} / {} / {}".format(
                SCOPE_LABELS[self._scope], MODE_LABELS[self._mode],
                ACTION_LABELS[self._action], WORLD_LABELS[self._world])
        )
        self._append_log("命令: " + format_task_command(command))
        self._append_log(f"工作目录: {REPO}")

        if not self._allow_spawn:
            self._append_log("smoke 模式：只拼命令，不启动任务子进程。")
            return

        if self._action == "confirm":
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm: real task",
                "即将以“执行”模式启动：\n\n"
                f"{format_task_command(command)}\n\n"
                "这会驱动机械臂做真实运动，确认继续？",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                self._append_log("已取消执行，未启动子进程。")
                return

        self._pending_output = b""
        self._stop_requested = False
        self._append_log("启动子进程 ...")
        self._process.start(command[0], command[1:])

    def _stop_task(self) -> None:
        if self._process.state() == QtCore.QProcess.NotRunning:
            self._append_log("当前没有运行中的任务。")
            return
        self._stop_requested = True
        pid = int(self._process.processId())
        children = _descendant_pids(pid) if pid else []
        if children:
            self._append_log(f"停止任务：先终止子进程树 {children}")
        self._append_log("停止任务：terminate()")
        for child in children:
            try:
                os.kill(child, signal.SIGTERM)
            except OSError:
                pass
        self._process.terminate()
        QtCore.QTimer.singleShot(3000, self._kill_if_running)

    def _kill_if_running(self) -> None:
        if self._process.state() != QtCore.QProcess.NotRunning:
            pid = int(self._process.processId())
            for child in (_descendant_pids(pid) if pid else []):
                try:
                    os.kill(child, signal.SIGKILL)
                except OSError:
                    pass
            self._append_log("子进程 3 s 内未退出，kill()")
            self._process.kill()

    def _on_task_started(self) -> None:
        pid = int(self._process.processId())
        self._task_started_at = time.time()
        self._append_log(f"子进程已启动 pid={pid}")
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self._set_state("run", f"运行中 pid={pid} · 00:00", "ok")

    def _drain_task_output(self) -> None:
        data = bytes(self._process.readAllStandardOutput())
        if not data:
            return
        self._pending_output += data
        *lines, rest = self._pending_output.split(b"\n")
        self._pending_output = rest
        for raw in lines:
            self._append_log(raw.decode("utf-8", "replace").replace("\r", ""))

    def _flush_task_output(self) -> None:
        if self._pending_output:
            self._append_log(
                self._pending_output.decode("utf-8", "replace").replace("\r", ""))
            self._pending_output = b""

    def _on_task_finished(self, exit_code: int, exit_status) -> None:
        self._flush_task_output()
        if self._stop_requested:
            self._append_log(f"进程结束：已停止（退出码 {exit_code}）")
            self._set_state("run", f"已停止 · 退出码 {exit_code}", "warn")
        elif exit_code == 0:
            self._append_log("进程结束：退出码 0（正常）")
            self._set_state("run", "已完成 · 退出码 0", "ok")
        else:
            self._append_log(f"进程结束：退出码 {exit_code}（异常）")
            self._set_state("run", f"失败 · 退出码 {exit_code}", "bad")
        self._stop_requested = False
        self._task_started_at = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def _on_task_error(self, error) -> None:
        if self._stop_requested and error == QtCore.QProcess.Crashed:
            # Expected: terminate()/kill() makes QProcess report a crash.
            self._append_log("子进程已按停止请求退出。")
            return
        names = {
            QtCore.QProcess.FailedToStart: "启动失败",
            QtCore.QProcess.Crashed: "进程崩溃",
            QtCore.QProcess.Timedout: "等待超时",
            QtCore.QProcess.WriteError: "写入失败",
            QtCore.QProcess.ReadError: "读取失败",
            QtCore.QProcess.UnknownError: "未知错误",
        }
        self._append_log(f"子进程错误: {names.get(error, error)}")
        if error == QtCore.QProcess.FailedToStart:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self._task_started_at = None
            self._set_state("run", "启动失败", "bad")

    # ------------------------------------------------------------- ROS callbacks
    @staticmethod
    def _image_to_array(msg: Image, channels: int, dtype) -> np.ndarray:
        """Decode a raw sensor_msgs/Image without cv_bridge.

        cv_bridge pulls in OpenCV, and OpenCV ships its own Qt plugin directory,
        which hijacks PyQt5's xcb plugin and aborts the whole window
        ("Could not load the Qt platform plugin xcb").  Raw decoding keeps the
        panel independent of OpenCV.
        """

        expected = msg.height * msg.step
        if len(msg.data) < expected:
            raise ValueError(f"image data {len(msg.data)} < {expected}")
        array = np.frombuffer(msg.data, dtype=np.uint8)
        array = array[:expected].reshape(msg.height, msg.step)
        row = msg.width * channels
        array = array[:, :row].reshape(msg.height, msg.width, channels)
        return array

    def _on_color_image(self, msg: Image) -> None:
        try:
            encoding = msg.encoding.lower()
            if encoding in ("bgr8", "rgb8"):
                frame = self._image_to_array(msg, 3, np.uint8)
                self._color_frame = frame if encoding == "bgr8" else frame[:, :, ::-1].copy()
            elif encoding in ("mono8", "8uc1"):
                grey = self._image_to_array(msg, 1, np.uint8)[:, :, 0]
                self._color_frame = np.dstack([grey, grey, grey])
            else:
                raise ValueError(f"unsupported colour encoding {msg.encoding}")
            self._frame_counter += 1
            now = time.time()
            self._frame_stamps = [t for t in self._frame_stamps if now - t < 3.0] + [now]
        except Exception as exc:  # noqa: BLE001
            self._append_log(f"彩色解码失败: {exc}")

    def _on_depth_image(self, msg: Image) -> None:
        try:
            if msg.encoding.lower() not in ("16uc1", "mono16"):
                self._depth_stats = (-1.0, -1.0, -1.0)
                return
            depth = self._image_to_array(msg, 2, np.uint16).view(np.uint16)[:, :, 0]
            valid = depth[depth > 0]
            ratio = float(valid.size) / float(depth.size) if depth.size else 0.0
            median = float(np.median(valid)) / 1000.0 if valid.size else 0.0
            self._depth_stats = (ratio, median, float(depth.size))
        except Exception as exc:  # noqa: BLE001
            self._append_log(f"深度解码失败: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._cam_info = msg

    def _on_tcp_pose(self, msg: PoseStamped) -> None:
        pose = msg.pose
        self._tcp = (pose.position.x, pose.position.y, pose.position.z)
        self._tcp_quat = (pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w)
        self._tcp_stamp = time.time()

    def _on_joint_state(self, msg: JointState) -> None:
        self._joints = dict(zip(msg.name, msg.position))

    def _on_controller(self, msg: JointTrajectoryControllerState) -> None:
        self._controller = {
            "reference": list(msg.reference.positions),
            "feedback": list(msg.feedback.positions),
            "error": list(msg.error.positions),
        }

    def _on_pose(self, key: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self._poses[key] = ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], time.time())

    # ---------------------------------------------------------------- timers
    def _tick(self) -> None:
        """The single 500 ms refresh of the whole panel.

        Draining several callbacks in one tick keeps the displayed camera fps
        useful even though the panel only wakes up twice per second.
        """
        for _ in range(SPIN_CALLS_PER_TICK):
            try:
                rclpy.spin_once(self._ros, timeout_sec=0.0)
            except Exception:
                break
        self._refresh_tcp()
        self._refresh_tables()
        self._refresh_status()
        self._refresh_flow_card()
        self._feed_plots()
        self._refresh_image()
        self._update_image_button()
        if time.time() - self._last_gripper_poll > 4.0:
            self._refresh_gripper()

    def _fps(self) -> float:
        if len(self._frame_stamps) < 2:
            return 0.0
        return (len(self._frame_stamps) - 1) / max(
            self._frame_stamps[-1] - self._frame_stamps[0], 1e-6)

    def _refresh_status(self) -> None:
        """One row per subsystem, all on the single state timer."""
        joint_count = sum(1 for name in JOINT_ORDER if name in self._joints)
        if joint_count:
            # An all-zero pose is a mock / not-enabled controller, not a healthy arm.
            values = [abs(self._joints.get(name, 0.0)) for name in JOINT_ORDER]
            if max(values) < 1e-6:
                self._set_state("robot", f"{joint_count}/6 关节 · 全零(未使能)", "warn")
            else:
                text = f"在线 · {joint_count}/6 关节"
                errors = self._controller.get("error") if self._controller else None
                if errors:
                    text += f" · 跟踪误差 {max(abs(v) for v in errors):.3f} rad"
                self._set_state("robot", text, "ok")
        else:
            self._set_state("robot", "等待 /joint_states ...", "warn")

        info = self._cam_info
        if self._color_frame is not None:
            height, width = self._color_frame.shape[:2]
            resolution = (f"{info.width}x{info.height}" if info is not None
                          else f"{width}x{height}")
            text = f"{resolution} · {self._fps():.1f} fps"
            if self._depth_stats is not None:
                text += f" · 深度有效 {self._depth_stats[0] * 100:.0f}%"
            self._set_state("camera", text, "ok")
        else:
            self._set_state(
                "camera", "等待 /camera/color/image_raw ...", "warn")

        world = WORLD_LABELS[self._world]
        if self._last_cam_pose is not None:
            self._set_state(
                "env",
                "{} · 相机在基座系 {:.3f}, {:.3f}, {:.3f} m".format(
                    world, *self._last_cam_pose),
                "ok",
            )
        else:
            self._set_state("env", f"{world} · 等待 TCP / 手眼标定", "warn")

        if self._process.state() != QtCore.QProcess.NotRunning:
            started = self._task_started_at or time.time()
            elapsed = max(0, int(time.time() - started))
            self._set_state(
                "run",
                f"运行中 pid={int(self._process.processId())} · "
                f"{elapsed // 60:02d}:{elapsed % 60:02d}",
                "ok",
            )

    def _refresh_tcp(self) -> None:
        """TCP from /tcp_pose_broadcaster/pose -- TF is still split into two trees."""

        if self._tcp is None or self._tcp_quat is None:
            self.tcp_label.setText("TCP: 等待 /tcp_pose_broadcaster/pose ...")
            return
        x, y, z = self._tcp
        if not all(math.isfinite(value) for value in (*self._tcp, *self._tcp_quat)):
            self.tcp_label.setText(
                "TCP: 话题数据无效（nan），检查 /tcp_pose_broadcaster/pose / 机械臂驱动")
            self._last_cam_pose = None
            return
        self.tcp_label.setText(
            f"TCP: {x: .4f}, {y: .4f}, {z: .4f} m · "
            f"{time.strftime('%H:%M:%S', time.localtime(self._tcp_stamp))}"
        )
        self.tcp_label.setToolTip(
            "话题 /tcp_pose_broadcaster/pose；frame: base → my_end_effector_link")
        try:
            calib = np.array(json.loads(
                (REPO / "config/handeye/tcp_to_camera.json").read_text())["T_tcp_cam"],
                dtype=float)
            rotation = Rotation.from_quat(list(self._tcp_quat))
            camera = np.array([x, y, z]) + rotation.as_matrix() @ calib[:3, 3]
            self._last_cam_pose = (float(camera[0]), float(camera[1]), float(camera[2]))
        except Exception:
            self._last_cam_pose = None

    def _refresh_tables(self) -> None:
        self.joint_table.setRowCount(len(JOINT_ORDER))
        for row, name in enumerate(JOINT_ORDER):
            actual = self._joints.get(name)
            ref = fb = err = None
            if self._controller:
                # controller_state is in the controller's own joint order.
                try:
                    idx = self._controller_reference_order().index(name)
                    ref = self._controller["reference"][idx]
                    fb = self._controller["feedback"][idx]
                    err = self._controller["error"][idx]
                except (ValueError, IndexError):
                    pass
            values = [
                name.replace("_joint", ""),
                "" if actual is None else f"{actual: .3f}",
                "" if ref is None else f"{ref: .3f}",
                "" if fb is None else f"{fb: .3f}",
                "" if err is None else f"{err: .3f}",
            ]
            for column, value in enumerate(values):
                item = self.joint_table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.joint_table.setItem(row, column, item)
                item.setText(value)
                item.setForeground(QtGui.QColor(
                    "#ff8d95" if (err is not None and abs(err) > 0.02) else "#dbe4ee"))

        info = self._cam_info
        depth = self._depth_stats
        fps = 0.0
        if len(self._frame_stamps) >= 2:
            fps = (len(self._frame_stamps) - 1) / max(
                self._frame_stamps[-1] - self._frame_stamps[0], 1e-6)
        values = {
            "color": "运行中" if self._color_frame is not None else "无数据",
            "depth": "运行中" if depth is not None else "无数据",
            "resolution": f"{info.width}x{info.height}" if info is not None else "—",
            "fps": f"{fps:.1f} fps" if fps else "—",
            "valid": f"{depth[0] * 100:.1f}%" if depth is not None else "—",
            "depth_median": f"{depth[1]:.3f} m" if depth is not None else "—",
            "intrinsics": (
                f"{info.k[0]:.1f} / {info.k[4]:.1f}" if info is not None else "—"),
            "stamp": (
                time.strftime("%H:%M:%S", time.localtime(self._frame_stamps[-1]))
                if self._frame_stamps else "—"),
        }
        for key, label in self._camera_value_labels.items():
            label.setText(values[key])
        if info is not None:
            self._camera_value_labels["resolution"].setToolTip(
                f"frame_id: {info.header.frame_id}")
        self._camera_pose_label.setText(
            "—" if self._last_cam_pose is None
            else "{:.3f}, {:.3f}, {:.3f}".format(*self._last_cam_pose))

    def _controller_reference_order(self) -> list[str]:
        # /arm_controller/controller_state is published in this standard order.
        return list(JOINT_ORDER)

    # -------------------------------------------------------------- gripper
    def _gripper_call(self, command: str, value: float | None = None) -> dict:
        """Call the gripper through the one-shot PowerShell helper.

        The helper opens COM9 per call, so the panel keeps working even when the
        persistent TCP bridge on :7500 is not running.
        """
        verb = command.split()[0].lower()
        args = [
            "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", GRIPPER_CMD_PS1, "-Cmd", verb,
        ]
        if value is not None:
            args += ["-Value", str(value)]
        try:
            completed = subprocess.run(args, capture_output=True, timeout=25)
        except Exception as exc:  # noqa: BLE001 - surfaced in the log panel
            raise RuntimeError(str(exc)) from exc

        def decode(payload: bytes) -> str:
            """PowerShell writes Chinese messages in the console code page (GBK)."""
            for encoding in ("utf-8", "gbk", "cp936"):
                try:
                    return payload.decode(encoding)
                except UnicodeDecodeError:
                    continue
            return payload.decode("utf-8", "replace")

        stdout = decode(completed.stdout or b"")
        stderr = decode(completed.stderr or b"")
        lines = [line for line in stdout.strip().splitlines() if line.strip()]
        if not lines:
            raise RuntimeError(stderr.strip() or "empty response")
        return json.loads(lines[-1])

    def _refresh_gripper(self, force: bool = False) -> None:
        if not force and time.time() - self._last_gripper_poll < 3.0:
            return
        self._last_gripper_poll = time.time()
        try:
            data = self._gripper_call("status")
            self._last_gripper = data
            if data.get("ok"):
                self._set_badge(self.gripper_badge, "夹爪在线", "ok")
                self.gripper_position.setText(
                    f"位置: {data.get('position_mm', '--')} mm"
                )
                state = {0: "到位", 1: "运动中", 2: "夹持中", 3: "掉落"}.get(
                    data.get("state"), str(data.get("state"))
                )
                self.gripper_state.setText(f"状态: {state}")
                init_value = data.get("init")
                init_ok = init_value == 5
                self.gripper_init.setText(
                    f"初始化: {'完成' if init_ok else '未完成'} ({init_value})"
                )
                self.open_button.setEnabled(init_ok)
                self.close_button.setEnabled(init_ok)
                self.close_button.setToolTip(
                    "" if init_ok else "请先初始化夹爪；未初始化时闭合可能不到位"
                )
            else:
                self._set_badge(self.gripper_badge, "夹爪错误", "bad")
        except Exception as exc:
            self._set_badge(self.gripper_badge, "夹爪离线", "bad")
            self._append_log(f"夹爪桥不可用: {exc}")

    def _command_gripper(self, command: str, target: int) -> None:
        if command == "grip":
            text = f"确认夹紧到 {target} mm？低速低电流，遇阻会停。"
        elif command == "init":
            text = "确认初始化/回零夹爪？初始化期间夹爪会执行校准动作，请确认周围安全。"
        elif command == "close":
            text = "确认收拢/闭合夹爪？用于进入抓取位前收拢两指；回零请用初始化，不要用直接闭合到 0。"
        else:
            text = "确认张开夹爪到 50 mm？"
        answer = QtWidgets.QMessageBox.question(
            self, "Confirm: gripper action", text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            data = self._gripper_call(command, target if command == "grip" else None)
            self._append_log(
                f"gripper {command} → {json.dumps(data, ensure_ascii=False)}"
            )
            QtCore.QTimer.singleShot(700, lambda: self._refresh_gripper(force=True))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Gripper bridge",
                                          f"命令失败: {exc}")

    # ---------------------------------------------------------------- image
    def _refresh_image(self) -> None:
        """Push the live ROS colour frame into the independent camera window.

        The window is not refreshed while hidden; the next 500 ms tick updates
        it as soon as the operator reopens it.
        """
        if not self._image_window.isVisible():
            return
        if self._color_frame is None:
            self._image_window.set_source_status(
                "等待 /camera/color/image_raw（检查相机节点与 QoS）")
            return
        if self._frame_counter == self._shown_frame:
            return
        try:
            frame = self._color_frame
            height, width = frame.shape[:2]
            # The view scales with a smooth transform; keep the source frame
            # untouched so the operator sees exactly what the sensor produced.
            rgb = np.ascontiguousarray(frame[:, :, ::-1])
            qimage = QtGui.QImage(
                rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0],
                QtGui.QImage.Format_RGB888,
            ).copy()
            self._image_window.image_view.set_pixmap(
                QtGui.QPixmap.fromImage(qimage))
            self._shown_frame = self._frame_counter
            depth_text = ""
            if self._depth_stats is not None:
                ratio, median, _count = self._depth_stats
                depth_text = f" · 深度有效 {ratio * 100:.0f}% 中位 {median:.3f} m"
            self._image_window.set_source_status(
                f"/camera/color/image_raw {width}x{height}{depth_text} · "
                f"{self._fps():.1f} fps")
        except Exception as exc:  # noqa: BLE001
            self._image_window.set_source_status("图像显示失败，见主面板日志")
            self._append_log(f"图像错误: {exc}")

    def _update_image_button(self) -> None:
        if not hasattr(self, "image_window_button"):
            return
        opened = self._image_window.isVisible()
        fps = self._fps()
        fps_text = f"{fps:.1f} fps" if fps else "无帧"
        self.image_window_button.setText(
            f"图像窗口 · {'已打开' if opened else '已关闭'} · {fps_text}")
        self.image_window_button.setProperty(
            "open", "true" if opened else "false")
        self._restyle(self.image_window_button)

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _restyle(widget: QtWidgets.QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _set_badge(self, badge: QtWidgets.QLabel, text: str, level: str) -> None:
        badge.setText(text)
        badge.setProperty("level", level)
        self._restyle(badge)

    def _set_state(self, key: str, text: str, level: str) -> None:
        label = self._status_values[key]
        if label.text() == text and label.property("level") == level:
            return
        label.setText(text)
        label.setProperty("level", level)
        self._restyle(label)

    def _append_log(self, text: str) -> None:
        for line in str(text).splitlines() or [""]:
            self.log.append_line(line)

    # -------------------------------------------------------------- smoke
    def _geometry_report(self) -> list[str]:
        """Print card geometry and prove the compact-panel acceptance numbers."""
        central = self.centralWidget()
        cards = (
            ("标题栏", self._header_widget),
            ("状态卡", self._status_card),
            ("任务模式", self._mode_card),
            ("当前流程", self._flow_card),
            ("监视卡", self._monitor_card),
            ("日志卡", self._log_card),
        )
        width_ok = self.width() <= MAIN_MAX_WIDTH
        lines = [
            f"[geometry] 主面板 {self.width()}x{self.height()} px · "
            f"控制列宽 {self.width()} px（要求 <= {MAIN_MAX_WIDTH} px: "
            f"{'OK' if width_ok else 'FAIL'}）"
        ]
        region = QtGui.QRegion(central.rect())
        for name, widget in cards:
            top_left = widget.mapTo(central, QtCore.QPoint(0, 0))
            rect = QtCore.QRect(top_left, widget.size())
            region = region.subtracted(QtGui.QRegion(rect))
            lines.append(
                f"[geometry]   {name:<4} x={rect.x():>3} y={rect.y():>3} "
                f"w={rect.width():>3} h={rect.height():>3}")

        log_height = self._log_card.height()
        lines.append(
            f"[geometry] 日志卡高度 {log_height} px（要求 >= 180 px: "
            f"{'OK' if log_height >= 180 else 'FAIL'}）")

        image_w, image_h = self._image_window.width(), self._image_window.height()
        target_w, target_h = IMAGE_WINDOW_SIZE
        scale = min(image_w / target_w, image_h / target_h)
        aspect_ok = image_h > 0 and abs(image_w / image_h - target_w / target_h) < 0.01
        lines.append(
            f"[geometry] 图像窗口 {image_w}x{image_h} px（目标 {target_w}x{target_h}，"
            f"等比 {scale:.3f}，4:3 {'OK' if aspect_ok else 'FAIL'}）")

        rects = region.rects()
        largest = max(rects, key=lambda r: r.width() * r.height()) if rects else None
        blank_ok = all(r.width() <= 200 or r.height() <= 200 for r in rects)
        largest_text = ("0x0" if largest is None else
                        f"{largest.width()}x{largest.height()} @ "
                        f"({largest.x()},{largest.y()})")
        lines.append(
            f"[geometry] 卡片外最大空白 {largest_text} px；是否存在 >200x200 "
            f"空白: {'否 (OK)' if blank_ok else '是 (FAIL)'}")
        return lines

    def _save_smoke(self) -> None:
        """Save the panel and the camera window, print sizes, then quit."""
        if self._screenshot is None:
            return
        self.show_image_window()
        QtWidgets.QApplication.processEvents()
        self._refresh_image()
        QtWidgets.QApplication.processEvents()

        main_path = Path(self._screenshot)
        image_path = main_path.with_name(f"{main_path.stem}_image.png")
        main_pixmap = self.grab()
        image_pixmap = self._image_window.grab()
        for ok, path in ((main_pixmap.save(str(main_path)), main_path),
                         (image_pixmap.save(str(image_path)), image_path)):
            if not ok:
                print(f"[smoke] 截图保存失败: {path}", file=sys.stderr)

        task_idle = self._process.state() == QtCore.QProcess.NotRunning
        print(f"[smoke] 主面板截图: {main_path.resolve()} "
              f"窗口 {self.width()}x{self.height()} · "
              f"PNG {main_pixmap.width()}x{main_pixmap.height()}")
        print(f"[smoke] 图像窗口截图: {image_path.resolve()} "
              f"窗口 {self._image_window.width()}x{self._image_window.height()} · "
              f"PNG {image_pixmap.width()}x{image_pixmap.height()}")
        print("[smoke] 任务子进程: " + ("未启动 (OK)" if task_idle
                                        else "运行中 (FAIL)"))
        for line in self._geometry_report():
            print(line)

        self._append_log(f"smoke 截图完成: {main_path} / {image_path}")
        QtWidgets.QApplication.quit()

    def closeEvent(self, event) -> None:  # noqa: N802
        self._timer.stop()
        if getattr(self, "_image_window", None) is not None:
            self._image_window.shutdown()
        if hasattr(self, "_process") and \
                self._process.state() != QtCore.QProcess.NotRunning:
            self._process.terminate()
            if not self._process.waitForFinished(1500):
                self._process.kill()
                self._process.waitForFinished(1000)
        try:
            self._ros.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--smoke", metavar="PNG",
                        help="离屏截图后退出（不启动任何任务子进程）")
    parser.add_argument("--print-command-only", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--scope", choices=SCOPES, help=argparse.SUPPRESS)
    parser.add_argument("--mode", choices=MODES, help=argparse.SUPPRESS)
    parser.add_argument("--action", choices=ACTIONS, help=argparse.SUPPRESS)
    parser.add_argument("--world", choices=WORLDS, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    if args.print_command_only:
        return _print_command_demo(args)

    if not ROS_AVAILABLE:
        print("ROS 2 Python 依赖不可用：" + ROS_IMPORT_ERROR, file=sys.stderr)
        print("请先执行 source scripts/source_dev_env.sh --full 后重试；"
              "--print-command-only 不需要 ROS 环境。", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = Dashboard(screenshot=args.smoke, allow_spawn=args.smoke is None)
    window.show()
    window.show_image_window()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
