#!/usr/bin/env python3
"""CS625 双模式操作面板（紧凑控制台，PyQt5）。

两种模式跑同一条示教链路：

  安全模式  bash scripts/resume_teaching.sh        门检/静置/fitness 全开，失败自动重启并续跑
  效率模式  python3 scripts/teaching/teaching_debug.py --from A --to B [--no-gates] ...

界面规则（所有者 2026-11-07）：
  · 不写解释性小字：只有「标签 + 数值」，说明放 tooltip
  · 一屏放下：阶段表固定高度不出滚动条，遥测/场景/日志同屏
  · 编号与阶段名固定列宽、左对齐
  · 3D 交给 RViz（config/rviz/cs625_teaching.rviz）；面板只显示数据源状态卡，
    见 scripts/panel_status.py

用法：
    python3 scripts/cs625_panel.py
    python3 scripts/cs625_panel.py --smoke out.png     # 离屏渲染，不需要 ROS
    python3 scripts/cs625_panel.py --no-probe
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import re
import signal
import subprocess
import sys
import time
from typing import Sequence

from PyQt5 import QtCore, QtGui, QtWidgets

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from panel_status import RvizStatusCard, source_summary   # noqa: E402
from panel_camera import CameraView                        # noqa: E402

RUN_STATE = REPO / "runtime" / "teaching_run_state.json"
DEBUG_RUNS = REPO / "runtime" / "debug_runs"
DEBUG = REPO / "scripts" / "teaching" / "teaching_debug.py"
RESUME = REPO / "scripts" / "resume_teaching.sh"

DEFAULT_SIZE = (2200, 1250)      # 4K 屏上会被 target_size() 覆盖
MIN_SIZE = (1280, 760)

# 界面缩放：4K（纵向 2160）下取 1.7，1080p 下为 1.0。字体/行高/控件高度都乘这个系数，
# 所以在 4K 上不是"把 1080p 的界面拉大"，而是字号本身变大、信息量保持。
SCALE = [1.0]


def set_scale(_screen_height: int) -> float:
    """默认不额外放大：字号用逻辑点（pt），Qt 会按系统 DPI 缩放。

    强行按 4K 放大字号会把布局撑得很空（所有者 2026-11-07 反馈），所以界面保持紧凑，
    只让窗口占据更大的可用区域。
    """
    SCALE[0] = 1.0
    return SCALE[0]


def target_size(screen: QtCore.QRect) -> tuple[int, int]:
    """窗口占据可用逻辑区域的 ~92%（不做物理像素换算，Qt 已处理 DPI）。"""
    width = min(int(screen.width() * 0.92), 2560)
    height = min(int(screen.height() * 0.92), 1400)
    return max(width, MIN_SIZE[0]), max(height, MIN_SIZE[1])

UI_FAMILY = "Noto Sans CJK SC, Source Han Sans SC, WenQuanYi Micro Hei, sans-serif"
MONO_FAMILY = "JetBrains Mono, DejaVu Sans Mono, monospace"

C_BG = "#0b1015"
C_PANEL = "#121b24"
C_BORDER = "#1f2c39"
C_TEXT = "#dbe7f1"
C_MUTED = "#7d93a6"
C_OK = "#3ec98a"
C_WARN = "#d6a52f"
C_BAD = "#e05252"
C_ACCENT = "#2fd6c9"

# 示教链阶段（第 2 步「插槽粗观测」已删除，不再出现在面板里）
STAGES = [
    (0, "环境模型"), (1, "插槽精观测"), (3, "工件粗观测"), (4, "工件精观测"),
    (5, "预抓取"), (6, "进入抓取"), (7, "张爪夹持"), (8, "抬升"),
    (9, "抬到安全高度"), (10, "transit"), (11, "staging"), (12, "旋转"),
    (13, "下降到预插入"), (14, "插入"), (15, "转平"), (16, "释放"),
]
SPEED_PHASES = ["pre_grasp", "grasp", "lift", "transit", "staging",
                "pre_insert", "insert", "straighten"]

# 每一阶段的关键约束/产物，直接写在表里，省得去翻文档
STAGE_NOTES = {
    0: "model.json，门禁 < 2 h",
    1: "T1 · fitness ≥ 0.9 · 位姿库比对",
    3: "T2 · 只定相机站位 ≥ 0.8",
    4: "T3 · 抓取位 · fitness ≥ 0.9",
    5: "抓取位上方 150 mm · 门 1.5 mm",
    6: "竖直下降 · 门 0.4°",
    7: "闭合到位 · 挂载工件到场景",
    8: "0.15 m · 持件 ≤ 0.013 m/s",
    9: "z = 0.53，转向前先抬高",
    10: "持件转运，环境场景已清空",
    11: "槽前分级位",
    12: "绕工具轴 −10°",
    13: "门 1.0 mm/0.4° + 目标正确性",
    14: "0.002 m/s · 6 段 · 力矩监督",
    15: "绕 TCP y 轴 0.5°/步，遇阻回退",
    16: "需 --allow-release",
}

def build_style(scale: float) -> str:
    body = 9.0 * scale
    small = 8.0 * scale
    mono = 9.0 * scale
    title = 11.0 * scale
    return f"""
QWidget {{ background: {C_BG}; color: {C_TEXT}; font-family: {UI_FAMILY}; font-size: {body:.1f}pt; }}
QLabel {{ background: transparent; }}
QLabel#Title {{ font-size: {title:.1f}pt; font-weight: 700; }}
QLabel#Mono {{ font-family: {MONO_FAMILY}; font-size: {mono:.1f}pt; }}
QLabel#Cap {{ color: {C_MUTED}; font-size: {small:.1f}pt; }}
QGroupBox {{ background: {C_PANEL}; border: 1px solid {C_BORDER}; border-radius: 2px;
    margin-top: {int(9*scale)}px; padding: 2px; font-size: {small:.1f}pt; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 6px; padding: 0 3px;
    color: {C_MUTED}; font-size: {small:.1f}pt; }}
QFrame#Chip {{ background: {C_PANEL}; border: 1px solid {C_BORDER}; border-radius: 2px; }}
QLabel#ChipValue[level="ok"] {{ color: {C_OK}; }}
QLabel#ChipValue[level="warn"] {{ color: {C_WARN}; }}
QLabel#ChipValue[level="bad"] {{ color: {C_BAD}; }}
QPlainTextEdit {{ background: #0a0f14; border: 1px solid {C_BORDER};
    font-family: {MONO_FAMILY}; font-size: {small + 0.5:.1f}pt; color: #b9c9d8; }}
QTableWidget {{ background: #0a0f14; alternate-background-color: #0e151c;
    gridline-color: {C_BORDER}; border: 1px solid {C_BORDER};
    font-family: {MONO_FAMILY}; font-size: {small + 0.5:.1f}pt; }}
QTableWidget::item {{ padding: 0 3px; }}
QHeaderView::section {{ background: #16202a; color: {C_MUTED}; border: none;
    padding: 1px 3px; font-size: {small:.1f}pt; }}
QPushButton {{ background: #17222d; border: 1px solid #263444; border-radius: 2px;
    padding: 2px 8px; font-size: {body:.1f}pt; min-height: {int(22*scale)}px; }}
QPushButton:hover {{ background: #1f2e3c; }}
QPushButton:disabled {{ color: #4f606e; }}
QPushButton[role="go"] {{ background: #10423e; border-color: {C_ACCENT}; font-weight: 700; }}
QPushButton[role="stop"] {{ background: #471a1e; border-color: #a83c44; font-weight: 700; }}
QCheckBox {{ font-size: {small + 0.5:.1f}pt; spacing: 4px; }}
QLineEdit {{ background: #0a0f14; border: 1px solid {C_BORDER}; padding: 1px 3px;
    font-family: {MONO_FAMILY}; font-size: {small + 0.5:.1f}pt; }}
QProgressBar {{ background: #0a0f14; border: 1px solid {C_BORDER}; text-align: center;
    font-family: {MONO_FAMILY}; font-size: {small:.1f}pt; }}
QProgressBar::chunk {{ background: #1f6f6a; }}
QTabBar::tab {{ background: #16202a; color: #c8d6e2; padding: 3px 14px;
    border: 1px solid {C_BORDER}; border-bottom: none; font-size: {body:.1f}pt; }}
QTabBar::tab:selected {{ background: {C_PANEL}; color: {C_ACCENT}; font-weight: 700; }}
QSplitter::handle {{ background: {C_BORDER}; }}
"""


STYLE = build_style(1.0)          # 启动时按真实屏幕重算（见 main）


# --------------------------------------------------------------------------- helpers
def run_cmd(args: Sequence[str], timeout: float = 6.0) -> tuple[int, str]:
    try:
        done = subprocess.run(args, capture_output=True, text=True, timeout=timeout, cwd=str(REPO))
        return done.returncode, (done.stdout or "") + (done.stderr or "")
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return 1, ""


def drive_channels() -> int:
    count = 0
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            lines = pathlib.Path(path).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 4 or fields[3] != "01":
                continue
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
            except (IndexError, ValueError):
                continue
            if port in (50001, 50003, 50004):
                count += 1
    return count


def read_run_state() -> dict:
    try:
        return json.loads(RUN_STATE.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def topic_mode(topic: str, timeout: float = 2.5) -> int | None:
    code, text = run_cmd(["timeout", str(timeout), "ros2", "topic", "echo", topic, "--once"],
                         timeout=timeout + 1.5)
    if code != 0:
        return None
    match = re.search(r"^mode:\s*(-?\d+)", text, re.MULTILINE)
    return int(match.group(1)) if match else None


class Poller(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, period_s: float = 2.0, parent=None) -> None:
        super().__init__(parent)
        self._period = period_s
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        while not self._stop:
            payload = {
                "channels": drive_channels(),
                "safety": topic_mode("/io_and_status_controller/safety_mode"),
                "robot": topic_mode("/io_and_status_controller/robot_mode"),
            }
            self.result.emit(payload)
            for _ in range(int(self._period / 0.1)):
                if self._stop:
                    return
                time.sleep(0.1)


# --------------------------------------------------------------------------- widgets
class Chip(QtWidgets.QFrame):
    def __init__(self, title: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("Chip")
        self.setFixedHeight(int(24 * SCALE[0]))
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(6, 0, 6, 0)
        layout.setSpacing(6)
        label = QtWidgets.QLabel(title)
        label.setObjectName("Cap")
        self.value = QtWidgets.QLabel("—")
        self.value.setObjectName("Mono")
        self.value.setProperty("level", "idle")
        layout.addWidget(label, 0)
        layout.addStretch(1)
        layout.addWidget(self.value, 0)

    def set_value(self, level: str, text: str) -> None:
        self.value.setText(text)
        self.value.setProperty("level", level)
        self.value.style().unpolish(self.value)
        self.value.style().polish(self.value)


class StageTable(QtWidgets.QTableWidget):
    ROW_H = 18          # 实际行高由字体决定，构造后按实测值设置

    def __init__(self, parent=None) -> None:
        super().__init__(len(STAGES), 5, parent)
        self.setHorizontalHeaderLabels(["#", "阶段", "状态", "耗时", "说明"])
        self.verticalHeader().setVisible(False)
        self.setShowGrid(False)
        self.setAlternatingRowColors(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        header = self.horizontalHeader()
        header.setStretchLastSection(False)
        for column, width in ((0, 34), (1, 150), (2, 70), (3, 58)):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            self.setColumnWidth(column, width)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.verticalHeader().setDefaultSectionSize(22)
        fixed_height = 22 * len(STAGES) + 30
        self.setFixedHeight(fixed_height)
        self._rows: dict[int, int] = {}
        self._state: dict[int, str] = {}
        header = self.horizontalHeader()
        for column in range(5):
            item = self.horizontalHeaderItem(column)
            if item is not None:
                align = (QtCore.Qt.AlignRight if column in (0, 2, 3)
                         else QtCore.Qt.AlignLeft)
                item.setTextAlignment(align | QtCore.Qt.AlignVCenter)
        for row, (number, name) in enumerate(STAGES):
            self._rows[number] = row
            self._state[number] = "idle"
            for column, text in enumerate((f"{number:02d}", name, "未开始", "",
                                           STAGE_NOTES.get(number, ""))):
                item = QtWidgets.QTableWidgetItem(text)
                item.setTextAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                if column in (0, 2):
                    item.setForeground(QtGui.QColor(C_MUTED))
                self.setItem(row, column, item)

    def set_stage(self, number: int, state: str, seconds: float | None = None) -> None:
        row = self._rows.get(number)
        if row is None:
            return
        self._state[number] = state
        if seconds is not None:
            cell = self.item(row, 3)
            if cell is not None:
                cell.setText(f"{seconds:.1f}")
        colors = {"run": C_ACCENT, "ok": C_OK, "bad": C_BAD, "idle": C_MUTED}
        labels = {"run": "进行中", "ok": "完成", "bad": "失败", "idle": "未开始"}
        item = self.item(row, 2)
        if item is not None:
            item.setText(labels.get(state, state))
            item.setForeground(QtGui.QColor(colors.get(state, C_MUTED)))

    def reset(self) -> None:
        for number in self._rows:
            self.set_stage(number, "idle")

    def mark_current(self, number: int) -> None:
        for n, state in list(self._state.items()):
            if state == "run" and n != number:
                self.set_stage(n, "idle")
        self.set_stage(number, "run")

    def complete(self) -> set[int]:
        return {n for n, state in self._state.items() if state == "ok"}


class EffortBars(QtWidgets.QWidget):
    """六关节力矩：固定量程的横条 + 数值，一眼能看出哪一轴在吃力。"""

    ORDER = [("shoulder_pan_joint", "J1"), ("shoulder_lift_joint", "J2"),
             ("elbow_joint", "J3"), ("wrist_1_joint", "J4"),
             ("wrist_2_joint", "J5"), ("wrist_3_joint", "J6")]
    FULL_SCALE = 60.0          # N·m；超过就按满格画

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._values = {name: 0.0 for name, _ in self.ORDER}

    def set_efforts(self, efforts: dict) -> None:
        for name, _ in self.ORDER:
            self._values[name] = abs(float(efforts.get(name, 0.0)))
        self.update()

    def paintEvent(self, event) -> None:            # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0f14"))
        font = painter.font()
        font.setPointSizeF(8.0)
        painter.setFont(font)
        rows = len(self.ORDER)
        row_h = self.height() / rows
        label_w, value_w = 26, 46
        track_x = label_w + 6
        track_w = max(10.0, self.width() - track_x - value_w - 14)
        for index, (name, label) in enumerate(self.ORDER):
            value = self._values[name]
            top = index * row_h
            mid = top + row_h * 0.5
            painter.setPen(QtGui.QColor(C_MUTED))
            painter.drawText(4, int(mid + 4), label)
            # 轨道 + 填充
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor("#16202a"))
            painter.drawRoundedRect(QtCore.QRectF(track_x, mid - row_h * 0.22, track_w, row_h * 0.44), 2, 2)
            ratio = min(1.0, value / self.FULL_SCALE)
            if ratio > 0:
                colour = C_ACCENT if ratio < 0.7 else (C_WARN if ratio < 0.9 else C_BAD)
                painter.setBrush(QtGui.QColor(colour))
                painter.drawRoundedRect(
                    QtCore.QRectF(track_x, mid - row_h * 0.22, track_w * ratio, row_h * 0.44), 2, 2)
            painter.setPen(QtGui.QColor(C_TEXT))
            painter.drawText(int(self.width() - value_w), int(mid + 4), f"{value:5.1f}")
        painter.end()


class WrenchGrid(QtWidgets.QWidget):
    LABELS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._values: dict | None = None

    def set_wrench(self, values: dict | None) -> None:
        self._values = values
        self.update()

    def paintEvent(self, event) -> None:            # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0f14"))
        font = painter.font()
        font.setPointSizeF(7.5)
        painter.setFont(font)
        cell_w = self.width() / 3
        cell_h = self.height() / 2
        for index, label in enumerate(self.LABELS):
            column, row = index % 3, index // 3
            x = column * cell_w + 4
            y = row * cell_h
            painter.setPen(QtGui.QColor("#5d7284"))
            painter.drawText(int(x), int(y + 12), label)
            if self._values is None:
                painter.setPen(QtGui.QColor("#465866"))
                text = "未采集"
            else:
                painter.setPen(QtGui.QColor(C_TEXT))
                text = f"{self._values.get(label, 0.0):+.1f}"
            painter.drawText(int(x + 18), int(y + 12), text)
        painter.end()


class TcpGripLabel(QtWidgets.QLabel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("Mono")
        self.setStyleSheet("background:#0a0f14; padding:2px 4px; color:#c2d3e2;")
        self._tcp = None
        self._grip = None
        self._render()

    def set_tcp(self, tcp) -> None:
        self._tcp = tcp
        self._render()

    def set_grip(self, grip) -> None:
        self._grip = grip
        self._render()

    def _render(self) -> None:
        tcp = "—" if self._tcp is None else \
            f"{self._tcp[0]:.3f}  {self._tcp[1]:.3f}  {self._tcp[2]:.3f}"
        grip = "—" if self._grip is None else str(self._grip)
        self.setText(f"TCP   {tcp}\n夹爪  {grip}")


class LogView(QtWidgets.QPlainTextEdit):
    COLORS = {"err": C_BAD, "warn": C_WARN, "ok": C_OK, "info": "#b9c9d8"}

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(3000)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)

    def append_line(self, line: str, level: str | None = None) -> None:
        if level is None:
            low = line.lower()
            if "error" in low or "failed" in low or "traceback" in low:
                level = "err"
            elif "warn" in low:
                level = "warn"
            elif ": ok" in low or low.strip() == "ok":
                level = "ok"
            else:
                level = "info"
        color = self.COLORS.get(level, C_TEXT)
        self.appendHtml(f'<span style="color:{color}">{line}</span>')


class EffortChart(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[tuple[float, float]] = []

    def load(self, csv_path: pathlib.Path) -> None:
        rows: list[tuple[float, float]] = []
        if csv_path.exists():
            try:
                with csv_path.open() as handle:
                    for record in csv.DictReader(handle):
                        try:
                            rows.append((float(record["t"]), float(record["fast"])))
                        except (KeyError, ValueError):
                            continue
            except OSError:
                rows = []
        self._rows = rows
        self.update()

    def paintEvent(self, event) -> None:            # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0f14"))
        font = painter.font()
        font.setPointSizeF(7.5)
        painter.setFont(font)
        if not self._rows:
            painter.setPen(QtGui.QColor("#465866"))
            painter.drawText(self.rect().adjusted(6, 0, 0, 0),
                             QtCore.Qt.AlignVCenter, "无 effort 数据")
            painter.end()
            return
        times = [row[0] for row in self._rows]
        values = [row[1] for row in self._rows]
        span_t = max(1e-6, times[-1] - times[0])
        lo, hi = min(values), max(values)
        span_v = max(1e-6, hi - lo)
        width = self.width() - 12
        height = self.height() - 16
        path = QtGui.QPainterPath()
        for index, (t, v) in enumerate(self._rows):
            x = 6 + width * (t - times[0]) / span_t
            y = 8 + height * (1 - (v - lo) / span_v)
            path.moveTo(x, y) if index == 0 else path.lineTo(x, y)
        painter.setPen(QtGui.QPen(QtGui.QColor(C_ACCENT), 1.4))
        painter.drawPath(path)
        painter.setPen(QtGui.QColor("#5d7284"))
        painter.drawText(6, self.height() - 3, f"{lo:.1f} … {hi:.1f}")
        painter.end()


def make_group(title: str, widget: QtWidgets.QWidget) -> QtWidgets.QGroupBox:
    box = QtWidgets.QGroupBox(title)
    layout = QtWidgets.QVBoxLayout(box)
    layout.setContentsMargins(2, 3, 2, 2)
    layout.addWidget(widget)
    return box


# --------------------------------------------------------------------------- tabs
class SafeTab(QtWidgets.QWidget):
    def __init__(self, probe: bool = True, parent=None) -> None:
        super().__init__(parent)
        self._process: QtCore.QProcess | None = None
        self._started_at = 0.0
        self._probe = probe
        self._current: int | None = None
        self._build()
        if probe:
            self._poller = Poller(parent=self)
            self._poller.result.connect(self._on_status)
            self._poller.start()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        self._refresh_run_state()

    # ------------------------------------------------------------------ ui
    def _build(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(3)

        top = QtWidgets.QHBoxLayout()
        top.setSpacing(4)
        self.chips = {
            "safety": Chip("safety_mode"),
            "robot": Chip("robot_mode"),
            "channels": Chip("反控"),
            "grip": Chip("夹爪"),
        }
        for chip in self.chips.values():
            top.addWidget(chip, 1)
        self.btn_start = QtWidgets.QPushButton("开始示教")
        self.btn_start.setProperty("role", "go")
        self.btn_start.setToolTip("bash scripts/resume_teaching.sh")
        self.btn_start.clicked.connect(self._start)
        self.btn_resume = QtWidgets.QPushButton("续跑")
        self.btn_resume.setToolTip("只续跑，不重启驱动栈")
        self.btn_resume.clicked.connect(self._resume)
        self.btn_abort = QtWidgets.QPushButton("中止")
        self.btn_abort.setProperty("role", "stop")
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self._abort)
        for button in (self.btn_start, self.btn_resume, self.btn_abort):
            button.setFixedWidth(92)
            top.addWidget(button, 0)

        # 相机：RVS 与 ROS 相机互斥，这里只负责启动/停止/查看状态
        self.btn_camera = QtWidgets.QPushButton("相机 ▾")
        self.btn_camera.setFixedWidth(76)
        self.btn_camera.setToolTip("启动/停止 Percipio 相机（与 RVS 互斥）")
        menu = QtWidgets.QMenu(self.btn_camera)
        menu.addAction("启动（fast，无深度）", lambda: self._camera("fast"))
        menu.addAction("启动（modeling，1280x960）", lambda: self._camera("modeling"))
        menu.addAction("停止", lambda: self._camera("stop"))
        menu.addSeparator()
        menu.addAction("状态", lambda: self._camera("status"))
        self.btn_camera.setMenu(menu)
        top.addWidget(self.btn_camera, 0)
        root.addLayout(top)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setHandleWidth(4)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_right())
        splitter.setStretchFactor(0, 38)
        splitter.setStretchFactor(1, 62)
        splitter.setSizes([820, 1340])
        root.addWidget(splitter, 1)

    def _build_left(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        self.stage_table = StageTable()
        stage_box = make_group("阶段", self.stage_table)
        stage_box.setFixedHeight(self.stage_table.height() + 22)
        layout.addWidget(stage_box, 0)

        self.state_label = QtWidgets.QLabel("空闲")
        self.state_label.setObjectName("Mono")
        self.resume_label = QtWidgets.QLabel("")
        self.resume_label.setObjectName("Cap")
        info = QtWidgets.QWidget()
        info_layout = QtWidgets.QHBoxLayout(info)
        info_layout.setContentsMargins(2, 0, 2, 0)
        info_layout.addWidget(self.state_label, 0)
        info_layout.addStretch(1)
        info_layout.addWidget(self.resume_label, 0)
        layout.addWidget(info, 0)

        self.log = LogView()
        layout.addWidget(make_group("日志", self.log), 2)
        return panel

    def _build_right(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        # 遥测三卡等高
        self.effort_bars = EffortBars()
        self.wrench = WrenchGrid()
        self.tcp_label = TcpGripLabel()
        telemetry = QtWidgets.QWidget()
        telemetry_row = QtWidgets.QHBoxLayout(telemetry)
        telemetry_row.setContentsMargins(0, 0, 0, 0)
        telemetry_row.setSpacing(3)
        gripper_box = QtWidgets.QGroupBox("夹爪控制")
        gripper_layout = QtWidgets.QVBoxLayout(gripper_box)
        gripper_layout.setContentsMargins(6, 4, 6, 4)
        gripper_layout.setSpacing(3)
        self.gripper_label = QtWidgets.QLabel("位置 — · 状态 —")
        self.gripper_label.setObjectName("Mono")
        gripper_layout.addWidget(self.gripper_label)
        gripper_buttons = QtWidgets.QHBoxLayout()
        gripper_buttons.setSpacing(3)
        for text, command in (("状态", "status"), ("初始化", "init"),
                              ("张开", "release"), ("闭合", "grip")):
            button = QtWidgets.QPushButton(text)
            button.setFixedHeight(24)
            button.setToolTip(f"scripts/teaching/gripper.py {command}")
            button.clicked.connect(lambda _checked=False, c=command: self._gripper(c))
            gripper_buttons.addWidget(button, 1)
        gripper_layout.addLayout(gripper_buttons)
        gripper_layout.addStretch(1)
        self.gripper_box = gripper_box

        for widget, title in ((self.effort_bars, "关节力矩 N·m（满量程 60）"),
                              (self.wrench, "六维力（未安装）"),
                              (self.tcp_label, "TCP")):
            box = make_group(title, widget)
            box.setFixedHeight(150)
            telemetry_row.addWidget(box, 1)
        telemetry_row.addWidget(self.gripper_box, 1)
        layout.addWidget(telemetry, 0)

        # 关键数值与最近采集：同高，一起撑满中段
        middle = QtWidgets.QWidget()
        middle_row = QtWidgets.QHBoxLayout(middle)
        middle_row.setContentsMargins(0, 0, 0, 0)
        middle_row.setSpacing(3)

        key_box = QtWidgets.QGroupBox("关键数值")
        key_grid = QtWidgets.QGridLayout(key_box)
        key_grid.setContentsMargins(10, 6, 10, 6)
        key_grid.setHorizontalSpacing(12)
        key_grid.setVerticalSpacing(4)
        self.key_labels: dict[str, QtWidgets.QLabel] = {}
        for row, name in enumerate(("T1 fitness", "T3 fitness", "最近采集",
                                    "上次阶段", "反控", "当前阶段",
                                    "相机", "RViz", "界面")):
            label = QtWidgets.QLabel(name)
            label.setObjectName("Cap")
            value = QtWidgets.QLabel("—")
            value.setObjectName("Mono")
            self.key_labels[name] = value
            key_grid.addWidget(label, row, 0)
            key_grid.addWidget(value, row, 1)
        key_grid.setColumnStretch(1, 1)
        key_grid.setRowStretch(len(self.key_labels), 1)
        key_box.setFixedHeight(236)
        middle_row.addWidget(key_box, 1)

        self.captures = QtWidgets.QTableWidget(0, 4)
        self.captures.setHorizontalHeaderLabels(["命令", "fitness", "z m", "时间"])
        self.captures.verticalHeader().setVisible(False)
        self.captures.setShowGrid(False)
        self.captures.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.captures.verticalHeader().setDefaultSectionSize(22)
        header = self.captures.horizontalHeader()
        header.setStretchLastSection(False)
        for column, width in {0: 52, 1: 74, 2: 66, 3: 78}.items():
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            self.captures.setColumnWidth(column, width)
        for column in range(4):
            item = self.captures.horizontalHeaderItem(column)
            if item is not None:
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
                                      if column else QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        session_box = QtWidgets.QGroupBox("会话")
        session_grid = QtWidgets.QGridLayout(session_box)
        session_grid.setContentsMargins(10, 6, 10, 6)
        session_grid.setVerticalSpacing(4)
        self.session_labels: dict[str, QtWidgets.QLabel] = {}
        for row, name in enumerate(("分支", "最近提交", "配置", "工作目录")):
            label = QtWidgets.QLabel(name)
            label.setObjectName("Cap")
            value = QtWidgets.QLabel("—")
            value.setObjectName("Mono")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.session_labels[name] = value
            session_grid.addWidget(label, row, 0)
            session_grid.addWidget(value, row, 1)
        session_grid.setColumnStretch(1, 1)
        session_grid.setRowStretch(4, 1)

        key_column = QtWidgets.QWidget()
        key_column_layout = QtWidgets.QVBoxLayout(key_column)
        key_column_layout.setContentsMargins(0, 0, 0, 0)
        key_column_layout.setSpacing(3)
        key_column_layout.addWidget(key_box, 0)
        key_column_layout.addWidget(session_box, 1)
        middle_row.addWidget(key_column, 1)
        middle_row.addWidget(make_group("最近采集", self.captures), 2)
        layout.addWidget(middle, 1)

        # 相机：彩色/深度按原始分辨率 1:1 显示，调试时不被缩放误导
        self.camera = CameraView(probe=self._probe)
        layout.addWidget(make_group("相机（1:1 原始像素）", self.camera), 1)

        # 3D 状态卡：只占必要高度
        self.status_card = RvizStatusCard(probe=self._probe)
        self.status_card.open_rviz.connect(self._open_rviz)
        self.status_card.diagnose.connect(self._diagnose_scene)
        if self._probe:
            self.status_card._worker.frame.connect(
                lambda payload: self._on_telemetry(payload))
        status_box = make_group("3D 状态（RViz）", self.status_card)
        status_box.setFixedHeight(196)
        layout.addWidget(status_box, 0)
        return panel

    # --------------------------------------------------------------- actions
    def _spawn(self, args: Sequence[str]) -> None:
        if self._process is not None:
            self.log.append_line("已有进程在运行", "warn")
            return
        self.log.append_line("$ " + " ".join(args), "info")
        self.stage_table.reset()
        self._current = None
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(REPO))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._drain)
        process.finished.connect(self._finished)
        process.start(args[0], list(args[1:]))
        self._process = process
        self._started_at = time.time()
        self.btn_abort.setEnabled(True)
        self.btn_start.setEnabled(False)
        self.btn_resume.setEnabled(False)

    def _drain(self) -> None:
        if self._process is None:
            return
        text = bytes(self._process.readAllStandardOutput()).decode("utf-8", "replace")
        for line in text.splitlines():
            self.log.append_line(line)
            self._parse(line)

    def _parse(self, line: str) -> None:
        match = re.match(r"^\[(\d+)\]", line.strip())
        if match:
            number = int(match.group(1))
            if number in dict(STAGES):
                self._current = number
                self.stage_table.mark_current(number)

        done = re.match(r"^\s+(.+?):\s*(OK|FAILED)", line)
        if done and self._current is not None:
            self.stage_table.set_stage(
                self._current, "ok" if done.group(2) == "OK" else "bad")

    def _finished(self, code: int, status) -> None:
        self.log.append_line(f"[面板] 进程结束 code={code}", "ok" if code == 0 else "err")
        self._process = None
        self.btn_abort.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_resume.setEnabled(True)
        self._refresh_run_state()

    def _gripper(self, command: str) -> None:
        """夹爪控制：调用 scripts/teaching/gripper.py，输出进日志。"""
        script = REPO / "scripts" / "teaching" / "gripper.py"
        self.log.append_line(f"$ python3 {script.name} {command}", "info")
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(REPO))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: [self.log.append_line(line) for line in
                     bytes(process.readAllStandardOutput()).decode("utf-8", "replace").splitlines()])
        process.finished.connect(
            lambda code, _status: self.log.append_line(
                f"[夹爪] {command} 结束 code={code}", "ok" if code == 0 else "err"))
        process.start(sys.executable, [str(script), command])

    def _camera(self, action: str) -> None:
        """在后台跑 camera_mode.py（子进程），输出进日志，不阻塞界面。"""
        command = [sys.executable, str(REPO / "scripts" / "camera_mode.py"), action]
        self.log.append_line("$ " + " ".join(command), "info")
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(REPO))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: [self.log.append_line(line) for line in
                     bytes(process.readAllStandardOutput()).decode("utf-8", "replace").splitlines()])
        process.finished.connect(lambda code, _status: self.log.append_line(
            f"[相机] {action} 结束 code={code}", "ok" if code == 0 else "err"))
        process.start(command[0], command[1:])

    def _diagnose_scene(self) -> None:
        """把 3D 四个数据源的真实状态打到日志里（每项都是 ros2 命令，失败不影响界面）。"""
        checks = (
            ("/joint_states", ["ros2", "topic", "info", "/joint_states"]),
            ("/compute_fk", ["ros2", "service", "list"]),
            ("点云 /sensors/camera/points",
             ["ros2", "topic", "info", "/sensors/camera/points"]),
            ("/robot_description", ["ros2", "topic", "info", "/robot_description"]),
        )
        self.log.append_line("[诊断] 开始检查 3D 数据源", "info")
        for label, command in checks:
            code, text = run_cmd(command, timeout=8.0)
            if code != 0:
                self.log.append_line(f"[诊断] {label}: 命令失败（ROS 未启动？）", "err")
                continue
            if label == "/compute_fk":
                hit = "/compute_fk" in text
                self.log.append_line(
                    f"[诊断] /compute_fk: {'可用' if hit else '缺失（move_group 未运行）'}",
                    "ok" if hit else "warn")
                continue
            publishers = re.search(r"Publisher count:\s*(\d+)", text)
            subscribers = re.search(r"Subscription count:\s*(\d+)", text)
            count = publishers.group(1) if publishers else "?"
            subs = subscribers.group(1) if subscribers else "?"
            level = "ok" if count not in ("0", "?") else "warn"
            self.log.append_line(f"[诊断] {label}: 发布者 {count} / 订阅者 {subs}", level)

    def _open_rviz(self) -> None:
        """在本面板之外打开标准 RViz（同一份 MoveIt 显示配置）。"""
        QtCore.QProcess.startDetached(
            "bash", [str(REPO / "scripts" / "open_rviz.sh")], str(REPO))
        self.log.append_line("[面板] 已请求打开 RViz（scripts/open_rviz.sh）", "info")

    def _start(self) -> None:
        self._spawn(["bash", str(RESUME)])

    def _resume(self) -> None:
        self._spawn(["bash", str(RESUME), "--no-restart"])

    def _abort(self) -> None:
        if self._process is None:
            return
        try:
            os.kill(int(self._process.processId()), signal.SIGINT)
            self.log.append_line("[面板] 已发送 SIGINT", "warn")
        except OSError as exc:
            self.log.append_line(f"[面板] 中止失败：{exc}", "err")

    # ---------------------------------------------------------------- update
    def _on_status(self, payload: dict) -> None:
        safety = payload.get("safety")
        level = "ok" if safety == 1 else ("warn" if safety in (2, 4, 5) else "bad")
        self.chips["safety"].set_value(level, "—" if safety is None else str(safety))
        robot = payload.get("robot")
        level = "ok" if robot == 7 else "bad"
        self.chips["robot"].set_value(level, "—" if robot is None else str(robot))
        channels = payload.get("channels", 0)
        self.chips["channels"].set_value(
            "ok" if channels >= 3 else ("warn" if channels else "bad"), f"{channels}/3")

    def _on_telemetry(self, payload: dict) -> None:
        efforts = payload.get("efforts") or {}
        if efforts:
            self.effort_bars.set_efforts(efforts)
        tcp = payload.get("tcp")
        if tcp is not None:
            self.tcp_label.set_tcp(tcp)

    def _refresh_captures(self) -> None:
        folder = REPO / "runtime" / "rvs_teaching"
        if not folder.is_dir():
            return
        files = sorted(folder.glob("*.json"), key=lambda q: q.stat().st_mtime)[-40:]
        self.captures.setRowCount(0)
        shown = 0
        for path in reversed(files):
            try:
                data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            # 目录里还混着手工记录与历史文件；只展示真正的 RVS 采集
            if data.get("command") not in ("T0", "T1", "T2", "T3"):
                continue
            if shown >= 16:
                break
            shown += 1
            row = self.captures.rowCount()
            self.captures.insertRow(row)
            position = data.get("position") or [0, 0, 0]
            fitness = data.get("fitness")
            command = str(data.get("command"))
            limit = {"T1": 0.9, "T3": 0.9, "T2": 0.8, "T0": 0.8}.get(command, 0.8)
            values = [command,
                      "—" if fitness is None else f"{fitness:.3f}",
                      f"{position[2]:.3f}" if len(position) > 2 else "—",
                      time.strftime("%H:%M:%S", time.localtime(path.stat().st_mtime))]
            for column, text in enumerate(values):
                item = QtWidgets.QTableWidgetItem(text)
                item.setTextAlignment(
                    QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter if column
                    else QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                if column == 1 and fitness is not None:
                    item.setForeground(QtGui.QColor(
                        C_OK if fitness >= limit else C_WARN))
                self.captures.setItem(row, column, item)

    def _refresh_session(self) -> None:
        """会话卡：分支 / 最近提交 / 配置 / 工作目录（只读信息，一次算好缓存）。"""
        if getattr(self, "_session_cache", None) is not None:
            return
        branch, commit, stamp = "—", "—", "—"
        try:
            branch = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"],
                                    cwd=str(REPO), capture_output=True, text=True,
                                    timeout=4).stdout.strip() or "—"
            log = subprocess.run(["git", "log", "-1", "--format=%h %s"],
                                 cwd=str(REPO), capture_output=True, text=True,
                                 timeout=4).stdout.strip()
            commit = log.split(" ", 1)[0] if log else "—"
            stamp = log.split(" ", 1)[1][:28] if " " in log else "—"
        except (OSError, subprocess.SubprocessError):
            pass
        self.session_labels["分支"].setText(branch)
        self.session_labels["最近提交"].setText(commit)
        self.session_labels["配置"].setText(stamp)
        self.session_labels["工作目录"].setText(str(REPO))
        self._session_cache = True

    def _refresh_keys(self) -> None:
        """用最新采集与运行状态填充关键数值卡（无数据时显示 —）。"""
        captures = sorted((REPO / "runtime" / "rvs_teaching").glob("*.json")) \
            if (REPO / "runtime" / "rvs_teaching").is_dir() else []
        newest: dict[str, tuple[float, dict]] = {}
        for path in captures[-40:]:
            try:
                data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            command = data.get("command")
            stamp = path.stat().st_mtime
            if command and (command not in newest or stamp > newest[command][0]):
                newest[command] = (stamp, data)
        for command, name in (("T1", "T1 fitness"), ("T3", "T3 fitness")):
            entry = newest.get(command)
            if entry and entry[1].get("fitness") is not None:
                self.key_labels[name].setText(f"{entry[1]['fitness']:.3f}")
            else:
                self.key_labels[name].setText("—")
        if newest:
            stamp = max(value[0] for value in newest.values())
            self.key_labels["最近采集"].setText(
                time.strftime("%H:%M:%S", time.localtime(stamp)))
        state = read_run_state()
        self.key_labels["上次阶段"].setText(str(state.get("last_phase") or "—"))
        self.key_labels["反控"].setText(self.chips["channels"].value.text())
        self.key_labels["当前阶段"].setText(
            str(self._current) if self._current is not None else "—")
        ready = sum(1 for row in self.status_card.rows.values()
                    if row.value.text() not in ("—", "缺失", "move_group 未运行"))
        self.key_labels["相机"].setText(
            "在线" if self.status_card.rows["cloud"].value.text().endswith("帧/秒") else "未就绪")
        self.key_labels["RViz"].setText(f"{ready}/4 就绪")
        self.key_labels["界面"].setText(f"{self.width()}×{self.height()}")
        self._refresh_session()

    def _tick(self) -> None:
        self._refresh_keys()
        self._refresh_captures()
        if self._process is None:
            self.state_label.setText("空闲")
            return
        elapsed = time.time() - self._started_at
        self.state_label.setText(f"运行 {elapsed:6.1f}s  pid={self._process.processId()}")

    def _refresh_run_state(self) -> None:
        state = read_run_state()
        last = state.get("last_phase")
        start = state.get("resume_start_from")
        stamp = (state.get("stamp") or "")[11:19]
        self.resume_label.setText(f"上次 {last} → {start}  {stamp}" if last else "")


class EfficiencyTab(QtWidgets.QWidget):
    """效率模式：左侧配置（阶段/速度/参数/执行），右侧看结果（状态/力矩/记录/日志）。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._process: QtCore.QProcess | None = None
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(2000)
        self._timer.timeout.connect(self._refresh_views)
        self._timer.start()
        self._refresh_views()

    def _build(self) -> None:
        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # ---------------- 左：配置 ----------------
        left = QtWidgets.QWidget()
        left.setFixedWidth(400)
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        stage_box = QtWidgets.QGroupBox("选择阶段（勾选后按最小—最大连续执行）")
        stage_grid = QtWidgets.QGridLayout(stage_box)
        stage_grid.setContentsMargins(8, 6, 8, 6)
        stage_grid.setHorizontalSpacing(10)
        stage_grid.setVerticalSpacing(4)
        self.stage_checks: dict[int, QtWidgets.QCheckBox] = {}
        row = 0
        for index, (number, name) in enumerate(STAGES):
            if number == 16:
                continue
            check = QtWidgets.QCheckBox(f"{number:02d} {name}")
            check.setChecked(number in (5, 6))
            self.stage_checks[number] = check
            stage_grid.addWidget(check, index // 2, index % 2)
        left_layout.addWidget(stage_box, 0)

        speed_box = QtWidgets.QGroupBox("速度覆盖 m/s（留空＝用默认）")
        speed_grid = QtWidgets.QGridLayout(speed_box)
        speed_grid.setContentsMargins(8, 6, 8, 6)
        speed_grid.setHorizontalSpacing(12)
        speed_grid.setVerticalSpacing(3)
        self.speed_edits: dict[str, QtWidgets.QLineEdit] = {}
        for index, phase in enumerate(SPEED_PHASES):
            label = QtWidgets.QLabel(phase)
            label.setObjectName("Cap")
            edit = QtWidgets.QLineEdit()
            edit.setPlaceholderText("默认")
            edit.setFixedWidth(66)
            self.speed_edits[phase] = edit
            speed_grid.addWidget(label, index // 2, (index % 2) * 2)
            speed_grid.addWidget(edit, index // 2, (index % 2) * 2 + 1)
        left_layout.addWidget(speed_box, 0)

        param_box = QtWidgets.QGroupBox("运行参数")
        param_grid = QtWidgets.QGridLayout(param_box)
        param_grid.setContentsMargins(8, 6, 8, 6)
        param_grid.setHorizontalSpacing(12)
        param_grid.setVerticalSpacing(4)
        self.repeat_edit = QtWidgets.QLineEdit("1")
        self.repeat_edit.setFixedWidth(56)
        self.check_nogates = QtWidgets.QCheckBox("跳过位姿门")
        self.check_nosettle = QtWidgets.QCheckBox("跳过静置")
        self.check_dry = QtWidgets.QCheckBox("预演（不动臂）")
        param_grid.addWidget(QtWidgets.QLabel("重复次数"), 0, 0)
        param_grid.addWidget(self.repeat_edit, 0, 1)
        param_grid.addWidget(self.check_nogates, 1, 0)
        param_grid.addWidget(self.check_nosettle, 1, 1)
        param_grid.addWidget(self.check_dry, 2, 0)
        left_layout.addWidget(param_box, 0)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(4)
        self.btn_run = QtWidgets.QPushButton("执行所选阶段")
        self.btn_run.setProperty("role", "go")
        self.btn_run.setFixedHeight(34)
        self.btn_run.clicked.connect(self._run)
        self.btn_abort = QtWidgets.QPushButton("中止")
        self.btn_abort.setProperty("role", "stop")
        self.btn_abort.setFixedHeight(34)
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self._abort)
        buttons.addWidget(self.btn_run, 3)
        buttons.addWidget(self.btn_abort, 1)
        left_layout.addLayout(buttons)

        self.log = LogView()
        left_layout.addWidget(make_group("日志", self.log), 1)
        root.addWidget(left, 0)

        # ---------------- 右：结果 ----------------
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        self.status_card = RvizStatusCard(probe=False)
        self.status_card.open_rviz.connect(
            lambda: QtCore.QProcess.startDetached(
                "bash", [str(REPO / "scripts" / "open_rviz.sh")], str(REPO)))
        status_box = make_group("3D 状态（RViz）", self.status_card)
        status_box.setFixedHeight(158)
        right_layout.addWidget(status_box, 0)

        self.effort_chart = EffortChart()
        right_layout.addWidget(make_group("插入力矩曲线（debug_runs/*/effort.csv）",
                                          self.effort_chart), 1)

        self.results = QtWidgets.QTableWidget(0, 6)
        self.results.setHorizontalHeaderLabels(["时间", "成功", "时长 s", "卡滞", "力峰值", "阶段"])
        self.results.verticalHeader().setVisible(False)
        self.results.setShowGrid(False)
        self.results.verticalHeader().setDefaultSectionSize(22)
        self.results.horizontalHeader().setStretchLastSection(True)
        self.results.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        right_layout.addWidget(make_group("运行记录", self.results), 1)
        root.addWidget(right, 1)

    def _build_command(self) -> list[str]:
        stages = [number for number, check in self.stage_checks.items() if check.isChecked()]
        first, last = (min(stages), max(stages)) if stages else (5, 5)
        command = [sys.executable, str(DEBUG), "--from", str(first), "--to", str(last)]
        if self.check_nogates.isChecked():
            command.append("--no-gates")
        if self.check_nosettle.isChecked():
            command.append("--no-settle")
        if self.check_dry.isChecked():
            command.append("--dry-run")
        repeat = self.repeat_edit.text().strip()
        if repeat.isdigit() and int(repeat) > 1:
            command += ["--repeat", repeat]
        for phase, edit in self.speed_edits.items():
            value = edit.text().strip()
            if value:
                command += ["--speed-override", f"{phase}={value}"]
        return command

    def _run(self) -> None:
        if self._process is not None:
            return
        command = self._build_command()
        self.log.append_line("$ " + " ".join(command), "info")
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(REPO))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._drain)
        process.finished.connect(self._finished)
        process.start(command[0], command[1:])
        self._process = process
        self.btn_run.setEnabled(False)
        self.btn_abort.setEnabled(True)

    def _drain(self) -> None:
        if self._process is None:
            return
        text = bytes(self._process.readAllStandardOutput()).decode("utf-8", "replace")
        for line in text.splitlines():
            self.log.append_line(line)

    def _finished(self, code: int, status) -> None:
        self.log.append_line(f"[面板] 结束 code={code}", "ok" if code == 0 else "err")
        self._process = None
        self.btn_run.setEnabled(True)
        self.btn_abort.setEnabled(False)
        self._refresh_views()

    def _abort(self) -> None:
        if self._process is None:
            return
        try:
            os.kill(int(self._process.processId()), signal.SIGINT)
        except OSError:
            pass

    def _refresh_views(self) -> None:
        runs = sorted(DEBUG_RUNS.glob("*/result.json")) if DEBUG_RUNS.is_dir() else []
        self.results.setRowCount(0)
        for path in runs[-10:]:
            try:
                data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            row = self.results.rowCount()
            self.results.insertRow(row)
            name = path.parent.name
            values = [name[9:15] if len(name) > 15 else name,
                      "是" if data.get("success") is True else
                      ("否" if data.get("success") is False else "预演"),
                      str(data.get("duration_s", "—")),
                      str(data.get("jam_count", "—")),
                      str(data.get("force_peak_n", "—"))]
            for column, text in enumerate(values):
                item = QtWidgets.QTableWidgetItem(text)
                item.setTextAlignment(
                    QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter if column
                    else QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                self.results.setItem(row, column, item)
        if runs:
            self.effort_chart.load(runs[-1].parent / "effort.csv")


class PanelWindow(QtWidgets.QMainWindow):
    def __init__(self, smoke: bool = False, probe: bool = True) -> None:
        super().__init__()
        self.setWindowTitle("CS625")
        self.setStyleSheet(build_style(SCALE[0]))
        self.setMinimumSize(*MIN_SIZE)   # 紧凑布局下 1280x760 依然够用
        screen = QtWidgets.QApplication.primaryScreen()
        geometry = screen.availableGeometry() if screen is not None else None
        self.resize(*(target_size(geometry) if geometry is not None else DEFAULT_SIZE))

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(5, 3, 5, 3)
        layout.setSpacing(2)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("CS625")
        title.setObjectName("Title")
        header.addWidget(title, 0)
        header.addStretch(1)
        layout.addLayout(header)

        self.tabs = QtWidgets.QTabWidget()
        self.safe = SafeTab(probe=probe and not smoke)
        self.efficient = EfficiencyTab()
        self.tabs.addTab(self.safe, "安全模式")
        self.tabs.addTab(self.efficient, "效率模式")
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

    def closeEvent(self, event) -> None:            # noqa: N802
        poller = getattr(self.safe, "_poller", None)
        if poller is not None:
            poller.stop()
            poller.wait(1200)
        for tab in (self.safe, self.efficient):
            card = getattr(tab, "status_card", None)
            if card is not None:
                card.stop()
            camera = getattr(tab, "camera", None)
            if camera is not None:
                camera.stop()
        super().closeEvent(event)


def run_smoke(app: QtWidgets.QApplication, window: PanelWindow, path: pathlib.Path) -> int:
    window.show()
    app.processEvents()
    QtCore.QTimer.singleShot(500, app.quit)
    app.exec_()
    pixmap = window.grab()
    pixmap.save(str(path))
    print(f"[smoke] {pixmap.width()}x{pixmap.height()} -> {path} ({path.stat().st_size} 字节)")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--smoke", metavar="PNG", help="离屏渲染截图后退出（不需要 ROS）")
    parser.add_argument("--no-probe", action="store_true", help="不读取 ROS 状态")
    parser.add_argument("--scale", type=float, default=None,
                        help="界面缩放系数（默认按屏幕高度自动：1080p=1.0，4K≈1.7）")
    parser.add_argument("--size", metavar="WxH", default=None,
                        help="强制窗口尺寸（离屏截图或单屏验证用）")
    args = parser.parse_args(argv)

    if args.smoke:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    screen = app.primaryScreen()
    height = screen.availableGeometry().height() if screen is not None else 1250
    if args.scale is not None:
        SCALE[0] = args.scale
    else:
        set_scale(height)
    app.setStyleSheet(build_style(SCALE[0]))
    print(f"[panel] 屏幕 {screen.availableGeometry().width()}x{height} "
          f"→ 缩放 {SCALE[0]:.2f}")
    window = PanelWindow(smoke=bool(args.smoke), probe=not args.no_probe)
    if args.size:
        width, _, height = args.size.lower().partition("x")
        window.resize(int(width), int(height))
    if args.smoke:
        return run_smoke(app, window, pathlib.Path(args.smoke))
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
