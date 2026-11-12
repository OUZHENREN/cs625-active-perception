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
import threading
import time
from typing import Sequence

import yaml

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
    # 4K 屏上直接把窗口开大（所有者 2026-11-08：基本都够用，别裁信息）
    width = min(int(screen.width() * 0.94), 3600)
    height = min(int(screen.height() * 0.94), 2000)
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
EFFICIENCY_SCOPE = "load"
EFFICIENCY_MODE = "efficient"
PHASE_LABELS = {
    "observe:workpiece_coarse": "工件粗观测",
    "observe:workpiece_fine": "工件精观测",
    "observe:tables": "桌面观测",
    "observe:slot_coarse": "插槽粗观测",
    "observe:slot_fine": "插槽精观测",
    "nbv:workpiece": "NBV 换精观测位",
    "measure:workpiece_fine": "工件精观测（原地）",
    "measure:slot_fine": "插槽精观测（原地）",
    "nbv:slot": "NBV 换槽精观测位",
    "baseline": "基准位",
    "pre_grasp": "预抓取",
    "gripper_init": "夹爪初始化",
    "grasp": "进入抓取",
    "grip": "张开涨紧",
    "lift": "抬升",
    "transit": "转运",
    "staging": "槽前分级",
    "pre_insert": "预插入",
    "insert": "插入",
    "straighten": "转正",
    "release": "释放",
    "withdraw": "退出",
}


def efficiency_phases() -> list[str]:
    """效率流程的真实阶段序列（读 config/tasks/<scope>_<mode>.yaml）。

    面板以前用的是示教链的静态阶段表，其中"插槽粗观测"被删掉了，
    所以效率模式里看不到这一步——其实流程配置里一直有。
    """
    path = REPO / "config" / "tasks" / f"{EFFICIENCY_SCOPE}_{EFFICIENCY_MODE}.yaml"
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        phases = [str(v) for v in (data.get("phases") or [])]
        return phases or []
    except OSError:
        return []


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
    body = 10.5 * scale
    small = 9.0 * scale
    mono = 10.5 * scale
    title = 13.0 * scale
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
        self.verticalHeader().setDefaultSectionSize(26)
        fixed_height = 26 * len(STAGES) + 32
        self.setFixedHeight(fixed_height)
        self._rows: dict[int, int] = {}
        self._state: dict[int, str] = {}
        header = self.horizontalHeader()
        for column in range(5):
            item = self.horizontalHeaderItem(column)
            if item is not None:
                align = (QtCore.Qt.AlignRight if column in (0, 3)
                         else QtCore.Qt.AlignLeft)
                item.setTextAlignment(align | QtCore.Qt.AlignVCenter)
        for row, (number, name) in enumerate(STAGES):
            self._rows[number] = row
            self._state[number] = "idle"
            for column, text in enumerate((f"{number:02d}", name, "未开始", "",
                                           STAGE_NOTES.get(number, ""))):
                item = QtWidgets.QTableWidgetItem(text)
                right = column in (0, 3)
                item.setTextAlignment((QtCore.Qt.AlignRight if right else QtCore.Qt.AlignLeft)
                                      | QtCore.Qt.AlignVCenter)
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


class WrenchChart(QtWidgets.QWidget):
    """六维力曲线：六条不同颜色的轨迹，带 0 轴与图例；无传感器时显示"未采集"。"""

    AXES = (("Fx", "#e05252"), ("Fy", "#4ec27a"), ("Fz", "#4a90d9"),
            ("Mx", "#d6a52f"), ("My", "#b07ad6"), ("Mz", "#2fd6c9"))
    WINDOW = 300          # 保留最近 300 个采样点

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._series: dict[str, list[float]] = {name: [] for name, _ in self.AXES}
        self._have_data = False

    def push(self, values: dict | None) -> None:
        if not values:
            self._have_data = False
        else:
            self._have_data = True
            for name, _ in self.AXES:
                series = self._series[name]
                series.append(float(values.get(name, 0.0)))
                del series[:-self.WINDOW]
        self.update()

    def paintEvent(self, event) -> None:            # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0f14"))
        font = painter.font()
        font.setPointSizeF(8.0)
        painter.setFont(font)
        width, height = self.width(), self.height()
        if not self._have_data:
            painter.setPen(QtGui.QColor("#465866"))
            painter.drawText(10, height // 2, "六维力未采集（传感器未安装）")
            self._draw_legend(painter, width)
            painter.end()
            return
        values = [abs(v) for series in self._series.values() for v in series] or [1.0]
        peak = max(1e-3, max(values))
        left, right, top, bottom = 8, width - 8, 8, height - 16
        mid = (top + bottom) / 2
        painter.setPen(QtGui.QPen(QtGui.QColor("#22303c"), 1))
        painter.drawLine(left, int(mid), right, int(mid))
        for name, colour in self.AXES:
            series = self._series[name]
            if len(series) < 2:
                continue
            path = QtGui.QPainterPath()
            for index, value in enumerate(series):
                x = left + (right - left) * index / max(1, self.WINDOW - 1)
                y = mid - (bottom - top) / 2 * (value / peak)
                path.moveTo(x, y) if index == 0 else path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor(colour), 1.3))
            painter.drawPath(path)
        painter.setPen(QtGui.QColor("#5d7284"))
        painter.drawText(left, height - 4, f"峰值 {peak:.1f}")
        self._draw_legend(painter, width)
        painter.end()

    def _draw_legend(self, painter: QtGui.QPainter, width: int) -> None:
        x = width - 250
        for name, colour in self.AXES:
            painter.setPen(QtGui.QColor(colour))
            painter.drawText(x, 12, name)
            x += 40


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

        # 效率模式的两个 RViz：一个看碰撞与规划，一个看点云（与示教模式的 RViz 分开）
        self.btn_rviz = QtWidgets.QPushButton("RViz ▾")
        self.btn_rviz.setFixedWidth(76)
        self.btn_rviz.setToolTip("效率模式观察用 RViz：碰撞/规划 与 点云 分开打开")
        rviz_menu = QtWidgets.QMenu(self.btn_rviz)
        rviz_menu.addAction("碰撞与规划", lambda: self._rviz("planning"))
        rviz_menu.addAction("点云", lambda: self._rviz("cloud"))
        rviz_menu.addSeparator()
        rviz_menu.addAction("两个都开", lambda: self._rviz("both"))
        self.btn_rviz.setMenu(rviz_menu)
        top.addWidget(self.btn_rviz, 0)
        root.addLayout(top)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setHandleWidth(4)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_right())
        splitter.addWidget(self._build_camera_column())
        splitter.setStretchFactor(0, 24)
        splitter.setStretchFactor(1, 30)
        splitter.setStretchFactor(2, 46)
        splitter.setSizes([460, 580, 900])
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
        """中列：力矩(整行) / 六维力(整行) / [TCP+夹爪控制] | [关键数值] / [采集伸到底] | [3D+摘要]。"""
        panel = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(panel)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(3)

        self.effort_bars = EffortBars()
        self.effort_bars.setFixedHeight(150)
        effort_box = make_group("关节力矩 N·m（满量程 60）", self.effort_bars)
        effort_box.setFixedHeight(178)
        outer.addWidget(effort_box, 0)

        self.wrench = WrenchChart()
        self.wrench.setFixedHeight(150)          # 与关节力矩同高
        wrench_box = make_group("六维力（未安装）", self.wrench)
        wrench_box.setFixedHeight(178)
        outer.addWidget(wrench_box, 0)

        # 下半部分：左右两列
        lower = QtWidgets.QWidget()
        lower_layout = QtWidgets.QHBoxLayout(lower)
        lower_layout.setContentsMargins(0, 0, 0, 0)
        lower_layout.setSpacing(3)

        left_column = QtWidgets.QVBoxLayout()
        left_column.setSpacing(3)
        self.tcp_label = TcpGripLabel()
        self.tcp_label.setFixedHeight(52)
        left_column.addWidget(make_group("TCP / 夹爪", self.tcp_label), 0)

        gripper_box = QtWidgets.QGroupBox("夹爪控制")
        gripper_layout = QtWidgets.QVBoxLayout(gripper_box)
        gripper_layout.setContentsMargins(8, 4, 8, 4)
        gripper_layout.setSpacing(3)
        self.gripper_label = QtWidgets.QLabel("位置 — · 状态 —")
        self.gripper_label.setObjectName("Mono")
        gripper_layout.addWidget(self.gripper_label)
        gripper_buttons = QtWidgets.QGridLayout()
        gripper_buttons.setSpacing(3)
        for index, (text, command) in enumerate((("状态", "status"), ("初始化", "init"),
                                                 ("张开", "release"), ("闭合", "grip"))):
            button = QtWidgets.QPushButton(text)
            button.setFixedHeight(28)
            button.clicked.connect(lambda _checked=False, c=command: self._gripper(c))
            gripper_buttons.addWidget(button, index // 2, index % 2)
        gripper_layout.addLayout(gripper_buttons)
        gripper_box.setFixedHeight(112)
        left_column.addWidget(gripper_box, 0)

        self.captures = QtWidgets.QTableWidget(0, 4)
        self.captures.setHorizontalHeaderLabels(["命令", "fitness", "z m", "时间"])
        self.captures.verticalHeader().setVisible(False)
        self.captures.setShowGrid(False)
        self.captures.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.captures.verticalHeader().setDefaultSectionSize(24)
        header = self.captures.horizontalHeader()
        header.setStretchLastSection(True)
        for column, width in {0: 46, 1: 66, 2: 60}.items():
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            self.captures.setColumnWidth(column, width)
        left_column.addWidget(make_group("最近采集", self.captures), 1)
        lower_layout.addLayout(left_column, 1)

        right_column = QtWidgets.QVBoxLayout()
        right_column.setSpacing(3)
        key_box = QtWidgets.QGroupBox("关键数值")
        key_grid = QtWidgets.QGridLayout(key_box)
        key_grid.setContentsMargins(8, 4, 8, 4)
        key_grid.setHorizontalSpacing(8)
        key_grid.setVerticalSpacing(2)
        self.key_labels: dict[str, QtWidgets.QLabel] = {}
        for row, name in enumerate(("T1 fitness", "T3 fitness", "最近采集", "上次阶段",
                                    "反控", "当前阶段", "相机", "RViz", "分支", "提交")):
            label = QtWidgets.QLabel(name)
            label.setObjectName("Cap")
            value = QtWidgets.QLabel("—")
            value.setObjectName("Mono")
            self.key_labels[name] = value
            key_grid.addWidget(label, row, 0)
            key_grid.addWidget(value, row, 1)
        key_grid.setColumnStretch(1, 1)
        key_box.setFixedHeight(268)
        right_column.addWidget(key_box, 0)

        self.status_card = RvizStatusCard(probe=self._probe)
        self.status_card.open_rviz.connect(self._open_rviz)
        self.status_card.diagnose.connect(self._diagnose_scene)
        if self._probe:
            self.status_card._worker.frame.connect(
                lambda payload: self._on_telemetry(payload))
        status_box = make_group("3D 状态（RViz）", self.status_card)
        status_box.setFixedHeight(206)
        right_column.addWidget(status_box, 0)

        # 长信息栏：最近一次运行摘要（放得下就多写几行）
        self.summary = LogView()
        right_column.addWidget(make_group("最近一次运行", self.summary), 1)
        lower_layout.addLayout(right_column, 1)

        outer.addWidget(lower, 1)
        return panel

    def _build_camera_column(self) -> QtWidgets.QWidget:
        """右列：相机整列高（原始像素 1:1）+ 最近采集 + 3D 状态。"""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        self.camera = CameraView(probe=self._probe)
        layout.addWidget(make_group("相机（1:1 原始像素）", self.camera), 1)
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

    def _rviz(self, mode: str) -> None:
        """打开效率模式的 RViz（独立进程，不阻塞面板）。"""
        command = ["bash", str(REPO / "scripts" / "rviz_efficiency.sh"), f"--{mode}"]
        self.log.append_line("$ " + " ".join(command), "info")
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(REPO))
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: [self.log.append_line(line) for line in
                     bytes(process.readAllStandardOutput()).decode("utf-8", "replace").splitlines()])
        process.finished.connect(lambda code, _status: self.log.append_line(
            f"[rviz] {mode} 结束 code={code}", "ok" if code == 0 else "err"))
        process.start(command[0], command[1:])

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
        self.key_labels["分支"].setText(branch)
        self.key_labels["提交"].setText(f"{commit} · {stamp}")
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


class VisionPipelineView(QtWidgets.QWidget):
    """视觉流程"摊开"视图：把每个算法步骤的输入/输出/状态/耗时列出来。

    数据来自 /vision_stage（measure_object 每个阶段发一条 JSON）。
    目的：让视觉不再是黑盒——能直接看到"现在走到哪一步、这一步吃进去什么、吐出来什么"。
    """

    STAGES = [
        ("capture", "① 采集", "组数×帧数", "帧数/分辨率"),
        ("segment", "② 分割", "原始点云", "剔桌面/高度过滤/球裁后点数"),
        ("planes", "③ 平面", "工件点云", "平面数/内点数/是否正交"),
        ("register", "④ 配准", "平面 + CAD", "选解/扶手分/覆盖率"),
        ("gate", "⑤ 质量门", "各组位姿", "判定/重复性/中心"),
    ]

    stage_received = QtCore.pyqtSignal(dict)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop = False
        self.stage_received.connect(self.on_stage)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("视觉流程（实时）")
        title.setObjectName("Cap")
        layout.addWidget(title)
        self.table = QtWidgets.QTableWidget(len(self.STAGES), 5)
        self.table.setHorizontalHeaderLabels(["步骤", "状态", "输入", "输出", "耗时"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setShowGrid(False)
        self.table.verticalHeader().setDefaultSectionSize(22)
        self.table.horizontalHeader().setStretchLastSection(True)
        self._started: dict[str, float] = {}
        for row, (key, label, _in, _out) in enumerate(self.STAGES):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(label))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem("待"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem("—"))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem("—"))
            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem("—"))
            self.table.setRowHeight(row, 52)
        self.table.resizeColumnsToContents()
        self.table.setColumnWidth(3, 320)
        layout.addWidget(self.table, 1)
        self._thread = threading.Thread(target=self._ros_loop, daemon=True)
        self._thread.start()

    def _ros_loop(self) -> None:
        """订阅 /vision_stage：measure_object 每完成一步就发一条 JSON。"""
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from panel_status import SPIN_LOCK
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("vision_pipeline_view")
            node.create_subscription(
                String, "/vision_stage",
                lambda msg: self.stage_received.emit(json.loads(msg.data or "{}")), 10)
            while not self._stop and rclpy.ok():
                with SPIN_LOCK:
                    rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:  # noqa: BLE001 - 没有 ROS 时保持空白即可
            return

    def reset(self) -> None:
        for row in range(self.table.rowCount()):
            self.table.item(row, 1).setText("待")
            self.table.item(row, 2).setText("—")
            self.table.item(row, 3).setText("—")
            self.table.item(row, 4).setText("—")
        self._started.clear()

    def on_stage(self, payload: dict) -> None:
        stage = str(payload.get("stage") or "")
        index = next((i for i, (key, *_rest) in enumerate(self.STAGES) if key == stage), None)
        if index is None:
            return
        now = time.time()
        started = self._started.setdefault(stage, now)
        elapsed = now - started
        _key, _label, input_hint, output_hint = self.STAGES[index]
        detail_in = []
        detail_out = []
        if stage == "segment":
            detail_in.append(f"{payload.get('points', '?')} 点")
            detail_out.append(f"外形 {payload.get('extent_mm')}")
            status = "OK"
        elif stage == "planes":
            detail_in.append(f"{payload.get('planes', '?')} 个面")
            detail_out.append(f"内点 {payload.get('inliers')}")
            detail_out.append("正交三面" if payload.get("orthogonal_triple") else "退化")
            status = "OK" if payload.get("orthogonal_triple") else "退化"
        elif stage == "register":
            detail_in.append("多起点 + CAD")
            detail_out.append(f"{payload.get('chosen')} 扶手 {payload.get('handle_score')}")
            detail_out.append(f"整体 {payload.get('overall')}")
            status = "OK" if payload.get("valid") else "可疑"
        elif stage == "gate":
            detail_in.append(f"{payload.get('credible')}/{payload.get('group_total')} 组")
            detail_out.append(f"{payload.get('verdict')} 覆盖 {payload.get('coverage')}")
            detail_out.append(f"{payload.get('translation_mm')} mm / "
                              f"{payload.get('rotation_deg')}°")
            status = "通过" if payload.get("verdict") == "OK" else "拒绝"
        else:
            status = "OK"
        colour = {"OK": C_OK, "通过": C_OK, "退化": C_WARN, "可疑": C_WARN,
                  "拒绝": C_BAD}.get(status, C_TEXT)
        self.table.item(index, 1).setText(status)
        self.table.item(index, 1).setForeground(QtGui.QColor(colour))
        self.table.item(index, 2).setText(" / ".join(detail_in) or input_hint)
        self.table.item(index, 3).setText(" / ".join(detail_out) or output_hint)
        self.table.item(index, 4).setText(f"{elapsed:.1f} s")


class VisionDataBox(QtWidgets.QGroupBox):
    """右上角的"视觉实验数据"：读 runtime/measurements/*.json，展示两类目标的测量结果。

    放什么（按调试时的顺序）：
      判定 / 覆盖率 / 组数(可信-退化) / 平移重复性 / 旋转重复性 / 中心坐标 / 更新时间
    这些正是 measure_object 的质量门指标，看板上一眼就能判断"这次测得准不准"。
    """

    TARGETS = (
        ("工件", REPO / "runtime" / "measurements" / "workpiece_latest.json"),
        ("插槽", REPO / "runtime" / "measurements" / "slot_latest.json"),
    )
    ROWS = ("判定", "覆盖率", "组数(可信/退化)", "平移重复性", "旋转重复性", "中心 (mm)", "更新时间")

    def __init__(self, parent=None) -> None:
        super().__init__("视觉实验数据", parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        self.table = QtWidgets.QTableWidget(len(self.ROWS), 3)
        self.table.setHorizontalHeaderLabels(["指标", "工件", "插槽"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        for row, name in enumerate(self.ROWS):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
        self.table.resizeColumnsToContents()
        layout.addWidget(self.table)
        self._last = 0.0
        self.refresh()

    def refresh(self) -> None:
        for column, (_label, path) in enumerate(self.TARGETS, start=1):
            data = {}
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                data = {}
            verdict = str(data.get("verdict") or "—")
            values = [
                verdict,
                f"{data.get('coverage_median', 0):.3f}" if data else "—",
                (f"{data.get('groups', 0) - data.get('degenerate_groups_dropped', 0)}"
                 f"/{data.get('degenerate_groups_dropped', 0)}") if data else "—",
                f"{data.get('repeatability', {}).get('translation_mm', 0):.2f} mm"
                if data else "—",
                f"{data.get('repeatability', {}).get('rotation_deg', 0):.2f}°"
                if data else "—",
                ", ".join(f"{v:.1f}" for v in (data.get("pose_base_centre_mm") or []))
                if data else "—",
                time.strftime("%H:%M:%S", time.localtime(path.stat().st_mtime))
                if path.exists() else "—",
            ]
            for row, text in enumerate(values):
                item = self.table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.table.setItem(row, column, item)
                item.setText(text)
                if row == 0:
                    level = "#3ec98a" if verdict == "OK" else (
                        "#d6a52f" if verdict == "—" else "#e05252")
                    item.setForeground(QtGui.QColor(level))
        self.table.resizeColumnsToContents()


class EfficiencyTab(QtWidgets.QWidget):
    """效率模式：左侧配置（阶段/速度/参数/执行），右侧看结果（状态/力矩/记录/日志）。"""

    def __init__(self, probe: bool = True, parent=None) -> None:
        super().__init__(parent)
        self._probe = probe
        self._process: QtCore.QProcess | None = None
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(2000)
        self._timer.timeout.connect(self._refresh_views)
        self._timer.timeout.connect(
            lambda: self.vision_data.refresh() if getattr(self, "vision_data", None) else None)
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
        # 阶段列表直接来自效率流程配置（含 observe:slot_coarse 等观测点）
        self.efficiency_phase_keys = efficiency_phases()
        # 注意：流程里 observe:slot_coarse 出现两次（开头和收尾），
        # 所以勾选框按【序号】而不是阶段名做 key，否则两个会互相覆盖。
        self.stage_checks: dict[int, QtWidgets.QCheckBox] = {}
        if not self.efficiency_phase_keys:
            fallback = QtWidgets.QLabel("读不到 config/tasks/load_efficient.yaml 的阶段表")
            fallback.setProperty("level", "bad")
            stage_grid.addWidget(fallback, 0, 0)
        for index, key in enumerate(self.efficiency_phase_keys):
            label = PHASE_LABELS.get(key, key)
            check = QtWidgets.QCheckBox(f"{index + 1:02d} {label}")
            check.setChecked(key in ("pre_grasp", "grasp", "grip"))
            self.stage_checks[index] = check
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

        # 相机：效率模式下同样需要看实时画面（1:1，不缩放）
        # 相机 + 视觉流程并排：把相机右侧那块空白用"算法每一步"占住
        camera_row = QtWidgets.QWidget()
        camera_row_layout = QtWidgets.QHBoxLayout(camera_row)
        camera_row_layout.setContentsMargins(0, 0, 0, 0)
        camera_row_layout.setSpacing(4)
        self.camera = CameraView(probe=self._probe)
        camera_row_layout.addWidget(make_group("相机（1:1 原始像素）", self.camera), 3)
        self.pipeline = VisionPipelineView()
        self.pipeline.setFixedWidth(430)
        camera_row_layout.addWidget(make_group("视觉流程（每一步的输入/输出）",
                                               self.pipeline), 0)
        right_layout.addWidget(camera_row, 3)

        # 视觉实验数据：测量判定/覆盖率/重复性/中心
        self.vision_data = VisionDataBox()
        self.vision_data.setFixedHeight(190)
        right_layout.addWidget(self.vision_data, 0)

        self.effort_chart = EffortChart()
        right_layout.addWidget(make_group("插入力矩曲线（debug_runs/*/effort.csv）",
                                          self.effort_chart), 1)
        self.wrench_chart = WrenchChart()
        self.wrench_chart.push(None)
        right_layout.addWidget(make_group("六维力（未安装）", self.wrench_chart), 1)

        self.results = QtWidgets.QTableWidget(0, 6)
        self.results.setHorizontalHeaderLabels(["时间", "成功", "时长 s", "卡滞", "力峰值", "备注"])
        self.results.verticalHeader().setVisible(False)
        self.results.setShowGrid(False)
        self.results.verticalHeader().setDefaultSectionSize(24)
        header = self.results.horizontalHeader()
        header.setStretchLastSection(True)
        for column, width in {0: 96, 1: 52, 2: 62, 3: 52, 4: 70}.items():
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            self.results.setColumnWidth(column, width)
        for column in range(6):
            item = self.results.horizontalHeaderItem(column)
            if item is not None:
                item.setTextAlignment((QtCore.Qt.AlignLeft if column in (0, 5)
                                       else QtCore.Qt.AlignRight) | QtCore.Qt.AlignVCenter)
        self.results.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)

        self.status_card = RvizStatusCard(probe=False)
        self.status_card.open_rviz.connect(
            lambda: QtCore.QProcess.startDetached(
                "bash", [str(REPO / "scripts" / "open_rviz.sh")], str(REPO)))

        bottom = QtWidgets.QWidget()
        bottom_row = QtWidgets.QHBoxLayout(bottom)
        bottom_row.setContentsMargins(0, 0, 0, 0)
        bottom_row.setSpacing(3)
        status_box = make_group("3D 状态（RViz）", self.status_card)
        status_box.setFixedHeight(206)
        bottom_row.addWidget(status_box, 1)
        results_box = make_group("运行记录", self.results)
        results_box.setFixedHeight(206)
        bottom_row.addWidget(results_box, 1)
        right_layout.addWidget(bottom, 0)

        root.addWidget(right, 1)

    def _build_command(self) -> list[str]:
        """效率模式跑的是【新流程】run_task.py，不再是示教链。

        选中的阶段用 --until 截断（runner 支持"跑到某个阶段就停"）；
        若要从中途开始，runner 目前没有 --start-from，这里只提示、仍然从流程开头跑。
        """
        selected = [self.efficiency_phase_keys[index]
                    for index, check in self.stage_checks.items() if check.isChecked()]
        command = [sys.executable, str(REPO / "scripts" / "run_task.py"),
                   "--scope", EFFICIENCY_SCOPE, "--mode", EFFICIENCY_MODE,
                   "--world", "real"]
        if self.check_dry.isChecked():
            command.append("--dry-run")
        else:
            command.append("--confirm")
        if selected:
            order = [k for k in self.efficiency_phase_keys if k in selected]
            if order:
                last = order[-1]
                command += ["--until", last]
                first = order[0]
                if first != self.efficiency_phase_keys[0]:
                    self.log.append_line(
                        f"[面板] 选中起点 {first}，但 runner 不支持 --start-from："
                        f"将从流程开头跑到 {last}", "warn")
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
                    (QtCore.Qt.AlignLeft if column in (0, 5) else QtCore.Qt.AlignRight)
                    | QtCore.Qt.AlignVCenter)
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
        self.efficient = EfficiencyTab(probe=probe and not smoke)
        # 两个标签页的相机都走同一条"把窗口撑够大"的路径：1:1 且不允许滚动，
        # 唯一能保证看得全的办法就是窗口/列必须放得下整幅图像。
        for tab in (self.safe, self.efficient):
            camera = getattr(tab, "camera", None)
            if camera is not None:
                camera.image_size.connect(self._ensure_camera_room)
                if camera._image is not None:
                    self._ensure_camera_room(camera._image.width(), camera._image.height())
        self.tabs.addTab(self.safe, "安全模式")
        self.tabs.addTab(self.efficient, "效率模式")
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

    def _ensure_camera_room(self, width: int, height: int) -> None:
        """1:1 不滚动的前提：窗口与最小尺寸必须能装下整幅图像。"""
        need_w = width + 1000          # 左列 + 中列 + 边距
        need_h = height + 220
        if self.minimumWidth() < need_w or self.minimumHeight() < need_h:
            self.setMinimumSize(max(MIN_SIZE[0], need_w), max(MIN_SIZE[1], need_h))
        if self.width() < need_w or self.height() < need_h:
            self.resize(max(self.width(), need_w), max(self.height(), need_h))

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
