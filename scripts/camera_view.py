#!/usr/bin/env python3
"""ROS 端相机查看器：直接看 /camera/*，并带模式/分辨率切换按钮。

为什么单独一个窗口
------------------
``cs625_panel.py`` 是"两模式操作面板"，相机只是它的一栏，而且订阅的是
``/sensors/camera/*``（需要 sensor_adapter 在跑）。今天这种"只开相机、不动夹爪"
的场合，需要的是一个只做相机事情的窗口：订阅驱动原始话题，按钮换档。

它只做三件事：
  1. 订阅相机（彩色 / 深度 / 红外），1:1 或铺满显示，状态栏给出分辨率与真实帧率；
  2. 通过 ``scripts/camera_mode.py`` 换档（fast / modeling / high）与查询状态，
     输出进下方日志，不阻塞界面；
  3. 截图存到 ``runtime/camera_snapshots/``。

安全边界（重要）
----------------
本窗口**只操作相机 launch 进程**（``runtime/camera.pid`` 记录的那一个），
不发任何机械臂指令、不启动夹爪、不碰 MoveIt。换档前后的相机重启由
``camera_mode.py`` 完成，它拒绝通配 kill，也不碰 ``cs625_bringup`` 的驱动栈。

用法（必须先在 source 过环境的 shell 里启动）：

    source scripts/source_dev_env.sh --full
    python3 scripts/camera_view.py

    # 也可以直接跑（不等图形界面就先测一条）：
    python3 scripts/camera_view.py --list-topics
"""

from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys
import time
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

REPO = pathlib.Path(__file__).resolve().parents[1]
CAMERA_MODE = REPO / "scripts" / "camera_mode.py"
SNAPSHOT_DIR = REPO / "runtime" / "camera_snapshots"

# 驱动话题优先；归一化话题（sensor_adapter 在跑时）作为后备。
TOPICS = {
    "color": ["/camera/color/image_raw", "/sensors/camera/color/image_raw"],
    "depth": ["/camera/depth/image_raw", "/sensors/camera/depth/image_raw"],
    "ir": ["/camera/left_ir/image_raw"],          # 需要 left_ir_enable:=true
}
KIND_LABEL = {"color": "彩色", "depth": "深度", "ir": "红外"}

C_BG = "#0b1015"
C_PANEL = "#121a22"
C_BORDER = "#1f2c39"
C_TEXT = "#dbe7f1"
C_MUTED = "#7d93a6"
C_ACCENT = "#2fd6c9"

# 深色主题 + 显式的按钮配色。之前只设了 QWidget 的文字颜色，按钮仍是系统浅色底，
# 浅色字落在浅色底上就看不清了；这里把按钮的底/边/悬停/按下/禁用全部写死。
def build_qss(scale: float = 1.0) -> str:
    """深色主题；所有字号/内边距按 ``scale`` 放大（4K 屏上默认界面太小）。"""
    def pt(value: float) -> str:
        return f"{value * scale:.1f}pt"

    def px(value: float) -> str:
        return f"{value * scale:.1f}px"

    return f"""
QWidget {{ background: {C_BG}; color: {C_TEXT}; }}
QMainWindow, QDialog {{ background: {C_BG}; }}
QPushButton {{
    background: #18232e; color: {C_TEXT};
    border: 1px solid #2f4256; border-radius: {px(4)};
    padding: {px(5)} {px(10)}; font-size: {pt(10)};
}}
QPushButton:hover {{ background: #22303d; border-color: {C_ACCENT}; }}
QPushButton:pressed {{ background: #101a23; }}
QPushButton:checked {{ background: #0f3a37; border-color: {C_ACCENT}; color: #b6fff8; }}
QPushButton:disabled {{ background: #111a22; color: #5b7186; border-color: {C_BORDER}; }}
QPlainTextEdit {{
    background: #0e1620; color: #a8bccd;
    border: 1px solid {C_BORDER}; font-size: {pt(10)};
}}
QScrollArea {{ border: 1px solid {C_BORDER}; background: #05080c; }}
QLabel {{ background: transparent; }}
QLabel#status {{ color: {C_MUTED}; font-size: {pt(10)}; }}
QLabel#status_live {{ color: {C_ACCENT}; font-size: {pt(10)}; font-weight: 600; }}
QLabel#footer {{ color: {C_MUTED}; font-size: {pt(9)}; }}
"""


APP_QSS = build_qss(1.0)


def detect_scale() -> float:
    """按屏幕逻辑分辨率猜一个界面缩放；4K 全屏（无系统缩放）会得到 2.0。"""
    screen = QtWidgets.QApplication.primaryScreen()
    if screen is None:
        return 1.0
    geo = screen.geometry()
    if geo.width() <= 0 or geo.height() <= 0:
        return 1.0
    return max(1.0, min(2.5, max(geo.width() / 1920.0, geo.height() / 1080.0)))


def pick_font_family() -> str:
    """挑一个能显示中文的字体；都没有时退回 DejaVu Sans。"""
    try:
        available = set(QtGui.QFontDatabase().families())
    except Exception:                           # noqa: BLE001
        return "DejaVu Sans"
    for name in ("Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei",
                 "WenQuanYi Zen Hei", "Microsoft YaHei", "DejaVu Sans"):
        if name in available:
            return name
    return "DejaVu Sans"


def convert_image(message) -> tuple[QtGui.QImage | None, dict]:
    """sensor_msgs/Image -> QImage + 状态信息（保持原始像素尺寸）。"""
    height, width, encoding = message.height, message.width, message.encoding
    info: dict = {"encoding": encoding, "width": width, "height": height}
    if encoding in ("bgr8", "rgb8"):
        pixels = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(height, width, 3)
        fmt = QtGui.QImage.Format_BGR888 if encoding == "bgr8" else QtGui.QImage.Format_RGB888
        stride = message.step or width * 3
        return QtGui.QImage(pixels.data, width, height, stride, fmt).copy(), info
    if encoding in ("mono8", "8UC1"):
        pixels = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(height, width)
        stride = message.step or width
        return QtGui.QImage(pixels.data, width, height, stride,
                            QtGui.QImage.Format_Grayscale8).copy(), info
    if encoding in ("16UC1", "mono16", "32FC1"):
        if encoding == "32FC1":
            metres = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(height, width)
            millimetres = np.where(np.isfinite(metres), metres * 1000.0, 0.0)
            unit = "m"
        else:
            millimetres = np.frombuffer(bytes(message.data), dtype=np.uint16).reshape(height, width)
            unit = "mm"
        valid = millimetres > 0.0
        info.update(unit=unit, valid=float(valid.mean()) if valid.size else 0.0,
                    median_mm=float(np.median(millimetres[valid])) if valid.any() else 0.0)
        # 0.2–2.0 m 映射到 0–255；0 是无效点
        level = np.clip((millimetres.astype(np.float32) - 200.0) * (255.0 / 1800.0), 0, 255)
        level = np.where(valid, level, 0.0).astype(np.uint8)
        return QtGui.QImage(level.data, width, height, width,
                            QtGui.QImage.Format_Grayscale8).copy(), info
    return None, info


class CameraWorker(QtCore.QThread):
    """订阅相机；只在**收到新帧**时发出，并在长时间无帧时重建订阅。

    为什么要重建订阅：相机换档会重启 launch，WSL + Fast DDS 下旧的订阅端点有时不会
    重新匹配（现象就是"画面停在最后一帧、状态一直显示等待图像"）。这里做一个看门狗：
    曾经收到过帧、又连续 8 s 没有新帧，就销毁节点重建一次订阅。
    """

    frame_ready = QtCore.pyqtSignal(str, object, dict)      # kind, QImage, info
    status = QtCore.pyqtSignal(str)

    RECREATE_AFTER_S = 8.0
    FIRST_FRAME_AFTER_S = 10.0

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop = False
        self._want = "color"
        self._arrived: dict[str, float] = {}
        self._frames: dict[str, deque] = {key: deque(maxlen=200) for key in
                                          (topic for topics in TOPICS.values() for topic in topics)}
        self._stamp: dict[str, int] = {}
        self._reliable = True          # camera_mode 启动驱动时用的是 reliable QoS
        self._rebuilds = 0
        self._emitted: dict[str, int] = {}
        self._last_frame_at = 0.0

    def set_kind(self, kind: str) -> None:
        self._want = kind if kind in TOPICS else "color"

    def stop(self) -> None:
        self._stop = True

    def _rate(self, topic: str) -> float:
        now = time.time()
        history = self._frames[topic]
        while history and now - history[0] > 5.0:
            history.popleft()
        if len(history) < 2:
            return 0.0
        span = max(1e-6, history[-1] - history[0])
        return (len(history) - 1) / span

    def _build_node(self, rclpy, Image, qos):
        node = rclpy.create_node("cs625_camera_view")
        latest: dict[str, object] = {}

        def make_callback(topic: str):
            def callback(message) -> None:
                key = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                if self._stamp.get(topic) == key:          # 同一帧重复投递
                    return
                self._stamp[topic] = key
                latest[topic] = message
                now = time.time()
                self._arrived[topic] = now
                self._frames[topic].append(now)
                self._last_frame_at = now
            return callback

        for topics in TOPICS.values():
            for topic in topics:
                node.create_subscription(Image, topic, make_callback(topic), qos)
        return node, latest

    def run(self) -> None:                     # noqa: C901 - one straightforward loop
        try:
            import rclpy
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from sensor_msgs.msg import Image
        except Exception as exc:                # noqa: BLE001
            self.status.emit(f"ROS 不可用：{type(exc).__name__}（先 source 环境）")
            return

        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init(args=None)
        # 1280x960 的彩色约 3.7 MiB、深度约 2.4 MiB，都大于 Fast DDS 默认共享内存段（~512 KiB）。
        # 用 BEST_EFFORT 订阅时这些大图会被静默丢掉 60-80%（项目里踩过的坑），
        # 表现为"偶尔来一帧、状态大多时候在等待"。所以默认用 RELIABLE，
        # 如果对端是 best-effort 发布（匹配不上、一直收不到），看门狗会切换到 best-effort 重试。
        def make_qos(reliable: bool):
            profile = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                 reliability=(ReliabilityPolicy.RELIABLE if reliable
                                              else ReliabilityPolicy.BEST_EFFORT),
                                 durability=DurabilityPolicy.VOLATILE)
            return profile

        qos = make_qos(self._reliable)
        node, latest = self._build_node(rclpy, Image, qos)
        self.status.emit(f"等待图像（{'reliable' if self._reliable else 'best_effort'} 订阅）")
        self.status.emit("等待图像（相机未启动 / RVS 占用 / 话题名不对都会是这一条）")

        last_status = 0.0
        started_at = time.time()
        try:
            while self._stop is False and rclpy.ok():
                now = time.time()
                # 看门狗：重建订阅能救两类问题——
                #   ① 相机换档/重启后旧端点不重连（收到过帧，随后一直没新帧）
                #   ② 一开始就匹配不上（QoS 不兼容），此时换另一种 QoS 再试
                stale = (self._last_frame_at and now - self._last_frame_at > self.RECREATE_AFTER_S)
                never = (not self._last_frame_at and now - started_at > self.FIRST_FRAME_AFTER_S)
                if stale or never:
                    self._rebuilds += 1
                    if not self._last_frame_at:
                        self._reliable = not self._reliable   # 从没收到过：换 QoS
                    node.destroy_node()
                    time.sleep(0.3)
                    qos = make_qos(self._reliable)
                    node, latest = self._build_node(rclpy, Image, qos)
                    self._last_frame_at = now if self._last_frame_at else 0.0
                    started_at = now
                    self._emitted.clear()
                    self.status.emit(
                        f"重新订阅相机（第 {self._rebuilds} 次，" 
                        f"{'reliable' if self._reliable else 'best_effort'}）…")

                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                candidates = TOPICS[self._want]
                live = [topic for topic in candidates
                        if topic in latest and now - self._arrived.get(topic, 0.0) < 3.0]
                if not live:
                    if now - last_status > 1.0:
                        last_status = now
                        first = candidates[0]
                        age = (f"已 {now - self._arrived[first]:.0f} s 无新帧"
                               if self._arrived.get(first) else "没有收到过数据")
                        self.status.emit(f"{KIND_LABEL[self._want]}：等待图像（{age}）")
                    continue
                topic = live[0]                 # 驱动话题优先，其次归一化话题
                if self._emitted.get(topic) == self._stamp.get(topic):
                    continue                    # 还没有新帧，别把同一帧反复画
                self._emitted[topic] = self._stamp.get(topic)
                image, info = convert_image(latest[topic])
                if image is None:
                    self.status.emit(f"{KIND_LABEL[self._want]}：不支持的编码 {info['encoding']}")
                    continue
                info["topic"] = topic
                info["fps"] = self._rate(topic)
                self.frame_ready.emit(self._want, image, info)
        except Exception as exc:                # noqa: BLE001
            self.status.emit(f"异常：{type(exc).__name__}: {exc}")
        finally:
            node.destroy_node()
            if owns_context and rclpy.ok():
                rclpy.shutdown()


class ModeRunner(QtCore.QThread):
    """后台跑 camera_mode.py <action>，输出逐行进日志。"""

    line = QtCore.pyqtSignal(str)
    finished_action = QtCore.pyqtSignal(str, int)

    def __init__(self, action: str, parent=None) -> None:
        super().__init__(parent)
        self._action = action

    def run(self) -> None:
        command = [sys.executable, str(CAMERA_MODE), self._action]
        self.line.emit(f"$ {' '.join(command)}")
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as exc:                # noqa: BLE001
            self.line.emit(f"启动失败：{exc}")
            self.finished_action.emit(self._action, -1)
            return
        assert process.stdout is not None
        for text in process.stdout:
            self.line.emit(text.rstrip())
        self.finished_action.emit(self._action, process.wait())


class Diagnostics(QtCore.QThread):
    """把"为什么看不到图像"一次性问清楚：节点、话题、发布者、域名、相机进程。"""

    line = QtCore.pyqtSignal(str)

    def run(self) -> None:
        self.line.emit("— 诊断开始 —")
        self.line.emit(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}   "
                       f"RMW={os.environ.get('RMW_IMPLEMENTATION', 'default')}")

        nodes = self._run(["ros2", "node", "list"])
        self.line.emit(f"节点 {len(nodes)} 个")
        for node in nodes[:12]:
            self.line.emit(f"  · {node}")
        if not nodes:
            self.line.emit("  ❌ 一个节点都没有：相机/驱动没起，或另一个终端用了不同的 "
                           "ROS_DOMAIN_ID。先在那个终端 ros2 topic list 对照。")

        topics = self._run(["ros2", "topic", "list"])
        camera_topics = [topic for topic in topics
                         if any(token in topic for token in ("camera", "sensors/camera"))]
        self.line.emit(f"话题 {len(topics)} 个，其中相机相关 {len(camera_topics)} 个")
        published = []
        for topic in camera_topics[:16]:
            count = self._publishers(topic)
            if count > 0:
                published.append(topic)
            note = "" if count > 0 else "（只有订阅端，比如本窗口自己）"
            self.line.emit(f"  · {topic}   发布者 {count}{note}")
        if not published:
            self.line.emit("  ❌ 没有任何相机话题有发布者：相机进程没起或已退出；"
                           "注意本窗口自己的订阅也会让话题出现在列表里。")
        for topic in (topic for group in TOPICS.values() for topic in group):
            if topic in camera_topics:
                continue
            self.line.emit(f"  · {topic}   没有这个 topic")

        ps = self._run(["pgrep", "-af", "percipio_camera"])
        self.line.emit("percipio_camera 进程：" + ("无" if not ps else ""))
        for pid_line in ps[:6]:
            self.line.emit(f"  · {pid_line}")
        self.line.emit("— 诊断结束 —")

    def _run(self, command: list[str]) -> list[str]:
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        except Exception as exc:                # noqa: BLE001
            self.line.emit(f"  {command[0]} 失败：{exc}")
            return []
        if result.returncode != 0:
            if command[0] == "pgrep" and result.returncode == 1:   # 没有匹配进程
                return []
            self.line.emit(f"  {command[0]} 退出码 {result.returncode}: "
                           f"{result.stderr.strip()[:160]}")
            return []
        return [line.strip() for line in result.stdout.splitlines() if line.strip()]

    def _publishers(self, topic: str) -> int:
        try:
            result = subprocess.run(["ros2", "topic", "info", topic],
                                    capture_output=True, text=True, timeout=15)
        except Exception:                       # noqa: BLE001
            return -1
        for line in result.stdout.splitlines():
            if "Publisher count" in line:
                try:
                    return int(line.split(":", 1)[1].strip())
                except (IndexError, ValueError):
                    return -1
        return -1


class ImageArea(QtWidgets.QWidget):
    """图像显示区：默认铺满，可切 1:1 原始像素。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._image: QtGui.QImage | None = None
        self._fit = True
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scroll.setStyleSheet(f"background:{C_BG}; border:1px solid {C_BORDER};")
        self.label = QtWidgets.QLabel("无图像")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setStyleSheet(f"color:{C_MUTED};")
        self.scroll.setWidget(self.label)
        layout.addWidget(self.scroll)

    def set_image(self, image: QtGui.QImage) -> None:
        self._image = image
        self._render()

    def set_fit(self, fit: bool) -> None:
        self._fit = fit
        self.scroll.setWidgetResizable(fit)
        policy = QtCore.Qt.ScrollBarAlwaysOff if fit else QtCore.Qt.ScrollBarAsNeeded
        self.scroll.setHorizontalScrollBarPolicy(policy)
        self.scroll.setVerticalScrollBarPolicy(policy)
        if not fit and self._image is not None:
            self.label.setFixedSize(self._image.size())
        self._render()

    def _render(self) -> None:
        if self._image is None:
            return
        if self._fit:
            target = self.scroll.viewport().size()
            pixmap = QtGui.QPixmap.fromImage(
                self._image.scaled(target, QtCore.Qt.KeepAspectRatio,
                                   QtCore.Qt.SmoothTransformation))
            self.label.setPixmap(pixmap)
        else:
            self.label.setFixedSize(self._image.size())
            self.label.setPixmap(QtGui.QPixmap.fromImage(self._image))

    def resizeEvent(self, event) -> None:      # noqa: N802
        super().resizeEvent(event)
        if self._fit:
            self._render()


class CameraWindow(QtWidgets.QMainWindow):
    def __init__(self, scale: float = 1.0) -> None:
        super().__init__()
        self._scale = max(0.8, min(2.5, float(scale)))
        # WSLg 的窗口装饰字体没有中文字形，中文标题会变成方块，所以标题用 ASCII。
        self.setWindowTitle("CS625 Camera - ROS view / mode switch")
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is not None:
            geo = screen.availableGeometry()
            self.resize(int(geo.width() * 0.72), int(geo.height() * 0.72))
        else:
            self.resize(int(960 * self._scale), int(760 * self._scale))
        self.setMinimumSize(int(760 * self._scale), int(600 * self._scale))
        self._worker: CameraWorker | None = None
        self._runner: ModeRunner | None = None
        self._diagnostics: Diagnostics | None = None
        self._kind = "color"
        self._last_frame = 0.0
        self._hinted = False

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        self.setCentralWidget(central)

        # --- 换档按钮 -----------------------------------------------------
        modes = QtWidgets.QHBoxLayout()
        modes.setSpacing(6)
        modes.addWidget(self._button("启动 · fast 640×480", lambda: self._run_mode("fast")))
        modes.addWidget(self._button("启动 · modeling 1280×960", lambda: self._run_mode("modeling")))
        modes.addWidget(self._button("启动 · high 2560×1440", lambda: self._run_mode("high")))
        modes.addWidget(self._button("停止", lambda: self._run_mode("stop")))
        modes.addWidget(self._button("状态", lambda: self._run_mode("status")))
        modes.addWidget(self._button("诊断", self._diagnose))
        self.mode_buttons = modes
        root.addLayout(modes)

        # --- 显示控制 -----------------------------------------------------
        display = QtWidgets.QHBoxLayout()
        display.setSpacing(6)
        self.kind_buttons = {}
        for kind in ("color", "depth", "ir"):
            button = self._button(KIND_LABEL[kind], lambda k=kind: self._set_kind(k),
                                  checkable=True)
            button.setChecked(kind == self._kind)
            self.kind_buttons[kind] = button
            display.addWidget(button)
        self.btn_fit = self._button("铺满", self._toggle_fit, checkable=True)
        self.btn_fit.setChecked(True)          # 默认铺满
        display.addWidget(self.btn_fit)
        display.addWidget(self._button("截图", self._snapshot))
        self.status_label = QtWidgets.QLabel("等待图像")
        self.status_label.setObjectName("status")
        display.addWidget(self.status_label, 1)
        root.addLayout(display)

        # --- 图像 ---------------------------------------------------------
        self.image_area = ImageArea()
        root.addWidget(self.image_area, 1)

        # --- 日志 ---------------------------------------------------------
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(int(round(150 * self._scale)))
        self.log.setStyleSheet(f"background:{C_PANEL}; color:{C_MUTED}; "
                               f"border:1px solid {C_BORDER}; font-size:8.5pt;")
        root.addWidget(self.log)

        footer = QtWidgets.QLabel(
            "只操作相机 launch 进程（runtime/camera.pid）；不发机械臂指令、不启动夹爪、不碰 MoveIt。"
            "  换档时相机会被重启，图像会短暂中断。")
        footer.setObjectName("footer")
        root.addWidget(footer)

        self.log.appendPlainText("就绪：选一档启动，或点『诊断』看为什么没有图像。")
        self._start_worker()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._check_frames)
        self._timer.start(3000)

    # ------------------------------------------------------------------
    def _button(self, text, slot, checkable=False):
        button = QtWidgets.QPushButton(text)
        button.setFixedHeight(int(round(30 * self._scale)))
        button.setCheckable(checkable)
        button.clicked.connect(slot)
        return button

    def _start_worker(self) -> None:
        if self._worker is not None:
            return
        self._worker = CameraWorker()
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.status.connect(self.status_label.setText)
        self._worker.start()

    def _set_kind(self, kind: str) -> None:
        self._kind = kind
        for key, button in self.kind_buttons.items():
            button.setChecked(key == kind)
        if self._worker is not None:
            self._worker.set_kind(kind)
        # 立刻给个反馈：深度/红外换过去到第一帧最长要等 1–2 s，不给提示会以为按钮没反应
        self.status_label.setObjectName("status")
        self.status_label.style().unpolish(self.status_label) if self.status_label.style() else None
        self.status_label.style().polish(self.status_label) if self.status_label.style() else None
        self.status_label.setText(f"{KIND_LABEL[kind]}：切换到该路，等待第一帧…")

    def _toggle_fit(self) -> None:
        # Qt 已经替我们翻转了 checked，这里按翻转后的状态处理
        fit = self.btn_fit.isChecked()
        self.btn_fit.setText("铺满" if fit else "1:1")
        self.image_area.set_fit(fit)

    def _on_frame(self, kind: str, image: QtGui.QImage, info: dict) -> None:
        if kind != self._kind:
            return
        self._last_frame = time.time()
        self._hinted = False
        if self.status_label.objectName() != "status_live":
            self.status_label.setObjectName("status_live")
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)
        self.image_area.set_image(image)
        parts = [f"{image.width()}×{image.height()}", info.get("encoding", "?")]
        fps = info.get("fps", 0.0)
        parts.append(f"{fps:.1f} fps" if fps > 0 else "fps 未测出")
        if "valid" in info:
            parts.append(f"有效点 {info['valid'] * 100:.1f}%")
            if info.get("median_mm"):
                parts.append(f"中位深度 {info['median_mm'] / 1000.0:.3f} m")
        parts.append(info.get("topic", ""))
        self.status_label.setText(" · ".join(str(p) for p in parts if p))

    def _snapshot(self) -> None:
        image = self.image_area._image
        if image is None:
            return
        SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
        path = SNAPSHOT_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{self._kind}.png"
        image.save(str(path))
        self.log.appendPlainText(f"截图 -> {path.relative_to(REPO)}")

    def _diagnose(self) -> None:
        """后台跑一次 ROS 体检，结果进日志；不阻塞界面。"""
        if self._diagnostics is not None and self._diagnostics.isRunning():
            self.log.appendPlainText("诊断还在跑，等它结束")
            return
        self._diagnostics = Diagnostics()
        self._diagnostics.line.connect(self.log.appendPlainText)
        self._diagnostics.start()

    def _check_frames(self) -> None:
        """超过 10 s 没有图像时提示一次，并说明常见原因。"""
        if self._hinted:
            return
        age = time.time() - self._last_frame if self._last_frame else 9999.0
        if age < 10.0:
            return
        self._hinted = True
        self.log.appendPlainText(
            "10 秒没有图像。常见原因：① 相机没起；② 另一个终端 ROS_DOMAIN_ID 不同；"
            "③ RVS 占着相机（Windows 端）；④ 话题名不是 /camera/*。点『诊断』看细节。")

    def _run_mode(self, action: str) -> None:
        if self._runner is not None and self._runner.isRunning():
            self.log.appendPlainText("上一档还在切换中，等它结束")
            return
        self._set_buttons_enabled(False)
        self._runner = ModeRunner(action)
        self._runner.line.connect(self.log.appendPlainText)
        self._runner.finished_action.connect(self._on_mode_finished)
        self._runner.start()

    def _on_mode_finished(self, action: str, code: int) -> None:
        self.log.appendPlainText(f"[{action}] 结束，退出码 {code}")
        self._set_buttons_enabled(True)

    def _set_buttons_enabled(self, enabled: bool) -> None:
        for index in range(self.mode_buttons.count()):
            widget = self.mode_buttons.itemAt(index).widget()
            if widget is not None:
                widget.setEnabled(enabled)

    def shutdown(self) -> None:
        """Stop the ROS thread and any running mode switch before the process exits.

        A QThread that is still running when Python tears down aborts the process
        ("terminate called without an active exception"), so this is wired to both
        closeEvent and aboutToQuit.
        """
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(2000)
            self._worker = None
        if self._runner is not None and self._runner.isRunning():
            self._runner.wait(2000)
        if self._diagnostics is not None and self._diagnostics.isRunning():
            self._diagnostics.wait(3000)

    def closeEvent(self, event) -> None:       # noqa: N802
        self.shutdown()
        super().closeEvent(event)


def list_topics() -> int:
    """不启动界面，先确认相机话题在不在。"""
    result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True)
    if result.returncode != 0:
        print("ros2 topic list 失败：先 source scripts/source_dev_env.sh --full", file=sys.stderr)
        return 1
    topics = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    wanted = [topic for group in TOPICS.values() for topic in group]
    print("相机相关话题：")
    for topic in wanted:
        print(f"  {'✅' if topic in topics else '  '} {topic}")
    missing = [topic for topic in wanted if topic not in topics]
    if missing:
        print("\n缺失的话题可能只是没开（红外要 left_ir_enable:=true）。")
    return 0 if len(missing) < len(wanted) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list-topics", action="store_true",
                        help="只检查相机话题，不打开窗口")
    parser.add_argument("--ui-scale", default="auto",
                        help="界面缩放：auto（按屏幕分辨率）或数字，例如 1.0 / 1.5 / 2.0")
    args = parser.parse_args()
    if args.list_topics:
        return list_topics()

    # 高 DPI 必须在 QApplication 之前设置，否则 4K 屏上是位图放大、字发虚
    for attribute in ("AA_EnableHighDpiScaling", "AA_UseHighDpiPixmaps"):
        flag = getattr(QtCore.Qt, attribute, None)
        if flag is not None:
            QtWidgets.QApplication.setAttribute(flag, True)

    app = QtWidgets.QApplication(sys.argv)
    try:
        scale = detect_scale() if args.ui_scale == "auto" else float(args.ui_scale)
    except ValueError:
        scale = 1.0
    scale = max(0.8, min(2.5, scale))
    app.setFont(QtGui.QFont(pick_font_family(), int(round(10 * scale))))
    app.setStyleSheet(build_qss(scale))
    window = CameraWindow(scale=scale)
    app.aboutToQuit.connect(window.shutdown)

    # Qt 的事件循环在 C++ 里跑，Python 信号默认得不到处理（kill 不生效，只能 kill -9）。
    # 一个 250 ms 的空定时器让 Python 有机会执行信号处理器，从而实现干净的退出。
    import signal
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: app.quit())
    heartbeat = QtCore.QTimer()
    heartbeat.timeout.connect(lambda: None)
    heartbeat.start(250)

    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
