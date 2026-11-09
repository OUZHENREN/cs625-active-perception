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
    """订阅相机；按界面选中的类型发出最新一帧。"""

    frame_ready = QtCore.pyqtSignal(str, object, dict)      # kind, QImage, info
    status = QtCore.pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop = False
        self._want = "color"
        self._arrived: dict[str, float] = {}
        self._frames: dict[str, deque] = {key: deque(maxlen=120) for key in
                                          (topic for topics in TOPICS.values() for topic in topics)}

    def set_kind(self, kind: str) -> None:
        self._want = kind if kind in TOPICS else "color"

    def stop(self) -> None:
        self._stop = True

    def _rate(self, topic: str) -> float:
        now = time.time()
        history = self._frames[topic]
        while history and now - history[0] > 3.0:
            history.popleft()
        if len(history) < 2:
            return 0.0
        span = max(1e-6, history[-1] - history[0])
        return (len(history) - 1) / span

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
        node = rclpy.create_node("cs625_camera_view")
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        latest: dict[str, object] = {}

        def make_callback(topic: str):
            def callback(message) -> None:
                latest[topic] = message
                self._arrived[topic] = time.time()
                self._frames[topic].append(time.time())
            return callback

        for topics in TOPICS.values():
            for topic in topics:
                node.create_subscription(Image, topic, make_callback(topic), qos)
        self.status.emit("等待图像（相机未启动 / RVS 占用 / 话题名不对都会是这一条）")

        last_emit = 0.0
        try:
            while self._stop is False and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                if now - last_emit < 0.1:       # 界面最多 10 Hz，不缩放图像
                    continue
                last_emit = now
                candidates = TOPICS[self._want]
                live = [topic for topic in candidates
                        if topic in latest and now - self._arrived.get(topic, 0.0) < 2.0]
                if not live:
                    self.status.emit(f"{KIND_LABEL[self._want]}：等待图像")
                    continue
                topic = live[0]                 # 驱动话题优先，其次归一化话题
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
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("CS625 相机 · ROS 端查看 / 换档")
        self.resize(960, 760)
        self._worker: CameraWorker | None = None
        self._runner: ModeRunner | None = None
        self._kind = "color"

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
        self.btn_fit = self._button("1:1", self._toggle_fit, checkable=True)
        display.addWidget(self.btn_fit)
        display.addWidget(self._button("截图", self._snapshot))
        self.status_label = QtWidgets.QLabel("等待图像")
        self.status_label.setStyleSheet(f"color:{C_MUTED}; font-size:9pt;")
        display.addWidget(self.status_label, 1)
        root.addLayout(display)

        # --- 图像 ---------------------------------------------------------
        self.image_area = ImageArea()
        root.addWidget(self.image_area, 1)

        # --- 日志 ---------------------------------------------------------
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        self.log.setStyleSheet(f"background:{C_PANEL}; color:{C_MUTED}; "
                               f"border:1px solid {C_BORDER}; font-size:8.5pt;")
        root.addWidget(self.log)

        footer = QtWidgets.QLabel(
            "只操作相机 launch 进程（runtime/camera.pid）；不发机械臂指令、不启动夹爪、不碰 MoveIt。"
            "  换档时相机会被重启，图像会短暂中断。")
        footer.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        root.addWidget(footer)

        self._start_worker()

    # ------------------------------------------------------------------
    def _button(self, text, slot, checkable=False):
        button = QtWidgets.QPushButton(text)
        button.setFixedHeight(28)
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

    def _toggle_fit(self) -> None:
        fit = not self.btn_fit.isChecked()
        self.btn_fit.setChecked(fit)
        self.btn_fit.setText("铺满" if fit else "1:1")
        self.image_area.set_fit(fit)

    def _on_frame(self, kind: str, image: QtGui.QImage, info: dict) -> None:
        if kind != self._kind:
            return
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
    args = parser.parse_args()
    if args.list_topics:
        return list_topics()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(f"QMainWindow{{background:{C_BG};}} QWidget{{color:{C_TEXT};}}")
    window = CameraWindow()
    app.aboutToQuit.connect(window.shutdown)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
