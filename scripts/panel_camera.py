#!/usr/bin/env python3
"""面板右侧的相机预览：彩色与深度都按**原始分辨率 1:1** 显示（不缩放）。

为什么独立成模块：
  · 相机图像的像素密度决定了调试时能不能看清边缘/深度台阶，任何缩放都会让人误判；
  · 订阅必须 best-effort（相机发布端就是 best effort），用 rclpy 单独一个线程，UI 不被拖慢；
  · 与 RVS 互斥：RVS 占着相机时这里会一直显示"无数据"，这是正常现象。

用法（面板内部）：
    view = CameraView(probe=True)      # 开始订阅
    view.set_source("color" | "depth")
"""

from __future__ import annotations

import time

from PyQt5 import QtCore, QtGui, QtWidgets

COLOR_TOPIC = "/sensors/camera/color/image_raw"
DEPTH_TOPIC = "/sensors/camera/depth/image_raw"

C_BG = "#0b1015"
C_BORDER = "#1f2c39"
C_MUTED = "#7d93a6"
C_TEXT = "#dbe7f1"


class CameraWorker(QtCore.QThread):
    """订阅彩色与深度图，发出原始尺寸的 QImage。"""

    image_ready = QtCore.pyqtSignal(object, str)      # QImage, source

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop = False
        self._source = "color"
        self._last = {"color": 0.0, "depth": 0.0}
        self.status = "未启动"

    def set_source(self, source: str) -> None:
        self._source = source if source in ("color", "depth") else "color"

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:                      # noqa: C901 - simple loop
        try:
            import rclpy
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   HistoryPolicy, DurabilityPolicy)
            from sensor_msgs.msg import Image
        except Exception as exc:                # noqa: BLE001
            self.status = f"ROS 不可用：{type(exc).__name__}"
            return

        try:
            rclpy.init(args=None)
            node = rclpy.create_node("panel_camera")
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            latest: dict[str, object] = {}

            def on_color(msg) -> None:
                latest["color"] = msg
                self._last["color"] = time.time()

            def on_depth(msg) -> None:
                latest["depth"] = msg
                self._last["depth"] = time.time()

            node.create_subscription(Image, COLOR_TOPIC, on_color, qos)
            node.create_subscription(Image, DEPTH_TOPIC, on_depth, qos)
            self.status = "等待图像"
            reemit = 0.0
            while not self._stop and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                if now - reemit < 0.1:           # 最多 10 Hz 刷新界面，图像不缩放
                    continue
                reemit = now
                # 当前选中的源优先；若它长时间没数据就退回另一个（例如只有彩色模式）
                source = self._source
                if source not in latest or now - self._last[source] > 1.5:
                    other = "depth" if source == "color" else "color"
                    source = other if other in latest and now - self._last[other] <= 1.5 else source
                message = latest.get(source)
                if message is None:
                    continue
                image = self._to_qimage(message)
                if image is not None:
                    self.status = f"{source} {image.width()}×{image.height()}"
                    self.image_ready.emit(image, source)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:                # noqa: BLE001
            self.status = f"异常：{type(exc).__name__}: {exc}"

    @staticmethod
    def _to_qimage(message) -> QtGui.QImage | None:
        """把 sensor_msgs/Image 转成 QImage，保持原始像素尺寸。"""
        height, width, encoding = message.height, message.width, message.encoding
        data = bytes(message.data)
        if encoding in ("rgb8", "bgr8"):
            fmt = QtGui.QImage.Format_RGB888 if encoding == "rgb8" else QtGui.QImage.Format_BGR888
            stride = message.step or width * 3
            image = QtGui.QImage(data, width, height, stride, fmt)
            return image.copy()
        if encoding in ("mono16", "16UC1"):      # 深度：毫米，按 0.2–2 m 映射到灰度
            stride = message.step or width * 2
            image = QtGui.QImage(data, width, height, stride, QtGui.QImage.Format_Grayscale16)
            image = image.copy()
            low, high = 200, 2000
            result = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
            for y in range(0, height, 2):        # 隔行处理，避免大图拖慢
                line_in = image.constScanLine(y).asstring(width * 2)
                values = memoryview(line_in).cast("H")
                for x in range(width):
                    depth = values[x]
                    level = 0 if depth == 0 else max(0, min(255, int(
                        (depth - low) * 255 / max(1, high - low))))
                    result.setPixel(x, y, QtGui.qRgb(level, level, level))
            return result
        if encoding in ("mono8", "8UC1"):
            stride = message.step or width
            image = QtGui.QImage(data, width, height, stride, QtGui.QImage.Format_Grayscale8)
            return image.copy()
        return None


class CameraView(QtWidgets.QWidget):
    """1:1 显示相机图像（不缩放），带"适应窗口"开关与状态行。"""

    def __init__(self, probe: bool = True, parent=None) -> None:
        super().__init__(parent)
        self.setStyleSheet(f"background:{C_BG}; border:1px solid {C_BORDER};")
        self._fit = False
        self._image: QtGui.QImage | None = None
        self._source = "color"

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(6)
        self.label_status = QtWidgets.QLabel("相机未启动")
        self.label_status.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        self.btn_color = QtWidgets.QPushButton("彩色")
        self.btn_depth = QtWidgets.QPushButton("深度")
        self.btn_fit = QtWidgets.QPushButton("适应窗口")
        for button in (self.btn_color, self.btn_depth, self.btn_fit):
            button.setFixedHeight(24)
            button.setCheckable(True)
            bar.addWidget(button, 0)
        self.btn_color.setChecked(True)
        bar.addWidget(self.label_status, 1)
        layout.addLayout(bar)

        self.canvas = QtWidgets.QScrollArea()
        self.canvas.setWidgetResizable(False)
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.inner = QtWidgets.QLabel("无图像（相机未启动或 RVS 占用）")
        self.inner.setStyleSheet(f"color:{C_MUTED}; font-size:9pt;")
        self.inner.setAlignment(QtCore.Qt.AlignCenter)
        self.canvas.setWidget(self.inner)
        layout.addWidget(self.canvas, 1)

        self.btn_color.clicked.connect(lambda: self._set_source("color"))
        self.btn_depth.clicked.connect(lambda: self._set_source("depth"))
        self.btn_fit.clicked.connect(self._toggle_fit)

        self._worker: CameraWorker | None = None
        if probe:
            self.start()

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._worker is not None:
            return
        self._worker = CameraWorker(parent=self)
        self._worker.image_ready.connect(self._on_image)
        self._worker.start()

    def stop(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(1500)
            self._worker = None

    def _set_source(self, source: str) -> None:
        self._source = source
        self.btn_color.setChecked(source == "color")
        self.btn_depth.setChecked(source == "depth")
        if self._worker is not None:
            self._worker.set_source(source)

    def _toggle_fit(self) -> None:
        self._fit = not self._fit
        self.btn_fit.setChecked(self._fit)
        self.canvas.setWidgetResizable(self._fit)
        self._render()

    def _on_image(self, image: QtGui.QImage, source: str) -> None:
        self._image = image
        self.label_status.setText(f"{source} · {image.width()}×{image.height()} · "
                                  f"{'适应窗口' if self._fit else '1:1'}")
        self._render()

    def _render(self) -> None:
        if self._image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._image)
        if self._fit:
            self.inner.setPixmap(pixmap.scaled(self.canvas.viewport().size(),
                                               QtCore.Qt.KeepAspectRatio,
                                               QtCore.Qt.SmoothTransformation))
        else:
            self.inner.setPixmap(pixmap)          # 原始像素，1:1
        self.inner.resize(pixmap.size() if not self._fit else self.inner.sizeHint())

    def resizeEvent(self, event) -> None:         # noqa: N802
        super().resizeEvent(event)
        if self._fit:
            self._render()
