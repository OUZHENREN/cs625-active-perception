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

from panel_status import SPIN_LOCK

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
            # 同进程里 status/telemetry 线程也可能先初始化过 rclpy，重复 init 会抛
            # "Context.init() must only be called once"，所以只在未初始化时初始化。
            self._owns_context = not rclpy.ok()
            if self._owns_context:
                rclpy.init(args=None)
            # 面板里有两个相机视图（安全页与效率页），各自一个节点。
            # 以前都叫 panel_camera，第二个会触发
            # "Publisher already registered for node name" 警告。
            suffix = f"_{self._node_suffix}" if self._node_suffix else ""
            node = rclpy.create_node(f"panel_camera{suffix}")
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            latest: dict[str, object] = {}

            versions: dict[str, int] = {"color": 0, "depth": 0}

            def on_color(msg) -> None:
                latest["color"] = msg
                self._last["color"] = time.time()
                versions["color"] += 1

            def on_depth(msg) -> None:
                latest["depth"] = msg
                self._last["depth"] = time.time()
                versions["depth"] += 1

            # launch_sensor_adapter:=false 时 /sensors/camera/* 不存在（归一化话题没发布），
            # 所以同时订阅相机原始话题——否则面板永远是"等待图像"。
            # 2026-11-25：COLOR_TOPIC 本身就是 /camera/color/image_raw，旧写法把同一个
            # 话题订阅了两次；这里用 fromkeys 去重，并把归一化话题排在前面。
            candidates = {
                "color": ["/sensors/camera/color/image", COLOR_TOPIC,
                          "/camera/color/image_raw"],
                "depth": ["/sensors/camera/depth/image", DEPTH_TOPIC,
                          "/camera/depth/image_raw"],
            }
            subscribed: list[str] = []
            for kind, names in candidates.items():
                for topic in dict.fromkeys(names):
                    node.create_subscription(
                        Image, topic,
                        on_color if kind == "color" else on_depth, qos)
                    subscribed.append(topic)
            self.status = "等待图像"
            last_version: dict[str, int] = {}
            reemit = 0.0
            started_at = time.time()
            warned = False
            while not self._stop and rclpy.ok():
                with SPIN_LOCK:
                    rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                # 2026-11-25：面板显示"无图像"时最难判断是"没订阅上"还是"没发布"。
                # 15 s 没有任何帧就打印一次诊断，把每条候选话题的发布者数量列出来。
                if (not latest) and not warned and now - started_at > 15.0:
                    warned = True
                    detail = ", ".join(
                        f"{t}={node.count_publishers(t)} 个发布者" for t in subscribed)
                    print(f"[panel_camera{'_' + self._node_suffix if self._node_suffix else ''}]"
                          f" 15 s 未收到图像：{detail}")
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
                # 2026-11-19 修：以前每个 0.1 s 都重新转换**同一帧**，深度图纯 Python
                # 转换一次要 1–3 s，于是线程永远在转旧帧，界面只能到 0.2–0.5 fps。
                # 现在只有真的收到新帧才转换。
                if versions.get(source, 0) == last_version.get(source, -1):
                    continue
                last_version[source] = versions[source]
                image = self._to_qimage(message)
                if image is not None:
                    self.status = f"{source} {image.width()}×{image.height()}"
                    self.image_ready.emit(image, source)
            node.destroy_node()
            if self._owns_context and rclpy.ok():
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
            # 2026-11-19 修：原来是纯 Python 逐像素 setPixel（1280x960 隔行仍要 61 万次
            # Qt 调用，一次 1–3 s），比 ROS 端查看器慢两个数量级。改成 numpy 向量化，
            # 与 scripts/camera_view.py 的做法一致。
            import numpy as np
            stride = message.step or width * 2
            raw = np.frombuffer(data, dtype=np.uint8)
            if stride == width * 2:
                millimetres = raw.view(np.uint16).reshape(height, width)
            else:                                 # 行有 padding：按 stride 逐行取
                millimetres = np.frombuffer(data, dtype=np.uint8).reshape(height, stride)
                millimetres = millimetres[:, :width * 2].copy().view(np.uint16)
            low, high = 200, 2000
            span = max(1, high - low)
            scaled = ((millimetres.astype(np.int32) - low) * 255 // span)
            scaled = np.clip(scaled, 0, 255).astype(np.uint8)
            scaled[millimetres == 0] = 0          # 无效深度保持全黑
            rgb = np.repeat(scaled[:, :, None], 3, axis=2)
            result = QtGui.QImage(rgb.data, width, height, width * 3,
                                  QtGui.QImage.Format_RGB888)
            return result.copy()
        if encoding in ("mono8", "8UC1"):
            stride = message.step or width
            image = QtGui.QImage(data, width, height, stride, QtGui.QImage.Format_Grayscale8)
            return image.copy()
        return None


class CameraView(QtWidgets.QWidget):
    """1:1 显示相机图像（不缩放、不滚动），窗口与分割器按图像尺寸腾地方。"""

    image_size = QtCore.pyqtSignal(int, int)      # 真实像素尺寸变化时发出

    def __init__(self, probe: bool = True, parent=None, fit: bool = False,
                 node_suffix: str = "") -> None:
        super().__init__(parent)
        self._node_suffix = node_suffix
        self.setStyleSheet(f"background:{C_BG}; border:1px solid {C_BORDER};")
        # fit=True：等比缩放（保持纵横比，只缩小不放大），供效率页使用；
        # fit=False：1:1 原始像素，安全页沿用。
        self._fit = bool(fit)
        self._image: QtGui.QImage | None = None
        self._source = "color"
        self._render_scheduled = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(6)
        self.label_status = QtWidgets.QLabel("相机未启动")
        self.label_status.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        self.btn_color = QtWidgets.QPushButton("彩色")
        self.btn_depth = QtWidgets.QPushButton("深度")
        self.btn_fit = QtWidgets.QPushButton("铺满")
        for button in (self.btn_color, self.btn_depth, self.btn_fit):
            button.setFixedHeight(24)
            button.setCheckable(True)
            bar.addWidget(button, 0)
        self.btn_color.setChecked(True)
        self.btn_fit.setChecked(False)
        bar.addWidget(self.label_status, 1)
        layout.addLayout(bar)

        self.canvas = QtWidgets.QScrollArea()
        self.canvas.setWidgetResizable(False)
        self.canvas.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.canvas.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.inner = QtWidgets.QLabel("无图像（相机未启动或 RVS 占用）")
        self.inner.setStyleSheet(f"color:{C_MUTED}; font-size:9pt;")
        self.inner.setAlignment(QtCore.Qt.AlignCenter)
        self.canvas.setWidget(self.inner)
        layout.addWidget(self.canvas, 1)

        self.btn_color.clicked.connect(lambda: self._set_source("color"))
        self.btn_depth.clicked.connect(lambda: self._set_source("depth"))
        self.btn_fit.clicked.connect(self._toggle_fit)
        self.canvas.viewport().installEventFilter(self)
        self._apply_fit_mode()

        self._worker: CameraWorker | None = None
        if probe:
            self.start()
        else:
            self._on_image(self._placeholder(), "color")

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

    @staticmethod
    def _placeholder() -> QtGui.QImage:
        """无相机时画一张 1280×960 网格：用于离线确认"1:1 不缩放"。"""
        image = QtGui.QImage(1280, 960, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("#11161c"))
        painter = QtGui.QPainter(image)
        painter.setPen(QtGui.QPen(QtGui.QColor("#243444"), 1))
        for x in range(0, 1280, 40):
            painter.drawLine(x, 0, x, 960)
        for y in range(0, 960, 40):
            painter.drawLine(0, y, 1280, y)
        painter.setPen(QtGui.QPen(QtGui.QColor("#2fd6c9"), 2))
        painter.drawRect(0, 0, 1279, 959)
        painter.setPen(QtGui.QColor("#7d93a6"))
        painter.drawText(12, 24, "占位图 1280×960（相机未启动，用于验证 1:1 不缩放）")
        painter.end()
        return image

    def _set_source(self, source: str) -> None:
        self._source = source
        self.btn_color.setChecked(source == "color")
        self.btn_depth.setChecked(source == "depth")
        if self._worker is not None:
            self._worker.set_source(source)

    @property
    def is_fit(self) -> bool:
        return self._fit

    def _toggle_fit(self, checked: bool | None = None) -> None:
        self._fit = bool(checked) if checked is not None else not self._fit
        self._apply_fit_mode()

    def _apply_fit_mode(self) -> None:
        """按当前模式设置画布尺寸策略 / 滚动条 / 控件最小尺寸。"""
        self.btn_fit.setChecked(self._fit)
        if self._fit:
            # 等比适应：画布跟随视口，图片在 _render 里按 KeepAspectRatio 缩放。
            self.canvas.setWidgetResizable(True)
            self.canvas.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.canvas.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            # 1:1 模式的 setFixedSize 会锁住画布尺寸，切回适应模式必须先解除。
            self.canvas.setMinimumSize(0, 0)
            self.canvas.setMaximumSize(16777215, 16777215)
            self.canvas.updateGeometry()
            self._schedule_render()
            # setFixedSize 会同时锁住 min/max，切回适应模式时必须解除。
            self.inner.setMinimumSize(0, 0)
            self.inner.setMaximumSize(16777215, 16777215)
            self.inner.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                     QtWidgets.QSizePolicy.Ignored)
            self.setMinimumSize(240, 200)
        else:
            # 1:1 原始像素：大图允许滚动查看，而不是被静默裁掉。
            self.canvas.setWidgetResizable(False)
            self.canvas.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
            self.canvas.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
            self.inner.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                     QtWidgets.QSizePolicy.Preferred)
            self.setMinimumSize(0, 0)
            if self._image is not None:
                self._set_native_size(self._image)
        self._render()

    def _set_native_size(self, image: QtGui.QImage) -> None:
        """1:1 模式：画布本身就是图像大小；控件最小尺寸给个上限。

        不再把 CameraView 的最小宽高锁成整幅图像：否则 2560×1920 会撑爆
        1314 px 宽的相机列（子控件比父控件还大，滚动条也救不了）。现在大图
        由 QScrollArea 的 AsNeeded 滚动条平移查看，小图仍按原生像素显示。
        """
        self.canvas.setFixedSize(image.width() + 2, image.height() + 2)
        self.inner.setFixedSize(image.width(), image.height())
        self.setMinimumWidth(min(image.width() + 24, 680))
        self.setMinimumHeight(min(image.height() + 56, 520))

    def _schedule_render(self) -> None:
        """布局完成后按新的视口尺寸再画一次（fit 模式专用）。"""
        if not self._fit or self._render_scheduled:
            return
        self._render_scheduled = True
        QtCore.QTimer.singleShot(0, self._render_now)

    def _render_now(self) -> None:
        self._render_scheduled = False
        self._render()

    def _on_image(self, image: QtGui.QImage, source: str) -> None:
        self._image = image
        if not self._fit:
            self._set_native_size(image)
        self.image_size.emit(image.width(), image.height())
        self.label_status.setText(
            f"{source} · {image.width()}×{image.height()} · "
            f"{'等比适应' if self._fit else '1:1'}")
        self._render()
        self._schedule_render()

    def _render(self) -> None:
        if self._image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._image)
        if self._fit:
            target = self.canvas.viewport().size()
            # 只缩小、不放大：小图保持原始像素（640 档不被拉糊），
            # 大图按视口等比缩小（2560 档不再被裁掉一半）。
            if target.width() > 4 and target.height() > 4 and (
                    pixmap.width() > target.width()
                    or pixmap.height() > target.height()):
                pixmap = pixmap.scaled(target, QtCore.Qt.KeepAspectRatio,
                                       QtCore.Qt.SmoothTransformation)
            self.inner.setPixmap(pixmap)
            self.label_status.setText(
                f"{self._source} · {self._image.width()}×{self._image.height()} · "
                f"等比适应（保持纵横比，只缩小不放大）")
            return
        self.inner.setPixmap(pixmap)      # 1:1，不缩放
        need_w = pixmap.width() + 1000
        room = self.window().width() if self.window() is not None else 0
        hint = "" if room >= need_w else f" · 需窗口 ≥{need_w} px 宽"
        self.label_status.setText(
            f"{self._source} · {pixmap.width()}×{pixmap.height()} · 1:1（不缩放、可滚动）{hint}")

    def eventFilter(self, obj, event) -> bool:    # noqa: N802
        """视口尺寸变化时在 fit 模式重新等比缩放（QLabel 不会自己缩放 pixmap）。"""
        if (obj is self.canvas.viewport()
                and event.type() == QtCore.QEvent.Resize
                and self._fit):
            self._schedule_render()
        return super().eventFilter(obj, event)

    def resizeEvent(self, event) -> None:         # noqa: N802
        super().resizeEvent(event)
        if self._fit:
            self._schedule_render()
