"""离线测试：面板相机图像转换（2026-11-19）。

真机问题：面板相机刷新不到 0.5 fps，而相机本身 0.79 fps。两个原因：
  ① 深度图用纯 Python 逐像素 setPixel（1280x960 一次约 0.4 s）；
  ② 主循环每 0.1 s 无条件重新转换**同一帧**，线程永远在转旧帧。
这里测转换的正确性与速度，防止回退。
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import time

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def panel_camera():
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    # panel_camera 直接 import 同目录的 panel_status，需要把 scripts 放进 sys.path
    if str(ROOT / "scripts") not in sys.path:
        sys.path.insert(0, str(ROOT / "scripts"))
    spec = importlib.util.spec_from_file_location(
        "panel_camera", ROOT / "scripts" / "panel_camera.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Message:
    def __init__(self, width, height, encoding, data, step=None):
        self.width = width
        self.height = height
        self.encoding = encoding
        self.data = data
        self.step = step if step is not None else width * len(data) // (width * height)


def depth_message(width=320, height=240):
    rng = np.random.default_rng(7)
    millimetres = rng.integers(700, 900, size=(height, width), dtype=np.uint16)
    millimetres[::17, ::13] = 0
    return Message(width, height, "16UC1", millimetres.tobytes(), step=width * 2)


def test_depth_is_converted_and_invalid_pixels_stay_black(panel_camera):
    message = depth_message()
    image = panel_camera.CameraWorker._to_qimage(message)
    assert image is not None
    assert (image.width(), image.height()) == (320, 240)
    # 无效深度（0）应映射为黑
    assert image.pixelColor(0, 0).red() == 0


def test_depth_conversion_is_fast_enough_for_the_camera(panel_camera):
    """相机 0.79 fps → 每帧预算约 1.2 s；转换必须远小于它（旧实现约 0.4 s/1280x960）。"""
    message = depth_message(1280, 960)
    started = time.perf_counter()
    panel_camera.CameraWorker._to_qimage(message)
    elapsed = time.perf_counter() - started
    assert elapsed < 0.15, f"深度转换 {elapsed:.3f} s，太慢（会拖垮刷新率）"


def test_color_is_converted(panel_camera):
    width, height = 64, 48
    data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    image = panel_camera.CameraWorker._to_qimage(
        Message(width, height, "bgr8", data, step=width * 3))
    assert image is not None and (image.width(), image.height()) == (width, height)


def test_unknown_encoding_returns_none(panel_camera):
    assert panel_camera.CameraWorker._to_qimage(
        Message(4, 4, "32FC1", b"\x00" * 64, step=16)) is None
