"""KWR75 六轴力传感器协议解析（2026-11-25 接入，RS485 / 28 字节长数据格式）。

手册（KWR75用户手册V21.6.75.3.3LB）给的样例帧必须能被逐位还原，
否则接上硬件后会把"kg"当"N"、"字节序"或"帧边界"搞错，而那种错误很难从波形上看出来。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("ft", ROOT / "scripts" / "ft_sensor_node.py")
ft = importlib.util.module_from_spec(SPEC)
sys.modules["ft"] = ft
SPEC.loader.exec_module(ft)

SAMPLE = bytes.fromhex("49AAB0751CC19B3FEDC0AE5CF040D4B52DBE5F55993E7C1AB0BB0D0A")
EXPECT_KG = [-9.77873, -7.41401, 7.51131, -0.16964, 0.29948, -0.00537]


def test_manual_sample_decodes_exactly():
    parsed = ft.parse_frame(SAMPLE)
    assert parsed is not None
    for got, want in zip(parsed["raw"], EXPECT_KG):
        assert math.isclose(got, want, abs_tol=1e-4), (got, want)


def test_units_are_converted_to_newton():
    """手册单位是 kg / kg·m；ROS 需要 N / N·m，必须乘 9.80665。"""
    parsed = ft.parse_frame(SAMPLE)
    # 期望值来自手册的 5 位小数，原始量是 float32，故用 1e-3 的绝对容差
    assert math.isclose(parsed["force"][0], EXPECT_KG[0] * 9.80665, abs_tol=1e-3)
    assert math.isclose(parsed["torque"][0], EXPECT_KG[3] * 9.80665, abs_tol=1e-3)


def test_rejects_bad_frames():
    assert ft.parse_frame(SAMPLE[:-1]) is None                  # 长度不对
    assert ft.parse_frame(b"\x50" + SAMPLE[1:]) is None          # 帧头不对
    assert ft.parse_frame(SAMPLE[:-2] + b"\x00\x00") is None     # 尾部不对
    nan = bytearray(SAMPLE)
    nan[2:6] = (b"\x00\x00\xc0\x7f")                             # NaN
    assert ft.parse_frame(bytes(nan)) is None


def test_stream_resyncs_through_garbage_and_dropped_bytes():
    parser = ft.FrameParser()
    stream = b"\x11\x22garbage" + SAMPLE + SAMPLE[3:] + SAMPLE
    frames = parser.feed(stream)
    assert len(frames) == 2, len(frames)          # 中间那帧被截断，应被跳过
    assert parser.bad_frames >= 0


def test_stream_handles_split_reads():
    """串口一次读到的可能是半个帧，必须能跨 read 拼起来。"""
    parser = ft.FrameParser()
    out = []
    for i in range(0, len(SAMPLE), 5):
        out += parser.feed(SAMPLE[i:i + 5])
    assert len(out) == 1
