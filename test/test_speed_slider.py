"""离线测试：速度滑块必须与我们的关节速度上限一致（2026-11-20 事故）。

事故：滑块被设成 100%，失控跳变时机械臂全速冲出安全区间（J6 到 157°）。
规矩：滑块比例 = 我们的上限 / 机器人额定速度，两者必须对得上。
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "set_speed_slider", ROOT / "scripts" / "set_speed_slider.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_fraction_leaves_tracking_headroom():
    """滑块必须比轨迹速度快几倍，否则伺服一直贴着极限跑会滞后/抖动。"""
    cap = tool.joint_speed_cap()
    rated = tool.DEFAULT_RATED_RAD_S
    fraction = cap * tool.TRACKING_HEADROOM / rated
    # 2.094 rad/s 额定 + 0.10 上限 + 3 倍余量 → 约 14.3%（17.2°/s）
    assert fraction == pytest.approx(0.1433, abs=0.005)
    assert fraction * rated > cap * 2, "余量必须明显大于 1 倍"
    assert fraction < 0.2, "仍要远低于额定，避免失控跳变全速"


def test_headroom_is_not_one():
    """等于轨迹速度（余量 1.0）是之前踩过的坑，必须大于 1。"""
    assert tool.TRACKING_HEADROOM > 2.0


def test_min_fraction_never_zero():
    """滑块不能是 0（那会让机器人完全动不了）。"""
    assert tool.MIN_FRACTION > 0


def test_cap_is_the_single_source_of_truth():
    """滑块换算必须复用执行器的常量，不能在别处写死。"""
    source = (ROOT / "scripts" / "set_speed_slider.py").read_text(encoding="utf-8")
    assert "MAX_JOINT_SPEED_RAD_S" in source
    assert "execute_assembly_chunks" in source
