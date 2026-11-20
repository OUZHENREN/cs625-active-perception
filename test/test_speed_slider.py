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


def test_fraction_matches_the_executor_cap():
    cap = tool.joint_speed_cap()
    rated = tool.DEFAULT_RATED_RAD_S
    fraction = cap / rated
    # 2.094 rad/s 额定 + 0.05 上限 → 约 2.4%
    assert fraction == pytest.approx(0.0239, abs=0.001)
    assert fraction * rated == pytest.approx(cap)


def test_min_fraction_never_zero():
    """滑块不能是 0（那会让机器人完全动不了）。"""
    assert tool.MIN_FRACTION > 0


def test_cap_is_the_single_source_of_truth():
    """滑块换算必须复用执行器的常量，不能在别处写死。"""
    source = (ROOT / "scripts" / "set_speed_slider.py").read_text(encoding="utf-8")
    assert "MAX_JOINT_SPEED_RAD_S" in source
    assert "execute_assembly_chunks" in source
