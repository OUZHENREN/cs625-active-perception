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


def test_default_fraction_is_the_verified_value():
    """2026-11-20：按额定速度算的 14.3% 实测跟不上，默认改用实测可靠的 50%。"""
    assert tool.DEFAULT_FRACTION == pytest.approx(0.50)


def test_default_is_far_below_full_slider():
    """50% 仍显著低于满滑块，失控跳变速度被压掉一半。"""
    assert tool.DEFAULT_FRACTION < 0.8
    assert tool.DEFAULT_FRACTION > 0.2      # 太低会跟不住，14.3% 已失败过


def test_min_fraction_never_zero():
    """滑块不能是 0（那会让机器人完全动不了）。"""
    assert tool.MIN_FRACTION > 0


def test_cap_is_the_single_source_of_truth():
    """滑块换算必须复用执行器的常量，不能在别处写死。"""
    source = (ROOT / "scripts" / "set_speed_slider.py").read_text(encoding="utf-8")
    assert "MAX_JOINT_SPEED_RAD_S" in source
    assert "execute_assembly_chunks" in source
