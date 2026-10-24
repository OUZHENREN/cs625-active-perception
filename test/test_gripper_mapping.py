"""EFG opening -> simulated finger joint mapping (pure, no ROS)."""

from __future__ import annotations

import importlib.util
import sys
import pathlib

import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent
SPEC = REPO / "scripts" / "run_pick_and_place.py"


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location("rpp_gripper", SPEC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rpp_gripper"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_closed_maps_to_zero(module):
    assert module.finger_position_m(0.0) == pytest.approx(0.0, abs=1e-12)


def test_full_opening_maps_to_the_cad_half_span(module):
    # 246.57 - 204.00 = 42.57 mm span change, half of it per finger
    assert module.finger_position_m(50.0) == pytest.approx(0.021285, abs=1e-9)


def test_half_opening_is_linear(module):
    assert module.finger_position_m(25.0) == pytest.approx(0.021285 / 2.0, abs=1e-9)


def test_commands_outside_the_range_are_clamped(module):
    assert module.finger_position_m(-5.0) == pytest.approx(0.0, abs=1e-12)
    assert module.finger_position_m(80.0) == pytest.approx(0.021285, abs=1e-9)


def test_zero_max_opening_is_rejected(module):
    with pytest.raises(ValueError):
        module.finger_position_m(10.0, max_opening_mm=0.0)


def test_config_carries_the_same_numbers(module):
    common = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text(encoding="utf-8"))
    assert common["gripper_max_opening_mm"] == pytest.approx(50.0)
    assert common["gripper_span_min_mm"] == pytest.approx(204.0)
    assert common["gripper_span_max_mm"] == pytest.approx(246.57)
    assert module.finger_position_m(
        common["gripper_max_opening_mm"], common["gripper_max_opening_mm"],
        common["gripper_span_min_mm"], common["gripper_span_max_mm"],
    ) == pytest.approx(0.021285, abs=1e-9)
