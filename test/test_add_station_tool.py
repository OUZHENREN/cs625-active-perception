"""离线测试：把精拍测量登记为站位条目（2026-11-19）。"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import types

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "add_station", ROOT / "scripts" / "add_station_from_measurement.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def payload(verdict="OK", coverage=0.972, yaw=-20.0) -> dict:
    return {
        "verdict": verdict, "coverage_median": coverage, "credible_groups": 2,
        "groups": 3, "workpiece_yaw_deg": yaw,
        "repeatability": {"translation_mm": 0.24, "rotation_deg": 0.053},
        "per_group": [{"group": "g00_m", "pose_base": {
            "translation": [0.6633, 0.6568, 0.1206],
            "quaternion": [0.2018, 0.9794, 0.0043, -0.0039]}}],
    }


def args(key="0@20261119", yaw=0.0, tier=1, min_coverage=0.95, yaw_tol=1.5):
    return types.SimpleNamespace(key=key, yaw=yaw, tier=tier,
                                 min_coverage=min_coverage, yaw_tol=yaw_tol,
                                 measurement="runtime/measurements/x.json")


def test_entry_contains_everything_the_selector_needs():
    entry = tool.build_entry(args(), payload(yaw=0.0), [0.63, 0.32, 0.46],
                             [0.47, -0.73, -0.27, -0.41], "test")
    assert entry["yaw_deg"] == 0.0
    assert entry["tier"] == 1
    assert entry["empirical_coverage"] == pytest.approx(0.972)
    assert entry["tcp"] == [0.63, 0.32, 0.46]
    assert entry["validated"] is True
    assert entry["workpiece_pose"]["translation"][0] == pytest.approx(0.6633)


def test_a_refused_measurement_is_not_registered():
    with pytest.raises(SystemExit, match="verdict"):
        tool.build_entry(args(), payload(verdict="REFUSE"), [0, 0, 0], [0, 0, 0, 1], "t")


def test_low_coverage_is_not_registered():
    with pytest.raises(SystemExit, match="覆盖率"):
        tool.build_entry(args(), payload(coverage=0.90), [0, 0, 0], [0, 0, 0, 1], "t")


def test_a_workpiece_that_was_not_rotated_to_the_requested_angle_is_refused():
    with pytest.raises(SystemExit, match="工件偏角"):
        tool.build_entry(args(yaw=0.0), payload(yaw=20.0), [0, 0, 0], [0, 0, 0, 1], "t")


def test_written_block_parses_and_keeps_the_existing_table():
    entry = tool.build_entry(args(), payload(yaw=0.0), [0.63, 0.32, 0.46],
                             [0.47, -0.73, -0.27, -0.41], "test")
    block = tool.render_block("0@20261119", entry)
    text = "workpiece_views:\n  '0':\n    tcp: [1, 2, 3]\n"
    merged = tool.insert_entry(text, block)
    parsed = yaml.safe_load(merged)
    views = parsed["workpiece_views"]
    assert "0" in views                      # 老条目还在
    assert views["0@20261119"]["tier"] == 1
    assert views["0@20261119"]["yaw_deg"] == 0.0
