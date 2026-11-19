"""离线测试：摆放约束（2026-11-19）。

2026-11-18 精观测失败的根因之一是工件相对站位参考位置平移了 58.7 mm，
第三面掠射到只剩 2 个面。现在选站后会检查这个偏差，超出就拒绝并提示摆回。
"""

from __future__ import annotations

import importlib.util
import json
import math
import pathlib

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "nbv_from_reference", ROOT / "scripts" / "nbv_from_reference.py")
nbv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(nbv)


def write_measurement(path: pathlib.Path, xyz, yaw_deg: float) -> None:
    yaw = math.radians(yaw_deg)
    payload = {"verdict": "OK", "per_group": [{
        "group": "g00_m",
        "pose_base": {"translation": list(xyz),
                      "quaternion": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]},
    }]}
    path.write_text(json.dumps(payload), encoding="utf-8")


def pose(xyz, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    m = np.eye(4)
    m[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0],
                 [math.sin(yaw), math.cos(yaw), 0],
                 [0, 0, 1]]
    m[:3, 3] = xyz
    return m


def test_no_reference_means_no_check(tmp_path):
    assert nbv.placement_gap({}, pose((0.6, 0.6, 0.12), 0.0)) is None


def test_gap_within_tolerance(tmp_path):
    ref = tmp_path / "ref.json"
    write_measurement(ref, (0.64, 0.60, 0.12), 0.0)
    entry = {"reference_measurement": str(ref)}
    gap_mm, gap_deg, name = nbv.placement_gap(entry, pose((0.65, 0.61, 0.12), 1.0))
    assert gap_mm == pytest.approx(14.1, abs=1.0)      # ~10 mm x, ~10 mm y
    assert gap_deg == pytest.approx(1.0, abs=0.2)
    assert name == "ref.json"


def test_the_2026_11_18_case_is_refused(tmp_path):
    """真实数据：工件比 −16 站位参考位置偏了约 58 mm → 必须超限。"""
    ref = tmp_path / "workpiece_minus15.json"
    write_measurement(ref, (0.632, 0.629, 0.1205), 162.51)
    entry = {"reference_measurement": str(ref)}
    current = pose((0.638, 0.687, 0.121), 159.01)
    gap_mm, gap_deg, _name = nbv.placement_gap(entry, current)
    assert gap_mm > nbv.PLACEMENT_MAX_MM            # 58.7 mm > 30 mm
    assert gap_deg == pytest.approx(nbv.PLACEMENT_MAX_DEG, abs=1e-3)  # yaw 3.50° 刚好在限内


def test_a_yaw_shift_alone_is_caught(tmp_path):
    ref = tmp_path / "ref.json"
    write_measurement(ref, (0.64, 0.60, 0.12), 0.0)
    entry = {"reference_measurement": str(ref)}
    _gap_mm, gap_deg, _name = nbv.placement_gap(entry, pose((0.64, 0.60, 0.12), 6.0))
    assert gap_deg > nbv.PLACEMENT_MAX_DEG


def test_unreadable_reference_is_skipped_not_fatal(tmp_path):
    entry = {"reference_measurement": str(tmp_path / "missing.json")}
    assert nbv.placement_gap(entry, pose((0.6, 0.6, 0.12), 0.0)) is None
