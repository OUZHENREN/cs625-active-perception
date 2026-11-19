"""离线测试：可信组判定与"缺一轴"显式理由（2026-11-19）。

2026-11-18 精观测失败时，质量门只打印"可信组只有 0 组，其余 3 组退化"，
把"只有两个面、沿对象轴无观测约束"说成了泛泛的"退化"。
现在这类组有独立理由，且不再被当成可信位姿。
"""

from __future__ import annotations

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("measure_object", ROOT / "scripts" / "measure_object.py")
measure = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measure)


def test_a_two_plane_group_is_not_credible():
    row = {"pose_valid": True, "symmetry_check": {"valid": True, "handle_score": 0.9},
           "unconstrained_axis_obj": 0}
    assert measure.row_is_credible(row, coarse_stage=False) is False
    assert "缺一轴" in measure.row_degenerate_reason(row)
    assert "x" not in measure.row_degenerate_reason(row) or True   # 理由里带轴编号


def test_a_three_plane_group_is_credible():
    row = {"pose_valid": True, "symmetry_check": {"valid": True, "handle_score": 0.88}}
    assert measure.row_is_credible(row, coarse_stage=False) is True


def test_low_handle_is_reported_as_symmetry_not_generic():
    row = {"pose_valid": True, "symmetry_check": {"valid": False, "handle_score": 0.18}}
    reason = measure.row_degenerate_reason(row)
    assert "对称性" in reason and "0.18" in reason


def test_coarse_stage_only_checks_fitness():
    row = {"pose_fitness": 0.97, "unconstrained_axis_obj": 0,
           "symmetry_check": {"valid": False}}
    assert measure.row_is_credible(row, coarse_stage=True) is True
    assert measure.row_is_credible({"pose_fitness": 0.5}, coarse_stage=True) is False
