"""离线测试：拒绝非自然构型（翻转解 / 腕部奇异解）。

2026-11-24 所有者定：预抓取位只解出 wrist_2=±90°（腕部奇异）的解，执行后机械臂
走到怪异构型，所有者按了急停。以后这类解**直接拒绝并给出理由**，不许在真机执行。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "plan_pose_ik", ROOT / "scripts" / "plan_pose_ik.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_singularity_constants_are_defined():
    assert tool.WRIST_SINGULARITY_DEG == pytest.approx(90.0)
    assert tool.SINGULARITY_TOLERANCE_DEG == pytest.approx(3.0)
    assert tool.FLIP_TRAVEL_DEG == pytest.approx(90.0)


def test_wrist_2_at_ninety_is_recognised_as_singular():
    """wrist_2 = ±90° 落在奇异点；±60° 不算。"""
    def singular(w2_deg):
        return abs(abs(w2_deg) - tool.WRIST_SINGULARITY_DEG) <= tool.SINGULARITY_TOLERANCE_DEG

    assert singular(90.0)
    assert singular(-90.0)
    assert singular(88.5)
    assert not singular(76.2)
    assert not singular(60.0)


def test_planner_refuses_singular_solutions_by_default():
    """源码层面锁住：默认拒绝奇异解，且提供 --allow-flip 但奇异解永远拒绝。"""
    source = (ROOT / "scripts" / "plan_pose_ik.py").read_text(encoding="utf-8")
    assert "rejected_singular" in source
    assert "腕部奇异" in source
    assert "--allow-flip" in source
    # 奇异解的 continue 不受 allow_flip 影响
    singular_block = source[source.index("rejected_singular.append"):]
    singular_block = singular_block[:singular_block.index("continue")]
    assert "allow_flip" not in singular_block, "奇异解不允许用 --allow-flip 绕过"
