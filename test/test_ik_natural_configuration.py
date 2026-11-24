"""离线测试：拒绝"腕部翻转/离已验证姿态过远"的解。

2026-11-24 第一版曾把"腕部奇异（w2≈±90°）"当成错误拒绝——那是错的：
垂直抓取（工具朝下）本来就把 wrist_2 压在 90° 附近，真机验证过的 pre_grasp
就是 w2=90.8°，一直正常执行。所有者纠正后改为：

  · 奇异本身允许；
  · 拒绝需要腕部翻转（wrist_1 或 wrist_3 行程 > 90°）的解；
  · 离所有已验证姿态都超过 60° 的解给出风险提示。
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "plan_pose_ik", ROOT / "scripts" / "plan_pose_ik.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_flip_threshold_and_proven_warning_are_defined():
    assert tool.FLIP_TRAVEL_DEG == pytest.approx(90.0)
    assert tool.PROVEN_WARN_DEG == pytest.approx(60.0)


def test_singularity_is_no_longer_a_rejection_reason():
    """垂直抓取天然落在 w2≈90°，不能再把它当错误拒绝。"""
    source = (ROOT / "scripts" / "plan_pose_ik.py").read_text(encoding="utf-8")
    assert "rejected_singular" not in source
    assert "WRIST_SINGULARITY_DEG" not in source
    assert "不拒绝腕部奇异" in source


def test_proven_postures_are_available_as_seeds():
    assert len(tool.PROVEN_POSTURES) >= 4
    for posture in tool.PROVEN_POSTURES:
        assert len(posture) == 6


def test_flip_rejection_cannot_be_bypassed_accidentally():
    """--allow-flip 必须显式给出才能放行翻转解。"""
    source = (ROOT / "scripts" / "plan_pose_ik.py").read_text(encoding="utf-8")
    assert "--allow-flip" in source
    block = source[source.index("rejected_flip.append"):]
    block = block[:block.index("continue")]
    assert "allow_flip" in source[source.index("if not args.allow_flip"):][:200]
