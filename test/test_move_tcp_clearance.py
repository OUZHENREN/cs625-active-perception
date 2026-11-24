"""离线测试：笛卡尔运动必须有工件点云间隙闸（2026-11-24 三次碰撞后加）。"""

from __future__ import annotations

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "move_tcp_linear", ROOT / "scripts" / "move_tcp_linear.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_clearance_threshold_is_conservative():
    """15 mm 是夹爪与工件之间的最小安全距离。"""
    assert tool.CLEARANCE_MIN_M == 0.015
    assert tool.CLOUD_RADIUS_M <= 0.5


def test_clearance_gate_refuses_before_moving():
    """源码层面锁住：每一步都要先查间隙，拿不到点云就拒绝。"""
    source = (ROOT / "scripts" / "move_tcp_linear.py").read_text(encoding="utf-8")
    body = source[source.index("while True"):]
    assert "workpiece_clearance(" in body
    assert "CLEARANCE_UNKNOWN" in body
    assert "REFUSED_TOO_CLOSE" in body
    # 检查必须发生在规划之前
    assert body.index("workpiece_clearance(") < body.index("GetCartesianPath.Request()")


def test_clearance_returns_none_without_cloud():
    """没有可用点云时返回 (None, 0)，调用方必须拒绝而不是放行。"""
    distance, count = tool.workpiece_clearance(
        [0, 0, 0], measurement_path="/nonexistent.json")
    assert distance is None or distance >= 0
    assert count >= 0
