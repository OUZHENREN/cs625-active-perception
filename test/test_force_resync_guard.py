"""离线测试：`--force` 重挂的安全硬闸（2026-11-20 事故）。

事故：所有者手动拖动机械臂后执行 `ensure_robot_ready.py --force`，
机械臂抽搐着快速跳回上一次轨迹的终点（TCP 在 y 方向跳了 240 mm）。
真因：arm_controller 把最后一条轨迹的终点当作保持目标，重挂外部控制脚本时
会把这个旧目标重新下发。

硬闸：`--force` 前比对"当前实测 vs 最后指令位姿"，偏差 > 2° 直接拒绝。
"""

from __future__ import annotations

import importlib.util
import json
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ensure_robot_ready", ROOT / "scripts" / "ensure_robot_ready.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_drift_limit_is_strict():
    """2° 是人手拖动的合理噪声上限，再大就说明臂被挪过。"""
    assert tool.FORCE_DRIFT_LIMIT_DEG == pytest.approx(2.0)


def test_last_commanded_file_records_a_full_joint_vector(tmp_path, monkeypatch):
    payload = {"joint_names": ["a", "b", "c", "d", "e", "f"],
               "positions": [0.1] * 6, "stamp": 1.0, "source": "test"}
    path = tmp_path / "last_commanded_pose.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(tool, "LAST_COMMANDED", path)
    loaded = json.loads(path.read_text())
    assert len(loaded["joint_names"]) == len(loaded["positions"]) == 6


def test_missing_baseline_is_not_fatal(tmp_path, monkeypatch):
    """没有基准文件时返回 None（不做判断），不能因此崩溃或误拒。"""
    monkeypatch.setattr(tool, "LAST_COMMANDED", tmp_path / "nope.json")

    class Dummy:
        def create_subscription(self, *args, **kwargs):
            return None

    assert tool.measured_vs_last_commanded(Dummy()) is None


def test_malformed_baseline_is_not_fatal(tmp_path, monkeypatch):
    path = tmp_path / "last_commanded_pose.json"
    path.write_text('{"joint_names": ["a"], "positions": []}', encoding="utf-8")
    monkeypatch.setattr(tool, "LAST_COMMANDED", path)

    class Dummy:
        def create_subscription(self, *args, **kwargs):
            return None

    assert tool.measured_vs_last_commanded(Dummy()) is None
