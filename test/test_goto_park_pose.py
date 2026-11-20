"""离线测试：停车位姿的读取与安全校验（2026-11-20）。

背景：J4 抱闸失效，所有者指定了一个"J4 转轴与重力平行"的停车位姿，
断电后重力不会带动 J4。一键回位任务读这个位姿并把机械臂送回去。
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "goto_park_pose", ROOT / "scripts" / "goto_park_pose.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_config_has_a_six_axis_park_pose():
    config = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    pose = config.get("j4_park_pose") or {}
    joints = pose.get("joints_deg")
    assert joints is not None and len(joints) == 6
    assert pose.get("j4_deg") == pytest.approx(joints[3], abs=0.01)


def test_every_park_joint_is_inside_the_absolute_safe_range():
    """停车位姿本身必须在安全区间内，否则回位等于把臂送到禁区。"""
    config = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    joints = config["j4_park_pose"]["joints_deg"]
    for name, value in zip(tool.JOINT_ORDER, joints):
        low, high = tool.ABSOLUTE_LIMITS_DEG[name]
        assert low <= value <= high, f"{name}={value} 超出 [{low}, {high}]"


def test_load_target_deg_returns_floats():
    target = tool.load_target_deg()
    assert len(target) == 6
    assert all(isinstance(value, float) for value in target)


def test_the_park_pose_differs_from_the_observation_pose_only_in_j4():
    """设计意图：其余五轴贴着粗观测位，只把 J4 转到与重力平行的角度。"""
    config = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    park = config["j4_park_pose"]["joints_deg"]
    observation = [34.08, -70.31, -148.27, 13.24, 104.80, 79.16]
    differences = [abs(a - b) for a, b in zip(park, observation)]
    assert differences[3] > 45.0, "J4 应当被明显转开（否则断电会摆动）"
    assert max(differences[:3] + differences[4:]) < 3.0, "其余五轴应贴着粗观测位"
