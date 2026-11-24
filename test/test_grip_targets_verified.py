"""离线测试：效率模式的抓取目标必须用所有者验证过的值。

2026-11-24 之前 grip_targets() 会沿工具 x 回退 10 mm 并自行 +150 mm 垂直算预抓取位，
结果既偏离验证过的抓取点，预抓取位又缺 10 mm 侧移（下降时电机保护座蹭工件）。
这里只读配置与源码文本，不导入 run_pick_and_place（它依赖运行时参数）。
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]


def config():
    return yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())


def test_approach_offset_is_150_up_and_10_lateral():
    """预抓取偏移 = 150 mm 垂直 + 10 mm 水平（−Y）。"""
    cfg = config()
    offset = np.array(cfg["pre_grasp"]) - np.array(cfg["grasp"])
    assert offset[2] == pytest.approx(0.150, abs=1e-6)
    assert np.linalg.norm(offset[:2]) == pytest.approx(0.010, abs=1e-6)
    assert offset[1] == pytest.approx(-0.010, abs=1e-6), "侧移应沿 −Y"


def test_grasp_in_workpiece_frame_is_configured():
    cfg = config()
    rel = cfg["grasp_in_workpiece_frame"]
    assert len(rel["translation"]) == 3
    assert len(rel["quaternion"]) == 4
    assert rel.get("joints_deg_at_verification"), "应记录验证时的关节角"


def test_relative_transform_reproduces_the_verified_grasp():
    """T_工件 @ T_相对 必须复现验证过的抓取点。"""
    from scipy.spatial.transform import Rotation as R
    cfg = config()
    ref = cfg["workpiece_reference"]
    rel = cfg["grasp_in_workpiece_frame"]
    T_obj = np.eye(4)
    T_obj[:3, :3] = R.from_quat(ref["quat"]).as_matrix()
    T_obj[:3, 3] = ref["xyz"]
    T_rel = np.eye(4)
    T_rel[:3, :3] = R.from_quat(rel["quaternion"]).as_matrix()
    T_rel[:3, 3] = rel["translation"]
    grasp = (T_obj @ T_rel)[:3, 3]
    assert np.linalg.norm(grasp - np.array(cfg["grasp"])) < 1e-6


def test_efficiency_mode_no_longer_shifts_the_grasp():
    """grip_targets 里不应再偏移验证过的抓取点，且应使用相对变换。"""
    source = (ROOT / "scripts" / "run_pick_and_place.py").read_text(encoding="utf-8")
    body = source[source.index("def grip_targets"):]
    body = body[:body.index("def record_verdict")]
    assert "grasp_pose_with_clearance(" not in body, "不应再对验证过的抓取点做工具 x 偏移"
    assert "grasp_in_workpiece_frame" in body, "应使用相对变换跟随实测工件位姿"
