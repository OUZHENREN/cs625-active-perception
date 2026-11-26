"""slot_origin 必须能反推出所有者手摆的插入位（2026-11-26）。

背景
----
插槽位姿的面内误差集中在 **y（通道长轴）**：RVS 的 slot 模型原点 = 夹具 CAD 原点
= 槽体顶面基准角点。用师兄的固定变换 T_s_tcp 可以反算真实的 slot 原点。
本测试把这个反算关系钉住，防止以后有人手改 slot_origin 却不同步更新说明。

判据：slot_origin + (-0.082, -0.193, -0.4125) 应与手摆的 S02.straighten 一致，
横向残差 ≤ 8 mm（x/z 是强约束方向，实测 5.3/2.2 mm），y 残差 ≤ 1 mm
（y 由这次修正直接确定，应当严格一致）。
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
T_S_TCP_XYZ = np.array([-0.082, -0.193, -0.4125])   # 师兄 tcp_server.py / pose_math.py


def _common() -> dict:
    return yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text(encoding="utf-8"))


def test_slot_origin_reproduces_hand_recorded_insert():
    calib_path = REPO / "runtime" / "insert_calibration.json"
    if not calib_path.exists():
        pytest.skip("没有 runtime/insert_calibration.json（手摆位姿只在真机上产生）")
    calib = json.loads(calib_path.read_text(encoding="utf-8"))
    poses = calib.get("sets", {}).get("S02", {}).get("poses", {})
    straighten = poses.get("straighten")
    if not straighten:
        pytest.skip("S02 缺少 straighten 位姿")

    slot = np.asarray(_common()["slot_origin"], dtype=float)
    derived = slot + T_S_TCP_XYZ
    hand = np.asarray(straighten["tcp"], dtype=float)
    residual_mm = (derived - hand) * 1000.0

    assert abs(residual_mm[1]) <= 1.0, (
        f"y 方向残差 {residual_mm[1]:+.1f} mm 超过 1 mm —— slot_origin.y 与手摆位姿不一致；"
        "y 是通道长轴（弱约束方向），必须由手摆真值反算，见手册第 77 条"
    )
    assert np.linalg.norm(residual_mm[[0, 2]]) <= 8.0, (
        f"x/z 残差 {np.round(residual_mm[[0, 2]], 1)} mm 超过 8 mm —— 检查 slot_origin 的 x/z"
    )


def test_slot_origin_axis_is_the_fixture_top_corner():
    """slot_origin 是夹具顶面基准角点：夹具模型在该点下方展开（z 全为负）。"""
    common = _common()
    slot_z = float(np.asarray(common["slot_origin"], dtype=float)[2])
    mesh = (REPO / "src" / "cs625_simulation" / "assets" / "cs625_real_cell"
            / "slot_fixture" / "meshes" / "slot_fixture_collision.stl")
    if not mesh.exists():
        pytest.skip("夹具网格不存在")
    import open3d as o3d
    verts = np.asarray(o3d.io.read_triangle_mesh(str(mesh)).vertices)
    # 夹具模型 z 最大值 = 0（原点在顶面）→ 世界系里最高点应约等于 slot_origin.z
    assert verts[:, 2].max() == pytest.approx(0.0, abs=1e-6), "夹具模型原点不在顶面"
    assert slot_z > 0.0
