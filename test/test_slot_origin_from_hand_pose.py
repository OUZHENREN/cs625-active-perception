"""slot_origin 的权威来源是**测量模板位姿**，不是手摆位姿（2026-11-26 修正）。

历史
----
最初这个测试断言"slot_origin 必须能反推出所有者手摆的插入位"。23:30 查明：
手摆插入位是在**张紧式抓取把工件撑偏约 10 mm 之后**记录的（手册第 83 条），
所以它本身就带 10 mm 系统偏差，不能当权威。

现在的口径
----------
· slot_origin = **测量模板位姿的中位**（`slot_latest.json` 的 per_group.pose_base）；
· 手摆插入位与它的差 ≈ 10 mm（x 方向），这是**已知且已记录的系统偏置**，
  测试里显式断言这个偏置的存在与量级，而不是要求两者相等。
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
T_S_TCP_XYZ = np.array([-0.082, -0.193, -0.4125])   # 师兄 tcp_server.py / pose_math.py
HAND_BIAS_DX_MM = 10.3        # 张紧式抓取撑偏工件导致的 x 系统偏置（手册 83）


def _common() -> dict:
    return yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text(encoding="utf-8"))


def test_slot_origin_follows_the_measured_template_pose():
    """slot_origin 必须等于最近一次插槽测量的模板位姿中位（±1.5 mm）。"""
    meas_path = REPO / "runtime" / "measurements" / "slot_latest.json"
    if not meas_path.exists():
        pytest.skip("没有 slot_latest.json")
    payload = json.loads(meas_path.read_text(encoding="utf-8"))
    groups = payload.get("per_group") or []
    poses = [g["pose_base"]["translation"] for g in groups if g.get("pose_base")]
    if len(poses) < 2:
        pytest.skip("slot_latest 里没有足够的 per_group 位姿")
    measured = np.median(np.asarray(poses, dtype=float), axis=0)
    configured = np.asarray(_common()["slot_origin"], dtype=float)
    residual_mm = (configured - measured) * 1000.0
    assert np.linalg.norm(residual_mm) <= 1.5, (
        f"slot_origin 与测量模板位姿相差 {np.round(residual_mm, 2)} mm，超过 1.5 mm —— "
        "config 没有跟上最近一次测量（手册第 77/83 条）"
    )


def test_hand_recorded_insert_carries_the_documented_grasp_bias():
    """手摆插入位相对测量推导位在 x 上偏约 10 mm —— 这是已知偏置，不是错误。"""
    calib_path = REPO / "runtime" / "insert_calibration.json"
    if not calib_path.exists():
        pytest.skip("没有手摆位姿记录")
    calib = json.loads(calib_path.read_text(encoding="utf-8"))
    straighten = (calib.get("sets", {}).get("S02", {}).get("poses", {}).get("straighten"))
    if not straighten:
        pytest.skip("S02 缺少 straighten 位姿")
    slot = np.asarray(_common()["slot_origin"], dtype=float)
    derived = slot + T_S_TCP_XYZ
    hand = np.asarray(straighten["tcp"], dtype=float)
    dx_mm = (derived[0] - hand[0]) * 1000.0
    assert abs(dx_mm - HAND_BIAS_DX_MM) <= 3.0, (
        f"手摆与推导的 x 偏置 {dx_mm:+.1f} mm 偏离记录的 {HAND_BIAS_DX_MM} mm 超过 3 mm —— "
        "若抓取方式变了（不再撑偏工件），需要重新标定手摆真值（手册第 83 条）"
    )
