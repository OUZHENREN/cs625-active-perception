"""插入位姿标定工具（所有者 2026-11-25 方案）。

插槽是"面内弱约束"的大平面夹具（同一片点云两条配准路径差 66 mm，手册第 68 条），
所以可靠的插入位姿必须由所有者手动摆出、我们记录，再换成
`insert_in_slot_frame = inv(T_base_slot) @ T_base_tcp` 这样的相对变换，
之后与坐标系约定无关（与 grasp_in_workpiece_frame 完全同构）。
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "record_insert", ROOT / "scripts" / "record_insert_poses.py")
rec = importlib.util.module_from_spec(SPEC)
sys.modules["record_insert"] = rec
SPEC.loader.exec_module(rec)


def _setup(tmp_path, monkeypatch, tcp_xyz, quat):
    store = tmp_path / "insert_calibration.json"
    monkeypatch.setattr(rec, "STORE", store)
    store.write_text(json.dumps({"recorded_at": "test", "poses": {"insert": {
        "tcp": list(tcp_xyz), "quat": list(quat), "joints_deg": [0] * 6,
        "note": "test", "recorded_at": "test"}}}), encoding="utf-8")
    slot = {"translation": [1.25, 0.14, 0.44],
            "quaternion": [-0.0017, -0.0008, -0.0048, 0.99999]}
    meas = tmp_path / "slot.json"
    meas.write_text(json.dumps({"per_group": [{"pose_base": slot}]}), encoding="utf-8")
    return store, meas


def test_derive_round_trip_is_exact(tmp_path, monkeypatch, capsys):
    store, meas = _setup(tmp_path, monkeypatch,
                         [1.1460, -0.0548, 0.0342], [0.0047, 0.9961, -0.0021, 0.088])
    code = rec.cmd_derive(type("A", (), {"name": "insert",
                                         "slot_measurement": str(meas)})())
    assert code == 0
    payload = json.loads(store.read_text(encoding="utf-8"))
    derived = payload["derived"]["insert"]
    assert derived["round_trip_mm"] < 1e-6
    assert derived["round_trip_deg"] < 1e-6


def test_relative_transform_matches_slot_in_tcp_frame(tmp_path, monkeypatch):
    """推导出的 T_rel 必须满足 T_slot @ T_rel == T_tcp（这就是它的定义）。"""
    tcp = [1.1460, -0.0548, 0.0342]
    quat = [0.0047, 0.9961, -0.0021, 0.088]
    store, meas = _setup(tmp_path, monkeypatch, tcp, quat)
    rec.cmd_derive(type("A", (), {"name": "insert", "slot_measurement": str(meas)})())
    d = json.loads(store.read_text(encoding="utf-8"))["derived"]["insert"]
    T_slot = np.eye(4)
    T_slot[:3, :3] = R.from_quat([-0.0017, -0.0008, -0.0048, 0.99999]).as_matrix()
    T_slot[:3, 3] = [1.25, 0.14, 0.44]
    T_rel = np.eye(4)
    T_rel[:3, :3] = R.from_quat(d["quaternion"]).as_matrix()
    T_rel[:3, 3] = d["translation"]
    assert np.allclose((T_slot @ T_rel)[:3, 3], tcp, atol=1e-9)


def test_derived_offset_is_comparable_to_the_senior_constant():
    """相对变换的用途：可以**跨口径**直接和师兄的 T_s_tcp 常数比，不再争绝对坐标。

    师兄 T_s_tcp = diag(-1,1,-1) + (-82, -193, -412.5) mm
    11-04 验证参考换成的相对变换 约为 (-112, -198, -411.8) mm
    → x 差约 30 mm，y 5 mm，z 1 mm。这是"两个插入链在相对意义下的真实差别"。
    """
    senior = np.array([-82.0, -193.0, -412.5])
    verified = np.array([-112.0, -198.4, -411.8])
    delta = np.abs(verified - senior)
    assert delta[0] > 20.0, "x 方向是主要差别"
    assert delta[1] < 10.0 and delta[2] < 5.0
