"""测量数据集要能直接用于环境建模（2026-11-25 所有者决定 A）。

背景：`measure_object` 只写 ``depth.npy``(米,float32) + ``meta.json``(无位姿)，
而 ``model_environment`` 需要 ``<stem>_depth.npy``(uint16 毫米) +
``metadata.jsonl``(每帧 tcp_matrix)。缺这两样时它对每组都报
"no metadata.jsonl (depth frames carry no pose)"，现有云点云完全无法用于建模。
"""

from __future__ import annotations

import importlib.util
import json
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "measure_object", ROOT / "scripts" / "measure_object.py")
measure = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measure)

HAND_EYE = np.array(json.loads(
    (ROOT / "config" / "handeye" / "tcp_to_camera.json").read_text())["T_tcp_cam"])


def _make_dataset(tmp_path, metres):
    """造一个 measure_object 风格的组目录：depth.npy 用的是**米**。"""
    group = tmp_path / "g00_m"
    group.mkdir()
    np.save(group / "depth.npy", np.asarray(metres, dtype=np.float32))
    return group


def test_depth_is_converted_to_millimetres(tmp_path):
    """必须转成 uint16 毫米：直接给米会让 model_environment 判"有效点不足"。"""
    group = _make_dataset(tmp_path, [[1.0, 2.0], [0.0, 0.5]])
    base_cam = np.eye(4)
    measure.write_env_metadata(tmp_path, base_cam)
    depth = np.load(group / "g00_m_depth.npy")
    assert depth.dtype == np.uint16
    assert depth[0, 0] == 1000 and depth[0, 1] == 2000
    assert depth[1, 0] == 0                      # 无效值保持 0
    assert depth[1, 1] == 500


def test_metadata_carries_tcp_matrix_not_tcp_cam(tmp_path):
    """model_environment 内部会再乘 T_tcp_cam，所以这里必须写 T_base_tcp。"""
    _make_dataset(tmp_path, [[1.0]])
    T_base_cam = np.eye(4)
    T_base_cam[:3, 3] = [0.6, 0.4, 0.5]
    measure.write_env_metadata(tmp_path, T_base_cam)
    rows = [json.loads(line) for line in
            (tmp_path / "g00_m" / "metadata.jsonl").read_text().splitlines()]
    assert len(rows) == 1 and rows[0]["file"] == "g00_m"
    expected = np.eye(4)
    expected[:3, 3] = [0.6, 0.4, 0.5]
    expected = expected @ np.linalg.inv(HAND_EYE)
    assert np.allclose(np.array(rows[0]["tcp_matrix"]).reshape(4, 4), expected, atol=1e-6)


def test_no_pose_means_no_metadata(tmp_path):
    """没有 T_base_cam 时不要写半套数据（否则建模会拿到错位姿）。"""
    group = _make_dataset(tmp_path, [[1.0]])
    measure.write_env_metadata(tmp_path, None)
    assert not (group / "metadata.jsonl").exists()
