"""方案 A（预教站位 + 按偏转角自动选站）的纯数学单元测试。

真机不可用时这些测试是唯一的离线保障：只测 `nbv_views` 里的决策数学
（yaw 约定、查表选站、0° 相对视角推导、一致性门），不 import rclpy、
不发任何运动命令、不读 runtime/ 数据（最后一条用 skipif 保护）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import nbv_views  # noqa: E402


def measurement_payload(T: np.ndarray, group: str = "g00_m") -> dict:
    """与 measure_object 输出同形的 per_group 结构。"""
    return {"per_group": [{"group": group, "pose_base": {
        "translation": np.round(T[:3, 3], 6).tolist(),
        "quaternion": np.round(R.from_matrix(T[:3, :3]).as_quat(), 9).tolist()}}]}


def pose(translation=(0.7, 0.4, 0.12), yaw_deg=0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = translation
    T[:3, :3] = R.from_euler("z", yaw_deg, degrees=True).as_matrix()
    return T


def make_views(ref_pose: np.ndarray, T_tcp_cam: np.ndarray) -> dict:
    """合成与 config/tasks/common.yaml 同形的视角表。

    每个 numeric key 的存储位姿都按"0° 相对视角推导"生成，因此与推导
    数学自洽；`+11` 由调用方覆盖为 taught 族以测独立站位路径。
    """
    cam0 = ref_pose.copy()
    cam0[:3, 3] = ref_pose[:3, 3] + np.array([0.1, -0.3, 0.3])
    views: dict = {"tolerance_deg": 12.0}
    for key in (0, -16, -23, -30, -45):
        cur = ref_pose.copy()
        cur[:3, :3] = R.from_euler("z", key, degrees=True).as_matrix() @ ref_pose[:3, :3]
        derived = cur @ np.linalg.inv(ref_pose) @ (cam0 @ T_tcp_cam) @ np.linalg.inv(T_tcp_cam)
        views[str(key)] = {
            "family": "relative_zero",
            "tcp": np.round(derived[:3, 3], 6).tolist(),
            "quat": np.round(R.from_matrix(derived[:3, :3]).as_quat(), 9).tolist(),
        }
    return views


@pytest.fixture()
def world():
    ref_pose = pose()
    T_tcp_cam = np.eye(4)
    T_tcp_cam[:3, 3] = [0.05, 0.0, 0.1]
    return ref_pose, T_tcp_cam, make_views(ref_pose, T_tcp_cam)


def test_yaw_sign_matches_measure_object_convention():
    ref, cur = pose(), pose(yaw_deg=-30.0)
    assert nbv_views.yaw_degrees_between(ref, cur) == pytest.approx(-30.0, abs=0.1)
    assert nbv_views.yaw_degrees_between(ref, pose(yaw_deg=+11.0)) == pytest.approx(11.0, abs=0.1)


def test_select_known_view_hits_nearest_numeric_station():
    assert nbv_views.select_known_view(-23.0)[1] == "-23"
    assert nbv_views.select_known_view(+11.3)[1] == "+11"
    assert nbv_views.select_known_view(-29.6)[1] == "-30"
    assert nbv_views.select_known_view(+90.0) is None


def test_pose_from_payload_roundtrip():
    payload = measurement_payload(pose(yaw_deg=-23.0))
    T = nbv_views.pose_from_payload(payload)
    assert T.shape == (4, 4)
    assert nbv_views.yaw_degrees_between(pose(), T) == pytest.approx(-23.0, abs=0.1)


def test_station_plan_selects_and_agrees_with_derivation(world):
    ref_pose, T_tcp_cam, views = world
    cur = pose(yaw_deg=-30.0)
    plan = nbv_views.station_plan(views, ref_pose, cur, T_tcp_cam)
    assert plan["yaw_deg"] == pytest.approx(-30.0, abs=0.1)
    assert plan["verdict"] == "station"
    assert plan["station"][1] == "-30"
    assert plan["delta_mm"] == pytest.approx(0.0, abs=0.5)
    assert plan["delta_deg"] == pytest.approx(0.0, abs=0.5)


def test_station_plan_refuses_when_no_station_covers_yaw(world):
    ref_pose, T_tcp_cam, views = world
    plan = nbv_views.station_plan(views, ref_pose, pose(yaw_deg=+90.0), T_tcp_cam)
    assert plan["verdict"] == "no_station"
    assert plan["station"] is None


def test_station_plan_refuses_when_workpiece_moved(world):
    ref_pose, T_tcp_cam, views = world
    moved = pose(translation=(0.76, 0.4, 0.12), yaw_deg=-30.0)
    plan = nbv_views.station_plan(views, ref_pose, moved, T_tcp_cam)
    assert plan["verdict"] == "station_moved"
    assert plan["delta_mm"] > 40.0


def test_station_plan_taught_family_skips_consistency_gate(world):
    ref_pose, T_tcp_cam, views = world
    views["+11"] = {
        "family": "taught",
        "tcp": [0.728, 0.338, 0.625],
        "quat": [0.274994, -0.821645, -0.278388, -0.414462],
    }
    plan = nbv_views.station_plan(views, ref_pose, pose(yaw_deg=+11.0), T_tcp_cam)
    assert plan["verdict"] == "station"
    assert plan["station"][1] == "+11"
    assert plan["delta_mm"] > 40.0  # 独立站位必然与 0° 推导不同，但不设门


def test_views_table_requires_zero_anchor(world):
    ref_pose, T_tcp_cam, views = world
    del views["0"]
    with pytest.raises(ValueError):
        nbv_views.station_plan(views, ref_pose, pose(yaw_deg=-30.0), T_tcp_cam)


@pytest.mark.skipif(
    not (REPO / "runtime" / "measurements" / "workpiece_zero.json").exists()
    or not (REPO / "runtime" / "measurements" / "workpiece_minus30.json").exists(),
    reason="需要真机测量产物（runtime/measurements，git 忽略）",
)
def test_real_measurements_validate_derivation_against_stored_stations():
    """真机数据回归：−30° 站位是"0° 相对视角推导"族，推导应与存储一致。"""
    import yaml

    config = yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text())
    handeye = json.loads((REPO / "config" / "handeye" / "tcp_to_camera.json").read_text())
    ref = nbv_views.load_pose(REPO / "runtime" / "measurements" / "workpiece_zero.json")
    cur = nbv_views.load_pose(REPO / "runtime" / "measurements" / "workpiece_minus30.json")
    plan = nbv_views.station_plan(config["workpiece_views"], ref, cur,
                                  np.array(handeye["T_tcp_cam"]))
    assert plan["yaw_deg"] == pytest.approx(-29.6, abs=0.5)
    assert plan["verdict"] == "station"
    assert plan["station"][1] == "-30"
    assert plan["delta_mm"] < 10.0
