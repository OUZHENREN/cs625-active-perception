"""Capture reference pose and camera-mode regressions (no ROS runtime needed).

2026-10-26: capture_dataset.py still used the 2026-10-24 workpiece pose.  The
measured pose was 180 mm away, so the framing gate and the clearance guard were
evaluated against the wrong part; every capture pose would have been rejected.
The same session removed the automatic camera-mode switching that restarted the
camera before every observation check.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"


@pytest.fixture(scope="module")
def capture():
    spec = importlib.util.spec_from_file_location(
        "capture_dataset_under_test", SCRIPTS / "capture_dataset.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["capture_dataset_under_test"] = module
    spec.loader.exec_module(module)
    return module


def test_part_centre_offset_matches_the_measured_model(capture):
    model = json.loads(
        (REPO / "runtime/environment/20261026_030037/model.json").read_text(encoding="utf-8"))
    xyz = np.array(model["workpiece"]["xyz"], dtype=float)
    quat = np.array(model["workpiece"]["quat"], dtype=float)
    centre = capture.part_centre_of(xyz, quat)
    # The model stores the geometric centre separately; both derivations agree to
    # a few millimetres (the offset is a CAD constant, the model fit is measured).
    stored = np.array(model["workpiece"]["centre"], dtype=float)
    assert np.linalg.norm(centre - stored) < 0.03


def test_fallback_reference_is_far_from_the_measured_pose(capture):
    model = json.loads(
        (REPO / "runtime/environment/20261026_030037/model.json").read_text(encoding="utf-8"))
    measured = capture.part_centre_of(
        np.array(model["workpiece"]["xyz"], dtype=float),
        np.array(model["workpiece"]["quat"], dtype=float))
    fallback = capture.part_centre_of(
        np.array([0.6803, 0.7166, 0.1245]), np.array([0.259, 0.9659, 0.0041, -0.0035]))
    # This is why the old defaults had to go: ~180 mm apart.
    assert np.linalg.norm(fallback - measured) > 0.15


def test_missing_pointer_falls_back(capture, monkeypatch, tmp_path):
    monkeypatch.setattr(capture, "MODEL_POINTER", tmp_path / "latest.json")
    assert capture.workpiece_pose_from_model() is None


def test_pointer_with_unreadable_model_falls_back(capture, monkeypatch, tmp_path):
    pointer = tmp_path / "latest.json"
    pointer.write_text(json.dumps({"model": "does_not_exist/model.json"}), encoding="utf-8")
    monkeypatch.setattr(capture, "MODEL_POINTER", pointer)
    assert capture.workpiece_pose_from_model() is None


def test_pointer_with_valid_model_returns_the_pose(capture, monkeypatch, tmp_path):
    model_dir = tmp_path / "20260101_000000"
    model_dir.mkdir()
    (model_dir / "model.json").write_text(json.dumps({
        "workpiece": {"xyz": [0.5, 0.6, 0.1], "quat": [0.0, 0.0, 0.0, 1.0]}
    }), encoding="utf-8")
    pointer = tmp_path / "latest.json"
    pointer.write_text(json.dumps({"model": "20260101_000000/model.json"}), encoding="utf-8")
    monkeypatch.setattr(capture, "MODEL_POINTER", pointer)
    pose = capture.workpiece_pose_from_model()
    assert pose is not None
    xyz, quat, source = pose
    assert np.allclose(xyz, [0.5, 0.6, 0.1])
    assert np.allclose(quat, [0.0, 0.0, 0.0, 1.0])
    assert source.endswith("model.json")


def test_distance_is_raised_until_the_clearance_guard_passes(capture):
    hand_eye = np.array(json.loads(
        (REPO / "config/handeye/tcp_to_camera.json").read_text(encoding="utf-8"))["T_tcp_cam"],
        dtype=float)
    model = json.loads(
        (REPO / "runtime/environment/20261026_030037/model.json").read_text(encoding="utf-8"))
    part = capture.part_centre_of(
        np.array(model["workpiece"]["xyz"], dtype=float),
        np.array(model["workpiece"]["quat"], dtype=float))
    xyz, _quat, distance_used, clearance = capture.target_for(
        part, 0.52, 0.0, 0.0, hand_eye, np.eye(4), part, 0.38)
    assert clearance >= 0.38 - 1e-9
    assert distance_used > 0.52
    assert distance_used <= capture.MAX_ORBIT_DISTANCE + 1e-9
    assert np.allclose(np.linalg.norm(xyz - part), clearance, atol=1e-9)


def test_target_for_stops_at_the_maximum_distance(capture):
    hand_eye = np.array(json.loads(
        (REPO / "config/handeye/tcp_to_camera.json").read_text(encoding="utf-8"))["T_tcp_cam"],
        dtype=float)
    model = json.loads(
        (REPO / "runtime/environment/20261026_030037/model.json").read_text(encoding="utf-8"))
    part = capture.part_centre_of(
        np.array(model["workpiece"]["xyz"], dtype=float),
        np.array(model["workpiece"]["quat"], dtype=float))
    _xyz, _quat, distance_used, clearance = capture.target_for(
        part, 0.52, 0.0, 0.0, hand_eye, np.eye(4), part, 5.0)
    assert distance_used == pytest.approx(capture.MAX_ORBIT_DISTANCE)
    assert clearance < 5.0


def test_angle_in_envelope_accepts_equivalent_representations(capture):
    # -346 deg is +14 deg; 221 deg is -139 deg (still outside).
    assert capture.angle_in_envelope(-346.0, -30.0, 120.0)
    assert capture.angle_in_envelope(14.0, -30.0, 120.0)
    assert not capture.angle_in_envelope(221.0, -30.0, 120.0)
    assert not capture.angle_in_envelope(127.0, -30.0, 120.0)


def test_task_entry_no_longer_restarts_the_camera():
    source = (SCRIPTS / "run_task.py").read_text(encoding="utf-8")
    assert "camera_mode.py" not in source
    assert "set_camera_mode" not in source


def test_runner_waits_for_a_frame_after_motion():
    source = (SCRIPTS / "run_pick_and_place.py").read_text(encoding="utf-8")
    assert "wait_camera_frame_after" in source
    assert "def set_camera_mode" not in source
    assert "/sensors/camera/color/image" in source
