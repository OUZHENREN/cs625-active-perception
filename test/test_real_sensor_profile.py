"""Real-cell sensor profile: the chain that turns camera topics into /sensors/*.

2026-10-26: every one of the four input topics in ``config/real.yaml`` was an
empty string (a Humble-era placeholder), so the adapter refused to configure and
none of the ``/sensors/camera/*`` contract topics existed.  These tests pin the
values that were measured on the cell and the camera-mode flag that makes the
driver publish a cloud at all.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import sys

import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"

INPUT_KEYS = (
    "input_color_topic",
    "input_depth_topic",
    "input_camera_info_topic",
    "input_points_topic",
)


@pytest.fixture(scope="module")
def real_profile():
    path = REPO / "src" / "cs625_bringup" / "config" / "real.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


@pytest.fixture(scope="module")
def common_profile():
    path = REPO / "src" / "cs625_bringup" / "config" / "common.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


@pytest.fixture(scope="module")
def camera_mode():
    # camera_mode resolves the device address at import time; give it a value so
    # the test does not depend on this machine's ~/.cs625_local.env.
    os.environ.setdefault("CS625_CAMERA_IP", "127.0.0.1")
    spec = importlib.util.spec_from_file_location(
        "camera_mode_under_test", SCRIPTS / "camera_mode.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["camera_mode_under_test"] = module
    spec.loader.exec_module(module)
    return module


def test_every_real_input_topic_is_set(real_profile):
    for key in INPUT_KEYS:
        value = str(real_profile.get(key, "")).strip()
        assert value, f"{key} is empty; the adapter will refuse to configure"
        assert value.startswith("/camera/"), f"{key}={value!r} is not a Percipio topic"


def test_real_inputs_use_the_measured_driver_names(real_profile):
    assert real_profile["input_color_topic"] == "/camera/color/image_raw"
    assert real_profile["input_depth_topic"] == "/camera/depth/image_raw"
    assert real_profile["input_camera_info_topic"] == "/camera/depth/camera_info"
    # Measured on the cell: the driver publishes this name only when
    # point_cloud_enable:=true.
    assert real_profile["input_points_topic"] == "/camera/depth_registered/points"


def test_normalized_outputs_come_from_common_profile(common_profile):
    assert common_profile["output_color_topic"] == "/sensors/camera/color/image"
    assert common_profile["output_depth_topic"] == "/sensors/camera/depth/image"
    assert common_profile["output_camera_info_topic"] == "/sensors/camera/depth/camera_info"
    assert common_profile["output_points_topic"] == "/sensors/camera/points"
    assert common_profile["status_topic"] == "/sensors/camera/status"


def test_modeling_mode_enables_the_point_cloud(camera_mode):
    assert camera_mode.MODELING_SPEC.get("point_cloud") is True
    command = camera_mode.build_launch_command(camera_mode.MODELING_SPEC)
    assert "point_cloud_enable:=true" in command
    assert "point_cloud_qos:=default" in command


def test_fast_modes_do_not_ask_for_a_cloud(camera_mode):
    for candidate in camera_mode.FAST_CANDIDATES:
        assert candidate.get("point_cloud") is False
        command = camera_mode.build_launch_command(candidate)
        assert "point_cloud_enable:=false" in command


def test_freshness_window_covers_the_measured_frame_rate(real_profile):
    """The cell runs at 0.8-1.0 fps; a 1 s window made the status flap."""
    assert float(real_profile.get("freshness_timeout_sec", 0.0)) >= 2.0


def test_camera_launch_matcher_ignores_the_bash_wrapper(tmp_path):
    """A shell wrapper and the launcher both contain 'ros2 launch ...'; only the
    launcher (comm 'ros2'/'python*') may be signalled."""
    source = (SCRIPTS / "camera_mode.py").read_text(encoding="utf-8")
    assert 'comm == "ros2" or comm.startswith("python")' in source
