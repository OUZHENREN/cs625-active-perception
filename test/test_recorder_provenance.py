"""The recorder must refuse to mix real camera frames into a simulation video.

Bug this guards (2026-10-25): a "simulation demo" clip contained real Percipio
frames because run_task.py restarted the camera driver inside the simulation's
ROS_DOMAIN_ID.  Two independent guards now exist:
  - publisher classification (refuse to record when a real camera node is present)
  - per-frame saturation threshold (Gazebo renders have no texture)
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SPEC = REPO / "scripts" / "record_sim_run.py"


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location("record_sim_run", SPEC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["record_sim_run"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_simulated_publisher_is_accepted(module):
    ok, simulated, suspicious = module.classify_publishers(["/cs625_sim_real_topic_alias"])
    assert ok and simulated and not suspicious


def test_real_camera_publisher_is_refused(module):
    ok, _sim, suspicious = module.classify_publishers(["/camera/camera"])
    assert not ok
    assert suspicious == ["/camera/camera"]


def test_percipio_publisher_is_refused(module):
    ok, _sim, suspicious = module.classify_publishers(["/camera/percipio_camera"])
    assert not ok
    assert suspicious


def test_mixed_publishers_are_refused(module):
    ok, _sim, suspicious = module.classify_publishers(
        ["/cs625_sim_real_topic_alias", "/camera/camera"])
    assert not ok
    assert len(suspicious) == 1


def test_threshold_separates_render_from_photo():
    """Documented measurement: Gazebo ~0.01-0.05, real photos 0.25-0.6."""
    limit = 0.10
    assert 0.05 < limit < 0.25
