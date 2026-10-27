"""Pure parts of the multi-view workpiece localizer (no ROS, no Open3D needed)."""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SPEC = REPO / "scripts" / "localize_workpiece_multiview.py"


@pytest.fixture(scope="module")
def loc():
    spec = importlib.util.spec_from_file_location("localize_multiview", SPEC)
    module = importlib.util.module_from_spec(spec)
    sys.modules["localize_multiview"] = module
    spec.loader.exec_module(module)
    return module


def test_centre_from_origin_round_trip(loc):
    origin = np.array([0.5029, 0.6351, 0.1254])
    rotation = np.eye(3)
    centre = loc.centre_from_origin(origin, rotation)
    assert np.allclose(centre - origin, loc.CAD_CENTRE_OFFSET)
    # and back
    assert np.allclose(centre - rotation @ loc.CAD_CENTRE_OFFSET, origin)


def test_quality_gate(loc):
    assert loc.quality_ok(0.5, 0.005, 3.0, 0.3, 0.010, 15.0)
    assert not loc.quality_ok(0.2, 0.005, 3.0, 0.3, 0.010, 15.0)
    assert not loc.quality_ok(0.5, 0.020, 3.0, 0.3, 0.010, 15.0)
    assert not loc.quality_ok(0.5, 0.005, 40.0, 0.3, 0.010, 15.0)


def test_crop_around_keeps_only_the_neighbourhood(loc):
    points = np.array([[0.72, 0.62, 0.10], [1.40, 0.62, 0.10], [0.72, 1.40, 0.10],
                       [0.72, 0.62, -0.20], [0.72, 0.62, 0.60]])
    kept = loc.crop_around(points, [0.72, 0.62], radius=0.42, z_min=-0.10, z_max=0.50)
    assert len(kept) == 1
    assert np.allclose(kept[0], [0.72, 0.62, 0.10])


def test_rotation_difference_detects_a_quarter_turn(loc):
    from scipy.spatial.transform import Rotation
    prior = np.array([0.0, 0.0, 0.0, 1.0])
    assert loc.rotation_difference_deg(np.eye(3), prior) == pytest.approx(0.0, abs=1e-6)
    quarter = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    assert loc.rotation_difference_deg(quarter, prior) == pytest.approx(90.0, abs=1e-6)
