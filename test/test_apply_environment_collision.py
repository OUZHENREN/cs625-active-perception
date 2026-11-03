"""Pure-logic tests for scripts/apply_environment_collision.py.

No ROS, no motion: only the AABB construction, padding, intersection and the
box filtering with subtraction regions are exercised (2026-11-02, environment
collision integration专项).
"""

from __future__ import annotations

import importlib.util
import pathlib

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location(
        "apply_environment_collision", REPO / "scripts" / "apply_environment_collision.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_region_aabb_identity_pose_and_xy_padding(module):
    lo, hi = module.region_aabb([0.0, 0.0, 0.0], [0.1, 0.2, 0.3],
                                [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0],
                                pad_xy=0.015)
    np.testing.assert_allclose(lo, [1.0 - 0.015, 2.0 - 0.015, 3.0])
    np.testing.assert_allclose(hi, [1.1 + 0.015, 2.2 + 0.015, 3.3])


def test_region_aabb_rotated_quaternion(module):
    # 90 deg about z: a bbox spanning x in [0, 0.1] maps onto y in [0, 0.1]
    lo, hi = module.region_aabb([0.0, 0.0, 0.0], [0.1, 0.0, 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)],
                                pad_xy=0.0)
    np.testing.assert_allclose(lo, [0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(hi, [0.0, 0.1, 0.0], atol=1e-6)


def test_boxes_intersect(module):
    lo = np.array([0.0, 0.0, 0.0]); hi = np.array([1.0, 1.0, 1.0])
    assert module.boxes_intersect(lo, hi, np.array([0.5] * 3), np.array([1.5] * 3))
    assert not module.boxes_intersect(lo, hi, np.array([1.0] * 3), np.array([2.0] * 3))
    assert not module.boxes_intersect(lo, hi, np.array([2.0] * 3), np.array([3.0] * 3))


def test_filter_boxes_drops_intersecting_region(module):
    boxes = [
        {"name": "keep_1", "center": [1.0, 1.0, -0.1], "size": [0.02] * 3},
        {"name": "drop_1", "center": [0.1, 0.1, -0.1], "size": [0.02] * 3},
        {"name": "keep_2", "center": [2.0, 2.0, -0.1], "size": [0.02] * 3},
    ]
    region = np.array([0.0, 0.0, -1.0]), np.array([0.2, 0.2, 1.0])
    kept, removed = module.filter_boxes(boxes, [("fixture", *region)])
    assert [b["name"] for b in kept] == ["keep_1", "keep_2"]
    assert removed == {"fixture": 1}


def test_filter_boxes_counts_per_region(module):
    boxes = [
        {"name": "a", "center": [0.1, 0.1, 0.0], "size": [0.02] * 3},
        {"name": "b", "center": [0.5, 0.1, 0.0], "size": [0.02] * 3},
    ]
    regions = [
        ("fixture", np.array([0.0, 0.0, -1.0]), np.array([0.2, 0.2, 1.0])),
        ("workpiece", np.array([0.4, 0.0, -1.0]), np.array([0.6, 0.2, 1.0])),
    ]
    kept, removed = module.filter_boxes(boxes, regions)
    assert kept == []
    assert removed == {"fixture": 1, "workpiece": 1}
