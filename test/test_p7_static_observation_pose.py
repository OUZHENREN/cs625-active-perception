"""Schema-level checks for the static P7 observation-pose solver."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_solve_static_observation_pose import (  # noqa: E402
    TARGET_LOCAL_CENTER_M,
    target_center_from_pose_record,
)


def test_p7_2_pose_is_already_a_cad_centre():
    record = {
        "pose": {
            "position_m": [0.695, 0.078, 0.051],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    }
    assert np.allclose(target_center_from_pose_record(record), (0.695, 0.078, 0.051))


def test_gazebo_model_origin_receives_ycb_local_centre_offset():
    record = {
        "pose": {
            "position": {"x": 0.72, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
    }
    assert np.allclose(
        target_center_from_pose_record(record),
        np.array((0.72, 0.0, 0.0)) + TARGET_LOCAL_CENTER_M,
    )
