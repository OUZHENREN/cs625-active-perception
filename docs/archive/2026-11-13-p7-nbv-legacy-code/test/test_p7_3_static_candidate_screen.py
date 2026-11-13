"""Unit tests for the P7.3 offline candidate-screen decision boundary."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_3_screen_static_candidates import preflight_decision, solver_command  # noqa: E402


def test_preflight_accepts_only_converged_low_residual_safe_in_frame_result():
    result = {
        "solver_converged": True,
        "max_abs_scaled_residual": 0.01,
        "minimum_distal_link_origin_z_m": 0.121,
        "projection": {"inside_image": True},
    }
    assert preflight_decision(
        result, maximum_scaled_residual=0.02, minimum_link_origin_z_m=0.12
    ) == []


def test_preflight_reports_each_independent_static_rejection():
    result = {
        "solver_converged": False,
        "max_abs_scaled_residual": 0.03,
        "minimum_distal_link_origin_z_m": 0.119,
        "projection": {"inside_image": False},
    }
    assert preflight_decision(
        result, maximum_scaled_residual=0.02, minimum_link_origin_z_m=0.12
    ) == [
        "STATIC_IK_NOT_CONVERGED",
        "STATIC_IK_RESIDUAL_EXCEEDED",
        "STATIC_FLOOR_CLEARANCE_INSUFFICIENT",
        "STATIC_TARGET_PROJECTION_OUT_OF_FRAME",
    ]


def test_solver_command_uses_fixed_point_for_negative_numerical_zero(tmp_path):
    candidate = {
        "candidate_pose_world_from_camera": {
            "translation_m": [0.1, 0.2, 0.3],
            "rotation_matrix": [[-1.2e-16, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
    class Arguments:
        urdf = tmp_path / "robot.urdf"
        seed_joints = tmp_path / "seed.yaml"
        target_pose = tmp_path / "target.json"
        minimum_link_origin_z_m = 0.12

    command = solver_command(tmp_path / "solver.py", Arguments(), candidate)
    assert "-0.00000000000000012" in command
    assert not any("e-" in token for token in command)
