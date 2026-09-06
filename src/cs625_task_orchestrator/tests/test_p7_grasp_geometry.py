import math

import pytest

from cs625_task_orchestrator.p7_grasp_geometry import (
    evaluate_grasp_geometry,
    target_pose_relative_to_grasp,
)


def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
    }


def test_geometry_uses_model_local_center_not_model_origin():
    result = evaluate_grasp_geometry(
        _pose(0.72, 0.0, 0.0),
        _pose(0.71, 0.084, 0.051),
        (-0.01, 0.084, 0.051),
    )
    assert result["target_to_grasp_center_error_m"] == pytest.approx(0.0)
    assert result["geometry_pass"]


def test_geometry_rotates_local_center_with_model_pose():
    result = evaluate_grasp_geometry(
        _pose(1.0, 2.0, 3.0, qz=math.sin(math.pi / 4), qw=math.cos(math.pi / 4)),
        _pose(1.0, 3.0, 3.0),
        (1.0, 0.0, 0.0),
    )
    assert result["target_center_world_m"] == pytest.approx([1.0, 3.0, 3.0])
    assert result["geometry_pass"]


def test_geometry_rejects_non_finite_input():
    with pytest.raises(ValueError, match="finite"):
        evaluate_grasp_geometry(
            _pose(float("nan"), 0.0, 0.0), _pose(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )


def test_relative_target_pose_preserves_exact_identity_alignment():
    relative = target_pose_relative_to_grasp(
        _pose(0.72, 0.0, 0.0),
        _pose(0.71, 0.084, 0.051),
        (-0.01, 0.084, 0.051),
    )
    assert list(relative["position"].values()) == pytest.approx([0.0, 0.0, 0.0])
    assert relative["orientation"] == pytest.approx(
        {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    )
