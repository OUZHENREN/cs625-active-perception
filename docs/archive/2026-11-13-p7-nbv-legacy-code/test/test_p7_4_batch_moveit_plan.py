from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = REPOSITORY_ROOT / "test" / "p7_4_batch_moveit_plan.py"
    spec = importlib.util.spec_from_file_location("p7_4_batch_moveit_plan", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r13_strict_candidates_and_offline_gate_helpers():
    module = load_module()
    screen_path = (
        REPOSITORY_ROOT
        / "docs"
        / "evidence"
        / "p7_3_geometry_nbv_20260904_r13_after_chroma"
        / "static_screen_all.json"
    )
    screen = json.loads(screen_path.read_text(encoding="utf-8"))
    candidates = module.strict_candidates(screen, 25)
    assert len(candidates) == 25
    assert candidates[0]["candidate_id"] == "geometry_nbv_e75_v-110_a22"

    joint_minimum = dict(zip(module.JOINT_NAMES, module.DEFAULT_JOINT_MIN))
    joint_maximum = dict(zip(module.JOINT_NAMES, module.DEFAULT_JOINT_MAX))
    for candidate in candidates:
        assert module.fov_gate(candidate)["passed"] is True
        position = candidate["geometry_candidate"]["candidate_pose_world_from_camera"][
            "translation_m"
        ]
        assert module.workspace_gate(
            position, module.DEFAULT_WORKSPACE_MIN, module.DEFAULT_WORKSPACE_MAX
        )["passed"] is True
        assert module.joint_limit_gate(
            candidate["static_fk"]["joint_positions_rad"],
            joint_minimum,
            joint_maximum,
        )["passed"] is True

    rotation = candidates[0]["geometry_candidate"]["candidate_pose_world_from_camera"][
        "rotation_matrix"
    ]
    quaternion = module.quaternion_from_rotation_matrix(rotation)
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)

    start = {name: 0.0 for name in module.JOINT_NAMES}
    first = [0.0] * 6
    first[0] = 1.0
    second = list(first)
    second[1] = 2.0
    points = [SimpleNamespace(positions=first), SimpleNamespace(positions=second)]
    assert module.joint_path_length(start, list(module.JOINT_NAMES), points) == pytest.approx(3.0)
