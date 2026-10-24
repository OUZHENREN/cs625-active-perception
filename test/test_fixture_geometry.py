#!/usr/bin/env python3
"""Unit tests for the fixture-geometry tooling (pure functions, no ROS).

Covered:
- STL parsing (ASCII and binary) and bbox extraction;
- PLY header/body offset and xyz extraction;
- mm -> m conversion;
- the slot-width probe (`through_gap`) on a synthetic channel;
- the point-cloud wall gap and its noise rejection;
- insertion-line sampling and profile summarising;
- the RVS placement rule (model origin at slot_origin, yaw from the quaternion);
- the committed collision mesh: metre-scale, <= 80k faces, bbox as documented;
- the tool-tip -> flange conversion used by the clearance verifier.

Run from the repository root:

    python3 -m pytest test/test_fixture_geometry.py -q
"""

from __future__ import annotations

import importlib.util
import pathlib
import struct
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def _load(name: str, relative: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


analyze = _load("analyze_fixture_geometry", "scripts/analyze_fixture_geometry.py")
builder = _load("build_fixture_collision_mesh", "scripts/build_fixture_collision_mesh.py")
verify = _load("verify_fixture_clearance", "scripts/verify_fixture_clearance.py")


# ---------------------------------------------------------------------------
# unit conversion / bbox
# ---------------------------------------------------------------------------


def test_unit_scale_mm_to_m_is_exactly_one_thousandth():
    assert analyze.unit_scale("mm") == pytest.approx(0.001, abs=0.0)
    assert analyze.unit_scale("m") == pytest.approx(1.0, abs=0.0)
    with pytest.raises(ValueError):
        analyze.unit_scale("cm")


def test_mm_to_m_scales_every_axis():
    points = np.array([[1000.0, 0.0, -500.0], [0.0, 250.0, 0.0]])
    converted = analyze.mm_to_m(points)
    assert converted.tolist() == [[1.0, 0.0, -0.5], [0.0, 0.25, 0.0]]


def test_bbox_and_size_on_a_box_corner_set():
    points = np.array([[-1.0, 2.0, 3.0], [4.0, -5.0, 6.0]])
    low, high = analyze.bbox(points)
    assert low.tolist() == [-1.0, -5.0, 3.0]
    assert high.tolist() == [4.0, 2.0, 6.0]
    assert analyze.bbox_size(points).tolist() == [5.0, 7.0, 3.0]
    with pytest.raises(ValueError):
        analyze.bbox(np.zeros((0, 3)))


# ---------------------------------------------------------------------------
# STL parsing
# ---------------------------------------------------------------------------


def _ascii_stl_bytes(triangles) -> bytes:
    lines = ["solid test"]
    for triangle in triangles:
        lines.append("  facet normal 0 0 1")
        lines.append("    outer loop")
        for vertex in triangle:
            lines.append("      vertex {:.9e} {:.9e} {:.9e}".format(*vertex))
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid test")
    return "\n".join(lines).encode("ascii")


def test_ascii_stl_is_detected_and_parsed():
    triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                          [[0, 0, 0], [0, 1, 0], [0, 0, 1]]], dtype=float)
    payload = _ascii_stl_bytes(triangles)
    assert analyze.is_ascii_stl(payload) is True
    parsed = analyze.parse_ascii_stl(payload)
    assert parsed.shape == (2, 3, 3)
    assert np.allclose(parsed, triangles, atol=1e-9)


def test_binary_stl_roundtrip_preserves_triangles_and_bbox(tmp_path):
    triangles = np.array([
        [[-0.2, -0.5, -0.1], [0.3, -0.5, -0.1], [0.3, 0.6, 0.4]],
        [[-0.2, -0.5, -0.1], [0.3, 0.6, 0.4], [-0.2, 0.6, 0.4]],
    ])
    output = tmp_path / "mesh.stl"
    written = builder.write_binary_stl(output, triangles)
    assert written == 2
    assert output.stat().st_size == 84 + 50 * 2

    payload = output.read_bytes()
    assert analyze.is_ascii_stl(payload) is False
    parsed = analyze.parse_binary_stl(payload)
    assert parsed.shape == (2, 3, 3)
    assert np.allclose(parsed, triangles, atol=1e-6)
    assert np.allclose(analyze.bbox_size(parsed), analyze.bbox_size(triangles), atol=1e-6)


def test_binary_stl_rejects_truncated_payload():
    payload = b"\0" * 80 + struct.pack("<I", 3) + b"\0" * 10
    with pytest.raises(ValueError, match="expected"):
        analyze.parse_binary_stl(payload)


# ---------------------------------------------------------------------------
# PLY parsing
# ---------------------------------------------------------------------------


def _minimal_ply(tmp_path: pathlib.Path, points: np.ndarray) -> pathlib.Path:
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
    ).encode("ascii")
    body = np.asarray(points, dtype="<f4").tobytes()
    path = tmp_path / "points.ply"
    path.write_bytes(header + body)
    return path


def test_ply_body_offset_skips_the_header_newline(tmp_path):
    points = np.array([[1.0, 2.0, 3.0], [-4.0, 5.5, -6.25]])
    path = _minimal_ply(tmp_path, points)
    parsed = analyze.read_ply_points(path)
    assert parsed.shape == (2, 3)
    assert np.allclose(parsed, points, atol=1e-6)


def test_ply_header_offset_points_at_first_vertex(tmp_path):
    points = np.array([[0.5, 0.0, 0.0]])
    path = _minimal_ply(tmp_path, points)
    payload = path.read_bytes()
    header, offset = analyze.parse_ply_header(payload)
    assert header["elements"][0]["name"] == "vertex"
    assert header["elements"][0]["count"] == 1
    assert payload[offset:offset + 12] == np.asarray(points, dtype="<f4").tobytes()


# ---------------------------------------------------------------------------
# slot width
# ---------------------------------------------------------------------------


def _channel_triangles(half_width: float = 0.2, extent: float = 0.5) -> np.ndarray:
    """两片平行于 x-z 的墙，分别在 y=±half_width（形成一个沿 y 的贯通槽）。"""

    triangles = []
    for sign in (+1.0, -1.0):
        y = sign * half_width
        corners = np.array([
            [-extent, y, -extent], [extent, y, -extent], [extent, y, extent],
            [-extent, y, -extent], [extent, y, extent], [-extent, y, extent],
        ])
        triangles.extend(corners.reshape(-1, 3, 3))
    return np.asarray(triangles)


def test_through_gap_measures_wall_to_wall_distance():
    triangles = _channel_triangles(0.2)
    gap = analyze.through_gap(triangles, [0.0, 0.0, 0.0], (0.0, 1.0, 0.0))
    assert gap == pytest.approx(0.4, abs=1e-9)


def test_through_gap_returns_none_when_one_side_is_open():
    triangles = _channel_triangles(0.2)
    # 探针在 +y 墙外侧：+y 方向没有墙
    gap = analyze.through_gap(triangles, [0.0, 0.3, 0.0], (0.0, 1.0, 0.0))
    assert gap is None


def test_ray_first_hit_respects_max_distance():
    triangles = _channel_triangles(0.2)
    assert analyze.ray_first_hit(triangles, [0.0, 0.0, 0.0], (0, 1, 0), t_max=0.1) is None
    assert analyze.ray_first_hit(triangles, [0.0, 0.0, 0.0], (0, 1, 0), t_max=0.3) == pytest.approx(0.2)


def test_histogram_wall_gap_ignores_sparse_flying_pixels():
    rng = np.random.default_rng(3)
    values = np.concatenate([
        rng.normal(0.20, 0.0004, 500),
        rng.normal(-0.20, 0.0004, 500),
        np.array([0.05]),  # 单点飞点，必须被 min_count 过滤
    ])
    gap, near, far = analyze.histogram_wall_gap(values, 0.0, bin_size=0.001, min_count=3)
    assert gap == pytest.approx(0.4, abs=0.002)
    assert near == pytest.approx(0.2, abs=0.002)
    assert far == pytest.approx(0.2, abs=0.002)


def test_cloud_through_gap_on_two_planes():
    rng = np.random.default_rng(4)
    x = rng.uniform(-0.03, 0.03, 4000)
    z = rng.uniform(-0.003, 0.003, 4000)
    upper = np.stack([x, np.full_like(x, 0.2), z], axis=1)
    lower = np.stack([x, np.full_like(x, -0.2), z], axis=1)
    points = np.vstack([upper, lower])
    gap, near, far = analyze.cloud_through_gap(points, [0.0, 0.0, 0.0], axis="y")
    assert gap == pytest.approx(0.4, abs=0.002)
    assert near == pytest.approx(0.2, abs=0.002)
    assert far == pytest.approx(0.2, abs=0.002)


def test_insertion_line_spans_pre_insert_to_insert():
    line = analyze.insertion_line([0.0, 0.0, 0.4], [0.0, 0.0, 0.0], levels=5)
    assert len(line) == 5
    assert line[0][1].tolist() == [0.0, 0.0, 0.4]
    assert line[-1][1].tolist() == [0.0, 0.0, 0.0]
    assert [round(item[0], 3) for item in line] == [0.4, 0.3, 0.2, 0.1, 0.0]


def test_summarise_profile_handles_missing_layers():
    profile = [{"z": 0.3, "gap": 0.45}, {"z": 0.2, "gap": None}, {"z": 0.1, "gap": 0.47}]
    summary = analyze.summarise_profile(profile)
    assert summary["valid"] == 2
    assert summary["levels"] == 3
    assert summary["median_gap"] == pytest.approx(0.46)
    assert summary["min_gap"] == pytest.approx(0.45)


# ---------------------------------------------------------------------------
# placement / tool frame
# ---------------------------------------------------------------------------


def test_rvs_placement_puts_model_origin_on_slot_origin():
    common = {
        "slot_origin": [1.241629, 0.181709, 0.443245],
        "slot_origin_q": [-0.0016287871280747132, 0.004253108458370272,
                          0.003384045809911108, 0.9999839030478886],
    }
    placement = analyze.rvs_placement(common)
    assert placement["xyz"] == pytest.approx(common["slot_origin"], abs=1e-12)
    assert placement["rule"] == "model_origin_on_slot_origin"
    assert abs(placement["yaw"]) < 0.01  # 标定四元数基本是单位四元数
    # yaw 矩阵与完整的 slot_origin_q 旋转在同一量级（差 < 0.6°）
    full = analyze.quat_xyzw_to_matrix(common["slot_origin_q"])
    delta = placement["rot"].T @ full
    angle = np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1)))
    assert angle < 0.6


def test_flange_position_moves_along_the_tool_axis():
    """夹爪尖 -> 法兰：沿工具 +Z 退 0.207 m（任务位姿是夹爪尖，MoveIt 的 EEF 是法兰）。"""

    tip = [1.0996, -0.0125, 0.3773]
    quat = [-0.004, 0.997, -0.001, 0.083]
    flange = verify.flange_position(tip, quat, 0.207)
    rot = verify.quat_to_matrix(quat)
    expected = np.asarray(tip) - rot @ np.array([0.0, 0.0, 0.207])
    assert flange == pytest.approx(expected.tolist(), abs=1e-12)
    # 这个姿态下工具 +Z 基本朝下，所以法兰应该比夹爪尖高 ~0.204 m
    assert flange[2] - tip[2] == pytest.approx(0.204, abs=0.005)


def test_flange_position_is_identity_for_zero_length():
    assert verify.flange_position([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 0.0) == [1.0, 2.0, 3.0]


# ---------------------------------------------------------------------------
# committed artefacts
# ---------------------------------------------------------------------------


COLLISION_MESH = (ROOT / "src" / "cs625_simulation" / "assets" / "cs625_real_cell"
                  / "slot_fixture" / "meshes" / "slot_fixture_collision.stl")


@pytest.mark.skipif(not COLLISION_MESH.is_file(), reason="collision mesh not generated yet")
def test_committed_collision_mesh_is_metre_scale_and_within_face_limit():
    triangles, fmt = analyze.read_stl_triangles(COLLISION_MESH)
    assert fmt == "binary"
    assert len(triangles) <= builder.MAX_FACES
    size = analyze.bbox_size(triangles.reshape(-1, 3))
    # RVS 网格：0.2000 x 0.7450 x 0.5077 m（与真机实测一致）
    assert size == pytest.approx([0.2000, 0.7450, 0.5077], abs=2e-3)


@pytest.mark.skipif(not COLLISION_MESH.is_file(), reason="collision mesh not generated yet")
def test_committed_collision_mesh_is_under_twenty_megabytes():
    assert COLLISION_MESH.stat().st_size < 20 * 1024 * 1024
