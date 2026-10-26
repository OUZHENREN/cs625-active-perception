"""Tests for scripts/publish_environment_point_cloud.py.

The point-cloud viewer is the only place that reads a PLY produced by
``scripts/model_environment.py``, so these tests pin the two things that would
silently corrupt a scene review: the PLY header walking (binary and ascii) and
the PointCloud2 layout (width, point_step, field offsets).  A wrong point_step
or a missing ``z`` field would still publish a message, and RViz would then show
nothing without any error, which is exactly the failure mode that is hard to
notice by eye.

The module imports ``rclpy``, so the whole file is skipped when ROS is not
sourced instead of failing the offline suite.
"""

from __future__ import annotations

import importlib.util
import pathlib
import struct

import numpy as np
import pytest

pytest.importorskip("rclpy")

REPO = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "publish_environment_point_cloud.py"
spec = importlib.util.spec_from_file_location("publish_environment_point_cloud", SCRIPT)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


BINARY_HEADER = """ply
format binary_little_endian 1.0
comment Created by Open3D
element vertex {count}
property double x
property double y
property double z
end_header
"""

ASCII_PLY = """ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
end_header
0 0 0
1 2 3
-1 -2 -3
"""


def write_binary_ply(path: pathlib.Path, points: np.ndarray) -> pathlib.Path:
    """Write the same header Open3D emits for scene.ply (double xyz, binary)."""
    body = b"".join(struct.pack("<ddd", *map(float, point)) for point in points)
    path.write_bytes(BINARY_HEADER.format(count=len(points)).encode("ascii") + body)
    return path


def test_binary_ply_round_trips_xyz(tmp_path):
    source = np.array([[0.1, -0.2, 0.3], [1.5, 2.5, -0.5], [0.0, 0.0, 0.0]])
    ply = write_binary_ply(tmp_path / "scene.ply", source)

    points = viewer.read_ply_xyz(ply)

    assert points.shape == (3, 3)
    assert np.allclose(points, source)


def test_ascii_ply_is_accepted(tmp_path):
    ply = tmp_path / "ascii.ply"
    ply.write_text(ASCII_PLY, encoding="ascii")

    points = viewer.read_ply_xyz(ply)

    assert np.allclose(points, [[0, 0, 0], [1, 2, 3], [-1, -2, -3]])


def test_non_finite_points_are_dropped(tmp_path):
    source = np.array([[0.0, 0.0, 0.0], [np.nan, 0.0, 0.0], [0.0, np.inf, 0.0]])
    ply = write_binary_ply(tmp_path / "scene.ply", source)

    points = viewer.read_ply_xyz(ply)

    assert points.shape == (1, 3)


def test_ply_without_vertex_element_is_an_error(tmp_path):
    ply = tmp_path / "broken.ply"
    ply.write_text(
        "ply\nformat ascii 1.0\nelement face 1\nproperty int a\nend_header\n0\n",
        encoding="ascii",
    )

    with pytest.raises(SystemExit):
        viewer.read_ply_xyz(ply)


def test_voxel_downsample_keeps_about_one_point_per_cell():
    grid = np.array([[x, y, z] for x in range(4) for y in range(4) for z in range(4)], dtype=float)
    grid *= 0.01  # 1 cm spacing

    points = viewer.voxel_downsample(grid, 0.02)

    # 4x4x4 points on a 1 cm lattice, 2 cm cells -> at most 2x2x2 cells
    assert 0 < points.shape[0] <= 8
    assert viewer.voxel_downsample(grid, 0.0).shape[0] == grid.shape[0]


def test_cloud_message_layout_matches_rviz_expectations():
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    message = viewer.build_cloud_message(points, "base_link", viewer.Time())

    assert message.header.frame_id == "base_link"
    assert message.width == 2 and message.height == 1
    assert message.point_step == 12
    assert message.row_step == 24
    assert [field.name for field in message.fields] == ["x", "y", "z"]
    assert [field.offset for field in message.fields] == [0, 4, 8]
    assert all(field.datatype == field.FLOAT32 for field in message.fields)
    assert len(message.data) == 24
    assert np.allclose(np.frombuffer(message.data, dtype="<f4").reshape(2, 3), points)


def test_resolve_scene_accepts_directory_and_file(tmp_path):
    directory = tmp_path / "20261025_070113"
    directory.mkdir()
    ply = write_binary_ply(directory / "scene.ply", np.zeros((1, 3)))

    assert viewer.resolve_scene(directory) == ply
    assert viewer.resolve_scene(ply) == ply


def test_resolve_scene_reports_missing_run(tmp_path):
    with pytest.raises(SystemExit):
        viewer.resolve_scene(tmp_path / "does_not_exist")


def test_rviz_profile_matches_the_published_topic():
    """The shipped profile must subscribe to the topic the script publishes."""
    import yaml

    profile = yaml.safe_load(viewer.DEFAULT_RVIZ_CONFIG.read_text(encoding="utf-8"))
    manager = profile["Visualization Manager"]
    clouds = [
        display
        for display in manager["Displays"]
        if display["Class"].endswith("PointCloud2")
    ]

    assert len(clouds) == 1
    assert clouds[0]["Topic"]["Value"] == viewer.DEFAULT_TOPIC
    assert clouds[0]["Topic"]["Reliability Policy"] == "Reliable"
    assert manager["Global Options"]["Fixed Frame"] == viewer.DEFAULT_FRAME
