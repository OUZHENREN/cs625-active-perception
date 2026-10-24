"""Regression tests for the real-profile camera pose constant.

The live chain is

    base <- TCP pose (topic) <- T_tcp_cam <- camera_color_optical_frame

and ``config/handeye/tcp_to_camera.json`` is the only place ``T_tcp_cam`` is
stored.  It was once hand-written (camera 50 mm from the TCP, tilted 45 deg),
which put every base-frame target about 1.4 m sideways, so these tests assert
that the stored file is the composition its generator claims it is, and that it
agrees with the simulation extrinsics derived from the same RVS calibration.

No machine-local RVS paths are touched: the committed file is the artefact under
test, exactly as a consumer reads it.
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


ROOT = pathlib.Path(__file__).resolve().parents[1]
HAND_EYE = ROOT / "config" / "handeye" / "tcp_to_camera.json"
SIM_EXTRINSICS = ROOT / "src" / "cs625_bringup" / "config" / "camera_extrinsics_sim.yaml"

# The URDF fixed joint flange -> my_end_effector_link
# (eli_cs_robot_description/urdf/cs625.urdf.xacro).
FLANGE_TO_TCP = np.array([-0.046, 0.0, 0.353])


@pytest.fixture(scope="module")
def chain() -> dict:
    return json.loads(HAND_EYE.read_text(encoding="utf-8"))


def as_matrix(rows) -> np.ndarray:
    return np.array(rows, dtype=float)


def test_tcp_cam_is_the_claimed_composition(chain: dict) -> None:
    """T_tcp_cam must equal inv(T_flange_tcp) @ T_flange_cam, not a guess."""

    tcp_cam = as_matrix(chain["T_tcp_cam"])
    expected = np.linalg.inv(as_matrix(chain["T_flange_tcp"])) @ as_matrix(
        chain["T_flange_cam_color_optical"])
    assert np.allclose(tcp_cam, expected, atol=1e-12), (
        "T_tcp_cam does not match the composition of the stored flange "
        "transforms; the file was probably edited by hand")


def test_flange_tcp_matches_the_urdf_joint(chain: dict) -> None:
    flange_tcp = as_matrix(chain["T_flange_tcp"])
    assert np.allclose(flange_tcp[:3, 3], FLANGE_TO_TCP, atol=1e-12)
    assert np.allclose(flange_tcp[:3, :3], np.eye(3), atol=1e-12), (
        "the flange -> TCP joint is a pure translation in the URDF")


@pytest.mark.parametrize("key", ["T_flange_tcp", "T_flange_cam_color_optical",
                                 "T_tcp_cam"])
def test_every_stored_transform_is_rigid(chain: dict, key: str) -> None:
    matrix = as_matrix(chain[key])
    rotation = matrix[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)
    assert np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)


def test_camera_looks_along_the_tool_axis(chain: dict) -> None:
    """The calibrated optical +Z comes out along the flange +Z.

    This is the physical claim that makes the RVS numbers an optical frame: an
    eye-in-hand camera at the wrist looks down the tool.  A 45 deg "guess" mount
    had the optical axis 45 deg off and fails here.
    """

    flange_cam = as_matrix(chain["T_flange_cam_color_optical"])
    optical_axis = flange_cam[:3, 2]
    assert float(optical_axis[2]) > 0.999, (
        f"calibrated optical axis {np.round(optical_axis, 3).tolist()} is not "
        "along the flange +Z")


def test_camera_sits_about_296_mm_behind_the_tcp(chain: dict) -> None:
    """Distance from TCP to camera; a 'camera almost on the TCP' regression fails."""

    distance = float(np.linalg.norm(as_matrix(chain["T_tcp_cam"])[:3, 3]))
    assert 0.25 < distance < 0.35, (
        f"camera is {distance * 1000:.0f} mm from the TCP; the hand-eye "
        "calibration says roughly 296 mm")


def test_real_chain_agrees_with_the_simulation_extrinsics(chain: dict) -> None:
    """Sim and real both come from the same RVS calibration, so they must agree."""

    yaml = pytest.importorskip("yaml")
    document = yaml.safe_load(SIM_EXTRINSICS.read_text(encoding="utf-8"))
    parameters = document["cs625_camera_extrinsics"]["ros__parameters"]
    sim_color = np.array(parameters["color_optical_in_flange"][:3], dtype=float)
    real_color = as_matrix(chain["T_flange_cam_color_optical"])[:3, 3]
    assert np.allclose(sim_color, real_color, atol=1e-6), (
        "the real-profile constant and the simulation extrinsics were derived "
        "from different calibrations")
