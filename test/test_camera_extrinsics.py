"""Regression tests for the RVS hand-eye extrinsic derivation.

The real camera calibration is external data the repository cannot re-derive, so
these tests pin the parsing, the unit cross-check and the frame algebra rather
than the numbers themselves.  The one physical claim they do assert is that the
calibrated optical axis points along the flange's +Z, because that is what makes
the "CameraToRobotTCP" frame an optical frame and not a body frame; if that ever
stops holding, the derivation is reading the transform the wrong way round.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts/camera_extrinsics.py"
spec = importlib.util.spec_from_file_location("camera_extrinsics", SCRIPT)
camera_extrinsics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera_extrinsics)


INI = """\
[General]
Eye%20In%20Hand=true

[colorToRobot]
x=96.95
y=42.212
z=97.54
rx=179.96
ry=179.215
rz=89.262

[depthToRobot]
x=96.754
y=17.803
z=97.328
rx=-179.861
ry=179.987
rz=89.451
"""

TXT = {
    "ColorToRobotTCP.txt": "0.09695 0.042212 0.09754 3.14089 3.12789 1.55792",
    "DepthToRobotTCP.txt": "0.096754 0.017803 0.097328 -3.13917 3.14137 1.56121",
}


@pytest.fixture()
def rvs_tree(tmp_path):
    (tmp_path / "HandEyeTool.ini").write_text(INI, encoding="utf-8")
    for name, text in TXT.items():
        (tmp_path / name).write_text(text, encoding="utf-8")
    return tmp_path


def test_units_are_millimetres_and_degrees_then_metres_and_radians(rvs_tree):
    ini = camera_extrinsics.calibrate_from_ini(rvs_tree / "HandEyeTool.ini")
    txt = camera_extrinsics.calibrate_from_txt(rvs_tree)
    camera_extrinsics.cross_check(ini, txt)

    assert ini["color"][0:3] == pytest.approx([0.09695, 0.042212, 0.09754])
    assert ini["color"][3] == pytest.approx(math.radians(179.96))
    assert txt["depth"][3:6] == pytest.approx(
        [-3.13917, 3.14137, 1.56121], abs=1e-9
    )


def test_a_unit_mismatch_between_the_two_encodings_aborts(rvs_tree):
    ini = camera_extrinsics.calibrate_from_ini(rvs_tree / "HandEyeTool.ini")
    txt = camera_extrinsics.calibrate_from_txt(rvs_tree)
    txt["color"][0] *= 1000.0  # as if the .txt had been written in millimetres
    with pytest.raises(SystemExit):
        camera_extrinsics.cross_check(ini, txt)


def test_derived_body_frame_reproduces_the_depth_optical_frame(rvs_tree):
    """camera_link anchors on the depth eye, so that is the identity that must hold.

    Gazebo's rgbd_camera hangs off camera_link and looks along that link's +X, and
    the point-cloud contract names camera_depth_optical_frame, so anchoring on the
    depth eye is what makes the simulated sensor and the declared optical frame
    coincide.  The colour eye then carries the stereo offset.
    """

    values = camera_extrinsics.load(rvs_tree)
    body = np.eye(4)
    body[:3, :3] = camera_extrinsics.rotation(values["camera_mount_rpy"])
    body[:3, 3] = values["camera_mount_xyz"]
    optical = np.eye(4)
    optical[:3, :3] = body[:3, :3] @ camera_extrinsics.rotation(
        camera_extrinsics.BODY_TO_OPTICAL_RPY
    )
    optical[:3, 3] = body[:3, 3]
    assert camera_extrinsics.to_pose(optical) == pytest.approx(
        values["depth_optical_in_flange"], abs=1e-12
    )
    # The body's +X, which is where Gazebo points the sensor, must be the depth
    # eye's optical +Z.
    assert (body[:3, :3] @ [1.0, 0.0, 0.0]) == pytest.approx(
        camera_extrinsics.rotation(values["depth_optical_in_flange"][3:6]) @ [0.0, 0.0, 1.0],
        abs=1e-12,
    )


def test_optical_axis_runs_along_the_tool_axis(rvs_tree):
    """The claim that decides optical frame versus body frame."""

    values = camera_extrinsics.load(rvs_tree)
    for eye in ("color_optical_in_flange", "depth_optical_in_flange"):
        pose = values[eye]
        axis = camera_extrinsics.rotation(pose[3:6]) @ [0.0, 0.0, 1.0]
        assert axis[2] > 0.99, f"{eye} does not look along the flange +Z: {axis}"
    # And the two eyes must be a stereo pair: close together, nearly parallel.
    # They are not perfectly parallel in the real calibration -- 0.79 degrees
    # apart -- which is why the comparison is on the angle, not on the matrices.
    assert values["baseline_m"] == pytest.approx(0.024411, abs=1e-6)
    colour = camera_extrinsics.rotation(values["color_optical_in_flange"][3:6])
    depth = camera_extrinsics.rotation(values["depth_optical_in_flange"][3:6])
    angle = math.degrees(
        math.acos(max(-1.0, min(1.0, float((colour @ [0, 0, 1]) @ (depth @ [0, 0, 1])))))
    )
    assert angle < 2.0, f"stereo eyes are {angle:.2f} degrees apart, not a rigid pair"


def test_camera_sits_near_the_tool_not_past_it(rvs_tree):
    """A sanity bound that rejects reading the transform the other way round.

    Applying the controller's (-46, 0, 353) mm TCP offset as well would place the
    camera at 450 mm, which is 243 mm past the real gripper's 207 mm tip.  The
    tool owner measured roughly 98 mm from the flange, so the raw numbers are the
    flange-frame pose and nothing else may be composed on top.
    """

    values = camera_extrinsics.load(rvs_tree)
    along_tool = values["camera_mount_xyz"][2]
    assert 0.05 < along_tool < 0.30, (
        "camera is not near the tool; check whether a TCP offset was applied"
    )
    # camera_link is the depth eye, so the depth X is the one at the flange frame.
    assert values["camera_mount_xyz"][0] == pytest.approx(0.096754)
    # The colour eye is the mirrored one, 24.4 mm away along +Y.
    assert values["camera_color_xyz"] == pytest.approx(
        [0.000207, 0.024410, 0.000037], abs=1e-6
    )


def test_check_mode_detects_a_drifted_config(rvs_tree, monkeypatch, tmp_path, capsys):
    config = tmp_path / "camera_extrinsics_sim.yaml"
    monkeypatch.setattr(camera_extrinsics, "CONFIG", config)

    monkeypatch.setattr("sys.argv", ["camera_extrinsics.py", "--rvs-dir", str(rvs_tree)])
    assert camera_extrinsics.main() == 0
    assert config.is_file()

    monkeypatch.setattr(
        "sys.argv", ["camera_extrinsics.py", "--rvs-dir", str(rvs_tree), "--check"]
    )
    assert camera_extrinsics.main() == 0

    config.write_text(
        config.read_text(encoding="utf-8").replace("0.09695", "0.19695"),
        encoding="utf-8",
    )
    assert camera_extrinsics.main() == 1


def test_missing_rvs_directory_is_reported_not_guessed(monkeypatch, tmp_path, capsys):
    monkeypatch.delenv("CS625_RVS_RUNTIME_DIR", raising=False)
    monkeypatch.setattr("sys.argv", ["camera_extrinsics.py"])
    assert camera_extrinsics.main() == 2
    assert "CS625_RVS_RUNTIME_DIR" in capsys.readouterr().err
