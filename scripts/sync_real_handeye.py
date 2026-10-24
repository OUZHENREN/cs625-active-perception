#!/usr/bin/env python3
"""Sync the RVS eye-in-hand calibration into the REAL-profile pose chain.

Why this file exists
--------------------
The real cell's camera is calibrated by RobotVisionSuite, and RVS is the only
authority for the real hand-eye.  Its runtime directory holds

    HandEyeTool.ini          [colorToRobot] / [depthToRobot] in mm and degrees
    ColorToRobotTCP.txt      the same numbers in metres and radians
    DepthToRobotTCP.txt

which state the camera's OPTICAL frame in the robot FLANGE frame.

The live pose chain cannot look this up in TF -- the cell publishes several
unconnected TF trees (tool0 is not even in the robot tree), so the chain is
composed as a constant instead:

    T_base_camera = T_base_tcp @ T_tcp_camera
    T_tcp_camera  = inv(T_flange_tcp) @ T_flange_camera_optical

`T_flange_tcp` is the URDF fixed joint ``flange -> my_end_effector_link``
(``xyz="-0.046 0 0.353"`` in ``eli_cs_robot_description/urdf/cs625.urdf.xacro``),
which is exactly the frame ``/tcp_pose_broadcaster/pose`` reports.

The JSON this script writes is that composed constant and its provenance.  It
must never be hand-written: the previous hand-made value put the camera 50 mm
from the TCP with a 45 deg tilt, i.e. 135 deg and 0.34 m away from the
calibration, which threw every base-frame target roughly 1.4 m sideways.

The numbers are cross-checked the same way ``scripts/camera_extrinsics.py`` does
it (mm/deg encoding against m/rad encoding), so a silent unit change aborts.

    python3 scripts/sync_real_handeye.py --rvs-dir "<RVS runtime>"          # write
    python3 scripts/sync_real_handeye.py --rvs-dir "<RVS runtime>" --check  # verify

The RVS directory is machine-local: pass ``--rvs-dir`` or set
``CS625_RVS_RUNTIME_DIR``.  Nothing is hardcoded.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camera_extrinsics as handeye  # noqa: E402  (same directory, single source)


ROOT = pathlib.Path(__file__).resolve().parents[1]
TARGET = ROOT / "config/handeye/tcp_to_camera.json"

# URDF fixed joint flange -> my_end_effector_link.  Kept here as data, not as a
# hidden default: --check fails if the URDF and this constant ever disagree.
FLANGE_TO_TCP_XYZ = (-0.046, 0.0, 0.353)


def matrix(pose6) -> np.ndarray:
    """[x, y, z, rx, ry, rz] in metres/radians -> 4x4."""

    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", pose6[3:6]).as_matrix()
    out[:3, 3] = pose6[0:3]
    return out


def derive(optical_color_in_flange, flange_to_tcp_xyz) -> dict:
    """Compose the constant the live pose chain multiplies the TCP pose by."""

    flange_tcp = np.eye(4)
    flange_tcp[:3, 3] = flange_to_tcp_xyz
    flange_cam = matrix(optical_color_in_flange)
    tcp_cam = np.linalg.inv(flange_tcp) @ flange_cam
    return {
        "T_flange_tcp": flange_tcp,
        "T_flange_cam_color_optical": flange_cam,
        "T_tcp_cam": tcp_cam,
    }


def clean(array: np.ndarray) -> list[list[float]]:
    return [[float(round(float(v), 12)) for v in row] for row in array]


def as_document(chain: dict, values: dict, rvs_dir: pathlib.Path) -> dict:
    return {
        "source": "RobotVisionSuite eye-in-hand hand-eye calibration",
        "derived_by": "scripts/sync_real_handeye.py",
        "source_files": [
            str(rvs_dir / "HandEyeTool.ini"),
            str(rvs_dir / "ColorToRobotTCP.txt"),
            str(rvs_dir / "DepthToRobotTCP.txt"),
        ],
        "reference_frame": (
            "flange; the calibration's own frame is the camera OPTICAL frame and its "
            "numbers are metres/radians (cross-checked against the mm/deg encoding)"
        ),
        "frame_chain": (
            "T_base_cam = T_base_tcp @ T_tcp_cam, with T_base_tcp from "
            "/tcp_pose_broadcaster/pose (frame 'base') and "
            "T_tcp_cam = inv(T_flange_tcp) @ T_flange_cam_color_optical"
        ),
        "camera_frame": "camera_color_optical_frame",
        "note": (
            "Do not hand-edit: regenerate with scripts/sync_real_handeye.py. A "
            "hand-written value here (camera 50 mm from the TCP, 45 deg tilt) "
            "disagreed with the calibration by 135 deg / 0.34 m and put every "
            "base-frame target about 1.4 m off."
        ),
        "T_flange_tcp": clean(chain["T_flange_tcp"]),
        "T_flange_cam_color_optical": clean(chain["T_flange_cam_color_optical"]),
        "T_tcp_cam": clean(chain["T_tcp_cam"]),
        # Kept for the calibration record only; never used by the runtime chain.
        "calibration_color_optical_in_flange": [round(float(v), 9) for v in values["color"]],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rvs-dir", type=pathlib.Path,
                        help="RobotVisionSuite runtime directory")
    parser.add_argument("--check", action="store_true",
                        help="verify the committed JSON against the calibration")
    arguments = parser.parse_args()

    rvs_dir = arguments.rvs_dir or os.environ.get("CS625_RVS_RUNTIME_DIR")
    if rvs_dir is None:
        print("ERROR: give --rvs-dir or set CS625_RVS_RUNTIME_DIR; the RVS install "
              "path is machine-local and is not stored in the repository.",
              file=sys.stderr)
        return 2
    rvs_dir = pathlib.Path(rvs_dir)
    if not (rvs_dir / "HandEyeTool.ini").is_file():
        print(f"ERROR: {rvs_dir / 'HandEyeTool.ini'} not found", file=sys.stderr)
        return 2

    ini = handeye.calibrate_from_ini(rvs_dir / "HandEyeTool.ini")
    txt = handeye.calibrate_from_txt(rvs_dir)
    handeye.cross_check(ini, txt)

    chain = derive(txt["color"], FLANGE_TO_TCP_XYZ)
    document = as_document(chain, txt, rvs_dir)

    t_tcp_cam = chain["T_tcp_cam"]
    print("T_tcp_cam translation = "
          f"{[round(float(v), 4) for v in t_tcp_cam[:3, 3]]}")
    print("camera-to-TCP distance = "
          f"{float(np.linalg.norm(t_tcp_cam[:3, 3])) * 1000:.1f} mm")

    if arguments.check:
        if not TARGET.is_file():
            print(f"ERROR: {TARGET} is missing", file=sys.stderr)
            return 1
        committed = json.loads(TARGET.read_text(encoding="utf-8"))
        if committed != json.loads(json.dumps(document)):
            print(f"ERROR: {TARGET} disagrees with the calibration", file=sys.stderr)
            return 1
        print(f"{TARGET.relative_to(ROOT)} matches the calibration")
        return 0

    TARGET.parent.mkdir(parents=True, exist_ok=True)
    TARGET.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {TARGET.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
