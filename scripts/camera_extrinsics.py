#!/usr/bin/env python3
"""Derive the URDF camera extrinsics from the RVS hand-eye calibration.

The real eye-in-hand camera was calibrated by RobotVisionSuite, not by this
repository, and the only authoritative record of that calibration is RVS's own
output:

    HandEyeTool.ini            [colorToRobot] / [depthToRobot] in mm and degrees
    ColorToRobotTCP.txt        the same numbers in metres and radians
    DepthToRobotTCP.txt

Both encodings are parsed and cross-checked against each other, so a silently
changed unit cannot pass: the two files agree to 4.5e-06 today, and a mismatch
aborts.

WHAT "RobotTCP" MEANS HERE.  The tool owner measured the camera at about 98 mm
from the flange, which matches the raw numbers, so the calibration is expressed
in the FLANGE frame and no controller-TCP offset is applied.  That distinction
matters: the controller's active TCP is (-46, 0, 353) mm from the flange, verified
against the recorded tool_data CSV to 0.0 mm, and composing the two would place
the camera 450 mm out, in mid-air past the tool.

WHAT THE CALIBRATION'S FRAME IS.  Its +Z is the optical axis and comes out along
the flange's +Z, which is what an eye-in-hand camera looking down the tool must
do, so the recorded pose is the camera's OPTICAL frame.  ROS wants camera_link in
the body convention, so the body pose is the optical pose with the standard
body->optical rotation removed.

    python3 scripts/camera_extrinsics.py --rvs-dir "/d/Program Files (x86)/RobotVisionSuite/runtime"
    python3 scripts/camera_extrinsics.py --check       # fail if the config drifted

The RVS directory is machine-local and is never hardcoded: it comes from --rvs-dir
or CS625_RVS_RUNTIME_DIR.  The numbers it derives are written to
src/cs625_bringup/config/camera_extrinsics_sim.yaml, which is what the launch reads.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import re
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/cs625_bringup/config/camera_extrinsics_sim.yaml"

# ROS camera convention: camera_link is x-forward/y-left/z-up, optical is
# x-right/y-down/z-forward, so optical = body * (-90 deg about X, -90 deg about Z).
BODY_TO_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

EYES = ("color", "depth")


def calibrate_from_ini(path: pathlib.Path) -> dict:
    """Read [colorToRobot] / [depthToRobot] in millimetres and degrees."""

    text = path.read_text(encoding="utf-8", errors="replace")
    result = {}
    for eye in EYES:
        match = re.search(
            rf"\[{eye}ToRobot\](.*?)(?=\n\s*\[|\Z)", text, flags=re.S | re.I
        )
        if match is None:
            raise SystemExit(f"ERROR: [{eye}ToRobot] not found in {path.name}")
        values = dict(re.findall(r"(\w+)\s*=\s*(-?[\d.]+)", match.group(1)))
        try:
            result[eye] = [
                float(values["x"]) / 1000.0,
                float(values["y"]) / 1000.0,
                float(values["z"]) / 1000.0,
                math.radians(float(values["rx"])),
                math.radians(float(values["ry"])),
                math.radians(float(values["rz"])),
            ]
        except KeyError as missing:
            raise SystemExit(f"ERROR: {eye}ToRobot is missing {missing}")
    return result


def calibrate_from_txt(directory: pathlib.Path) -> dict:
    """Read ColorToRobotTCP.txt / DepthToRobotTCP.txt in metres and radians."""

    result = {}
    for eye in EYES:
        path = directory / f"{eye.capitalize()}ToRobotTCP.txt"
        if not path.is_file():
            raise SystemExit(f"ERROR: {path.name} not found in {directory}")
        numbers = [float(v) for v in path.read_text(encoding="utf-8").split()]
        if len(numbers) != 6:
            raise SystemExit(f"ERROR: {path.name} holds {len(numbers)} numbers, expected 6")
        result[eye] = numbers
    return result


def cross_check(ini: dict, txt: dict) -> None:
    for eye in EYES:
        worst = max(abs(a - b) for a, b in zip(ini[eye], txt[eye]))
        if worst > 1e-4:
            raise SystemExit(
                f"ERROR: {eye} disagrees between mm/deg and m/rad encodings by "
                f"{worst:.3e}; one of the two files changed its unit"
            )


def display(path: pathlib.Path) -> str:
    """Path relative to the repository when possible, absolute otherwise.

    The config path is monkeypatched to a temporary file in the tests, so a bare
    relative_to(ROOT) would raise there and hide the real assertion.
    """

    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def clean(value) -> float:
    """Round to 12 decimals and hand back a plain float.

    numpy scalars leak out of scipy and numpy, and PyYAML refuses to serialise
    them, so every number that reaches the config goes through here.
    """

    return float(round(float(value), 12))


def rotation(rpy) -> np.ndarray:
    return Rotation.from_euler("xyz", rpy).as_matrix()


def to_pose(matrix) -> list[float]:
    return [float(v) for v in matrix[:3, 3]] + list(
        Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
    )


def derive(optical: dict) -> dict:
    """URDF-ready values from the two optical-frame poses in the flange frame."""

    flange_optical = {}
    for eye in EYES:
        matrix = np.eye(4)
        matrix[:3, :3] = rotation(optical[eye][3:6])
        matrix[:3, 3] = optical[eye][0:3]
        flange_optical[eye] = matrix

    # camera_link body frame sits at the colour eye so that the colour optical
    # frame is exactly the calibrated one after the standard body->optical turn.
    body = np.eye(4)
    body[:3, :3] = flange_optical["color"][:3, :3] @ rotation(BODY_TO_OPTICAL_RPY).T
    body[:3, 3] = flange_optical["color"][:3, 3]

    check = np.eye(4)
    check[:3, :3] = body[:3, :3] @ rotation(BODY_TO_OPTICAL_RPY)
    check[:3, 3] = body[:3, 3]
    if np.abs(check - flange_optical["color"]).max() > 1e-12:
        raise SystemExit("ERROR: body/optical composition does not reproduce the colour eye")

    depth = np.linalg.inv(body) @ flange_optical["depth"]
    return {
        "camera_mount_xyz": [float(v) for v in body[:3, 3]],
        "camera_mount_rpy": list(
            Rotation.from_matrix(body[:3, :3]).as_euler("xyz")
        ),
        "camera_optical_rpy": list(BODY_TO_OPTICAL_RPY),
        "camera_depth_xyz": [float(v) for v in depth[:3, 3]],
        "camera_depth_rpy": list(
            Rotation.from_matrix(depth[:3, :3]).as_euler("xyz")
        ),
        "baseline_m": float(np.linalg.norm(depth[:3, 3])),
        "color_optical_in_flange": to_pose(flange_optical["color"]),
        "depth_optical_in_flange": to_pose(flange_optical["depth"]),
    }


def load(rvs_dir: pathlib.Path) -> dict:
    ini = calibrate_from_ini(rvs_dir / "HandEyeTool.ini")
    txt = calibrate_from_txt(rvs_dir)
    cross_check(ini, txt)
    return derive(txt)


def as_yaml(values: dict, rvs_dir: pathlib.Path) -> str:
    document = {
        "cs625_camera_extrinsics": {
            "ros__parameters": {
                "source": "RobotVisionSuite hand-eye calibration, eye-in-hand",
                "source_files": [
                    str(rvs_dir / "HandEyeTool.ini"),
                    str(rvs_dir / "ColorToRobotTCP.txt"),
                    str(rvs_dir / "DepthToRobotTCP.txt"),
                ],
                "source_encoding": "mm and degrees in the .ini, metres and radians in the .txt",
                "reference_frame": (
                    "flange; the tool owner measured the camera at ~98 mm from the "
                    "flange, matching the raw numbers, so no controller-TCP offset applies"
                ),
                "calibration_frame": (
                    "the camera OPTICAL frame: its +Z is the optical axis and comes out "
                    "along the flange +Z"
                ),
                "note": (
                    "CameraToRobotTCP is stored as camera_link in the body convention; "
                    "camera_depth_* is the second stereo eye relative to camera_link. "
                    "Regenerate with scripts/camera_extrinsics.py; do not hand-edit."
                ),
                "camera_mount_xyz": [clean(v) for v in values["camera_mount_xyz"]],
                "camera_mount_rpy": [clean(v) for v in values["camera_mount_rpy"]],
                "camera_optical_rpy": [clean(v) for v in values["camera_optical_rpy"]],
                "camera_depth_xyz": [clean(v) for v in values["camera_depth_xyz"]],
                "camera_depth_rpy": [clean(v) for v in values["camera_depth_rpy"]],
                "stereo_baseline_m": clean(values["baseline_m"]),
                "color_optical_in_flange": [clean(v) for v in values["color_optical_in_flange"]],
                "depth_optical_in_flange": [clean(v) for v in values["depth_optical_in_flange"]],
            }
        }
    }
    return yaml.safe_dump(document, sort_keys=False, allow_unicode=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rvs-dir", type=pathlib.Path,
                        help="RobotVisionSuite runtime directory")
    parser.add_argument("--check", action="store_true",
                        help="verify the committed config against the calibration")
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

    values = load(rvs_dir)
    text = as_yaml(values, rvs_dir)

    print(f"camera_mount_xyz  = {[round(float(v), 6) for v in values['camera_mount_xyz']]}")
    print(f"camera_mount_rpy  = {[round(float(v), 9) for v in values['camera_mount_rpy']]}")
    print(f"camera_depth_xyz  = {[round(float(v), 6) for v in values['camera_depth_xyz']]}")
    print(f"stereo baseline   = {values['baseline_m'] * 1000:.3f} mm")

    if arguments.check:
        if not CONFIG.is_file():
            print(f"ERROR: {display(CONFIG)} is missing", file=sys.stderr)
            return 1
        committed = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        if committed != yaml.safe_load(text):
            print(f"\nERROR: {display(CONFIG)} disagrees with the calibration",
                  file=sys.stderr)
            return 1
        print(f"\n{display(CONFIG)} matches the calibration")
        return 0

    CONFIG.write_text(text, encoding="utf-8")
    print(f"\nwrote {display(CONFIG)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
