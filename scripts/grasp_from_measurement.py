#!/usr/bin/env python3
"""Derive (and check) the grasp poses from a MEASURED workpiece pose.

Why this exists
---------------
`config/tasks/common.yaml` stores ``pre_grasp`` / ``grasp`` as constants that were
derived once, on 2026-10-24, from the workpiece pose of that day.  The environment
model does not re-measure the workpiece pose either: ``model_environment.py`` takes
it from ``WORKPIECE_POSE`` (a constant) and the gate then compares the constant with
itself, so a moved workpiece is invisible.

On 2026-10-26 the arm drove into the workpiece during ``grasp`` and knocked it over:
the nominal pose was for a part that had shifted.  This tool closes that gap:

    grasp      = T_workpiece * p_part          (p_part: CAD-frame grasp point)
    pre_grasp  = grasp - R_workpiece * o_part  (o_part: approach offset, part frame)

``p_part`` and ``o_part`` are recovered from the reference pose + the reference
config, so the formula reproduces the proven values exactly (verified in tests).
Feeding a new measured pose gives the corrected poses; ``--check`` tells you whether
today's part is close enough to the reference to use the stored constants at all.

Usage
-----
    # 1) from a measured pose (xyz, quat xyzw) -- print the corrected YAML block
    python3 scripts/grasp_from_measurement.py --workpiece-xyz 0.503 ... --workpiece-quat ...

    # 2) measure from the camera (fine observation pose, arm still) and check drift
    python3 scripts/grasp_from_measurement.py --measure --check

    # 3) only check: exit 1 when the workpiece moved more than the tolerance
    python3 scripts/grasp_from_measurement.py --check --workpiece-xyz ... --workpiece-quat ...

Never moves the robot.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

REPO = pathlib.Path(__file__).resolve().parent.parent
COMMON = REPO / "config" / "tasks" / "common.yaml"
ICP = REPO / "scripts" / "icp_pose_once.py"
DEFAULT_WEIGHTS = pathlib.Path.home() / (
    "cs625_yolo_seg/runs/segment/runs/segment/box_slot_1280/weights/best.pt")
DEFAULT_TEMPLATE = pathlib.Path.home() / (
    "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL")

# Reference: the workpiece pose the stored grasp poses were derived from, and the
# config values derived from it (2026-10-24).
REFERENCE_WORKPIECE_XYZ = np.array([0.5029, 0.6351, 0.1254])
REFERENCE_WORKPIECE_QUAT = np.array([0.1910, 0.9816, 0.0019, -0.0068])
REFERENCE_GRASP = np.array([0.8316, 0.5918, -0.0583])
REFERENCE_PRE_GRASP = np.array([0.7677, 0.4903, -0.0595])
DEFAULT_TOL_MM = 5.0
DEFAULT_TOL_DEG = 1.0


def _transform(xyz, quat) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def part_point() -> np.ndarray:
    """CAD-frame grasp point (metres), recovered from the reference pose+config."""
    transform = _transform(REFERENCE_WORKPIECE_XYZ, REFERENCE_WORKPIECE_QUAT)
    return transform[:3, :3].T @ (REFERENCE_GRASP - transform[:3, 3])


def approach_offset_part() -> np.ndarray:
    """pre_grasp -> grasp offset expressed in the part frame (metres)."""
    rotation = _transform(REFERENCE_WORKPIECE_XYZ, REFERENCE_WORKPIECE_QUAT)[:3, :3]
    return rotation.T @ (REFERENCE_GRASP - REFERENCE_PRE_GRASP)


def poses_from_workpiece(xyz, quat) -> tuple[np.ndarray, np.ndarray]:
    transform = _transform(xyz, quat)
    grasp = transform[:3, :3] @ part_point() + transform[:3, 3]
    pre_grasp = grasp - transform[:3, :3] @ approach_offset_part()
    return pre_grasp, grasp


def orientation_for_grasp(current_quat=None):
    """The grasp orientation is unchanged by a translation; keep the proven one."""
    return np.asarray([-0.4835, 0.8753, 0.0075, 0.0018])


def drift(measured_xyz, measured_quat, reference_xyz=None, reference_quat=None):
    """Translation (mm) and rotation (deg) between measured and reference pose."""
    reference_xyz = (REFERENCE_WORKPIECE_XYZ if reference_xyz is None
                     else np.asarray(reference_xyz, dtype=float))
    reference_quat = (REFERENCE_WORKPIECE_QUAT if reference_quat is None
                      else np.asarray(reference_quat, dtype=float))
    delta_mm = float(np.linalg.norm(np.asarray(measured_xyz) - reference_xyz) * 1000.0)
    rotation = (Rotation.from_quat(reference_quat).inv()
                * Rotation.from_quat(np.asarray(measured_quat, dtype=float)))
    delta_deg = float(np.degrees(np.linalg.norm(rotation.as_rotvec())))
    return delta_mm, delta_deg


POSE_RE = re.compile(r"icp xyz=\[(?P<xyz>[^\]]+)\] quat=\[(?P<quat>[^\]]+)\]")


def measure_with_camera(weights: pathlib.Path, template: pathlib.Path,
                        timeout: float = 120.0) -> tuple[np.ndarray, np.ndarray]:
    command = [sys.executable, "-u", str(ICP),
               "--weights", str(weights), "--template", str(template)]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    text = (completed.stdout or "") + (completed.stderr or "")
    match = POSE_RE.search(text)
    if match is None:
        raise SystemExit("could not parse the ICP pose; icp_pose_once output:\n"
                         + "\n".join(text.strip().splitlines()[-6:]))
    xyz = np.array([float(v) for v in match.group("xyz").split(",")])
    quat = np.array([float(v) for v in match.group("quat").split(",")])
    return xyz, quat


def yaml_block(pre_grasp: np.ndarray, grasp: np.ndarray) -> str:
    quat = orientation_for_grasp()
    fmt = lambda v: ", ".join(f"{float(x):.4f}" for x in v)  # noqa: E731
    return ("pre_grasp: [" + fmt(pre_grasp) + "]\n"
            "grasp: [" + fmt(grasp) + "]\n"
            "grasp_q: [" + ", ".join(f"{float(x):.4f}" for x in quat) + "]")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--workpiece-xyz", type=float, nargs=3, default=None)
    parser.add_argument("--workpiece-quat", type=float, nargs=4, default=None,
                        help="xyzw")
    parser.add_argument("--measure", action="store_true",
                        help="measure the workpiece with the camera (arm must be still)")
    parser.add_argument("--check", action="store_true",
                        help="exit 1 when the workpiece drifted from the reference pose")
    parser.add_argument("--tol-mm", type=float, default=DEFAULT_TOL_MM)
    parser.add_argument("--tol-deg", type=float, default=DEFAULT_TOL_DEG)
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    parser.add_argument("--template", default=str(DEFAULT_TEMPLATE))
    args = parser.parse_args()

    xyz, quat = args.workpiece_xyz, args.workpiece_quat
    if args.measure:
        if xyz is not None or quat is not None:
            print("--measure and --workpiece-* are mutually exclusive")
            return 2
        xyz, quat = measure_with_camera(pathlib.Path(args.weights),
                                        pathlib.Path(args.template))
        print("measured workpiece xyz:", np.round(xyz, 4).tolist())
        print("measured workpiece quat:", np.round(quat, 4).tolist())
    if xyz is None or quat is None:
        print("give --measure or --workpiece-xyz/--workpiece-quat")
        return 2

    delta_mm, delta_deg = drift(xyz, quat)
    pre_grasp, grasp = poses_from_workpiece(xyz, quat)
    print(f"drift vs the reference pose used to derive the stored constants: "
          f"{delta_mm:.2f} mm / {delta_deg:.2f} deg")
    print("derived poses:")
    print(yaml_block(pre_grasp, grasp))

    if args.check:
        ok = delta_mm <= args.tol_mm and delta_deg <= args.tol_deg
        if not ok:
            print(f"REFUSE: workpiece moved more than {args.tol_mm:.1f} mm / "
                  f"{args.tol_deg:.1f} deg; update config/tasks/common.yaml with the "
                  "derived pre_grasp/grasp above before running the task")
            return 1
        print("OK: the stored grasp poses are valid for this workpiece pose")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
