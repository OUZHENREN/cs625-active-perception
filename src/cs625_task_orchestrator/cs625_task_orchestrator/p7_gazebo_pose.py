"""Parse the documented ``gz model --pose`` output for P7 evidence."""

from __future__ import annotations

import math
import re
from typing import Any


ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
SEPARATOR = r"(?:[ \t]*\|[ \t]*|[ \t]+)"
POSE = re.compile(
    rf"-\s*Pose\s*\[\s*XYZ\s*\(m\)\s*\]\s*\[\s*RPY\s*\(rad\)\s*\]\s*:\s*"
    rf"\[\s*({NUMBER}){SEPARATOR}({NUMBER}){SEPARATOR}({NUMBER})\s*\]\s*"
    rf"\[\s*({NUMBER}){SEPARATOR}({NUMBER}){SEPARATOR}({NUMBER})\s*\]",
    re.IGNORECASE | re.MULTILINE,
)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return an xyzw quaternion for fixed-axis roll, pitch, yaw."""

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_gz_model_pose(output: str, expected_model: str) -> dict[str, Any]:
    """Parse exactly one model pose and reject ambiguous or wrong output."""

    clean = ANSI.sub("", output)
    if f"Name: {expected_model}" not in clean:
        raise ValueError(f"Gazebo output does not identify model {expected_model!r}")
    matches = POSE.findall(clean)
    if len(matches) != 1:
        raise ValueError(f"expected one Gazebo model pose, observed {len(matches)}")
    x, y, z, roll, pitch, yaw = (float(value) for value in matches[0])
    values = (x, y, z, roll, pitch, yaw)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Gazebo model pose contains a non-finite value")
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        "rpy_rad": {"roll": roll, "pitch": pitch, "yaw": yaw},
    }
