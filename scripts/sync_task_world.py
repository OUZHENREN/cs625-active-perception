#!/usr/bin/env python3
"""Read, write and synchronise the CS625 task scene placement.

Gazebo does not write a dragged model back to its SDF file, so placing the scene
is always two steps: read the pose out of the GUI, then record it here.  This
script owns both halves.

    # what is recorded now, in both notations the GUI may show
    python3 scripts/sync_task_world.py --print

    # record a pose read from Gazebo's right-hand Pose panel, then sync the world
    python3 scripts/sync_task_world.py --set fixture \
        --position -0.720 0.000 0.262063 --rpy-deg 180 5.004 0
    python3 scripts/sync_task_world.py --set module \
        --position 0.450 0.450 0.221264 --quat 0.70385 -0.70385 -0.06780 0.06780

    # the world file and the config both hold these poses, so keep them in step
    python3 scripts/sync_task_world.py              # write the config's poses into the world
    python3 scripts/sync_task_world.py --check      # report drift without changing anything
    python3 test/contract_checks.py                 # should pass

The world file stays a reviewed, tracked artifact: nothing here regenerates it
silently at launch time.

Reading the pose from Gazebo: click the model in the Entity Tree, expand "Pose" in
the right panel.  The position is in the model's parent frame, which for these
models is the world, so the numbers transfer directly.  The orientation may be
shown as a quaternion or as roll/pitch/yaw; both forms are accepted here, and
--print shows both so they can be compared.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import re
import sys

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/cs625_bringup/config/cs625_task_scene.yaml"
WORLD = ROOT / "src/cs625_simulation/worlds/cs625_insertion_scene.sdf"

# Settable placements: (--set name, config section, pose key, world model name)
SETTABLE = {
    "fixture": ("fixture", "pose_world", "slot_fixture"),
    "module": ("module", "home_pose_world", "shielding_module"),
}
# Everything the world and config must agree on, settable or not.
BINDINGS = (
    ("slot_fixture", "fixture", "pose_world"),
    ("shielding_module", "module", "home_pose_world"),
)


# --- pose conversion -------------------------------------------------------

def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """SDF roll/pitch/yaw to an xyzw quaternion.

    SDF composes R = Rz(yaw) * Ry(pitch) * Rx(roll); this is the matching
    intrinsic Z-Y-X conversion.  test/test_sync_task_world.py pins it against the
    rotation matrix recovered from the SolidWorks assembly, so a wrong axis order
    cannot pass unnoticed.
    """

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """xyzw quaternion to the SDF roll/pitch/yaw triple."""

    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def format_pose(pose: list[float]) -> str:
    return " ".join(f"{float(value):.9f}" for value in pose)


def describe_pose(pose: list[float]) -> str:
    quaternion = quaternion_from_rpy(*pose[3:6])
    return (
        f"position ({pose[0]: .6f}, {pose[1]: .6f}, {pose[2]: .6f}) m   "
        f"rpy_deg ({math.degrees(pose[3]): .4f}, {math.degrees(pose[4]): .4f}, "
        f"{math.degrees(pose[5]): .4f})   "
        f"quat ({quaternion[0]: .6f}, {quaternion[1]: .6f}, "
        f"{quaternion[2]: .6f}, {quaternion[3]: .6f})"
    )


# --- config / world access -------------------------------------------------

def load_parameters() -> dict:
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "cs625_task_scene"
    ]["ros__parameters"]


def write_pose(parameters: dict, section: str, key: str, pose: list[float]) -> None:
    """Replace one pose in the config, preserving comments and layout."""

    text = CONFIG.read_text(encoding="utf-8")
    for indent in ("    ", "      "):
        pattern = re.compile(
            rf"^{indent}{re.escape(key)}: \[[^\]]*\]$", flags=re.M
        )
        if pattern.search(text):
            CONFIG.write_text(
                pattern.sub(f"{indent}{key}: [{', '.join(f'{v:.9f}' for v in pose)}]",
                            text, count=1),
                encoding="utf-8",
            )
            parameters[section][key] = pose
            return
    raise SystemExit(f"ERROR: could not find {key!r} in {CONFIG.name}")


def include_block(world_text: str, model_name: str) -> re.Match | None:
    return re.search(
        r"<include>\s*<uri>[^<]*</uri>\s*<name>"
        + re.escape(model_name)
        + r"</name>.*?<pose>([^<]*)</pose>",
        world_text,
        flags=re.S,
    )


def sync_world(parameters: dict, check_only: bool) -> int:
    world_text = WORLD.read_text(encoding="utf-8")
    updated = world_text
    drift = []
    for model_name, section, key in BINDINGS:
        expected = format_pose(parameters[section][key])
        match = include_block(updated, model_name)
        if match is None:
            print(f"ERROR: {model_name} <include> not found in {WORLD.name}", file=sys.stderr)
            return 2
        observed = match.group(1).strip()
        if observed != expected:
            drift.append((model_name, observed, expected))
            updated = updated[: match.start(1)] + expected + updated[match.end(1) :]

    if not drift:
        print("task world poses already match the config")
        return 0

    for model_name, observed, expected in drift:
        print(f"{model_name}:")
        print(f"  world  {observed}")
        print(f"  config {expected}")
    if check_only:
        print(f"\n{len(drift)} pose(s) out of sync; run without --check to fix")
        return 1

    WORLD.write_text(updated, encoding="utf-8")
    print(f"\nrewrote {len(drift)} pose(s) in {WORLD.relative_to(ROOT)}")
    print("now run: python3 test/contract_checks.py")
    return 0


# --- entry point -----------------------------------------------------------

def build_pose(arguments: argparse.Namespace) -> list[float]:
    if arguments.position is None:
        raise SystemExit("ERROR: --set requires --position X Y Z")
    if len(arguments.position) != 3:
        raise SystemExit("ERROR: --position takes exactly three numbers")
    position = [float(value) for value in arguments.position]

    if arguments.rpy_deg is not None:
        if len(arguments.rpy_deg) != 3:
            raise SystemExit("ERROR: --rpy-deg takes exactly three numbers")
        rpy = [math.radians(float(value)) for value in arguments.rpy_deg]
    elif arguments.quat is not None:
        if len(arguments.quat) != 4:
            raise SystemExit("ERROR: --quat takes exactly four numbers")
        rpy = list(rpy_from_quaternion(tuple(float(v) for v in arguments.quat)))
    else:
        raise SystemExit("ERROR: --set requires --rpy-deg or --quat")
    return position + rpy


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="report drift only")
    parser.add_argument("--print", dest="show", action="store_true",
                        help="print the recorded poses and exit")
    parser.add_argument("--set", dest="target", choices=sorted(SETTABLE),
                        help="which placement to overwrite")
    parser.add_argument("--position", nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--rpy-deg", nargs=3, metavar=("R", "P", "Y"),
                        help="orientation as roll/pitch/yaw in degrees")
    parser.add_argument("--quat", nargs=4, metavar=("QX", "QY", "QZ", "QW"),
                        help="orientation as a quaternion, as the GUI may show it")
    arguments = parser.parse_args()

    parameters = load_parameters()

    if arguments.show:
        for section, key, model in (
            ("fixture", "pose_world", "slot_fixture"),
            ("module", "home_pose_world", "shielding_module"),
        ):
            print(f"{model}  ({section}.{key})")
            print(f"  {describe_pose(parameters[section][key])}")
        print("\ninsertion.seated_pose_world is measured from the SolidWorks assembly")
        print("and is not a placement; it is not settable here.")
        return 0

    if arguments.target:
        section, key, model_name = SETTABLE[arguments.target]
        pose = build_pose(arguments)
        write_pose(parameters, section, key, pose)
        print(f"recorded {model_name}:")
        print(f"  {describe_pose(pose)}")

    return sync_world(parameters, arguments.check)


if __name__ == "__main__":
    sys.exit(main())
