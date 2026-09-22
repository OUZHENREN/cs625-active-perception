#!/usr/bin/env python3
"""Derive and verify the shielding module insertion sequence.

Turns the measured scene geometry and the grasp template into named world-frame
waypoints for grasp -> transfer -> insert -> release, and measures the insertion
axis and travel from the fixture and module meshes rather than assuming them.

    python3 scripts/insertion_sequence.py --print
    python3 scripts/insertion_sequence.py --write
    python3 scripts/insertion_sequence.py --check

How the insertion axis is found, and why it is not a free choice.  The module sits
in the fixture with one end proud of it, and that proud end is the opening.  In the
fixture's own frame the seated module spans Z -264.4 .. +206.0 mm while the fixture
spans -245.1 .. +260.7 mm, so the module protrudes at the LOW-Z end and the opening
faces -Z.  The fixture's +Z maps to the world's -Z because its stored pose is rotated
by pi about X, so withdrawing is the world's +Z: the module is lowered into the slot
from above and lifted back out.  That is worth stating because a sweep over candidate
directions does not find it -- moving away from a fixture never collides, so "which
direction is free" has no answer.

The travel is where the module's own extent along that axis stops overlapping the
fixture's: 470 mm of module plus the 19 mm it sits proud.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import struct
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = pathlib.Path(__file__).resolve().parents[1]
TASK_SCENE = ROOT / "src/cs625_bringup/config/cs625_task_scene.yaml"
GRASP_TEMPLATE = ROOT / "src/cs625_bringup/config/cs625_grasp_template.yaml"
CONFIG = ROOT / "src/cs625_bringup/config/cs625_insertion_sequence.yaml"
MODULE_MESH = (
    ROOT
    / "src/cs625_simulation/assets/cs625_task/shielding_module/meshes/shielding_module.stl"
)
FIXTURE_MESH = (
    ROOT / "src/cs625_simulation/assets/cs625_task/slot_fixture/meshes/slot_fixture.stl"
)

# The fixture's own frame is the assembly frame, so the module's seated pose in that
# frame is the recorded assembly invariant.
ASSEMBLY_POSITION = [0.1480452, 0.2085, -0.0143114]
ASSEMBLY_RPY = [-0.10472, 0.0, 1.570796327]

sys.path.insert(0, str(ROOT / "src/cs625_task_orchestrator"))
from cs625_task_orchestrator import task_insertion_sequence as sequence  # noqa: E402


def read_stl(path: pathlib.Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path.name} is too short to be a binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) < 84 + count * 50:
        raise ValueError(f"{path.name} is truncated")
    raw = np.frombuffer(data, dtype=np.uint8, count=count * 50, offset=84)
    return (
        raw.reshape(count, 50)[:, 12:48]
        .copy()
        .view("<f4")
        .reshape(count, 3, 3)
        .astype(np.float64)
    )


def measure_insertion(
    module: np.ndarray, fixture: np.ndarray, fixture_pose_world: list[float]
) -> dict:
    """Find the open end in the fixture frame and the travel that clears it.

    Two different rotations are in play and swapping them is silent.  Bringing the
    module mesh into the fixture's frame uses the MODULE's pose in that frame; turning
    the discovered axis into a world direction uses the FIXTURE's pose in the world.
    Using the fixture's rotation for both puts the module in the wrong place and
    reports three open ends; using the module's for both points the axis the wrong way
    and reports a clearance in the hundreds of millimetres, because the line then runs
    away from the fixture instead of down into it.
    """

    module_in_assembly = Rotation.from_euler("xyz", ASSEMBLY_RPY).as_matrix()
    module_world = (
        module_in_assembly @ module.T
    ).T + np.asarray(ASSEMBLY_POSITION, dtype=float)
    fixture_in_world = Rotation.from_euler("xyz", fixture_pose_world[3:6]).as_matrix()

    # Project both bodies onto each candidate axis, in the fixture frame.
    candidates = []
    for index, name in enumerate("XYZ"):
        low, high = module_world[:, index].min(), module_world[:, index].max()
        f_low, f_high = fixture[:, index].min(), fixture[:, index].max()
        low_proud = f_low - low
        high_proud = high - f_high
        if low_proud > 0.002 and high_proud <= 0.002:
            candidates.append((name, -1, low_proud))
        elif high_proud > 0.002 and low_proud <= 0.002:
            candidates.append((name, +1, high_proud))
    if len(candidates) != 1:
        raise SystemExit(
            "expected exactly one open end of the fixture, found "
            f"{[(c[0], c[1]) for c in candidates]}"
        )
    axis_name, outward_sign, proud = candidates[0]
    axis_assembly = np.zeros(3)
    axis_assembly["XYZ".index(axis_name)] = outward_sign

    # The module exits once its extent along the axis stops overlapping the fixture's.
    index = "XYZ".index(axis_name)
    low, high = module_world[:, index].min(), module_world[:, index].max()
    f_low, f_high = fixture[:, index].min(), fixture[:, index].max()
    travel = max(f_high - low, high - f_low)

    axis_world = fixture_in_world @ axis_assembly
    return {
        "axis_assembly": [float(v) for v in axis_assembly],
        "axis_world": [float(v) for v in axis_world],
        "axis_name": axis_name,
        "outward_sign": int(outward_sign),
        "proud_m": float(proud),
        "travel_m": float(travel),
        "module_extent_along_axis_m": float(high - low),
        "method": (
            "the module protrudes at exactly one end of the fixture in the fixture's "
            "own frame; that end is the opening, and the travel is where the module's "
            "extent along the axis stops overlapping the fixture's"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--print", dest="show", action="store_true")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()

    scene = yaml.safe_load(TASK_SCENE.read_text(encoding="utf-8"))[
        "cs625_task_scene"
    ]["ros__parameters"]
    template = yaml.safe_load(GRASP_TEMPLATE.read_text(encoding="utf-8"))[
        "cs625_grasp_template"
    ]["ros__parameters"]

    module = read_stl(MODULE_MESH).reshape(-1, 3)
    fixture = read_stl(FIXTURE_MESH).reshape(-1, 3)
    insertion = measure_insertion(module, fixture, scene["fixture"]["pose_world"])

    margin = 0.020
    result = sequence.build_sequence(
        module_home_world=scene["module"]["home_pose_world"],
        seated_module_world=scene["insertion"]["seated_pose_world"],
        grasp_template=template,
        insertion_axis_world=insertion["axis_world"],
        withdrawal_travel_m=insertion["travel_m"],
        approach_margin_m=margin,
    )
    result["insertion_measurement"] = insertion
    result["approach_margin_m"] = margin
    # The fixture mesh is in the assembly frame and the module is placed in the world,
    # so the fixture has to be carried into the world as well.  Comparing the two in
    # different frames reports a clearance of hundreds of millimetres, which is the
    # distance to a fixture that is not there.
    fixture_pose = scene["fixture"]["pose_world"]
    fixture_in_world = (
        Rotation.from_euler("xyz", fixture_pose[3:6]).as_matrix() @ fixture.T
    ).T + np.asarray(fixture_pose[:3], dtype=float)
    result["insertion_line"] = sequence.verify_insertion_line(
        scene["insertion"]["seated_pose_world"],
        insertion["axis_world"],
        insertion["travel_m"] + margin,
        module,
        fixture_in_world,
    )
    result["insertion_line"].pop("samples")

    document = yaml.safe_dump(
        {"cs625_insertion_sequence": {"ros__parameters": result}},
        sort_keys=False,
        allow_unicode=True,
    )

    if arguments.show and not (arguments.write or arguments.check):
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0

    if arguments.write:
        CONFIG.write_text(document, encoding="utf-8")
        print(f"wrote {CONFIG.relative_to(ROOT)}")
        print(
            f"  insertion axis {insertion['axis_name']}{'+' if insertion['outward_sign'] > 0 else '-'}"
            f" (assembly) -> world {[round(v, 4) for v in insertion['axis_world']]}"
        )
        print(f"  withdrawal travel {insertion['travel_m'] * 1000:.1f} mm")
        print(
            "  insertion line minimum clearance "
            f"{result['insertion_line']['minimum_clearance_m'] * 1000:.3f} mm"
        )
        return 0

    if arguments.check:
        if not CONFIG.is_file():
            print(f"ERROR: {CONFIG.relative_to(ROOT)} is missing", file=sys.stderr)
            return 1
        if yaml.safe_load(CONFIG.read_text(encoding="utf-8")) != yaml.safe_load(document):
            print(
                f"ERROR: {CONFIG.relative_to(ROOT)} disagrees with the derivation",
                file=sys.stderr,
            )
            return 1
        print(
            "insertion sequence matches the derivation: travel "
            f"{insertion['travel_m'] * 1000:.1f} mm"
        )
        line = result["insertion_line"]
        print(
            f"  insertion line minimum {line['minimum_clearance_m'] * 1000:.3f} mm at "
            f"{line['minimum_at_offset_m'] * 1000:.1f} mm above seated"
        )
        if not line["passes"]:
            # A config that reproduces is not the same as a motion that is safe, and
            # collapsing the two would let an unsafe sequence look certified.
            print(
                "NOT CERTIFIED: the straight insertion line comes within "
                f"{line['minimum_clearance_m'] * 1000:.3f} mm of the fixture, which is at "
                "the level of the mesh decimation error rather than clearly clear",
                file=sys.stderr,
            )
            return 2
        return 0

    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
