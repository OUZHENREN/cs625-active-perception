#!/usr/bin/env python3
"""Rewrite the task world's <include> poses from the task scene config.

The world file and the config both carry the fixture and module poses, and
``test/contract_checks.py`` fails when they disagree.  That is the right guard,
but it makes hand-editing two files error-prone during scene placement.  This
script makes the config the only thing a human edits:

    1. change fixture.pose_world / module.home_pose_world in
       src/cs625_bringup/config/cs625_task_scene.yaml
    2. python3 scripts/sync_task_world.py
    3. python3 test/contract_checks.py      # should pass

It is deliberately a host-side tool rather than a launch step: the world file
stays a reviewed, tracked artifact rather than something regenerated silently.

    python3 scripts/sync_task_world.py --check     # report drift, change nothing
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/cs625_bringup/config/cs625_task_scene.yaml"
WORLD = ROOT / "src/cs625_simulation/worlds/cs625_insertion_scene.sdf"

# (model name in the world, config section, pose key)
BINDINGS = (
    ("slot_fixture", "fixture", "pose_world"),
    ("shielding_module", "module", "home_pose_world"),
)


def format_pose(pose: list[float]) -> str:
    return " ".join(f"{float(value):.9f}" for value in pose)


def include_block(world_text: str, model_name: str) -> re.Match | None:
    return re.search(
        r"<include>\s*<uri>[^<]*</uri>\s*<name>"
        + re.escape(model_name)
        + r"</name>.*?<pose>([^<]*)</pose>",
        world_text,
        flags=re.S,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="report drift only")
    arguments = parser.parse_args()

    parameters = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "cs625_task_scene"
    ]["ros__parameters"]

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
            updated = (
                updated[: match.start(1)] + expected + updated[match.end(1) :]
            )

    if not drift:
        print("task world poses already match the config")
        return 0

    for model_name, observed, expected in drift:
        print(f"{model_name}:")
        print(f"  world  {observed}")
        print(f"  config {expected}")

    if arguments.check:
        print(f"\n{len(drift)} pose(s) out of sync; run without --check to fix")
        return 1

    WORLD.write_text(updated, encoding="utf-8")
    print(f"\nrewrote {len(drift)} pose(s) in {WORLD.relative_to(ROOT)}")
    print("now run: python3 test/contract_checks.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
