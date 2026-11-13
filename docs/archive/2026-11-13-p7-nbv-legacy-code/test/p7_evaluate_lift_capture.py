#!/usr/bin/env python3
"""Measure P7 target-centre lift from two captured Gazebo model poses."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from cs625_task_orchestrator.p7_grasp_geometry import target_center_world


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before-model-pose", required=True, type=Path)
    parser.add_argument("--after-model-pose", required=True, type=Path)
    parser.add_argument(
        "--target-local-center-m", required=True, nargs=3, type=float,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--minimum-height-m", type=float, default=0.10)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def load_pose(path: Path) -> dict:
    decoded = json.loads(path.read_text(encoding="utf-8"))
    pose = decoded.get("pose", decoded) if isinstance(decoded, dict) else None
    if not isinstance(pose, dict):
        raise ValueError(f"{path} does not contain a pose object")
    return pose


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    arguments = parse_arguments()
    local = tuple(arguments.target_local_center_m)
    before = target_center_world(load_pose(arguments.before_model_pose), local)
    after = target_center_world(load_pose(arguments.after_model_pose), local)
    height_delta = after[2] - before[2]
    success = height_delta >= arguments.minimum_height_m
    record = {
        "capture_schema": "p7_lift_measurement_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "sources": {
            "before_model_pose": {
                "path": str(arguments.before_model_pose),
                "sha256": sha256(arguments.before_model_pose),
            },
            "after_model_pose": {
                "path": str(arguments.after_model_pose),
                "sha256": sha256(arguments.after_model_pose),
            },
        },
        "target_local_center_m": list(local),
        "before_target_center_world_m": list(before),
        "after_target_center_world_m": list(after),
        "height_delta_m": height_delta,
        "minimum_height_m": arguments.minimum_height_m,
        "height_gate_pass": success,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
