#!/usr/bin/env python3
"""Evaluate two captured poses and atomically write the P7 geometry gate."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from cs625_task_orchestrator.p7_grasp_geometry import evaluate_grasp_geometry


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-pose", required=True, type=Path)
    parser.add_argument("--grasp-pose", required=True, type=Path)
    parser.add_argument(
        "--target-local-center-m", required=True, nargs=3, type=float,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--threshold-m", type=float, default=0.015)
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
    result = evaluate_grasp_geometry(
        load_pose(arguments.model_pose),
        load_pose(arguments.grasp_pose),
        tuple(arguments.target_local_center_m),
        arguments.threshold_m,
    )
    record = {
        "capture_schema": "p7_grasp_geometry_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "sources": {
            "model_pose": {
                "path": str(arguments.model_pose),
                "sha256": sha256(arguments.model_pose),
            },
            "grasp_pose": {
                "path": str(arguments.grasp_pose),
                "sha256": sha256(arguments.grasp_pose),
            },
        },
        **result,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    if not record["geometry_pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
