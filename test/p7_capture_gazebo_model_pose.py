#!/usr/bin/env python3
"""Capture one Gazebo model pose through the official ``gz model`` command."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess

from cs625_task_orchestrator.p7_gazebo_pose import parse_gz_model_pose


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="target_object")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    command = ["gz", "model", "-m", arguments.model, "--pose"]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=arguments.timeout_sec,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"gz model failed ({completed.returncode}): {completed.stderr.strip()}"
        )
    pose = parse_gz_model_pose(completed.stdout, arguments.model)
    record = {
        "capture_schema": "p7_gazebo_model_pose_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "model": arguments.model,
        "command": command,
        "runtime": {
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
        "pose": pose,
        "raw_output": completed.stdout,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))


if __name__ == "__main__":
    main()
