#!/usr/bin/env python3
"""Write a boot-scoped monotonic start/end marker for one P7 episode."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("start", "end"), required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    boot_id_path = Path("/proc/sys/kernel/random/boot_id")
    record = {
        "capture_schema": "p7_episode_marker_v1",
        "stage": arguments.stage,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "monotonic_ns": time.monotonic_ns(),
        "boot_id": boot_id_path.read_text(encoding="utf-8").strip(),
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
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
