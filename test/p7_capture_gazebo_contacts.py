#!/usr/bin/env python3
"""Capture Gazebo contact topics and classify robot/environment contacts.

This is deliberately independent from MoveIt collision checking.  It listens
to Gazebo's contact sensors while another process executes the selected view.
Only robot contacts are classified for the P7.4 view-motion metric; the
fixture's target/ground and target/occluder contacts are retained but are not
misreported as robot collisions.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys


DEFAULT_TOPICS = (
    "/p7/contacts/occluder",
    "/p7/contacts/target",
)


def pair_names(contact: dict) -> tuple[str, str]:
    return (
        str((contact.get("collision1") or {}).get("name", "")),
        str((contact.get("collision2") or {}).get("name", "")),
    )


def is_unexpected_robot_contact(pair: tuple[str, str]) -> bool:
    names = set(pair)
    robot_names = [name for name in names if name.startswith("cs::")]
    if not robot_names:
        return False
    other_names = names.difference(robot_names)
    # The fixed pedestal resting on the floor is the one expected robot-world
    # contact in both the start state and the selected NBV terminal state.
    if all("::base_link::" in name for name in robot_names) and any(
        name.startswith("ground_plane::") for name in other_names
    ):
        return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=25.0)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topic", action="append", dest="topics")
    args = parser.parse_args()
    topics = tuple(args.topics or DEFAULT_TOPICS)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite contact capture: {args.output}")
    if args.duration_sec <= 0.0:
        raise ValueError("duration must be positive")

    listed = subprocess.run(
        ["gz", "topic", "-l"], check=True, capture_output=True, text=True
    ).stdout.splitlines()
    advertised = {topic: topic in listed for topic in topics}
    if not all(advertised.values()):
        missing = [topic for topic, present in advertised.items() if not present]
        raise RuntimeError(f"Gazebo contact topics are not advertised: {missing}")

    processes: dict[str, subprocess.Popen[str]] = {}
    for topic in topics:
        processes[topic] = subprocess.Popen(
            [
                "gz",
                "topic",
                "--json-output",
                "-e",
                "-t",
                topic,
                "-d",
                str(args.duration_sec),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    print("P7_CONTACT_MONITOR_READY", flush=True)

    topic_records = {}
    pair_counts: dict[tuple[str, str], int] = {}
    parse_failures = 0
    for topic, process in processes.items():
        stdout, stderr = process.communicate(timeout=args.duration_sec + 15.0)
        message_count = 0
        contact_count = 0
        for line in stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                parse_failures += 1
                continue
            message_count += 1
            for contact in message.get("contact", []):
                pair = tuple(sorted(pair_names(contact)))
                if not all(pair):
                    continue
                contact_count += 1
                pair_counts[pair] = pair_counts.get(pair, 0) + 1
        topic_records[topic] = {
            "advertised": advertised[topic],
            "message_count": message_count,
            "contact_record_count": contact_count,
            "process_return_code": process.returncode,
            "stderr": stderr.strip(),
        }

    pairs = [
        {
            "collision_pair": list(pair),
            "record_count": count,
            "unexpected_robot_contact": is_unexpected_robot_contact(pair),
        }
        for pair, count in sorted(pair_counts.items())
    ]
    unexpected_pairs = [row for row in pairs if row["unexpected_robot_contact"]]
    monitor_available = all(
        row["advertised"] and row["process_return_code"] == 0
        for row in topic_records.values()
    )
    result = {
        "schema": "p7_gazebo_contact_capture_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_sec": args.duration_sec,
        "monitor_available": monitor_available,
        "classification_scope": "Gazebo runtime robot contacts with the severe-v5 occluder or target during selected view execution",
        "expected_robot_contact_rule": "no robot contact with the occluder or target during view motion",
        "topics": topic_records,
        "parse_failure_count": parse_failures,
        "collision_pairs": pairs,
        "unexpected_collision": bool(unexpected_pairs),
        "unexpected_collision_pair_count": len(unexpected_pairs),
        "claim_boundary": (
            "Gazebo physical-contact observation only; MoveIt state validity "
            "and collision rejection are reported separately"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, sort_keys=True))
    if not monitor_available or parse_failures:
        sys.exit(2)


if __name__ == "__main__":
    main()
