#!/usr/bin/env python3
"""Assemble one insertion episode from the per-leg receipts and the contact capture.

The insertion task reuses the P7 execution pieces rather than duplicating them: each
leg is driven by test/p7_execute_pose_capture.py, the arms by the P7 gripper adapter,
and contacts by test/p7_capture_gazebo_contacts.py.  What is specific to this task is
how those records are put together and what the result means, which is what this
script and the contract it feeds are for.

    python3 test/assemble_insertion_episode.py \
        --leg-dir <episode-dir>/legs \
        --contacts <episode-dir>/contacts.json \
        --sequence src/cs625_bringup/config/cs625_insertion_sequence.yaml \
        --output <episode-dir>/episode.json

The contact record is REQUIRED.  A failed descent is only a precision-fit jam if the
module touched the fixture on the way; without that record a tracking abort and a jam
are indistinguishable, and reporting the second as the first would be a claim the
evidence does not support.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "cs625_task_orchestrator"))

from cs625_task_orchestrator import insertion_episode_contract as contract  # noqa: E402


def load_leg(path: pathlib.Path) -> dict:
    """Reduce one p7_phase_receipt_v1 capture to what the contract reads."""

    record = json.loads(path.read_text(encoding="utf-8"))
    receipt = record.get("receipt")
    if not isinstance(receipt, dict):
        return {"result": "unrecorded", "code": "RECEIPT_MISSING"}
    return {
        "result": "succeeded" if receipt.get("success") is True else "failed",
        "code": str(receipt.get("code", "")),
        "receipt_schema": record.get("capture_schema"),
        "captured_at_utc": record.get("captured_at_utc"),
    }


def contact_summary(path: pathlib.Path) -> dict:
    """Reduce a contact capture to whether the module touched anything."""

    record = json.loads(path.read_text(encoding="utf-8"))
    contacts = record.get("contacts")
    if contacts is None:
        contacts = record.get("pairs")
    if isinstance(contacts, list):
        count = len(contacts)
    elif isinstance(contacts, dict):
        count = sum(len(v) for v in contacts.values() if isinstance(v, list))
    else:
        raise SystemExit(f"unsupported contact capture shape in {path.name}")
    return {
        "contacted": count > 0,
        "contact_count": count,
        "capture_schema": record.get("schema"),
        "topics": record.get("topics"),
        "duration_sec": record.get("duration_sec"),
    }


def assemble(leg_dir: pathlib.Path, contacts: pathlib.Path, sequence: pathlib.Path) -> dict:
    sequence_parameters = yaml.safe_load(sequence.read_text(encoding="utf-8"))[
        "cs625_insertion_sequence"
    ]["ros__parameters"]

    legs = {}
    for name in contract.REQUIRED_LEGS:
        path = leg_dir / f"{name}.json"
        legs[name] = load_leg(path) if path.is_file() else {
            "result": "unrecorded",
            "code": "LEG_NOT_RUN",
        }

    comparison = sequence_parameters.get("reference_mesh_comparison", {})
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "precision_fit": {
            "minimum_clearance_m": comparison.get("minimum_clearance_m"),
            "repository_asset_clearance_m": comparison.get(
                "repository_asset_minimum_clearance_m"
            ),
            "source": comparison.get("source"),
            "decision": (
                "recorded as a real constraint; the simulation is expected to show the "
                "position-controlled descent failing here"
            ),
        },
        "insertion_axis_world": sequence_parameters.get("insertion_axis_world"),
        "insertion_travel_m": sequence_parameters.get("insertion_travel_m"),
        "legs": legs,
        "contact_during_descent": contact_summary(contacts),
        "order": sequence_parameters.get("order"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg-dir", required=True, type=pathlib.Path)
    parser.add_argument("--contacts", required=True, type=pathlib.Path)
    parser.add_argument("--sequence", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    arguments = parser.parse_args()

    if not arguments.contacts.is_file():
        print(f"ERROR: {arguments.contacts} is missing", file=sys.stderr)
        return 2
    episode = assemble(arguments.leg_dir, arguments.contacts, arguments.sequence)
    verdict = contract.evaluate_insertion_episode(episode)
    episode["verdict"] = verdict

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(episode, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(verdict, sort_keys=True))
    # A jam is the expected outcome for a position-controlled descent through this
    # fit, and a clean pass is a surprise but still a result.  Only a verdict that
    # supports neither conclusion is an error.
    return 1 if verdict["verdict"] == "OTHER_FAILURE" else 0


if __name__ == "__main__":
    sys.exit(main())
