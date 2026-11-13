#!/usr/bin/env python3
"""Validate the P7.2 estimator-to-NBV handoff without treating abstention as success."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--full-evaluation",
        action="append",
        required=True,
        metavar="STRATUM=PATH",
        help="A five-window external evaluation for an observable stratum.",
    )
    parser.add_argument("--severe-estimate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite handoff record: {args.output}")

    full_records = {}
    failures = []
    for item in args.full_evaluation:
        if "=" not in item:
            raise ValueError(f"expected STRATUM=PATH, got: {item}")
        stratum, raw_path = item.split("=", 1)
        record = load_json(Path(raw_path))
        full_records[stratum] = {
            "path": raw_path,
            "estimate_count": record.get("estimate_count"),
            "full_se3_pose_success_rate": record.get("full_se3_pose_success_rate"),
            "gate_pass": record.get("gate_pass"),
        }
        if record.get("estimate_count") != 5:
            failures.append(f"{stratum}:EXPECTED_FIVE_WINDOWS")
        if record.get("gate_pass") is not True:
            failures.append(f"{stratum}:FULL_SE3_GATE_FAILED")

    severe = load_json(args.severe_estimate)
    severe_quality = severe.get("quality", {})
    severe_covariance = severe.get("covariance_6x6", [])
    severe_codes = set(severe.get("failure_codes", []))
    severe_safe_reject = (
        severe.get("ground_truth_read") is False
        and severe.get("full_se3_gate_pass") is False
        and severe_quality.get("pose_valid_for_grasp") is False
        and severe_quality.get("texture_yaw_observable") is False
        and "POSE_YAW_UNOBSERVABLE" in severe_codes
        and len(severe_covariance) == 6
        and len(severe_covariance[5]) == 6
        and severe_covariance[5][5] >= math.pi**2 / 3.0 * 0.99
    )
    if not severe_safe_reject:
        failures.append("severe:UNOBSERVABILITY_REJECTION_INVALID")

    result = {
        "schema": "p7_2_handoff_gate_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "gate": "P7.2_POSE_ESTIMATION_HANDOFF",
        "observable_strata": full_records,
        "severe_rejection": {
            "path": str(args.severe_estimate),
            "ground_truth_read": severe.get("ground_truth_read"),
            "full_se3_gate_pass": severe.get("full_se3_gate_pass"),
            "pose_valid_for_grasp": severe_quality.get("pose_valid_for_grasp"),
            "texture_yaw_observable": severe_quality.get("texture_yaw_observable"),
            "yaw_variance_rad2": severe_covariance[5][5],
            "failure_codes": sorted(severe_codes),
            "safe_reject": severe_safe_reject,
        },
        "interface_gate_pass": not failures,
        "full_single_view_severe_pose_pass": False,
        "handoff_decision": (
            "P7.3_REQUIRED_REOBSERVATION"
            if not failures
            else "STOP_BEFORE_P7_3"
        ),
        "failure_codes": failures,
        "claim_boundary": (
            "A safe severe-view abstention is not full SE(3) success and is not "
            "a grasp authorization. It only permits P7.3 to seek another view."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
