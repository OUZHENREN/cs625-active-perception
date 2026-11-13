#!/usr/bin/env python3
"""Assemble one P7 fixture episode from recorded files without filling gaps."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from cs625_task_orchestrator.p7_episode_contract import evaluate_episode


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-id", default="p7_ycb_tomato_light")
    parser.add_argument("--strategy", default="fixture_pose_control_only")
    parser.add_argument("--random-seed", type=int, default=0)
    parser.add_argument(
        "--single-attempt-runner",
        action="store_true",
        help="assert that the tracked abort-on-failure runner issued each phase once",
    )
    for name in (
        "preflight", "episode_start", "detach", "scene_full", "pregrasp", "scene_approach",
        "approach", "gripper", "geometry", "attachment", "scene_attached",
        "lift", "lift_measurement", "hold", "episode_end",
    ):
        parser.add_argument(f"--{name.replace('_', '-')}", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    decoded = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return decoded


def receipt(record: dict[str, Any], schema: str) -> dict[str, Any]:
    if record.get("capture_schema") != schema or not isinstance(record.get("receipt"), dict):
        raise ValueError(f"expected {schema} receipt envelope")
    return record["receipt"]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_sum(values: list[Any]) -> float | None:
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        for value in values
    ):
        return None
    return sum(float(value) for value in values)


def main() -> None:
    arguments = parse_arguments()
    paths = {
        name: getattr(arguments, name)
        for name in (
            "preflight", "episode_start", "detach", "scene_full", "pregrasp", "scene_approach",
            "approach", "gripper", "geometry", "attachment", "scene_attached",
            "lift", "lift_measurement", "hold", "episode_end",
        )
    }
    records = {name: load(path) for name, path in paths.items()}
    e0 = receipt(records["detach"], "p7_attachment_receipt_v1")
    e1 = receipt(records["pregrasp"], "p7_phase_receipt_v1")
    e2 = receipt(records["approach"], "p7_phase_receipt_v1")
    e3_gripper = receipt(records["gripper"], "p7_gripper_receipt_v1")
    e3_attachment = receipt(records["attachment"], "p7_attachment_receipt_v1")
    e4 = receipt(records["lift"], "p7_phase_receipt_v1")
    geometry = records["geometry"]
    lift_measurement = records["lift_measurement"]
    hold = records["hold"]

    scene_modes_ok = all(
        records[name].get("success") is True
        and records[name].get("mode") == expected_mode
        for name, expected_mode in (
            ("scene_full", "full"),
            ("scene_approach", "approach"),
            ("scene_attached", "attached"),
        )
    )
    evidence = {
        "scene_id": arguments.scene_id,
        "strategy": arguments.strategy,
        "random_seed": arguments.random_seed,
        "perception": {
            "fresh_rgbd": False,
            "visible_ratio": None,
            "translation_error_m": None,
            "rotation_error_deg": None,
            "source": "not_executed_fixture_pose_control_only",
        },
        "pregrasp": {
            "plan_success": e1.get("success") is True,
            "execution_success": e1.get("success") is True,
            "receipt": e1,
        },
        "approach": {
            "plan_success": e2.get("success") is True,
            "execution_success": e2.get("success") is True,
            "receipt": e2,
        },
        "gripper": {
            "command_reached": e3_gripper.get("success") is True,
            "geometry_measurement_available": geometry.get("geometry_measurement_available") is True,
            "target_to_grasp_center_error_m": geometry.get("target_to_grasp_center_error_m"),
            "geometry_pass": geometry.get("geometry_pass") is True,
            "receipt": e3_gripper,
        },
        "attachment": {
            "attached": e3_attachment.get("attached") is True
            and e3_attachment.get("success") is True,
            "relative_translation_error_m": geometry.get("target_to_grasp_center_error_m"),
            "mechanism": e3_attachment.get("mechanism"),
            "receipt": e3_attachment,
        },
        "lift": {
            "plan_success": e4.get("success") is True,
            "execution_success": e4.get("success") is True,
            "height_delta_m": lift_measurement.get("height_delta_m"),
            "receipt": e4,
        },
        "hold": {
            "duration_sec": hold.get("duration_sec"),
            "stable": hold.get("stable") is True,
            "max_height_drift_m": hold.get("max_height_drift_m"),
        },
        "contact": {
            "monitor_available": False,
            "unexpected_collision": None,
            "source": "not_instrumented",
        },
    }
    engineering_chain_success = all((
        records["preflight"].get("success") is True,
        e0.get("success") is True and e0.get("attached") is False,
        scene_modes_ok,
        e1.get("success") is True,
        e2.get("success") is True,
        e3_gripper.get("success") is True,
        geometry.get("geometry_pass") is True,
        e3_attachment.get("success") is True and e3_attachment.get("attached") is True,
        e4.get("success") is True,
        lift_measurement.get("height_gate_pass") is True,
        hold.get("stable") is True,
    ))
    phase_sum = finite_sum([
        e1.get("total_time_sec"),
        e2.get("total_time_sec"),
        e3_gripper.get("elapsed_sec"),
        e3_attachment.get("elapsed_sec"),
        e4.get("total_time_sec"),
        hold.get("duration_sec"),
    ])
    start_marker = records["episode_start"]
    end_marker = records["episode_end"]
    marker_identity_ok = (
        start_marker.get("capture_schema") == "p7_episode_marker_v1"
        and start_marker.get("stage") == "start"
        and end_marker.get("capture_schema") == "p7_episode_marker_v1"
        and end_marker.get("stage") == "end"
        and start_marker.get("boot_id")
        and start_marker.get("boot_id") == end_marker.get("boot_id")
    )
    episode_wall_time = None
    if marker_identity_ok:
        episode_wall_time = finite_sum([
            end_marker.get("monotonic_ns"), -start_marker.get("monotonic_ns")
        ])
        if episode_wall_time is not None:
            episode_wall_time /= 1_000_000_000.0
            if episode_wall_time < 0.0:
                episode_wall_time = None
    evaluation = evaluate_episode(evidence)
    record = {
        "episode_schema": "p7_attachment_assisted_fixture_episode_v1",
        "assembled_at_utc": datetime.now(timezone.utc).isoformat(),
        **evidence,
        "scene_transitions": {
            "full": records["scene_full"],
            "approach": records["scene_approach"],
            "attached": records["scene_attached"],
        },
        "timing": {
            "recorded_phase_sum_sec": phase_sum,
            "episode_wall_time_sec": episode_wall_time,
            "episode_wall_time_reason": (
                "boot-scoped monotonic markers"
                if episode_wall_time is not None
                else "invalid or cross-boot episode markers"
            ),
            "replan_after_execution_failure_count": (
                0 if arguments.single_attempt_runner else None
            ),
            "replan_count_reason": (
                "tracked runner aborts on the first failed phase; this successful run issued each phase once"
                if arguments.single_attempt_runner
                else "manual chain without an attempt event log"
            ),
        },
        "engineering_acceptance": {
            "attachment_assisted_kinematic_chain_success": engineering_chain_success,
            "carried_object_collision_proxy_applied": records["scene_attached"].get("mode") == "attached",
            "real_contact_grasp_validated": False,
            "perception_aware_grasp_validated": False,
            "formal_p7_episode_accepted": evaluation["task_success"],
        },
        "contract_evaluation": evaluation,
        "source_files": {
            name: {"path": str(path), "sha256": sha256(path)}
            for name, path in paths.items()
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    if not engineering_chain_success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
