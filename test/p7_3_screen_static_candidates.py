#!/usr/bin/env python3
"""Screen geometry-NBV candidates with offline CS625 URDF/FK only.

This is a P7.3 pre-screen, not a P7.4 planning result.  It invokes the
existing static pose solver for a bounded prefix of the *already geometry-
ranked* candidates and records every rejection reason.  It never imports ROS,
MoveIt, or a trajectory interface.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


def preflight_decision(
    result: dict[str, Any], *, maximum_scaled_residual: float, minimum_link_origin_z_m: float
) -> list[str]:
    """Return explicit static-FK rejection codes for one solver result."""
    failures: list[str] = []
    if not result.get("solver_converged", False):
        failures.append("STATIC_IK_NOT_CONVERGED")
    if float(result.get("max_abs_scaled_residual", float("inf"))) > maximum_scaled_residual:
        failures.append("STATIC_IK_RESIDUAL_EXCEEDED")
    if float(result.get("minimum_distal_link_origin_z_m", float("-inf"))) < minimum_link_origin_z_m:
        failures.append("STATIC_FLOOR_CLEARANCE_INSUFFICIENT")
    if not result.get("projection", {}).get("inside_image", False):
        failures.append("STATIC_TARGET_PROJECTION_OUT_OF_FRAME")
    return failures


def solver_command(
    solver: Path,
    arguments: argparse.Namespace,
    candidate: dict[str, Any],
) -> list[str]:
    transform = candidate["candidate_pose_world_from_camera"]
    # argparse treats a negative scientific-notation token (for example
    # ``-1.2e-16``) as another option.  Candidate rotations commonly contain
    # these numerical-zero artefacts, so pass fixed-point literals instead.
    decimal = lambda value: format(float(value), ".17f")
    rotation = [decimal(value) for row in transform["rotation_matrix"] for value in row]
    translation = [decimal(value) for value in transform["translation_m"]]
    return [
        sys.executable,
        str(solver),
        "--urdf", str(arguments.urdf),
        "--seed-joints", str(arguments.seed_joints),
        "--target-pose", str(arguments.target_pose),
        "--desired-camera-position-m", *translation,
        "--desired-camera-rotation-matrix", *rotation,
        "--minimum-link-origin-z-m", str(arguments.minimum_link_origin_z_m),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rank", required=True, type=Path)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--seed-joints", required=True, type=Path)
    parser.add_argument("--target-pose", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-candidates", type=int, default=24)
    parser.add_argument("--maximum-scaled-residual", type=float, default=0.02)
    parser.add_argument("--minimum-link-origin-z-m", type=float, default=0.12)
    arguments = parser.parse_args()
    if arguments.output.exists():
        raise FileExistsError(f"refusing to overwrite screen result: {arguments.output}")
    if arguments.max_candidates < 1:
        raise ValueError("--max-candidates must be positive")

    rank = json.loads(arguments.rank.read_text(encoding="utf-8"))
    candidates = sorted(
        rank["candidates"], key=lambda item: float(item["selection_score"]), reverse=True
    )[: arguments.max_candidates]
    solver = Path(__file__).with_name("p7_solve_static_observation_pose.py")
    records = []
    for candidate in candidates:
        completed = subprocess.run(
            solver_command(solver, arguments, candidate),
            check=True,
            text=True,
            capture_output=True,
        )
        solver_result = json.loads(completed.stdout)
        failures = preflight_decision(
            solver_result,
            maximum_scaled_residual=arguments.maximum_scaled_residual,
            minimum_link_origin_z_m=arguments.minimum_link_origin_z_m,
        )
        records.append(
            {
                "candidate_id": candidate["candidate_id"],
                "selection_score": candidate["selection_score"],
                "geometry_candidate": candidate,
                "static_fk": solver_result,
                "preflight_pass": not failures,
                "failure_codes": failures,
            }
        )
    accepted = next((record for record in records if record["preflight_pass"]), None)
    result = {
        "schema": "p7_3_static_candidate_screen_v1",
        "gate": "P7.3_NBV_STATIC_PREFLIGHT",
        "claim_boundary": (
            "offline URDF/FK and image projection only; this is neither a P7.4 "
            "planning result nor trajectory execution evidence"
        ),
        "rank_input": str(arguments.rank),
        "screened_candidate_count": len(records),
        "maximum_scaled_residual": arguments.maximum_scaled_residual,
        "minimum_link_origin_z_m": arguments.minimum_link_origin_z_m,
        "recommended_candidate_id": None if accepted is None else accepted["candidate_id"],
        "gate_decision": (
            "P7.3_NO_STATIC_FEASIBLE_CANDIDATE"
            if accepted is None
            else "P7.3_STATIC_CANDIDATE_READY_FOR_RUNTIME_REOBSERVATION"
        ),
        "records": records,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "screened_candidate_count": len(records),
        "recommended_candidate_id": result["recommended_candidate_id"],
        "gate_decision": result["gate_decision"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
