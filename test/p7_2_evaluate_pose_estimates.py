#!/usr/bin/env python3
"""Evaluate completed P7.2 estimates; ground truth is read only in this scorer."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


LOCAL_CENTER = np.array((-0.0091685, 0.0840170, 0.0510065), dtype=float)


def quaternion(record: dict) -> Rotation:
    return Rotation.from_quat([record[key] for key in ("x", "y", "z", "w")])


def obj_vertices(path: Path) -> np.ndarray:
    vertices = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if line.startswith("v "):
            vertices.append([float(value) for value in line.split()[1:4]])
    if not vertices:
        raise ValueError("OBJ has no vertices")
    points = np.asarray(vertices, dtype=float) - LOCAL_CENTER
    return points[:: max(1, len(points) // 4000)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate-dir", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--model-obj", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--occlusion-stratum", default="unspecified")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite evaluation: {args.output}")

    gt_record = json.loads(args.ground_truth.read_text(encoding="utf-8"))["pose"]
    gt_rotation = quaternion(gt_record["orientation"])
    gt_origin = np.array([gt_record["position"][key] for key in ("x", "y", "z")])
    gt_center = gt_origin + gt_rotation.apply(LOCAL_CENTER)
    model = obj_vertices(args.model_obj)
    gt_cloud = gt_rotation.apply(model) + gt_center
    rows = []
    for path in sorted(args.estimate_dir.glob("window_*_estimate.json")):
        estimate = json.loads(path.read_text(encoding="utf-8"))
        position = np.asarray(estimate["pose"]["position_m"], dtype=float)
        estimated_rotation = Rotation.from_quat(
            estimate["pose"]["orientation_xyzw"]
        )
        rotation_error = (gt_rotation.inv() * estimated_rotation).magnitude()
        estimate_cloud = estimated_rotation.apply(model) + position
        add = float(np.linalg.norm(estimate_cloud - gt_cloud, axis=1).mean())
        adds = float(cKDTree(gt_cloud).query(estimate_cloud, workers=-1)[0].mean())
        translation_error = float(np.linalg.norm(position - gt_center))
        full_observable = bool(estimate["quality"]["texture_yaw_observable"])
        rows.append(
            {
                "window": path.stem.replace("_estimate", ""),
                "translation_error_m": translation_error,
                "rotation_error_deg": float(np.degrees(rotation_error)),
                "add_m": add,
                "add_s_m": adds,
                "translation_success_20mm": bool(translation_error <= 0.020),
                "metric_success": bool(
                    translation_error <= 0.020
                    and rotation_error <= np.radians(5.0)
                    and adds <= 0.020
                ),
                "full_se3_observable": full_observable,
                "gate_window_pass": bool(
                    full_observable
                    and translation_error <= 0.020
                    and rotation_error <= np.radians(5.0)
                    and adds <= 0.020
                ),
            }
        )
    failure_codes = []
    if not rows:
        failure_codes.append("POSE_ESTIMATE_MISSING")
    if any(not row["full_se3_observable"] for row in rows):
        failure_codes.append("POSE_YAW_UNOBSERVABLE")
    if any(not row["translation_success_20mm"] for row in rows):
        failure_codes.append("POSE_TRANSLATION_ERROR")
    if any(row["rotation_error_deg"] > 5.0 for row in rows):
        failure_codes.append("POSE_ROTATION_ERROR")
    if any(row["add_s_m"] > 0.020 for row in rows):
        failure_codes.append("POSE_ADD_S_ERROR")
    result = {
        "schema": "p7_2_pose_evaluation_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "ground_truth_use": "evaluation_only_after_estimates_were_immutable",
        "estimate_count": len(rows),
        "occlusion_stratum": args.occlusion_stratum,
        "thresholds": {
            "translation_m": 0.020,
            "rotation_deg": 5.0,
            "add_s_m": 0.020,
        },
        "windows": rows,
        "translation_pose_success_rate": float(
            np.mean([row["translation_success_20mm"] for row in rows])
        ),
        "full_se3_pose_success_rate": float(
            np.mean([row["gate_window_pass"] for row in rows])
        ),
        "gate_pass": bool(rows) and all(row["gate_window_pass"] for row in rows),
        "failure_codes": sorted(set(failure_codes)),
        "claim_boundary": (
            f"single frozen scene in occlusion stratum {args.occlusion_stratum}; "
            "no NBV, MoveIt execution, or grasp was evaluated"
        ),
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
