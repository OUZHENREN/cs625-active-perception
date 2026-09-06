#!/usr/bin/env python3
"""Generate P7 grasp candidates from an accepted RGB-D pose, without ROS.

Usage (world-frame estimate and an explicitly captured base <- world TF)::

    python3 test/p7_capture_tf_pose.py --parent-frame base_link \
        --child-frame world --output base_from_world.json
    python3 test/p7_generate_grasp_candidates.py \
        --estimate post_move_pose/window_01_estimate.json \
        --frame-transform base_from_world.json --output grasp_candidates.json

An estimate already in ``--target-frame`` needs no transform. The transform
must use the existing p7_tf_pose_v1 capture format: parent_frame is the target
frame, child_frame is the estimate's pose_frame, and pose is T_parent_child.
No identity world/base relationship is inferred. --lift-height-m describes
translation along target-frame +Z (default base_link +Z); it is a commanded
offset, not a measured object-height result.

The tomato template reuses the historical P7 r5 tool orientation and pregrasp
offset from run_p7_static_episode_capture.sh. Two additional templates use a
straight approach along tool +X with configurable standoff distances. All
positions/orientations are transformed from the current estimated CAD centre;
the YCB model-origin-to-centre offset must NOT be added again. These candidates
still require current-state IK, collision checking, planning and execution.
This script reads no Gazebo pose or external ground-truth evaluation file.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path


# Known-object grasp templates, not scene/world goal positions.
TEMPLATE_ORIENTATION_XYZW = (0.6199534, 0.7749167, -0.0769224, 0.0961499)
HISTORICAL_PREGRASP_OFFSET_M = (-0.0108315, -0.4240170, 0.2490725)
TEMPLATE_SOURCE = "test/run_p7_static_episode_capture.sh (historical tomato r5 template)"


def vector(value, size: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must contain {size} numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
        raise ValueError(f"{label} must contain finite numeric values")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{label} must contain finite numeric values")
    return result


def unit_quaternion(value, label: str) -> tuple[float, ...]:
    result = vector(value, 4, label)
    norm = math.sqrt(sum(v * v for v in result))
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError(f"{label} must be a unit xyzw quaternion")
    return tuple(v / norm for v in result)


def multiply(left, right) -> tuple[float, ...]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate(point, quaternion) -> tuple[float, ...]:
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    return multiply(multiply(quaternion, (*point, 0.0)), inverse)[:3]


def add(left, right) -> tuple[float, ...]:
    return tuple(a + b for a, b in zip(left, right))


def pose_record(position, orientation, frame: str) -> dict:
    return {
        "frame_id": frame,
        "position": dict(zip("xyz", position)),
        "orientation": dict(zip("xyzw", orientation)),
    }


def generate_candidates(
    estimate: dict,
    frame_transform: dict | None = None,
    *,
    target_frame: str = "base_link",
    physical_tool_frame: str = "p7_grasp_center_link",
    lift_height_m: float = 0.12,
    standoffs_m: tuple[float, ...] = (0.15, 0.25),
    template_yaws_deg: tuple[float, ...] = (0.0,),
    level_side_yaws_deg: tuple[float, ...] = (),
    top_grasp_yaws_deg: tuple[float, ...] = (),
    top_grasp_height_offset_m: float = 0.03,
    top_grasp_pitch_deg: float = 90.0,
    top_grasp_x_offset_m: float = 0.0,
) -> dict:
    """Transform known-object templates using only the estimator's own output."""
    if estimate.get("schema") != "p7_2_pose_estimate_v1":
        raise ValueError("expected p7_2_pose_estimate_v1 estimator output")
    if estimate.get("ground_truth_read") is not False:
        raise ValueError("estimate must explicitly declare ground_truth_read=false")
    if estimate.get("full_se3_gate_pass") is not True:
        raise ValueError("estimator full_se3_gate_pass is not true")
    if (estimate.get("quality") or {}).get("pose_valid_for_grasp") is not True:
        raise ValueError("estimator quality.pose_valid_for_grasp is not true")
    source_frame = estimate.get("pose_frame")
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ValueError("estimate needs an explicit pose_frame")
    if not target_frame.strip() or not physical_tool_frame.strip():
        raise ValueError("target and physical tool frames must be explicit")
    lift_height = vector((lift_height_m,), 1, "lift_height_m")[0]
    if lift_height <= 0.0:
        raise ValueError("lift_height_m must be positive")
    distances = vector(standoffs_m, len(standoffs_m), "standoffs_m")
    if any(distance <= 0.0 for distance in distances):
        raise ValueError("standoffs_m must be positive")

    pose = estimate.get("pose") or {}
    center = vector(pose.get("position_m"), 3, "pose.position_m")
    object_q = unit_quaternion(pose.get("orientation_xyzw"), "pose.orientation_xyzw")
    if source_frame == target_frame:
        if frame_transform is not None:
            raise ValueError("same-frame estimate needs no frame transform")
        transform_record = {"kind": "same_frame", "frame": target_frame}
    else:
        if frame_transform is None:
            raise ValueError(f"explicit {target_frame} <- {source_frame} TF required")
        if (
            frame_transform.get("capture_schema") != "p7_tf_pose_v1"
            or frame_transform.get("parent_frame") != target_frame
            or frame_transform.get("child_frame") != source_frame
        ):
            raise ValueError("TF schema or direction does not match target <- estimate frame")
        tf_pose = frame_transform.get("pose") or {}
        translation = tf_pose.get("position") or {}
        orientation = tf_pose.get("orientation") or {}
        tf_p = vector([translation.get(a) for a in "xyz"], 3, "TF translation")
        tf_q = unit_quaternion([orientation.get(a) for a in "xyzw"], "TF orientation")
        center = add(tf_p, rotate(center, tf_q))
        object_q = multiply(tf_q, object_q)
        transform_record = {"kind": "captured_tf", "record": frame_transform}

    template_q = unit_quaternion(TEMPLATE_ORIENTATION_XYZW, "template quaternion")
    approach_axis = rotate((1.0, 0.0, 0.0), template_q)
    templates = [("historical_r5", HISTORICAL_PREGRASP_OFFSET_M)]
    templates.extend(
        (f"tool_x_standoff_{index + 1}", tuple(-distance * v for v in approach_axis))
        for index, distance in enumerate(distances)
    )
    candidates = []
    yaw_angles = vector(template_yaws_deg, len(template_yaws_deg), "template_yaws_deg")
    for yaw_deg in yaw_angles:
      half_angle = math.radians(yaw_deg) / 2.0
      yaw_q = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
      template_frame_q = multiply(object_q, yaw_q)
      grasp_q = unit_quaternion(multiply(template_frame_q, template_q), "grasp quaternion")
      for name, pregrasp_offset in templates:
        yaw_label = "" if yaw_deg == 0.0 else f"yaw{yaw_deg:g}_"
        candidates.append({
            "candidate_id": f"tomato_{yaw_label}{name}",
            "physical_tool_frame": physical_tool_frame,
            "template_yaw_deg": yaw_deg,
            "pregrasp_offset_object_m": list(rotate(pregrasp_offset, yaw_q)),
            "pregrasp": pose_record(add(center, rotate(pregrasp_offset, template_frame_q)), grasp_q, target_frame),
            "grasp": pose_record(center, grasp_q, target_frame),
            "lift": pose_record(add(center, (0.0, 0.0, lift_height)), grasp_q, target_frame),
            "feasibility_status": "NOT_EVALUATED",
        })
    # Geometric side templates keep tool Z along the estimated can axis.
    # They retain all subsequent IK, collision and physical-contact gates.
    side_yaws = vector(level_side_yaws_deg, len(level_side_yaws_deg), "level_side_yaws_deg")
    for yaw_deg in side_yaws:
        half_angle = math.radians(yaw_deg) / 2.0
        yaw_q = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
        grasp_q = unit_quaternion(multiply(object_q, yaw_q), "side grasp quaternion")
        for index, distance in enumerate(distances):
            offset = rotate((-distance, 0.0, 0.0), yaw_q)
            candidates.append({
                "candidate_id": f"tomato_level_yaw{yaw_deg:g}_standoff_{index + 1}",
                "physical_tool_frame": physical_tool_frame,
                "template_source": "CS625 gripper CAD: side approach, tool Z along can axis",
                "template_yaw_deg": yaw_deg,
                "pregrasp_offset_object_m": list(offset),
                "pregrasp": pose_record(add(center, rotate(offset, object_q)), grasp_q, target_frame),
                "grasp": pose_record(center, grasp_q, target_frame),
                "lift": pose_record(add(center, (0.0, 0.0, lift_height)), grasp_q, target_frame),
                "feasibility_status": "NOT_EVALUATED",
            })
    # Top-down pad contact on the upper can wall. Moving the reference 30 mm
    # above the CAD centre leaves the palm above the 101.855 mm tall can;
    # the pads still overlap its wall. The inherited flange offset is kept.
    top_yaws = vector(top_grasp_yaws_deg, len(top_grasp_yaws_deg), "top_grasp_yaws_deg")
    top_height = vector((top_grasp_height_offset_m,), 1, "top_grasp_height_offset_m")[0]
    if not 0.0 <= top_height <= 0.04:
        raise ValueError("top grasp height must stay on the known can wall")
    top_x = vector((top_grasp_x_offset_m,), 1, "top_grasp_x_offset_m")[0]
    pitch = vector((top_grasp_pitch_deg,), 1, "top_grasp_pitch_deg")[0]
    if abs(top_x) > 0.015 or not 60.0 <= pitch <= 120.0:
        raise ValueError("top template must retain pad overlap with the known can")
    top_center = add(center, rotate((top_x, 0.0, top_height), object_q))
    pitch_half = math.radians(pitch) / 2.0
    pitch_q = (0.0, math.sin(pitch_half), 0.0, math.cos(pitch_half))
    for yaw_deg in top_yaws:
        half_angle = math.radians(yaw_deg) / 2.0
        yaw_q = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
        grasp_q = unit_quaternion(multiply(object_q, multiply(yaw_q, pitch_q)), "top grasp quaternion")
        for index, distance in enumerate(distances):
            candidates.append({
                "candidate_id": f"tomato_top_yaw{yaw_deg:g}_standoff_{index + 1}",
                "physical_tool_frame": physical_tool_frame,
                "template_source": "CS625 gripper CAD and 101.855 mm can: top approach to upper sidewall",
                "template_yaw_deg": yaw_deg,
                "grasp_offset_object_m": [top_x, 0.0, top_height],
                "template_pitch_deg": pitch,
                "pregrasp": pose_record(add(top_center, rotate((-distance, 0.0, 0.0), grasp_q)), grasp_q, target_frame),
                "grasp": pose_record(top_center, grasp_q, target_frame),
                "lift": pose_record(add(top_center, (0.0, 0.0, lift_height)), grasp_q, target_frame),
                "feasibility_status": "NOT_EVALUATED",
            })
    return {
        "schema": "p7_grasp_candidates_v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "target_object": "005_tomato_soup_can",
        "source": {
            "estimator": estimate.get("estimator"),
            "captured_at_utc": estimate.get("captured_at_utc"),
            "input_window": estimate.get("input_window"),
            "pose_frame": source_frame,
            "pose": pose,
            "covariance_6x6_in_source_frame": estimate.get("covariance_6x6"),
            "ground_truth_read": False,
        },
        "frame_transform": transform_record,
        "template_source": TEMPLATE_SOURCE,
        "template_orientation_object_xyzw": list(template_q),
        "target_center_target_frame_m": list(center),
        "target_frame": target_frame,
        "lift_offset_target_frame_m": [0.0, 0.0, lift_height],
        "candidate_count": len(candidates),
        "candidates": candidates,
        "claim_boundary": "Estimated-pose-conditioned grasp candidates only; no GT scoring, motion, contact or grasp success is inferred",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--frame-transform", type=Path)
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--physical-tool-frame", default="p7_grasp_center_link")
    parser.add_argument("--lift-height-m", type=float, default=0.12)
    parser.add_argument("--standoffs-m", type=float, nargs="+", default=(0.15, 0.25))
    parser.add_argument("--template-yaws-deg", type=float, nargs="+", default=(0.0,))
    parser.add_argument("--level-side-yaws-deg", type=float, nargs="+", default=())
    parser.add_argument("--top-grasp-yaws-deg", type=float, nargs="+", default=())
    parser.add_argument("--top-grasp-height-offset-m", type=float, default=0.03)
    parser.add_argument("--top-grasp-pitch-deg", type=float, default=90.0)
    parser.add_argument("--top-grasp-x-offset-m", type=float, default=0.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    estimate = json.loads(args.estimate.read_text(encoding="utf-8"))
    transform = json.loads(args.frame_transform.read_text(encoding="utf-8")) if args.frame_transform else None
    result = generate_candidates(
        estimate, transform, target_frame=args.target_frame,
        physical_tool_frame=args.physical_tool_frame,
        lift_height_m=args.lift_height_m, standoffs_m=tuple(args.standoffs_m),
        template_yaws_deg=tuple(args.template_yaws_deg),
        level_side_yaws_deg=tuple(args.level_side_yaws_deg),
        top_grasp_yaws_deg=tuple(args.top_grasp_yaws_deg),
        top_grasp_height_offset_m=args.top_grasp_height_offset_m,
        top_grasp_pitch_deg=args.top_grasp_pitch_deg,
        top_grasp_x_offset_m=args.top_grasp_x_offset_m,
    )
    result["source"]["estimate_file"] = str(args.estimate)
    if args.frame_transform:
        result["frame_transform"]["source_file"] = str(args.frame_transform)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "candidate_count": result["candidate_count"]}))


if __name__ == "__main__":
    main()
