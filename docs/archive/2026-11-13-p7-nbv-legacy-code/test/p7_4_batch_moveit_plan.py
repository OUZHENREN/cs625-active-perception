#!/usr/bin/env python3
"""Batch-screen strict P7.3 candidates through the P7.4 MoveIt gates.

Every candidate is evaluated from the same explicit, versioned arm start
state.  The tool calls MoveIt IK, state-validity and motion-plan services but
never sends a controller goal.  It therefore measures MoveIt planning
feasibility and collision rejection, not Gazebo physical contact or execution
success.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
WRAPPED_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}
DEFAULT_WORKSPACE_MIN = (-1.20, -1.20, 0.05)
DEFAULT_WORKSPACE_MAX = (1.80, 1.20, 1.80)
DEFAULT_JOINT_MIN = (
    -6.283185307,
    -6.283185307,
    -3.141592654,
    -6.283185307,
    -6.283185307,
    -6.283185307,
)
DEFAULT_JOINT_MAX = (
    6.283185307,
    6.283185307,
    3.141592654,
    6.283185307,
    6.283185307,
    6.283185307,
)
GATE_ORDER = ("fov", "workspace", "ik", "joint_limit", "collision", "planning")

MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_ACQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -31: "NO_IK_SOLUTION",
}
EXPLICIT_COLLISION_CODES = {-10, -12}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--screen", required=True, type=Path)
    parser.add_argument("--start-joints", required=True, type=Path)
    parser.add_argument(
        "--scene-receipt",
        required=True,
        type=Path,
        help="successful p7_apply_fixture_scene.py receipt for the active scene",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--expected-candidate-count", type=int, default=25)
    parser.add_argument("--required-scene-profile", default="severe_v5")
    parser.add_argument("--planning-frame", default="base_link")
    parser.add_argument("--camera-frame", default="camera_depth_optical_frame")
    parser.add_argument("--planning-tip", default="my_end_effector_link")
    parser.add_argument("--planning-group", default="cs625_arm")
    parser.add_argument("--workspace-min", nargs=3, type=float, default=DEFAULT_WORKSPACE_MIN)
    parser.add_argument("--workspace-max", nargs=3, type=float, default=DEFAULT_WORKSPACE_MAX)
    parser.add_argument("--joint-limit-min", nargs=6, type=float, default=DEFAULT_JOINT_MIN)
    parser.add_argument("--joint-limit-max", nargs=6, type=float, default=DEFAULT_JOINT_MAX)
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--state-validity-service", default="/check_state_validity")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--service-wait-sec", type=float, default=15.0)
    parser.add_argument("--request-timeout-sec", type=float, default=5.0)
    parser.add_argument("--ik-timeout-sec", type=float, default=1.0)
    parser.add_argument("--planning-time-sec", type=float, default=2.0)
    parser.add_argument("--planning-attempts", type=int, default=1)
    parser.add_argument("--goal-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--velocity-scale", type=float, default=0.25)
    parser.add_argument("--acceleration-scale", type=float, default=0.25)
    arguments = parser.parse_args()
    if arguments.expected_candidate_count < 1:
        parser.error("--expected-candidate-count must be positive")
    if arguments.planning_attempts < 1:
        parser.error("--planning-attempts must be positive")
    for name in (
        "service_wait_sec",
        "request_timeout_sec",
        "ik_timeout_sec",
        "planning_time_sec",
        "goal_tolerance_rad",
    ):
        if not math.isfinite(getattr(arguments, name)) or getattr(arguments, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if any(lower >= upper for lower, upper in zip(arguments.workspace_min, arguments.workspace_max)):
        parser.error("each workspace minimum must be below its maximum")
    if any(lower >= upper for lower, upper in zip(arguments.joint_limit_min, arguments.joint_limit_max)):
        parser.error("each joint minimum must be below its maximum")
    if arguments.output.exists():
        parser.error(f"refusing to overwrite output: {arguments.output}")
    return arguments


def strict_candidates(screen: dict[str, Any], expected_count: int) -> list[dict[str, Any]]:
    """Select only records that passed every frozen P7.3 static gate."""

    if screen.get("schema") != "p7_3_static_candidate_screen_v1":
        raise ValueError("P7_4_SCREEN_SCHEMA_UNSUPPORTED")
    selected = [
        record
        for record in screen.get("records", [])
        if record.get("preflight_pass") is True and not record.get("failure_codes")
    ]
    identifiers = [str(record.get("candidate_id", "")) for record in selected]
    if len(selected) != expected_count:
        raise ValueError(
            f"P7_4_STRICT_CANDIDATE_COUNT_MISMATCH expected={expected_count} actual={len(selected)}"
        )
    if not all(identifiers) or len(set(identifiers)) != len(identifiers):
        raise ValueError("P7_4_CANDIDATE_IDS_INVALID")
    return selected


def load_joint_map(path: Path) -> dict[str, float]:
    decoded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError("P7_4_START_STATE_INVALID")
    try:
        result = {name: float(decoded[name]) for name in JOINT_NAMES}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("P7_4_START_STATE_INVALID") from error
    if not all(math.isfinite(value) for value in result.values()):
        raise ValueError("P7_4_START_STATE_INVALID")
    return result


def validate_scene_receipt(path: Path, required_profile: str) -> dict[str, Any]:
    receipt = json.loads(path.read_text(encoding="utf-8"))
    if (
        receipt.get("capture_schema") != "p7_planning_scene_transition_v1"
        or receipt.get("success") is not True
        or receipt.get("mode") != "full"
        or receipt.get("profile") != required_profile
    ):
        raise ValueError("P7_4_SCENE_RECEIPT_INVALID")
    return receipt


def fov_gate(record: dict[str, Any]) -> dict[str, Any]:
    projection = record.get("static_fk", {}).get("projection", {})
    try:
        u = float(projection["u_px"])
        v = float(projection["v_px"])
        width = int(projection["width"])
        height = int(projection["height"])
    except (KeyError, TypeError, ValueError):
        return {"evaluated": True, "passed": False, "reason": "FOV_EVIDENCE_MISSING"}
    passed = (
        projection.get("inside_image") is True
        and width > 0
        and height > 0
        and math.isfinite(u)
        and math.isfinite(v)
        and 0.0 <= u < width
        and 0.0 <= v < height
    )
    return {
        "evaluated": True,
        "passed": passed,
        "reason": None if passed else "FOV_REJECTED",
        "u_px": u,
        "v_px": v,
        "width": width,
        "height": height,
    }


def workspace_gate(
    position: list[float] | tuple[float, ...],
    minimum: list[float] | tuple[float, ...],
    maximum: list[float] | tuple[float, ...],
) -> dict[str, Any]:
    try:
        xyz = tuple(float(value) for value in position)
    except (TypeError, ValueError):
        xyz = ()
    passed = (
        len(xyz) == 3
        and all(math.isfinite(value) for value in xyz)
        and all(lower <= value <= upper for value, lower, upper in zip(xyz, minimum, maximum))
    )
    return {
        "evaluated": True,
        "passed": passed,
        "reason": None if passed else "WORKSPACE_REJECTED",
        "camera_position_m": list(xyz),
        "workspace_min_m": list(minimum),
        "workspace_max_m": list(maximum),
    }


def joint_limit_gate(
    joints: dict[str, float],
    minimum: dict[str, float],
    maximum: dict[str, float],
) -> dict[str, Any]:
    violations = []
    for name in JOINT_NAMES:
        value = joints.get(name, float("nan"))
        if not math.isfinite(value) or value < minimum[name] or value > maximum[name]:
            violations.append(
                {"joint": name, "position_rad": value, "minimum_rad": minimum[name], "maximum_rad": maximum[name]}
            )
    return {
        "evaluated": True,
        "passed": not violations,
        "reason": None if not violations else "JOINT_LIMIT_REJECTED",
        "violations": violations,
    }


def quaternion_from_rotation_matrix(matrix: list[list[float]]) -> tuple[float, float, float, float]:
    """Convert a proper 3x3 rotation matrix to a normalized x/y/z/w quaternion."""

    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("P7_4_CAMERA_ROTATION_INVALID")
    m = [[float(value) for value in row] for row in matrix]
    if not all(math.isfinite(value) for row in m for value in row):
        raise ValueError("P7_4_CAMERA_ROTATION_INVALID")
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m[2][1] - m[1][2]) / scale,
            (m[0][2] - m[2][0]) / scale,
            (m[1][0] - m[0][1]) / scale,
            0.25 * scale,
        )
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        quaternion = (
            0.25 * scale,
            (m[0][1] + m[1][0]) / scale,
            (m[0][2] + m[2][0]) / scale,
            (m[2][1] - m[1][2]) / scale,
        )
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        quaternion = (
            (m[0][1] + m[1][0]) / scale,
            0.25 * scale,
            (m[1][2] + m[2][1]) / scale,
            (m[0][2] - m[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        quaternion = (
            (m[0][2] + m[2][0]) / scale,
            (m[1][2] + m[2][1]) / scale,
            0.25 * scale,
            (m[1][0] - m[0][1]) / scale,
        )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("P7_4_CAMERA_ROTATION_INVALID")
    return tuple(value / norm for value in quaternion)


def multiply_quaternion(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(
    vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = multiply_quaternion(
        multiply_quaternion(quaternion, (*vector, 0.0)), inverse
    )
    return rotated[0], rotated[1], rotated[2]


def compose_pose(base: Pose, relative: Pose) -> Pose:
    base_q = (
        base.orientation.x,
        base.orientation.y,
        base.orientation.z,
        base.orientation.w,
    )
    relative_q = (
        relative.orientation.x,
        relative.orientation.y,
        relative.orientation.z,
        relative.orientation.w,
    )
    translated = rotate_vector(
        (relative.position.x, relative.position.y, relative.position.z), base_q
    )
    result = Pose()
    result.position.x = base.position.x + translated[0]
    result.position.y = base.position.y + translated[1]
    result.position.z = base.position.z + translated[2]
    (
        result.orientation.x,
        result.orientation.y,
        result.orientation.z,
        result.orientation.w,
    ) = multiply_quaternion(base_q, relative_q)
    return result


def pose_from_transform(transform) -> Pose:
    result = Pose()
    result.position.x = transform.translation.x
    result.position.y = transform.translation.y
    result.position.z = transform.translation.z
    result.orientation = transform.rotation
    return result


def candidate_camera_pose(record: dict[str, Any], frame_id: str) -> PoseStamped:
    transform = record["geometry_candidate"]["candidate_pose_world_from_camera"]
    quaternion = quaternion_from_rotation_matrix(transform["rotation_matrix"])
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
        float(value) for value in transform["translation_m"]
    )
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = quaternion
    return pose


def duration_message(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1.0e9)))


def moveit_error_name(code: int | None) -> str | None:
    if code is None:
        return None
    return MOVEIT_ERROR_NAMES.get(int(code), f"UNKNOWN_MOVEIT_ERROR_{int(code)}")


def nearest_wrapped_solution(
    solution: JointState, reference: dict[str, float]
) -> dict[str, float]:
    decoded = {name: float(value) for name, value in zip(solution.name, solution.position)}
    result = {}
    for name in JOINT_NAMES:
        if name not in decoded:
            raise ValueError(f"P7_4_IK_SOLUTION_MISSING_JOINT:{name}")
        value = decoded[name]
        if name in WRAPPED_JOINTS:
            value += round((reference[name] - value) / (2.0 * math.pi)) * (2.0 * math.pi)
        result[name] = value
    return result


def goal_constraints(joints: dict[str, float], tolerance: float) -> Constraints:
    result = Constraints()
    for name in JOINT_NAMES:
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = joints[name]
        constraint.tolerance_above = tolerance
        constraint.tolerance_below = tolerance
        constraint.weight = 1.0
        result.joint_constraints.append(constraint)
    return result


def joint_path_length(
    start: dict[str, float], trajectory_joint_names: list[str], trajectory_points: list[Any]
) -> float:
    """Return cumulative Euclidean arc length in joint space, in radians."""

    indices = [trajectory_joint_names.index(name) for name in JOINT_NAMES]
    previous = [start[name] for name in JOINT_NAMES]
    total = 0.0
    for point in trajectory_points:
        current = [float(point.positions[index]) for index in indices]
        total += math.sqrt(sum((right - left) ** 2 for left, right in zip(previous, current)))
        previous = current
    return total


def skipped_gate(after: str) -> dict[str, Any]:
    return {"evaluated": False, "passed": False, "reason": f"SKIPPED_AFTER_{after}"}


def service_call(node: Node, client, request, timeout_sec: float):
    started = time.monotonic()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    elapsed = time.monotonic() - started
    if not future.done():
        return None, elapsed, "SERVICE_TIMEOUT"
    try:
        response = future.result()
    except Exception as error:  # ROS surfaces transport failures on the future.
        return None, elapsed, f"SERVICE_ERROR:{type(error).__name__}"
    if response is None:
        return None, elapsed, "SERVICE_EMPTY_RESPONSE"
    return response, elapsed, None


def state_validity_request(group: str, joints: dict[str, float]) -> GetStateValidity.Request:
    request = GetStateValidity.Request()
    request.group_name = group
    request.robot_state.is_diff = True
    request.robot_state.joint_state.name = list(JOINT_NAMES)
    request.robot_state.joint_state.position = [joints[name] for name in JOINT_NAMES]
    return request


def state_validity_record(response, elapsed_sec: float) -> dict[str, Any]:
    contacts = sorted(
        {
            "<->".join(sorted((contact.contact_body_1, contact.contact_body_2)))
            for contact in response.contacts
        }
    )
    return {
        "valid": bool(response.valid),
        "contacts": contacts,
        "contact_count": len(response.contacts),
        "wall_time_sec": round(elapsed_sec, 6),
    }


def set_remaining_skipped(gates: dict[str, Any], after: str) -> None:
    start = GATE_ORDER.index(after) + 1
    for name in GATE_ORDER[start:]:
        gates[name] = skipped_gate(after.upper())


def evaluate_candidate(
    *,
    node: Node,
    record: dict[str, Any],
    arguments: argparse.Namespace,
    camera_to_tip: Pose,
    start_joints: dict[str, float],
    start_validity: dict[str, Any],
    ik_client,
    validity_client,
    plan_client,
    joint_minimum: dict[str, float],
    joint_maximum: dict[str, float],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "candidate_id": record["candidate_id"],
        "selection_score": record.get("selection_score"),
        "passed": False,
        "motion_commanded": False,
        "gates": {},
        "failure_reason": None,
        "moveit_collision_rejected": False,
        "moveit_raw_error_code": None,
        "moveit_error_name": None,
        "planning_time_sec": None,
        "joint_path_length_rad_l2": None,
    }
    gates = result["gates"]

    gates["fov"] = fov_gate(record)
    if not gates["fov"]["passed"]:
        result["failure_reason"] = gates["fov"]["reason"]
        set_remaining_skipped(gates, "fov")
        return result

    camera_transform = record["geometry_candidate"]["candidate_pose_world_from_camera"]
    gates["workspace"] = workspace_gate(
        camera_transform["translation_m"], arguments.workspace_min, arguments.workspace_max
    )
    if not gates["workspace"]["passed"]:
        result["failure_reason"] = gates["workspace"]["reason"]
        set_remaining_skipped(gates, "workspace")
        return result

    camera_pose = candidate_camera_pose(record, arguments.planning_frame)
    tip_pose = PoseStamped()
    tip_pose.header = camera_pose.header
    tip_pose.pose = compose_pose(camera_pose.pose, camera_to_tip)
    ik_request = GetPositionIK.Request()
    ik_request.ik_request.group_name = arguments.planning_group
    ik_request.ik_request.ik_link_name = arguments.planning_tip
    ik_request.ik_request.pose_stamped = tip_pose
    ik_request.ik_request.robot_state.is_diff = True
    ik_request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
    ik_request.ik_request.robot_state.joint_state.position = [
        start_joints[name] for name in JOINT_NAMES
    ]
    # Collision is deliberately evaluated by the following state-validity
    # gate so an IK failure is not mislabeled as a collision rejection.
    ik_request.ik_request.avoid_collisions = False
    ik_request.ik_request.timeout = duration_message(arguments.ik_timeout_sec)
    ik_response, ik_elapsed, ik_error = service_call(
        node, ik_client, ik_request, arguments.request_timeout_sec
    )
    ik_code = None if ik_response is None else int(ik_response.error_code.val)
    gates["ik"] = {
        "evaluated": True,
        "passed": ik_error is None and ik_code == MoveItErrorCodes.SUCCESS,
        "reason": ik_error,
        "raw_error_code": ik_code,
        "error_name": moveit_error_name(ik_code),
        "wall_time_sec": round(ik_elapsed, 6),
        "avoid_collisions": False,
    }
    if not gates["ik"]["passed"]:
        gates["ik"]["reason"] = ik_error or moveit_error_name(ik_code) or "IK_FAILED"
        result["failure_reason"] = f"IK_{gates['ik']['reason']}"
        result["moveit_raw_error_code"] = ik_code
        result["moveit_error_name"] = moveit_error_name(ik_code)
        set_remaining_skipped(gates, "ik")
        return result
    try:
        solution = nearest_wrapped_solution(ik_response.solution.joint_state, start_joints)
    except ValueError as error:
        gates["ik"]["passed"] = False
        gates["ik"]["reason"] = str(error)
        result["failure_reason"] = str(error)
        set_remaining_skipped(gates, "ik")
        return result
    gates["ik"]["solution_joint_positions_rad"] = {
        name: round(solution[name], 9) for name in JOINT_NAMES
    }

    gates["joint_limit"] = joint_limit_gate(solution, joint_minimum, joint_maximum)
    if not gates["joint_limit"]["passed"]:
        result["failure_reason"] = gates["joint_limit"]["reason"]
        set_remaining_skipped(gates, "joint_limit")
        return result

    goal_validity_response, validity_elapsed, validity_error = service_call(
        node,
        validity_client,
        state_validity_request(arguments.planning_group, solution),
        arguments.request_timeout_sec,
    )
    goal_validity = (
        None
        if goal_validity_response is None
        else state_validity_record(goal_validity_response, validity_elapsed)
    )
    collision_passed = (
        start_validity["valid"] is True
        and validity_error is None
        and goal_validity is not None
        and goal_validity["valid"] is True
    )
    if not start_validity["valid"]:
        collision_reason = "MOVEIT_START_STATE_COLLISION"
    elif validity_error is not None:
        collision_reason = f"MOVEIT_STATE_VALIDITY_{validity_error}"
    elif goal_validity is None or not goal_validity["valid"]:
        collision_reason = "MOVEIT_GOAL_STATE_COLLISION"
    else:
        collision_reason = None
    gates["collision"] = {
        "evaluated": True,
        "passed": collision_passed,
        "reason": collision_reason,
        "start_state": start_validity,
        "goal_state": goal_validity,
    }
    if not collision_passed:
        result["failure_reason"] = collision_reason
        result["moveit_collision_rejected"] = (
            not start_validity["valid"]
            or (goal_validity is not None and not goal_validity["valid"])
        )
        set_remaining_skipped(gates, "collision")
        return result

    plan_request = GetMotionPlan.Request()
    motion = plan_request.motion_plan_request
    motion.group_name = arguments.planning_group
    motion.start_state.is_diff = True
    motion.start_state.joint_state.name = list(JOINT_NAMES)
    motion.start_state.joint_state.position = [start_joints[name] for name in JOINT_NAMES]
    motion.goal_constraints = [goal_constraints(solution, arguments.goal_tolerance_rad)]
    motion.num_planning_attempts = arguments.planning_attempts
    motion.allowed_planning_time = arguments.planning_time_sec
    motion.max_velocity_scaling_factor = arguments.velocity_scale
    motion.max_acceleration_scaling_factor = arguments.acceleration_scale
    plan_response, plan_elapsed, plan_error = service_call(
        node, plan_client, plan_request, arguments.request_timeout_sec
    )
    response = None if plan_response is None else plan_response.motion_plan_response
    plan_code = None if response is None else int(response.error_code.val)
    trajectory = None if response is None else response.trajectory.joint_trajectory
    points = [] if trajectory is None else list(trajectory.points)
    plan_passed = (
        plan_error is None
        and plan_code == MoveItErrorCodes.SUCCESS
        and bool(points)
    )
    if plan_error is not None:
        planning_reason = f"PLANNING_{plan_error}"
    elif plan_code != MoveItErrorCodes.SUCCESS:
        planning_reason = moveit_error_name(plan_code) or "PLANNING_FAILED"
    elif not points:
        planning_reason = "PLANNING_EMPTY_TRAJECTORY"
    else:
        planning_reason = None
    reported_time = None if response is None else float(response.planning_time)
    gates["planning"] = {
        "evaluated": True,
        "passed": plan_passed,
        "reason": planning_reason,
        "raw_error_code": plan_code,
        "error_name": moveit_error_name(plan_code),
        "moveit_reported_planning_time_sec": reported_time,
        "wall_time_sec": round(plan_elapsed, 6),
        "planning_attempts": arguments.planning_attempts,
        "trajectory_point_count": len(points),
    }
    result["moveit_raw_error_code"] = plan_code
    result["moveit_error_name"] = moveit_error_name(plan_code)
    result["planning_time_sec"] = reported_time
    if plan_code in EXPLICIT_COLLISION_CODES:
        result["moveit_collision_rejected"] = True
    if not plan_passed:
        result["failure_reason"] = planning_reason
        return result

    path_length = joint_path_length(start_joints, list(trajectory.joint_names), points)
    duration = points[-1].time_from_start
    terminal = {
        name: round(points[-1].positions[list(trajectory.joint_names).index(name)], 9)
        for name in JOINT_NAMES
    }
    gates["planning"].update(
        {
            "joint_path_length_rad_l2": round(path_length, 9),
            "trajectory_duration_sec": round(
                duration.sec + duration.nanosec / 1_000_000_000.0, 6
            ),
            "terminal_joint_positions_rad": terminal,
        }
    )
    result["joint_path_length_rad_l2"] = round(path_length, 9)
    result["passed"] = True
    return result


def main() -> None:
    arguments = parse_arguments()
    screen = json.loads(arguments.screen.read_text(encoding="utf-8"))
    candidates = strict_candidates(screen, arguments.expected_candidate_count)
    start_joints = load_joint_map(arguments.start_joints)
    scene_receipt = validate_scene_receipt(
        arguments.scene_receipt, arguments.required_scene_profile
    )
    joint_minimum = dict(zip(JOINT_NAMES, arguments.joint_limit_min))
    joint_maximum = dict(zip(JOINT_NAMES, arguments.joint_limit_max))
    start_limit = joint_limit_gate(start_joints, joint_minimum, joint_maximum)
    if not start_limit["passed"]:
        raise ValueError("P7_4_START_STATE_JOINT_LIMIT_REJECTED")

    rclpy.init()
    node = Node("p7_4_batch_moveit_plan")
    ik_client = node.create_client(GetPositionIK, arguments.ik_service)
    validity_client = node.create_client(GetStateValidity, arguments.state_validity_service)
    plan_client = node.create_client(GetMotionPlan, arguments.plan_service)
    listener = None
    try:
        for name, client in (
            (arguments.ik_service, ik_client),
            (arguments.state_validity_service, validity_client),
            (arguments.plan_service, plan_client),
        ):
            if not client.wait_for_service(timeout_sec=arguments.service_wait_sec):
                raise RuntimeError(f"P7_4_MOVEIT_SERVICE_UNAVAILABLE:{name}")

        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, node)
        deadline = time.monotonic() + arguments.service_wait_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if tf_buffer.can_transform(
                arguments.camera_frame, arguments.planning_tip, rclpy.time.Time()
            ):
                break
        try:
            camera_to_tip = pose_from_transform(
                tf_buffer.lookup_transform(
                    arguments.camera_frame, arguments.planning_tip, rclpy.time.Time()
                ).transform
            )
        except TransformException as error:
            raise RuntimeError(
                f"P7_4_CAMERA_TO_TIP_TF_UNAVAILABLE:{arguments.camera_frame}->{arguments.planning_tip}"
            ) from error

        start_response, start_elapsed, start_error = service_call(
            node,
            validity_client,
            state_validity_request(arguments.planning_group, start_joints),
            arguments.request_timeout_sec,
        )
        if start_error is not None or start_response is None:
            raise RuntimeError(f"P7_4_START_STATE_VALIDITY_{start_error or 'FAILED'}")
        start_validity = state_validity_record(start_response, start_elapsed)

        records = [
            evaluate_candidate(
                node=node,
                record=candidate,
                arguments=arguments,
                camera_to_tip=camera_to_tip,
                start_joints=start_joints,
                start_validity=start_validity,
                ik_client=ik_client,
                validity_client=validity_client,
                plan_client=plan_client,
                joint_minimum=joint_minimum,
                joint_maximum=joint_maximum,
            )
            for candidate in candidates
        ]
    finally:
        # Retain the listener until all service work has completed.
        _ = listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failure_counts = Counter(
        record["failure_reason"] for record in records if record["failure_reason"]
    )
    collision_evaluated = sum(
        record["gates"]["collision"]["evaluated"] for record in records
    )
    collision_rejected = sum(record["moveit_collision_rejected"] for record in records)
    planning_times = [
        record["planning_time_sec"]
        for record in records
        if record["planning_time_sec"] is not None
    ]
    passed = [record for record in records if record.get("passed") is True]
    result = {
        "schema": "p7_4_batch_moveit_planning_v1",
        "gate": "P7.4_MOVEIT_PLANNING_ONLY",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "claim_boundary": (
            "collision-checked MoveIt planning only; no trajectory was commanded and "
            "no Gazebo physical-contact collision was measured"
        ),
        "motion_commanded": False,
        "gate_order": list(GATE_ORDER),
        "screen_input": str(arguments.screen),
        "scene_receipt": str(arguments.scene_receipt),
        "scene_profile": scene_receipt["profile"],
        "start_state_source": str(arguments.start_joints),
        "start_state_joint_positions_rad": start_joints,
        "start_state_joint_limit_gate": start_limit,
        "start_state_moveit_validity": start_validity,
        "configuration": {
            "planning_frame": arguments.planning_frame,
            "camera_frame": arguments.camera_frame,
            "planning_tip": arguments.planning_tip,
            "planning_group": arguments.planning_group,
            "workspace_min_m": list(arguments.workspace_min),
            "workspace_max_m": list(arguments.workspace_max),
            "joint_limit_min_rad": joint_minimum,
            "joint_limit_max_rad": joint_maximum,
            "planning_time_limit_sec": arguments.planning_time_sec,
            "planning_attempts_per_candidate": arguments.planning_attempts,
            "external_replanning_count": 0,
        },
        "summary": {
            "candidate_count": len(records),
            "planning_success_count": len(passed),
            "planning_success_rate": len(passed) / len(records),
            "moveit_collision_rejection_count": collision_rejected,
            "moveit_collision_evaluated_count": collision_evaluated,
            "moveit_collision_rejection_rate": (
                collision_rejected / collision_evaluated if collision_evaluated else None
            ),
            "planning_time_sec_total": sum(planning_times),
            "first_feasible_candidate_id": (
                None if not passed else passed[0]["candidate_id"]
            ),
            "failure_reason_counts": dict(sorted(failure_counts.items())),
        },
        "records": records,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(result["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
