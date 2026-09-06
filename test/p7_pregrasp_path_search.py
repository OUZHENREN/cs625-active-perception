#!/usr/bin/env python3
"""Read-only, target-conditioned P7 pregrasp path search.

The requested poses describe ``p7_grasp_center_link`` (the nominal object
centre between the pads), rather than the historical ``tool0`` frame.  Every candidate is
converted by live TF to the SRDF planning tip, then evaluated by collision
aware IK *and* a full MoveIt motion-plan request.  This tool never sends a
trajectory action and is therefore safe to use for repeatable preflight
selection before a P7 episode.
"""

from __future__ import annotations

import argparse
import math
import json
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetPositionFK, GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict
from cs625_motion_adapter.p7_arm_motion_adapter import pose_errors
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
WRAPPED_JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "wrist_1_joint",
    "wrist_2_joint", "wrist_3_joint",
}
PHYSICAL_FRAME = "p7_grasp_center_link"
PLANNING_TIP = "my_end_effector_link"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--position", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="probe exactly one physical grasp-centre position instead of the default pregrasp grid",
    )
    parser.add_argument(
        "--orientation", nargs=4, type=float, metavar=("QX", "QY", "QZ", "QW"),
        help="unit quaternion for a single-position probe",
    )
    parser.add_argument("--candidates", type=Path, help="estimated-pose grasp candidate JSON")
    parser.add_argument("--output", type=Path, help="save every attempted candidate and first planned joint branch")
    parser.add_argument("--pose-key", choices=("pregrasp", "grasp"), default="pregrasp")
    parser.add_argument("--candidate-id", help="restrict an estimated candidate set to one named template")
    parser.add_argument("--ik-seed-grid", action="store_true",
                        help="try current plus eight bounded shoulder/elbow/wrist seeds with the same MoveIt solver")
    parser.add_argument("--ik-seed-file", type=Path, help="start IK from a previously accepted branch, not an execution goal")
    parser.add_argument("--state-diagnostic", action="store_true",
                        help="solve IK without collision filtering, then report explicit state contacts; no planning")
    arguments = parser.parse_args()
    if arguments.orientation is not None:
        norm = math.sqrt(sum(value * value for value in arguments.orientation))
        if abs(norm - 1.0) > 1.0e-3:
            parser.error("--orientation must be a unit quaternion")
    return arguments


def multiply(left: tuple[float, float, float, float], right: tuple[float, float, float, float]):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lz * rw + lx * ry - ly * rx,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate(vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = multiply(multiply(quaternion, (*vector, 0.0)), inverse)
    return rotated[:3]


def pose_from_transform(transform) -> Pose:
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation
    return pose


def compose_pose(base: Pose, relative: Pose) -> Pose:
    result = Pose()
    translated = rotate(
        (relative.position.x, relative.position.y, relative.position.z),
        (base.orientation.x, base.orientation.y, base.orientation.z, base.orientation.w),
    )
    result.position.x = base.position.x + translated[0]
    result.position.y = base.position.y + translated[1]
    result.position.z = base.position.z + translated[2]
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = multiply(
        (base.orientation.x, base.orientation.y, base.orientation.z, base.orientation.w),
        (relative.orientation.x, relative.orientation.y, relative.orientation.z, relative.orientation.w),
    )
    return result


def joint_bounds_from_urdf(xml_text: str) -> dict[str, tuple[float, float]]:
    result = {}
    for joint in ET.fromstring(xml_text).findall("joint"):
        if joint.attrib["name"] not in JOINT_NAMES:
            continue
        limit = joint.find("limit")
        lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
        soft = joint.find("safety_controller")
        if soft is not None:
            lower = max(lower, float(soft.attrib.get("soft_lower_limit", lower)))
            upper = min(upper, float(soft.attrib.get("soft_upper_limit", upper)))
        result[joint.attrib["name"]] = (lower, upper)
    if set(result) != set(JOINT_NAMES):
        raise ValueError("active MoveIt URDF does not expose six arm joint bounds")
    return result


def normalise_solution(solution: JointState, observed: dict[str, float],
                       bounds: dict[str, tuple[float, float]]) -> None:
    for index, name in enumerate(solution.name):
        if name in WRAPPED_JOINTS and name in observed:
            value = solution.position[index]
            lower, upper = bounds[name]
            candidates = [value + turn * 2.0 * math.pi for turn in range(-3, 4)
                          if lower <= value + turn * 2.0 * math.pi <= upper]
            if not candidates:
                raise ValueError(f"no bounded angular equivalent for {name}")
            solution.position[index] = min(candidates, key=lambda angle: abs(angle - observed[name]))


def constraints(solution: JointState) -> Constraints:
    result = Constraints()
    for name, position in zip(solution.name, solution.position):
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = position
        constraint.tolerance_above = 0.001
        constraint.tolerance_below = 0.001
        constraint.weight = 1.0
        result.joint_constraints.append(constraint)
    return result


def roll_about_tool_x(base: tuple[float, float, float, float], angle: float) -> tuple[float, float, float, float]:
    return multiply(base, (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)))


def main() -> None:
    arguments = parse_arguments()
    rclpy.init()
    node = rclpy.create_node("p7_pregrasp_path_search")
    observed: dict[str, float] = {}
    node.create_subscription(JointState, "/joint_states", lambda message: observed.update(zip(message.name, message.position)), 10)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    fk_client = node.create_client(GetPositionFK, "/compute_fk")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    parameters_client = node.create_client(GetParameters, "/move_group/get_parameters")
    if not ik_client.wait_for_service(timeout_sec=12.0) or not plan_client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError("MoveIt IK or planner service unavailable")
    if not parameters_client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("active MoveIt URDF parameters unavailable")
    parameters_request = GetParameters.Request()
    parameters_request.names = ["robot_description"]
    parameters_future = parameters_client.call_async(parameters_request)
    rclpy.spin_until_future_complete(node, parameters_future, timeout_sec=5.0)
    bounds = joint_bounds_from_urdf(parameters_future.result().values[0].string_value)
    deadline = time.monotonic() + 3.0
    while len(observed) < len(JOINT_NAMES) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(observed) < len(JOINT_NAMES):
        raise RuntimeError("six-axis joint state unavailable")
    # A newly constructed TF listener has no transform cache until its
    # executor has processed a few /tf messages.  Wait deterministically
    # before the single static-chain lookup below.
    tf_deadline = time.monotonic() + 1.5
    while time.monotonic() < tf_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf_buffer.can_transform(PHYSICAL_FRAME, PLANNING_TIP, rclpy.time.Time()):
            break
    try:
        # lookup(physical, tip) = T_physical_tip; desired T_base_tip is
        # therefore T_base_physical T_physical_tip.
        physical_to_tip = pose_from_transform(
            tf_buffer.lookup_transform(PHYSICAL_FRAME, PLANNING_TIP, rclpy.time.Time()).transform
        )
    except TransformException as error:
        raise RuntimeError(f"{PHYSICAL_FRAME} -> {PLANNING_TIP} TF unavailable: {error}") from error

    # The YCB can centre is at x=0.72 m.  These are 0.15--0.35 m stand-off
    # poses on its unobstructed (-y) side, above table height.  They are not
    # grasp claims: this probe selects a collision-free pregrasp only.
    positions = (
        ((tuple(arguments.position),) if arguments.position is not None else (
            (0.70, -0.26, 0.24), (0.78, -0.26, 0.24),
            (0.70, -0.34, 0.30), (0.78, -0.34, 0.30),
            (0.70, -0.26, 0.38), (0.78, -0.26, 0.38),
            (0.70, -0.34, 0.44), (0.78, -0.34, 0.44),
        ))
    )
    # Historic target-facing orientation, plus axial rolls to avoid an
    # otherwise valid IK branch whose forearm/wrist self-collides in planning.
    historic = tuple(arguments.orientation) if arguments.orientation is not None else (
        -0.0961499, 0.0769224, 0.7749167, 0.6199534
    )
    rolls = (0.0,) if arguments.orientation is not None else (
        0.0, math.pi / 2.0, -math.pi / 2.0, math.pi
    )
    pose_candidates = [
        (str(index), position, roll_about_tool_x(historic, roll))
        for index, position in enumerate(positions) for roll in rolls]
    if arguments.candidates:
        source = json.loads(arguments.candidates.read_text(encoding="utf-8"))
        if source.get("target_frame") != "base_link":
            raise ValueError("grasp candidates must explicitly be in base_link")
        pose_candidates = [
            (candidate["candidate_id"],
             tuple(candidate[arguments.pose_key]["position"][a] for a in "xyz"),
             tuple(candidate[arguments.pose_key]["orientation"][a] for a in "xyzw"))
            for candidate in source["candidates"]
            if not arguments.candidate_id or candidate["candidate_id"] == arguments.candidate_id]
    seeded_candidates = []
    for index, position, quaternion in pose_candidates:
        if arguments.ik_seed_file:
            saved = json.loads(arguments.ik_seed_file.read_text(encoding="utf-8"))
            if saved.get("success") is not True:
                raise ValueError("IK seed file must describe an accepted branch")
            seed = saved["joint_goal_positions_rad"]
            if any(not bounds[name][0] <= seed[name] <= bounds[name][1] for name in JOINT_NAMES):
                raise ValueError("saved IK seed is outside active joint bounds")
            seeded_candidates.append((index, position, quaternion, str(arguments.ik_seed_file), seed))
        seeded_candidates.append((index, position, quaternion, "current", dict(observed)))
        if arguments.ik_seed_grid:
            azimuth = math.atan2(position[1], position[0])
            for pan in (azimuth, azimuth - math.pi):
                for elbow in (-1.7, 1.7):
                    for wrist in (-math.pi / 2.0, math.pi / 2.0):
                        seed = dict(zip(JOINT_NAMES, (pan, -1.0, elbow, -1.5, wrist, 0.0)))
                        seed = {name: min(bounds[name][1], max(bounds[name][0], value))
                                for name, value in seed.items()}
                        label = f"pan{pan:.3f}_elbow{elbow:.1f}_wrist{wrist:.3f}"
                        seeded_candidates.append((index, position, quaternion, label, seed))
    records = []
    for index, position, quaternion, seed_label, seed_values in seeded_candidates:
            x, y, z = position
            qx, qy, qz, qw = quaternion
            record = {"candidate_id": index, "position_m": list(position),
                      "orientation_xyzw": list(quaternion), "ik_success": False,
                      "planning_success": False}
            records.append(record)
            record["ik_seed_label"] = seed_label
            record["ik_seed_joint_positions_rad"] = {name: seed_values[name] for name in JOINT_NAMES}
            physical = Pose()
            physical.position.x, physical.position.y, physical.position.z = x, y, z
            physical.orientation.x, physical.orientation.y = qx, qy
            physical.orientation.z, physical.orientation.w = qz, qw
            tip = compose_pose(physical, physical_to_tip)
            ik_request = GetPositionIK.Request()
            ik = ik_request.ik_request
            ik.group_name = "cs625_arm"
            ik.ik_link_name = PLANNING_TIP
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = "base_link"
            ik.pose_stamped.pose = tip
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state.name = list(JOINT_NAMES)
            ik.robot_state.joint_state.position = [seed_values[name] for name in JOINT_NAMES]
            ik.avoid_collisions = not arguments.state_diagnostic
            ik.timeout = Duration(nanosec=700_000_000)
            ik_future = ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(node, ik_future, timeout_sec=1.5)
            ik_response = ik_future.result()
            record["ik_error_code"] = None if ik_response is None else ik_response.error_code.val
            if ik_response is None or ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            record["ik_success"] = True
            record["joint_bounds_from_active_moveit_urdf"] = bounds
            record["raw_ik_joint_positions_rad"] = dict(zip(
                ik_response.solution.joint_state.name, ik_response.solution.joint_state.position))
            normalise_solution(ik_response.solution.joint_state, observed, bounds)
            record["bounded_ik_joint_positions_rad"] = dict(zip(
                ik_response.solution.joint_state.name, ik_response.solution.joint_state.position))
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = "base_link"
            fk_request.fk_link_names = [PHYSICAL_FRAME, PLANNING_TIP, "flange",
                                       "wrist_1_link", "camera_link"]
            fk_request.robot_state = ik_response.solution
            fk_future = fk_client.call_async(fk_request)
            rclpy.spin_until_future_complete(node, fk_future, timeout_sec=3.0)
            fk_response = fk_future.result()
            record["requested_physical_pose"] = message_to_ordereddict(physical)
            record["requested_tip_pose"] = message_to_ordereddict(tip)
            record["physical_to_tip_tf"] = message_to_ordereddict(physical_to_tip)
            if fk_response is None or not fk_response.pose_stamped:
                record["failure_reason"] = "FK_UNAVAILABLE"
                continue
            record["fk_poses"] = [message_to_ordereddict(p) for p in fk_response.pose_stamped]
            position_error, rotation_error = pose_errors(physical, fk_response.pose_stamped[0].pose)
            record["ik_physical_fk_position_error_m"] = position_error
            record["ik_physical_fk_orientation_error_rad"] = rotation_error
            if position_error > 0.008 or rotation_error > 0.05:
                record["failure_reason"] = "IK_FK_MISMATCH"
                continue
            validity_request = GetStateValidity.Request()
            validity_request.group_name = "cs625_arm"
            validity_request.robot_state = ik_response.solution
            validity_future = validity_client.call_async(validity_request)
            rclpy.spin_until_future_complete(node, validity_future, timeout_sec=3.0)
            validity = validity_future.result()
            record["state_valid"] = validity is not None and validity.valid
            record["state_contacts"] = [] if validity is None else sorted({
                "<->".join(sorted((c.contact_body_1, c.contact_body_2))) for c in validity.contacts})
            if arguments.state_diagnostic or not record["state_valid"]:
                continue
            plan_request = GetMotionPlan.Request()
            request = plan_request.motion_plan_request
            request.group_name = "cs625_arm"
            request.num_planning_attempts = 1
            request.allowed_planning_time = 2.0
            request.max_velocity_scaling_factor = 0.25
            request.max_acceleration_scaling_factor = 0.25
            request.start_state.is_diff = True
            request.start_state.joint_state.name = list(JOINT_NAMES)
            request.start_state.joint_state.position = [observed[name] for name in JOINT_NAMES]
            request.goal_constraints = [constraints(ik_response.solution.joint_state)]
            plan_future = plan_client.call_async(plan_request)
            rclpy.spin_until_future_complete(node, plan_future, timeout_sec=3.0)
            plan_response = plan_future.result()
            record["plan_error_code"] = None if plan_response is None else plan_response.motion_plan_response.error_code.val
            if plan_response is None or plan_response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            trajectory = plan_response.motion_plan_response.trajectory.joint_trajectory
            if not trajectory.points:
                continue
            duration = trajectory.points[-1].time_from_start
            terminal = {
                name: round(position, 7)
                for name, position in zip(trajectory.joint_names, trajectory.points[-1].positions)
            }
            requested_joints = record["bounded_ik_joint_positions_rad"]
            if any(abs(value - requested_joints[name]) > 0.0015 for name, value in terminal.items()):
                record["failure_reason"] = "PLANNER_ALTERED_JOINT_GOAL"
                continue
            record["planning_success"] = True
            record["joint_goal_positions_rad"] = terminal
            if arguments.output:
                arguments.output.parent.mkdir(parents=True, exist_ok=True)
                arguments.output.write_text(json.dumps({
                    "schema": "p7_pregrasp_branch_selection_v1", "success": True,
                    "candidate_id": index, "motion_commanded": False,
                    "joint_goal_positions_rad": terminal, "records": records,
                    "source_candidates": str(arguments.candidates),
                }, indent=2) + "\n", encoding="utf-8")
            print(
                "PATH_OK "
                f"candidate={index} "
                f"physical_center=({x:.4f},{y:.4f},{z:.4f}) "
                f"physical_q=({qx:.7f},{qy:.7f},{qz:.7f},{qw:.7f}) "
                f"points={len(trajectory.points)} duration_sec={duration.sec + duration.nanosec / 1e9:.3f} "
                f"terminal_joints={json.dumps(terminal, sort_keys=True, separators=(',', ':'))}"
            )
            node.destroy_node()
            rclpy.shutdown()
            return
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps({
            "schema": "p7_pregrasp_branch_selection_v1", "success": False,
            "motion_commanded": False, "records": records,
            "source_candidates": str(arguments.candidates),
        }, indent=2) + "\n", encoding="utf-8")
    print(f"PATH_NONE poses={len(pose_candidates)} seeded_attempts={len(seeded_candidates)}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
