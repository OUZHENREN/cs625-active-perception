#!/usr/bin/env python3
"""Plan one move to a TCP pose, LOCKED to a collision-checked IK solution.

Replaces plan_pose_once for the teaching chain (2026-11-02 audit, hole 1):

  * IK is solved with avoid_collisions=True, seeded from the current joints, so
    a pose whose only solution self-collides (RVS S2 on 2026-11-02: forearm_link
    <-> my_end_effector_link, 5.3 mm) is REFUSED before any motion;
  * the plan is a JOINT-space goal at that solution (tolerance --goal-tol), so
    the branch cannot flip to a 734-point wrapped-wrist path;
  * if no collision-free IK exists the script exits 2.

Usage:
    plan_joint_locked.py X Y Z QX QY QZ QW --velocity 0.05 \
        [--goal-tol 0.004] --output /tmp/plan.json

Exit codes: 0 plan written; 1 plan failed; 2 no collision-free IK; 3 service down.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, PositionIKRequest,
                             RobotState)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]



# Proven postures (degrees) used as extra IK seeds and as the reference for the
# configuration sanity rules.  Every one of these has executed on the real cell.
PROVEN_POSTURES = {
    "observation":  [43.1, -57.4, -146.3, 14.3, 92.8, 89.8],
    "pre_grasp":    [37.1, -120.3, -107.2, -42.4, 90.8, 32.6],
    "grasp":        [43.0, -123.0, -101.7, -45.0, 90.7, 38.4],
    "baseline":     [6.8, -87.9, -91.9, -50.5, 94.4, 91.2],
}


def sanity_violation(candidate: list[float], seed: list[float], args) -> str | None:
    """Why this IK solution must not be executed, or None when it is sane.

    Two rules, both calibrated on 2026-11-03 on this cell:
      * an absolute joint angle past +-max_abs is a wrap-around branch
        (the RVS fine-view pose only solves to J1=-204 / wrist_3=+332 deg);
      * a single move that swings one joint past max_delta is not an approach
        (a wrapped branch shows ~180-250 deg jumps).
    """
    goal_deg = np.degrees(np.array(candidate))
    delta_deg = goal_deg - np.degrees(np.array(seed))
    if np.max(np.abs(goal_deg)) > args.max_abs_joint_deg:
        worst = int(np.argmax(np.abs(goal_deg)))
        return (f"{ORDER[worst]} = {goal_deg[worst]:+.0f} deg is past "
                f"+-{args.max_abs_joint_deg:.0f} deg (wrapped branch)")
    if np.max(np.abs(delta_deg)) > args.max_joint_delta_deg:
        worst = int(np.argmax(np.abs(delta_deg)))
        return (f"{ORDER[worst]} would move {delta_deg[worst]:+.0f} deg "
                f"(limit +-{args.max_joint_delta_deg:.0f})")
    return None


def collision_violation(client, node, candidate: list[float]) -> str | None:
    """Explicit collision check of an IK solution (replaces avoid_collisions IK).

    Returns a reason string when the configuration is in collision, else None.
    A failure to answer counts as a violation: never move on an unverified pose.
    """
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    state = RobotState()
    joints = JointState()
    joints.name = ORDER
    joints.position = list(candidate)
    state.joint_state = joints
    request = GetStateValidity.Request()
    request.robot_state = state
    request.group_name = "cs625_arm"
    request.constraints = Constraints()
    future = client.call_async(request)
    deadline = time.time() + 10.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    response = future.result()
    if response is None:
        return "state validity check timed out"
    if not response.valid:
        pairs = "; ".join(f"{c.contact_body_1}<->{c.contact_body_2}"
                          for c in response.contacts[:3])
        return f"configuration in collision ({pairs})"
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--velocity", type=float, default=0.05,
                        help="velocity AND acceleration scaling for the plan")
    parser.add_argument("--goal-tol", type=float, default=0.004,
                        help="joint-space goal tolerance in radians (insert steps use 0.0003)")
    parser.add_argument("--max-joint-delta-deg", type=float, default=120.0,
                        help="refuse a single move that swings one joint more than "
                             "this (a wrapped branch moves ~180 deg)")
    parser.add_argument("--max-abs-joint-deg", type=float, default=180.0,
                        help="refuse any solution whose absolute joint angle exceeds "
                             "this (wrap-around branch)")
    parser.add_argument("--max-plan-seconds", type=float, default=40.0,
                        help="refuse a plan longer than this (a wrapped branch "
                             "produces 100+ s paths)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("plan_joint_locked")
    current: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: current.update(dict(zip(m.name, m.position))), 10)
    deadline = time.time() + 8.0
    while len([k for k in ORDER if k in current]) < 6 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len([k for k in ORDER if k in current]) < 6:
        print("NO_JOINT_STATES")
        return 3
    seed = [current[k] for k in ORDER]
    print("seed J:", [round(np.degrees(v), 1) for v in seed])

    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not ik.wait_for_service(timeout_sec=40):
        print("NO /compute_ik")
        return 3
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    if not validity.wait_for_service(timeout_sec=40):
        print("NO /check_state_validity")
        return 3
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = args.pose[:3]
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = args.pose[3:]

    # Multi-seed IK (2026-11-03): a seed-based solver converges to whatever
    # branch is nearest, and "nearest" is sometimes a wrap-around that the sanity
    # rules below must reject.  Trying the proven postures as extra seeds makes
    # the guard practical: if ANY seed yields a sane branch, the move is legal.
    seeds = [("current", seed)]
    for name, posture in PROVEN_POSTURES.items():
        seeds.append((name, [np.radians(v) for v in posture]))

    goal = None
    last_reason = "no seed produced a collision-free solution"
    for label, candidate in seeds:
        request = PositionIKRequest()
        request.group_name = "cs625_arm"
        request.ik_link_name = "my_end_effector_link"
        # Kinematics only.  MoveIt's avoid_collisions IK short-circuits to -31
        # (or to a wrapped branch) when the FIRST solution it finds is in
        # collision, hiding a perfectly valid branch -- verified 2026-11-03:
        # the RVS fine-view pose IS reachable (J=[3.8,-89.2,-113.5,-42.4,143.3,
        # 21.4], valid) but avoid_collisions=True never returned it.  So we take
        # the raw solution and validate it ourselves below.
        request.avoid_collisions = False
        request.pose_stamped = pose
        state = RobotState()
        joints = JointState()
        joints.name = ORDER
        joints.position = list(candidate)
        state.joint_state = joints
        request.robot_state = state
        future = ik.call_async(GetPositionIK.Request(ik_request=request))
        deadline = time.time() + 20.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result()
        if response is None or response.error_code.val != 1:
            last_reason = f"no collision-free IK from seed '{label}' " \
                          f"(code={response.error_code.val if response else -1})"
            continue
        candidate_solution = [dict(zip(response.solution.joint_state.name,
                                       response.solution.joint_state.position))[k]
                              for k in ORDER]
        reason = sanity_violation(candidate_solution, seed, args)
        if reason is not None:
            last_reason = f"seed '{label}': {reason}"
            continue
        reason = collision_violation(validity, node, candidate_solution)
        if reason is not None:
            last_reason = f"seed '{label}': {reason}"
            continue
        goal = candidate_solution
        print(f"ik goal J (seed '{label}'):",
              [round(np.degrees(v), 1) for v in goal])
        break
    if goal is None:
        print(f"REFUSED: {last_reason}; refusing to plan")
        return 2

    # Configuration sanity gate (2026-11-03, completing audit hole 1).
    # Locking the plan to the IK solution is not enough when the IK solution
    # itself is a wrapped branch: verified on 2026-11-03, the pose RVS asks for
    # produced J1=-204 deg / J5=-326 deg and the planner happily returned a
    # 1302-point, 130-second path around the cell.  A commanded move must stay
    # near the current configuration; anything else is a refusal, not a plan.
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not planner.wait_for_service(timeout_sec=40):
        print("NO /plan_kinematic_path")
        return 3
    plan_request = GetMotionPlan.Request().motion_plan_request
    plan_request.group_name = "cs625_arm"
    plan_request.num_planning_attempts = 20
    plan_request.allowed_planning_time = 20.0
    plan_request.max_velocity_scaling_factor = args.velocity
    plan_request.max_acceleration_scaling_factor = args.velocity
    constraints = Constraints()
    for name, value in zip(ORDER, goal):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = name
        joint_constraint.position = float(value)
        joint_constraint.tolerance_above = args.goal_tol
        joint_constraint.tolerance_below = args.goal_tol
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    plan_request.goal_constraints.append(constraints)
    future = planner.call_async(GetMotionPlan.Request(motion_plan_request=plan_request))
    deadline = time.time() + 60.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    planned = future.result().motion_plan_response
    print(f"plan code {planned.error_code.val} "
          f"points {len(planned.trajectory.joint_trajectory.points)}")
    if planned.error_code.val != 1:
        return 1
    trajectory = planned.trajectory.joint_trajectory
    if trajectory.points:
        last = trajectory.points[-1].time_from_start
        planned_seconds = last.sec + last.nanosec * 1e-9
        if planned_seconds > args.max_plan_seconds:
            print(f"PLAN_TOO_LONG: {planned_seconds:.1f} s > "
                  f"{args.max_plan_seconds:.0f} s; this is a wrap-around path, "
                  "refusing to plan")
            return 1
    points = [{"q": list(p.positions),
               "t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9}
              for p in trajectory.points]
    json.dump({"joint_names": list(trajectory.joint_names), "points": points},
              open(args.output, "w"))
    print("duration:", round(points[-1]["t"], 2),
          "last J:", [round(np.degrees(v), 1) for v in points[-1]["q"]])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
