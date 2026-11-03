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
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--velocity", type=float, default=0.05,
                        help="velocity AND acceleration scaling for the plan")
    parser.add_argument("--goal-tol", type=float, default=0.004,
                        help="joint-space goal tolerance in radians (insert steps use 0.0003)")
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
    request = PositionIKRequest()
    request.group_name = "cs625_arm"
    request.ik_link_name = "my_end_effector_link"
    request.avoid_collisions = True
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = args.pose[:3]
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = args.pose[3:]
    request.pose_stamped = pose
    state = RobotState()
    joints = JointState()
    joints.name = ORDER
    joints.position = seed
    state.joint_state = joints
    request.robot_state = state
    future = ik.call_async(GetPositionIK.Request(ik_request=request))
    deadline = time.time() + 20.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    response = future.result()
    if response is None or response.error_code.val != 1:
        code = response.error_code.val if response else -1
        print(f"NO_COLLISION_FREE_IK (code={code}): this pose cannot be adopted "
              "without self-collision; refusing to plan")
        return 2
    solution = dict(zip(response.solution.joint_state.name,
                        response.solution.joint_state.position))
    goal = [solution[k] for k in ORDER]
    print("ik goal J:", [round(np.degrees(v), 1) for v in goal])

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
