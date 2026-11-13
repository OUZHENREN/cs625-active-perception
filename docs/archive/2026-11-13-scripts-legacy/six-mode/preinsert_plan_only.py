#!/usr/bin/env python3
"""Plan-only adapter for the slot pre-insert pose.

This mirrors the senior `cs625_trajectory_tools` PlanToTarget joint-target path:
  1. take /slot_target_pose;
  2. rebuild insert -> rotated(-10 deg local y) -> pre-insert (-350 mm local z);
  3. solve IK at the exact pre-insert pose;
  4. plan in joint space to the IK solution;
  5. write the trajectory to /tmp/preinsert_plan.json.

It never sends a controller goal.

Run:
  source scripts/source_dev_env.sh --full
  python3 scripts/preinsert_plan_only.py
"""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

T_S_TCP = np.array(
    [
        [-1.0, 0.0, 0.0, -82.0 / 1000.0],
        [0.0, 1.0, 0.0, -193.0 / 1000.0],
        [0.0, 0.0, -1.0, -412.5 / 1000.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def matrix_from_pose(pose: Pose) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    ).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def pose_from_matrix(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in matrix[:3, 3])
    q = R.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(q[0]), float(q[1]), float(q[2]), float(q[3])
    )
    return pose


def main() -> int:
    rclpy.init()
    node = Node("preinsert_plan_only")
    joints: JointState | None = None
    slot: PoseStamped | None = None

    node.create_subscription(
        JointState, "/joint_states", lambda m: _set("joints", m), 10
    )
    node.create_subscription(
        PoseStamped, "/slot_target_pose", lambda m: _set("slot", m), 10
    )

    global _joints, _slot
    _joints = None
    _slot = None

    def _set(key, msg):
        global _joints, _slot
        if key == "joints":
            _joints = msg
        else:
            _slot = msg

    deadline = time.time() + 60.0
    while (_joints is None or _slot is None) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if _joints is None or _slot is None:
        print("MISSING joint_states or slot_target_pose")
        return 2

    T_slot = matrix_from_pose(_slot.pose)
    T_insert = T_slot @ T_S_TCP
    T_rot = np.eye(4)
    T_rot[:3, :3] = R.from_euler("y", -10.0, degrees=True).as_matrix()
    T_shift = np.eye(4)
    T_shift[2, 3] = -0.350
    T_pre = T_insert @ T_rot @ T_shift
    target = PoseStamped()
    target.header.frame_id = "world"
    target.pose = pose_from_matrix(T_pre)
    print("preinsert_position", np.round(T_pre[:3, 3], 6).tolist())

    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    for client in (ik_client, plan_client):
        while not client.wait_for_service(timeout_sec=1.0):
            pass

    ik_req = GetPositionIK.Request()
    ik_req.ik_request.group_name = "cs625_arm"
    ik_req.ik_request.robot_state.joint_state = _joints
    ik_req.ik_request.pose_stamped = target
    ik_req.ik_request.timeout.sec = 3
    future = ik_client.call_async(ik_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    ik_res = future.result()
    print("ik_error", None if ik_res is None else ik_res.error_code.val)
    if ik_res is None or ik_res.error_code.val != 1:
        return 3

    solution = dict(
        zip(ik_res.solution.joint_state.name, ik_res.solution.joint_state.position)
    )
    constraints = Constraints()
    for name in JOINTS:
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = solution[name]
        joint.tolerance_above = 0.02
        joint.tolerance_below = 0.02
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)

    plan_req = GetMotionPlan.Request()
    motion = plan_req.motion_plan_request
    motion.group_name = "cs625_arm"
    motion.start_state.joint_state = _joints
    motion.goal_constraints.append(constraints)
    motion.num_planning_attempts = 8
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.05
    motion.max_acceleration_scaling_factor = 0.05
    future = plan_client.call_async(plan_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    plan_res = future.result()
    print(
        "joint_plan_error",
        None if plan_res is None else plan_res.motion_plan_response.error_code.val,
    )
    if plan_res is None or plan_res.motion_plan_response.error_code.val != 1:
        return 4

    trajectory = plan_res.motion_plan_response.trajectory.joint_trajectory
    output = {
        "joint_names": list(trajectory.joint_names),
        "points": [
            {
                "q": list(point.positions),
                "t": point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9,
            }
            for point in trajectory.points
        ],
    }
    path = "/tmp/preinsert_plan.json"
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(output, handle)
    print("saved", path, "points", len(trajectory.points))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
