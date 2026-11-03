#!/usr/bin/env python3
"""Drive the real TCP onto a nominal pose, iterating until it is within tolerance.

Why this is a tracked script (2026-11-03)
----------------------------------------
A joint-space plan stops when every joint is inside the goal tolerance, which is
not the same as the TCP being on the pose: measured on this cell, 0.0003 rad of
joint error still left 10.8 mm of TCP error (353 mm lever), and the pre-grasp
gate (0.5 mm / 0.05 deg) rejects that.  This script closes the last millimetre:
IK for the exact pose -> joint-space plan -> slow segmented execution -> measure,
repeating until the gate passes.

A 12 deg roll error from an unrelated wiggle test was corrected by this loop on
2026-11-03; it is the tool that turns "arrived at the pose" into "arrived at the
pose with the gripper square to the part".

Usage:
    python3 scripts/teaching/adjust_tcp.py X Y Z QX QY QZ QW \
        [--pos-tol-mm 0.5] [--rot-tol-deg 0.05] [--iterations 4] \
        [--velocity 0.008] [--dry-run]

Exit codes: 0 inside tolerance; 1 not reached; 2 services unavailable;
3 planning/execution failed; 5 envelope check refused.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
EXECUTOR = REPO / "scripts" / "execute_assembly_chunks.py"
ENVELOPE = REPO / "scripts" / "check_tool_envelope.py"
JOINT_TOL = 0.0004


def measure(node, latest) -> tuple[np.ndarray, np.ndarray, dict]:
    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if "j" in latest and "p" in latest:
            break
    pose = latest["p"].pose
    xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
    quat = np.array([pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w])
    joints = dict(zip(latest["j"].name, latest["j"].position))
    return xyz, quat, joints


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--pos-tol-mm", type=float, default=0.5)
    parser.add_argument("--rot-tol-deg", type=float, default=0.05)
    parser.add_argument("--iterations", type=int, default=4)
    parser.add_argument("--velocity", type=float, default=0.008)
    parser.add_argument("--boost", type=float, default=1.0,
                        help="gain on the measured error; 1.0 converges in one step "
                             "when the servo is perfect, 0.5 damps oscillation at the "
                             "tracking-error floor")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    target_xyz = np.array(args.pose[:3], dtype=float)
    target_quat = np.array(args.pose[3:], dtype=float)
    target_rot = R.from_quat(target_quat)

    rclpy.init()
    node = rclpy.create_node("adjust_tcp")
    latest: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.__setitem__("j", m), 10)
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: latest.__setitem__("p", m), 10)
    ik = node.create_client(GetPositionIK, "/compute_ik")
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    for client, name in ((ik, "compute_ik"), (planner, "plan_kinematic_path")):
        if not client.wait_for_service(timeout_sec=40):
            print(f"NO {name}")
            return 2

    for iteration in range(1, args.iterations + 1):
        xyz, quat, joints = measure(node, latest)
        pos_err = float(np.linalg.norm(target_xyz - xyz) * 1000.0)
        rot_err = float((R.from_quat(quat).inv() * target_rot).magnitude() * 180.0 / np.pi)
        print(f"[iter {iteration}] TCP {np.round(xyz, 4)} "
              f"err {pos_err:.3f} mm / {rot_err:.3f} deg")
        if pos_err <= args.pos_tol_mm and rot_err <= args.rot_tol_deg:
            print("TCP_GATE_OK")
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if args.dry_run:
            print("dry run: no motion")
            node.destroy_node()
            rclpy.shutdown()
            return 0

        # Error-boosted target (2026-11-03).  Re-solving the SAME pose returns the
        # same joints, and KDL's solution carries ~0.5 mm of numerical residual:
        # the planner then emits a 1-point "already there" trajectory and the TCP
        # never moves (observed: stuck at 0.539 mm for four iterations).  So aim
        # at target + (target - measured) instead -- a closed-loop step that
        # drives the measured TCP onto the target.
        measured_rot = R.from_quat(quat)
        error_rot = R.from_rotvec(args.boost * (target_rot * measured_rot.inv()).as_rotvec())
        commanded_pos = target_xyz + args.boost * (target_xyz - xyz)
        commanded_quat = (error_rot * target_rot).as_quat()
        print(f"  commanding {np.round(commanded_pos, 5)} "
              f"(boost by {np.round((target_xyz - xyz) * 1000, 2)} mm)")

        request = PositionIKRequest()
        request.group_name = "cs625_arm"
        request.ik_link_name = "my_end_effector_link"
        request.avoid_collisions = False
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = commanded_pos
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = commanded_quat
        request.pose_stamped = pose
        state = RobotState()
        joints_state = JointState()
        joints_state.name = ORDER
        joints_state.position = [joints[k] for k in ORDER]
        state.joint_state = joints_state
        request.robot_state = state
        future = ik.call_async(GetPositionIK.Request(ik_request=request))
        deadline = time.time() + 20.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result()
        if response is None or response.error_code.val != 1:
            print("IK failed for the correction")
            node.destroy_node()
            rclpy.shutdown()
            return 3
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        goal = [solution[k] for k in ORDER]

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
            joint_constraint.tolerance_above = JOINT_TOL
            joint_constraint.tolerance_below = JOINT_TOL
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        plan_request.goal_constraints.append(constraints)
        future = planner.call_async(GetMotionPlan.Request(motion_plan_request=plan_request))
        deadline = time.time() + 60.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        planned = future.result().motion_plan_response
        print(f"  plan code {planned.error_code.val}, "
              f"{len(planned.trajectory.joint_trajectory.points)} points")
        if planned.error_code.val != 1:
            node.destroy_node()
            rclpy.shutdown()
            return 3
        trajectory = planned.trajectory.joint_trajectory
        points = [{"q": list(p.positions),
                   "t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9}
                  for p in trajectory.points]
        out = pathlib.Path("/tmp/adjust_tcp_plan.json")
        out.write_text(json.dumps({"joint_names": list(trajectory.joint_names),
                                   "points": points}))
        guard = subprocess.run([sys.executable, str(ENVELOPE), str(out),
                                "--sample", "2", "--margin", "0.02"],
                               capture_output=True, text=True, timeout=600)
        for line in (guard.stdout or "").strip().splitlines()[-2:]:
            print("  envelope:", line)
        if guard.returncode != 0:
            print("  envelope check refused this correction")
            node.destroy_node()
            rclpy.shutdown()
            return 5
        done = subprocess.run([sys.executable, str(EXECUTOR), str(out),
                               "--time-scale", "2.0", "--goal-tolerance", str(JOINT_TOL),
                               "--verify-final", str(JOINT_TOL), "--fault-poll", "0.2"],
                              capture_output=True, text=True, timeout=600)
        output = (done.stdout or "") + (done.stderr or "")
        if "ALL_ASSEMBLY_CHUNKS_OK" not in output:
            print("  execution failed")
            for line in output.strip().splitlines()[-2:]:
                print("   ", line)
            node.destroy_node()
            rclpy.shutdown()
            return 3
        print("  corrected")

    xyz, quat, _ = measure(node, latest)
    pos_err = float(np.linalg.norm(target_xyz - xyz) * 1000.0)
    rot_err = float((R.from_quat(quat).inv() * target_rot).magnitude() * 180.0 / np.pi)
    print(f"TCP_GATE_NOT_MET after {args.iterations} iterations: "
          f"{pos_err:.3f} mm / {rot_err:.3f} deg")
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
