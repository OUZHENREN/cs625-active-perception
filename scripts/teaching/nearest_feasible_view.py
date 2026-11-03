#!/usr/bin/env python3
"""Find the nearest ADOPTABLE stance for a view pose RVS asks for.

Why this exists (2026-11-03)
----------------------------
RVS returns a fine-view pose (S2).  With the real SolidWorks tool (294.6 mm wide,
motor seat included) that exact pose folds the tool into the forearm by 8.1 mm at
J5=143 deg, so it cannot be adopted.  The senior's system could stand there only
because his tool mesh was 55 mm narrower.

This script keeps the INTENT of the request -- aim the camera at the target from
about the requested distance -- while allowing a small offset in position and
orientation.  It searches a deterministic ladder of offsets, and for each
candidate that is reachable and collision-free it computes the CAMERA AIM ERROR
against the target and picks the smallest offset whose aim stays within
--max-aim-error-deg.

Data flow:
    requested TCP pose + target point
      -> candidate ladder (position offsets, then small rotations about the
         camera axes)
      -> IK (kinematics only) + /check_state_validity   [drop the infeasible]
      -> camera aim error via config/handeye/tcp_to_camera.json
      -> nearest candidate within the aim budget

Usage:
    python3 scripts/teaching/nearest_feasible_view.py \
        0.7844 0.2928 0.5723 -0.5935 0.6472 0.2667 0.3971 \
        --target 0.6146 0.5763 0.3542 \
        [--max-aim-error-deg 12] [--max-position-offset 0.15] [--json /tmp/view.json]

Exit codes: 0 found (printed + optionally written); 1 none within the budget;
2 MoveIt services unavailable.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]   # scripts/teaching/<file> -> repo root
HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
TASK_CONFIG = REPO / "config" / "tasks" / "common.yaml"
ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def load_limits() -> tuple[float, float]:
    task = yaml.safe_load(TASK_CONFIG.read_text())
    return (float(task.get("tool_flange_to_tcp_mm", 353.0)),
            float(task.get("tool_body_forward_from_flange_mm", 207.0)))


def camera_pose(tcp_xyz, quat, t_tcp_cam) -> tuple[np.ndarray, np.ndarray]:
    rotation = R.from_quat(quat).as_matrix()
    position = np.asarray(tcp_xyz) + rotation @ t_tcp_cam[:3, 3]
    look = rotation @ (t_tcp_cam[:3, :3] @ np.array([0.0, 0.0, 1.0]))
    return position, look / np.linalg.norm(look)


def aim_error_deg(tcp_xyz, quat, target, t_tcp_cam) -> float:
    position, look = camera_pose(tcp_xyz, quat, t_tcp_cam)
    to_target = np.asarray(target) - position
    distance = float(np.linalg.norm(to_target))
    if distance < 1e-6:
        return 0.0
    cos_angle = float(np.clip(np.dot(look, to_target / distance), -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))


def look_at_candidates(requested_xyz, requested_quat, target, t_tcp_cam,
                       distance: float, max_aim_error_deg: float):
    """Generate stances that AIM AT the target, ordered by distance from the
    requested position.

    Substituting near the requested pose is not enough when that pose is stale:
    measured 2026-11-03 the RVS fine-view pose aimed 22.7 deg away from the
    workpiece, so no small offset could rescue it.  These candidates keep the
    requested CAMERA DISTANCE and sweep a small angular neighbourhood of the
    requested viewing direction, then convert camera -> TCP through the measured
    hand-eye transform.
    """
    camera_requested, _ = camera_pose(requested_xyz, requested_quat, t_tcp_cam)
    base_direction = (camera_requested - target)
    norm = float(np.linalg.norm(base_direction))
    if norm < 1e-6:
        base_direction = np.array([0.0, 0.0, 1.0])
    else:
        base_direction = base_direction / norm

    # Orthonormal basis around the requested direction for the angular sweep.
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(helper, base_direction))) > 0.9:
        helper = np.array([1.0, 0.0, 0.0])
    right = np.cross(base_direction, helper)
    right /= np.linalg.norm(right)
    up = np.cross(right, base_direction)

    angles = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0]
    inverse_hand_eye = np.linalg.inv(t_tcp_cam)
    seen: set[tuple[float, float]] = set()
    candidates = []
    for theta in angles:
        for phi in angles:
            for sign_theta in ((1.0,) if theta == 0 else (-1.0, 1.0)):
                for sign_phi in ((1.0,) if phi == 0 else (-1.0, 1.0)):
                    key = (sign_theta * theta, sign_phi * phi)
                    if key in seen:
                        continue
                    seen.add(key)
                    rt, rp = np.radians(key)
                    direction = (base_direction * np.cos(rt) * np.cos(rp)
                                 + right * np.sin(rt) * np.cos(rp)
                                 + up * np.sin(rp))
                    camera = target + direction * distance
                    z_axis = -direction
                    x_axis = np.cross(up, z_axis)
                    if np.linalg.norm(x_axis) < 1e-6:
                        x_axis = np.cross(right, z_axis)
                    x_axis /= np.linalg.norm(x_axis)
                    y_axis = np.cross(z_axis, x_axis)
                    world_cam = np.eye(4)
                    world_cam[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
                    world_cam[:3, 3] = camera
                    world_tcp = world_cam @ inverse_hand_eye
                    xyz = world_tcp[:3, 3]
                    quat = R.from_matrix(world_tcp[:3, :3]).as_quat()
                    error = aim_error_deg(xyz, quat, target, t_tcp_cam)
                    if error > max_aim_error_deg:
                        continue
                    displacement = float(np.linalg.norm(xyz - requested_xyz))
                    candidates.append((displacement, f"aim(az{key[0]:+.0f},el{key[1]:+.0f})",
                                       xyz, quat, error))
    candidates.sort(key=lambda item: item[0])
    return candidates


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--target", nargs=3, type=float, required=True,
                        help="point the camera must keep looking at (metres, world)")
    parser.add_argument("--max-aim-error-deg", type=float, default=12.0,
                        help="the substitute stance may aim this far off the target")
    parser.add_argument("--max-position-offset", type=float, default=0.15)
    parser.add_argument("--max-rotation-deg", type=float, default=20.0)
    parser.add_argument("--json", default=None, help="write the chosen pose here")
    args = parser.parse_args()

    requested_xyz = np.array(args.pose[:3], dtype=float)
    requested_quat = np.array(args.pose[3:], dtype=float)
    target = np.array(args.target, dtype=float)
    t_tcp_cam = np.array(json.loads(HAND_EYE.read_text())["T_tcp_cam"], dtype=float)

    base_error = aim_error_deg(requested_xyz, requested_quat, target, t_tcp_cam)
    print(f"requested pose aims {base_error:.1f} deg off the target")

    rclpy.init()
    node = rclpy.create_node("nearest_feasible_view")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    for client, name in ((ik, "compute_ik"), (validity, "check_state_validity")):
        if not client.wait_for_service(timeout_sec=40):
            print(f"NO {name}")
            return 2

    current: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: current.update(dict(zip(m.name, m.position))), 10)
    deadline = time.time() + 8.0
    while len([k for k in ORDER if k in current]) < 6 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len([k for k in ORDER if k in current]) < 6:
        print("NO_JOINT_STATES")
        return 2
    seed = [current[k] for k in ORDER]

    distance = float(np.linalg.norm(
        camera_pose(requested_xyz, requested_quat, t_tcp_cam)[0] - target))
    candidates = look_at_candidates(requested_xyz, requested_quat, target, t_tcp_cam,
                                    distance, args.max_aim_error_deg)
    print(f"candidate stances aiming at the target from {distance:.2f} m: "
          f"{len(candidates)} (nearest first)")

    chosen = None
    tried = 0
    for displacement, label, xyz, quat, error in candidates:
        if displacement > args.max_position_offset:
            break
        tried += 1
        request = PositionIKRequest()
        request.group_name = "cs625_arm"
        request.ik_link_name = "my_end_effector_link"
        request.avoid_collisions = False
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [float(v) for v in xyz]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = [float(v) for v in quat]
        request.pose_stamped = pose
        state = RobotState()
        joints = JointState()
        joints.name = ORDER
        joints.position = seed
        state.joint_state = joints
        request.robot_state = state
        future = ik.call_async(GetPositionIK.Request(ik_request=request))
        deadline = time.time() + 15.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result()
        if response is None or response.error_code.val != 1:
            continue
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        candidate_joints = [solution[k] for k in ORDER]
        if max(abs(v) for v in np.degrees(candidate_joints)) > 180.0:
            continue
        check = RobotState()
        joints = JointState()
        joints.name = ORDER
        joints.position = candidate_joints
        check.joint_state = joints
        valid_request = GetStateValidity.Request()
        valid_request.robot_state = check
        valid_request.group_name = "cs625_arm"
        valid_request.constraints = Constraints()
        future = validity.call_async(valid_request)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result()
        if response is None or not response.valid:
            continue
        chosen = {
            "offset": label,
            "xyz": [float(v) for v in xyz],
            "quat": [float(v) for v in quat],
            "joints_deg": [round(float(v), 2) for v in np.degrees(candidate_joints)],
            "aim_error_deg": round(error, 2),
            "distance_m": round(distance, 3),
            "displacement_from_request_m": round(displacement, 3),
        }
        break

    node.destroy_node()
    rclpy.shutdown()

    if chosen is None:
        print(f"NO_FEASIBLE_VIEW within {args.max_aim_error_deg:.0f} deg aim / "
              f"{args.max_position_offset:.2f} m / {args.max_rotation_deg:.0f} deg "
              f"({tried} candidates were checked)")
        return 1
    print(f"chosen: {chosen['offset']} "
          f"({chosen['displacement_from_request_m']} m from the requested pose, "
          f"camera distance {chosen['distance_m']} m)")
    print(f"  TCP  {[round(v, 4) for v in chosen['xyz']]}  "
          f"quat {[round(v, 4) for v in chosen['quat']]}")
    print(f"  aim error {chosen['aim_error_deg']:.2f} deg "
          f"(requested {base_error:.2f}, budget {args.max_aim_error_deg:.0f})")
    print(f"  joints {chosen['joints_deg']} deg")
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(chosen, indent=2))
        print(f"  written to {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
