#!/usr/bin/env python3
"""Plan a single TCP pose goal and dump the joint trajectory (plan only).

Why this is a tracked script
----------------------------
The coarse->fine pipeline used to shell out to ``/tmp/plan_pose.py``.  A pipeline
that depends on an untracked file in ``/tmp`` breaks silently after a reboot, and
that script also hard-coded assembly-grade tolerances (0.5 mm position sphere,
0.001 rad orientation).  Those tolerances are right for insertion, where a
"successful" 2.9 deg misalignment is a false success, but they make an ordinary
transit to an observation pose fail: the planner has to hit one exact point in
Cartesian space and OMPL usually gives up (measured: code 99999 on a pose whose
IK exists and whose path is collision-free).

An observation goal only has to put the camera in the right place - the next
stage re-derives the target from the new frame - so 5 mm / 0.02 rad is enough.

Plan only: nothing is executed here.

Usage:
  python3 scripts/plan_pose_once.py X Y Z QX QY QZ QW \
      [--velocity 0.10] [--position-tolerance 0.005] \
      [--orientation-tolerance 0.02] [--output /tmp/jm.json]

Prints ``code <MoveItErrorCodes> points <n> duration <s>`` and exits 0 on
success.  The trajectory JSON is written for scripts/execute_assembly_chunks.py.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (BoundingVolume, Constraints,
                             OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetMotionPlan as GetMotionPlanSrv
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

sys.stdout.reconfigure(line_buffering=True)

FRAME = "world"          # the MoveIt model frame; identity with base on this cell
GROUP = "cs625_arm"
TIP = "my_end_effector_link"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--velocity", type=float, default=0.05,
                        help="velocity AND acceleration scaling for the plan.  On this "
                             "cell 0.05 IS the normal speed (the owner's words), so do "
                             "not treat it as a slow-down; assembly segments drop to "
                             "0.005-0.01")
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="position sphere radius in metres")
    parser.add_argument("--orientation-tolerance", type=float, default=0.02,
                        help="per-axis orientation tolerance in radians")
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--budget", type=float, default=15.0, help="planning seconds")
    parser.add_argument("--output", default="/tmp/jm.json")
    parser.add_argument("--frame", default=FRAME)
    arguments = parser.parse_args()

    xyz = arguments.pose[:3]
    quat = arguments.pose[3:]

    rclpy.init()
    node = Node("plan_pose_once")
    client = node.create_client(GetMotionPlanSrv, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=10.0):
        print("plan service unavailable")
        return 4

    constraints = Constraints()
    position = PositionConstraint()
    position.header.frame_id = arguments.frame
    position.link_name = TIP
    volume = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [arguments.position_tolerance]
    volume.primitives.append(sphere)
    centre = Pose()
    centre.position.x, centre.position.y, centre.position.z = xyz
    centre.orientation.w = 1.0
    volume.primitive_poses.append(centre)
    position.constraint_region = volume
    position.weight = 1.0
    constraints.position_constraints.append(position)

    orientation = OrientationConstraint()
    orientation.header.frame_id = arguments.frame
    orientation.link_name = TIP
    (orientation.orientation.x, orientation.orientation.y,
     orientation.orientation.z, orientation.orientation.w) = quat
    orientation.absolute_x_axis_tolerance = arguments.orientation_tolerance
    orientation.absolute_y_axis_tolerance = arguments.orientation_tolerance
    orientation.absolute_z_axis_tolerance = arguments.orientation_tolerance
    orientation.weight = 1.0
    constraints.orientation_constraints.append(orientation)

    request = GetMotionPlanSrv.Request()
    motion = request.motion_plan_request
    motion.group_name = GROUP
    motion.goal_constraints.append(constraints)
    motion.num_planning_attempts = arguments.attempts
    motion.allowed_planning_time = arguments.budget
    motion.max_velocity_scaling_factor = arguments.velocity
    motion.max_acceleration_scaling_factor = arguments.velocity

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=arguments.budget + 25.0)
    response = future.result()
    rclpy.shutdown()
    if response is None:
        print("plan timed out")
        return 5

    code = response.motion_plan_response.error_code.val
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    points = trajectory.points
    if code != 1 or not points:
        print(f"code {code} points {len(points)}")
        return 6

    duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
    pathlib.Path(arguments.output).write_text(json.dumps({
        "joint_names": list(trajectory.joint_names),
        "points": [{"q": list(p.positions), "t": p.time_from_start.sec
                    + p.time_from_start.nanosec * 1e-9} for p in points],
    }))
    print(f"code {code} points {len(points)} duration {duration:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
