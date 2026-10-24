#!/usr/bin/env python3
"""Represent the carried workpiece as world boxes and validate a candidate pose.

Why a world box instead of an attached object: in this cell MoveIt keeps the
attached object but the mesh/mesh_poses offset is not usable, so planning does
not know where the carried workpiece really is.  World collision objects *are*
honoured internally by the collision checker (verified 2026-10-23), therefore
the carried part is published as a world box positioned at

    T_world_box = T_world_tcp x T_LINK_BOX

and the box is split into ``--slices`` slabs along its local x axis with an
``--inset-x`` removed from the gripper side, so the slab does not overlap the
fingers/wrist (that overlap is physically unavoidable and would otherwise make
every plan "colliding").

Verification uses /check_state_validity with a temporary world box, so it never
relies on reading poses back from /get_planning_scene (which zeroes them).

Usage (no robot motion):
  python3 scripts/validate_carried_pose.py                 # current pose
  python3 scripts/validate_carried_pose.py --xyz ... --quat ... --keep
  python3 scripts/validate_carried_pose.py --clear
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import SolidPrimitive

sys.stdout.reconfigure(line_buffering=True)

T_LINK_BOX = np.array(
    [
        [-1.0, 0.0, 0.0, -0.009],
        [0.0, 1.0, 0.0, 0.215],
        [0.0, 0.0, -1.0, -0.411],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
BOX_MIN = np.array([-0.038, -0.4355, -0.459])
BOX_MAX = np.array([0.076, 0.0055, 0.006])
PREFIX = "carried_box_"


class Scene:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("carried_pose_check")
        self.apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.valid = self.node.create_client(GetStateValidity, "/check_state_validity")
        for client in (self.apply, self.ik, self.valid):
            while not client.wait_for_service(timeout_sec=1.0):
                pass

    def pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        latest: dict = {}
        sub = self.node.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose",
            lambda msg: latest.__setitem__("m", msg), 10,
        )
        deadline = time.time() + 4.0
        while "m" not in latest and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        msg = latest.get("m")
        if msg is None:
            return None
        p, q = msg.pose.position, msg.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])

    def _apply(self, objects: list[CollisionObject]) -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        result = future.result()
        return bool(result and result.success)

    def clear(self) -> None:
        removes = []
        for index in range(16):
            obj = CollisionObject()
            obj.id = f"{PREFIX}{index}"
            obj.header.frame_id = "world"
            obj.operation = CollisionObject.REMOVE
            removes.append(obj)
        self._apply(removes)

    def publish_box(self, tcp_xyz: np.ndarray, tcp_quat: np.ndarray,
                    inset_x: float, slices: int) -> list[str]:
        transform = np.eye(4)
        transform[:3, :3] = R.from_quat(tcp_quat).as_matrix()
        transform[:3, 3] = tcp_xyz
        world = transform @ T_LINK_BOX
        low = np.array([BOX_MIN[0] + inset_x, BOX_MIN[1], BOX_MIN[2]])
        high = BOX_MAX.copy()
        span = high[0] - low[0]
        if span <= 0.002:
            raise SystemExit("inset too large: no box left")
        ids = []
        objects = []
        rot = world[:3, :3]
        quat = R.from_matrix(rot).as_quat()
        for index in range(slices):
            lo = low[0] + span * index / slices
            hi = low[0] + span * (index + 1) / slices
            centre_local = np.array([(lo + hi) / 2, (low[1] + high[1]) / 2,
                                     (low[2] + high[2]) / 2])
            centre = rot @ centre_local + world[:3, 3]
            dims = [float(hi - lo), float(high[1] - low[1]), float(high[2] - low[2])]
            obj = CollisionObject()
            obj.id = f"{PREFIX}{index}"
            obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dims
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = centre
            pose.orientation.x, pose.orientation.y = quat[0], quat[1]
            pose.orientation.z, pose.orientation.w = quat[2], quat[3]
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            objects.append(obj)
            ids.append(obj.id)
        self._apply(objects)
        return ids

    def ik_for(self, xyz: np.ndarray, quat: np.ndarray):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in quat)
        request = GetPositionIK.Request()
        request.ik_request.group_name = "cs625_arm"
        request.ik_request.pose_stamped.header.frame_id = "world"
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.timeout.sec = 3
        future = self.ik.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=12)
        result = future.result()
        if result is None or result.error_code.val != 1:
            return None
        return result.solution.joint_state

    def check(self, joints, ignore_prefixes: tuple[str, ...]) -> tuple[bool, list]:
        request = GetStateValidity.Request()
        request.group_name = "cs625_arm"
        state = RobotState()
        state.joint_state = joints
        request.robot_state = state
        future = self.valid.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=12)
        result = future.result()
        if result is None:
            return False, []
        # Only world-object contacts matter here.  Robot self pairs are reported
        # by this build even for pairs disabled in the SRDF, so filtering them
        # out is required; the carried slabs are the "tool" side of the check.
        robot_prefixes = ("base", "shoulder", "upperarm", "forearm", "wrist",
                          "my_end_effector", "flange", "tool", "camera")
        world_contacts = []
        tool_world = []
        for contact in result.contacts:
            pair = (contact.contact_body_1, contact.contact_body_2)
            is_tool = any(name.startswith(ignore_prefixes) for name in pair)
            both_robot = all(any(name.startswith(robot_prefixes) for name in (n,))
                             for n in pair)
            if both_robot:
                continue
            entry = (pair, round(contact.depth, 4))
            if is_tool:
                tool_world.append(entry)
            else:
                world_contacts.append(entry)
        return (not tool_world), tool_world


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xyz", nargs=3, type=float)
    parser.add_argument("--quat", nargs=4, type=float)
    parser.add_argument("--inset-x", type=float, default=0.060,
                        help="metres removed from the gripper side of the box")
    parser.add_argument("--slices", type=int, default=2)
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--keep", action="store_true",
                        help="leave the box in the scene after checking")
    args = parser.parse_args()

    scene = Scene()
    try:
        if args.clear:
            scene.clear()
            print("cleared")
            return 0
        if args.xyz and args.quat:
            xyz = np.array(args.xyz, dtype=float)
            quat = np.array(args.quat, dtype=float)
        else:
            measured = scene.pose()
            if measured is None:
                print("NO_POSE")
                return 2
            xyz, quat = measured
        scene.clear()
        ids = scene.publish_box(xyz, quat, args.inset_x, args.slices)
        joints = scene.ik_for(xyz, quat)
        if joints is None:
            print("IK_FAIL")
            if not args.keep:
                scene.clear()
            return 3
        ok, contacts = scene.check(joints, tuple(ids))
        print(f"box slabs: {ids}")
        print(f"pose ok: {ok}")
        for pair, depth in contacts[:8]:
            print(f"  {pair[0]} <-> {pair[1]}  {depth}")
        if not args.keep:
            scene.clear()
        return 0 if ok else 1
    finally:
        scene.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
