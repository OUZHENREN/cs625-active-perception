#!/usr/bin/env python3
"""Geometric clearance check for the carried workpiece.

The attached-object collision model is not reliable in this cell (MoveIt stores
mesh_poses as identity), so planning does not actually know where the workpiece
is.  This script computes it directly:

    T_world_box = T_world_tcp x T_LINK_BOX

and tests the resulting oriented box against the world collision boxes that are
currently in the planning scene (table voxels, slot, ...).

Usage:
  python3 scripts/check_carried_clearance.py                 # check current TCP
  python3 scripts/check_carried_clearance.py --xyz 0.83 0.59 -0.05
  python3 scripts/check_carried_clearance.py --quat x y z w

Exit codes: 0 clear, 1 collision or too close, 2 no data.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

# box origin relative to the TCP link while grasped (metres)
T_LINK_BOX = np.array(
    [
        [-1.0, 0.0, 0.0, -0.009],
        [0.0, 1.0, 0.0, 0.215],
        [0.0, 0.0, -1.0, -0.411],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
# workpiece local bounding box (metres), from box_collision.STL
BOX_MIN = np.array([-0.038, -0.4355, -0.459])
BOX_MAX = np.array([0.076, 0.0055, 0.006])
CLEARANCE_MIN = 0.005  # metres


def current_pose(node: Node) -> tuple[np.ndarray, np.ndarray] | None:
    latest: dict = {}
    sub = node.create_subscription(
        PoseStamped, "/tcp_pose_broadcaster/pose",
        lambda m: latest.__setitem__("m", m), 10,
    )
    deadline = time.time() + 4.0
    while "m" not in latest and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    msg = latest.get("m")
    if msg is None:
        return None
    p, q = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def world_boxes(node: Node) -> list[tuple[np.ndarray, np.ndarray, str]]:
    """Return (aabb_min, aabb_max, name) for every box/mesh object in the scene."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    while not client.wait_for_service(timeout_sec=1.0):
        pass
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.WORLD_OBJECT_NAMES
    )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    scene = future.result()
    boxes: list[tuple[np.ndarray, np.ndarray, str]] = []
    if scene is None:
        return boxes
    for obj in scene.scene.world.collision_objects:
        for primitive, pose in zip(obj.primitives, obj.primitive_poses):
            half = np.array(primitive.dimensions) / 2.0
            rot = R.from_quat([pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w]).as_matrix()
            centre = np.array([pose.position.x, pose.position.y, pose.position.z])
            # conservative AABB of the oriented box
            extent = np.abs(rot) @ half
            boxes.append((centre - extent, centre + extent, obj.id))
        for mesh, pose in zip(obj.meshes, obj.mesh_poses):
            if not mesh.vertices:
                continue
            verts = np.array([[v.x, v.y, v.z] for v in mesh.vertices])
            rot = R.from_quat([pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w]).as_matrix()
            centre = np.array([pose.position.x, pose.position.y, pose.position.z])
            world = (rot @ verts.T).T + centre
            boxes.append((world.min(axis=0), world.max(axis=0), obj.id))
    return boxes


def obb_min_clearance(tcp_xyz: np.ndarray, tcp_quat: np.ndarray,
                      boxes: list[tuple[np.ndarray, np.ndarray, str]]) -> tuple[float, str]:
    """Smallest gap between the carried box OBB and the scene AABBs."""
    T_world_tcp = np.eye(4)
    T_world_tcp[:3, :3] = R.from_quat(tcp_quat).as_matrix()
    T_world_tcp[:3, 3] = tcp_xyz
    T_world_box = T_world_tcp @ T_LINK_BOX
    rot = T_world_box[:3, :3]
    origin = T_world_box[:3, 3]

    corners = []
    for x in (BOX_MIN[0], BOX_MAX[0]):
        for y in (BOX_MIN[1], BOX_MAX[1]):
            for z in (BOX_MIN[2], BOX_MAX[2]):
                corners.append(rot @ np.array([x, y, z]) + origin)
    corners = np.array(corners)

    best = float("inf")
    worst_name = ""
    for lo, hi, name in boxes:
        # separating-axis test on the three world axes (conservative for OBB)
        gap = max(float(np.max(lo - corners, axis=1).max()) if False else 0.0, 0.0)
        overlap_x = min(corners[:, 0].max(), hi[0]) - max(corners[:, 0].min(), lo[0])
        overlap_y = min(corners[:, 1].max(), hi[1]) - max(corners[:, 1].min(), lo[1])
        overlap_z = min(corners[:, 2].max(), hi[2]) - max(corners[:, 2].min(), lo[2])
        if overlap_x > 0 and overlap_y > 0 and overlap_z > 0:
            return -1.0, name  # intersecting
        separation = min(
            max(lo[0] - corners[:, 0].max(), corners[:, 0].min() - hi[0]),
            max(lo[1] - corners[:, 1].max(), corners[:, 1].min() - hi[1]),
            max(lo[2] - corners[:, 2].max(), corners[:, 2].min() - hi[2]),
        )
        if separation < best:
            best = separation
            worst_name = name
        del gap
    return best, worst_name


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xyz", nargs=3, type=float)
    parser.add_argument("--quat", nargs=4, type=float)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_carried_clearance")
    try:
        if args.xyz and args.quat:
            xyz = np.array(args.xyz, dtype=float)
            quat = np.array(args.quat, dtype=float)
        else:
            measured = current_pose(node)
            if measured is None:
                print("NO_POSE")
                return 2
            xyz, quat = measured
        boxes = world_boxes(node)
        print(f"world boxes: {len(boxes)}")
        if not boxes:
            print("NO_SCENE")
            return 2
        clearance, name = obb_min_clearance(xyz, quat, boxes)
        if clearance < 0:
            print(f"COLLISION with {name}")
            return 1
        status = "CLEAR" if clearance >= CLEARANCE_MIN else "TOO_CLOSE"
        print(f"min clearance {clearance * 1000:.1f} mm near {name}: {status}")
        return 0 if clearance >= CLEARANCE_MIN else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
