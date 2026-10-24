#!/usr/bin/env python3
"""Keep a world-frame box_collision glued to the grasped workpiece.

MoveIt stores an attached object's mesh pose as identity when applied through
/apply_planning_scene in this setup, which makes the collision model sit at the
link origin instead of the real workpiece.  This helper instead re-places the
world box_collision at the current workpiece pose before each planning step:

    T_world_box = T_world_tcp @ T_link_box

where T_link_box is the grasp-time rigid relation (also ~ inv(T_b_tcp)).

Run:
  python3 scripts/update_grasped_box.py [--detach]
"""

from __future__ import annotations

import argparse
import os
import struct
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, PlanningScene,
                             AllowedCollisionEntry)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import Mesh, MeshTriangle

BOX_STL = os.environ.get(
    "CS625_BOX_COLLISION_STL",
    str(Path("box_collision.STL")),
)
SCALE = 1.00
LINK = "my_end_effector_link"

# grasp-time relation between the TCP link and the workpiece origin
TCP_GRASP = np.array([0.8289, 0.5876, -0.0583])
TCP_GRASP_Q = (-0.4835, 0.8753, 0.0075, 0.0018)
BOX_P = np.array([0.653422, 0.703592, 0.355183])
BOX_Q = (
    0.007481638933337939,
    -0.0017592429190824202,
    0.48351536911719123,
    0.875302129536247,
)


def matrix(xyz, quat) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = R.from_quat(quat).as_matrix()
    out[:3, 3] = xyz
    return out


T_LINK_BOX = np.linalg.inv(matrix(TCP_GRASP, TCP_GRASP_Q)) @ matrix(BOX_P, BOX_Q)


def load_stl(path: str):
    raw = open(path, "rb").read()
    if raw[:5].lower() == b"solid" and b"facet" in raw[:2000]:
        verts = []
        for line in raw.decode("ascii", "replace").splitlines():
            parts = line.strip().split()
            if parts and parts[0] == "vertex":
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        v = np.array(verts, float)
        return v, np.arange(len(v)).reshape(-1, 3)
    count = struct.unpack("<I", raw[80:84])[0]
    dtype = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    data = np.frombuffer(raw[84:], dtype=dtype, count=count)
    return data["v"].reshape(-1, 3).astype(float), np.arange(count * 3).reshape(-1, 3)


def mesh_message() -> Mesh:
    verts, faces = load_stl(BOX_STL)
    verts = verts * 0.001 * SCALE
    mesh = Mesh()
    for x, y, z in verts:
        point = Point()
        point.x, point.y, point.z = float(x), float(y), float(z)
        mesh.vertices.append(point)
    for tri in faces:
        face = MeshTriangle()
        face.vertex_indices = [int(tri[0]), int(tri[1]), int(tri[2])]
        mesh.triangles.append(face)
    return mesh


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--detach", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = Node("update_grasped_box")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    while not client.wait_for_service(timeout_sec=1.0):
        pass

    def apply(scene: PlanningScene) -> bool:
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        result = future.result()
        return bool(result and result.success)

    detach = AttachedCollisionObject()
    detach.link_name = LINK
    detach.object.id = "box_collision"
    detach.object.header.frame_id = LINK
    detach.object.operation = CollisionObject.REMOVE
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects.append(detach)
    print("detach:", apply(scene))
    if args.detach:
        return 0

    transform = None
    for _ in range(80):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buffer.lookup_transform(
                "world", LINK, Time(), timeout=RDuration(seconds=0.3)
            ).transform
            transform = np.eye(4)
            transform[:3, :3] = R.from_quat(
                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
            ).as_matrix()
            transform[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
            break
        except Exception:
            continue
    if transform is None:
        print("NO_TF")
        return 2

    T_world_box = transform @ T_LINK_BOX
    obj = CollisionObject()
    obj.id = "box_collision"
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.ADD
    obj.meshes = [mesh_message()]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = T_world_box[:3, 3]
    q = R.from_matrix(T_world_box[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    obj.mesh_poses = [pose]

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(obj)
    print("world box placed at", np.round(T_world_box[:3, 3], 4).tolist())

    # The collision mesh is a solid block that also covers the finger gap, so the
    # robot links that physically sit inside that gap must be exempt from
    # self-collision with the box (equivalent to the senior touch_links).
    link_names = [
        "my_end_effector_link", "flange", "tool0",
        "wrist_1_link", "wrist_2_link", "wrist_3_link",
        "gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link",
    ]
    acm = scene.allowed_collision_matrix
    acm.entry_names = ["box_collision"] + link_names
    total = len(acm.entry_names)
    rows = []
    for i in range(total):
        row = AllowedCollisionEntry()
        row.enabled = []
        for j in range(total):
            if i == 0 and j == 0:
                row.enabled.append(True)          # box vs itself: check
            elif i == 0 or j == 0:
                row.enabled.append(False)         # box vs robot links: allowed
            else:
                row.enabled.append(True)          # robot self pairs: still checked
        rows.append(row)
    acm.entry_values = rows
    print("acm entries:", len(acm.entry_names))
    print("apply:", apply(scene))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
