#!/usr/bin/env python3
"""Publish the grasped workpiece collision model as voxel boxes glued to the TCP.

The model comes from the pre-grasp point cloud (`workpiece_voxels.json`), so the
handle gap contains no boxes and the gripper/wrist no longer self-collide, while
the real outer surface still blocks the environment.

Each call re-places the voxels using the current TCP transform:

    T_world_box = T_world_tcp @ T_link_box

Run:
  python3 scripts/update_grasped_workpiece.py [--json PATH] [--clear]
"""

from __future__ import annotations

import argparse
import os
import json
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import SolidPrimitive

DEFAULT_JSON = os.environ.get(
    "CS625_WORKPIECE_VOXELS",
    str(Path("runs/20261022_workpiece_model/workpiece_voxels.json")),
)
LINK = "my_end_effector_link"
PREFIX = "wp_voxel_"
# box origin relative to the TCP link at grasp time (from tcp_server T_b_tcp)
T_LINK_BOX = np.array(
    [
        [-1.0, 0.0, 0.0, -0.009],
        [0.0, 1.0, 0.0, 0.215],
        [0.0, 0.0, -1.0, -0.411],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", default=DEFAULT_JSON)
    parser.add_argument("--clear", action="store_true")
    args = parser.parse_args()

    data = json.load(open(args.json, encoding="utf-8"))
    centers = np.array(data["centers"], dtype=float)
    leaf = float(data["leaf"])

    rclpy.init()
    node = Node("update_grasped_workpiece")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    while not client.wait_for_service(timeout_sec=1.0):
        pass

    def apply(scene: PlanningScene) -> bool:
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        result = future.result()
        return bool(result and result.success)

    clear = PlanningScene()
    clear.is_diff = True
    for index in range(600):
        obj = CollisionObject()
        obj.id = f"{PREFIX}{index:03d}"
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.REMOVE
        clear.world.collision_objects.append(obj)
    old = CollisionObject()
    old.id = "box_collision"
    old.header.frame_id = "world"
    old.operation = CollisionObject.REMOVE
    clear.world.collision_objects.append(old)
    print("clear:", apply(clear))
    if args.clear:
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
    rotation = T_world_box[:3, :3]
    origin = T_world_box[:3, 3]
    scene = PlanningScene()
    scene.is_diff = True
    for index, center in enumerate(centers):
        world_center = rotation @ center + origin
        obj = CollisionObject()
        obj.id = f"{PREFIX}{index:03d}"
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [leaf, leaf, leaf]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = world_center
        q = R.from_matrix(rotation).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj)
    print(f"publishing {len(centers)} workpiece voxels; box origin {np.round(origin,4).tolist()}")
    print("apply:", apply(scene))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
