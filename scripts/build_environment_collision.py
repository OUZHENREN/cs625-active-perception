#!/usr/bin/env python3
"""Build MoveIt collision objects from a merged environment PCD.

Replaces hand-drawn table boxes with voxel boxes derived from the measured
point cloud, so the collision scene follows the real table shape instead of a
draft bounding box that overlaps free space.

Only points in the configured z band (table surface band) are used.

Run:
  source scripts/source_dev_env.sh --full
  python3 scripts/build_environment_collision.py \
      --pcd /tmp/environment_20261022.pcd --apply
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def load_pcd_xyz(path: str) -> np.ndarray:
    raw = open(path, "rb").read()
    marker = b"DATA binary\n"
    end = raw.find(marker)
    if end < 0:
        raise SystemExit("only binary PCD is supported")
    header = raw[:end].decode("ascii", "replace")
    fields = []
    count = 0
    for line in header.splitlines():
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("POINTS"):
            count = int(line.split()[1])
    stride = 4 * len(fields)
    body = raw[end + len(marker):][: count * stride]
    table = np.frombuffer(body, dtype=np.uint8).reshape(count, stride)
    columns = []
    for index, name in enumerate(fields):
        if name in ("x", "y", "z"):
            chunk = table[:, index * 4:(index + 1) * 4].copy().view("<f4").ravel()
            columns.append(chunk)
    return np.column_stack(columns).astype(np.float64)


def box_object(object_id: str, center, size) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(v) for v in size]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
    pose.orientation.w = 1.0
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


def remove_object(object_id: str) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.REMOVE
    return obj


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--z-min", type=float, default=-0.13)
    parser.add_argument("--z-max", type=float, default=-0.07)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--min-points", type=int, default=8)
    parser.add_argument("--max-boxes", type=int, default=400)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--keep", nargs="*", default=[],
        help="world object ids to keep untouched (e.g. box_collision)",
    )
    parser.add_argument(
        "--drop", nargs="*", default=["workpiece_table", "slot_table"],
        help="world object ids to remove",
    )
    args = parser.parse_args()

    points = load_pcd_xyz(args.pcd)
    band = points[
        (points[:, 2] > args.z_min)
        & (points[:, 2] < args.z_max)
        & np.isfinite(points).all(axis=1)
    ]
    if len(band) == 0:
        print("no points in the configured z band")
        return 2
    print(f"band points: {len(band)}")

    cells: dict[tuple[int, int], int] = {}
    for x, y, _z in band:
        key = (int(np.floor(x / args.voxel)), int(np.floor(y / args.voxel)))
        cells[key] = cells.get(key, 0) + 1
    kept = {k: v for k, v in cells.items() if v >= args.min_points}
    ordered = sorted(kept.items(), key=lambda item: -item[1])[: args.max_boxes]
    print(f"voxels: {len(cells)} kept: {len(kept)} published: {len(ordered)}")

    objects = [remove_object(oid) for oid in args.drop]
    height = args.z_max - args.z_min
    center_z = (args.z_min + args.z_max) / 2.0
    for index, ((ix, iy), _count) in enumerate(ordered):
        center = ((ix + 0.5) * args.voxel, (iy + 0.5) * args.voxel, center_z)
        size = (args.voxel, args.voxel, max(height, 0.02))
        objects.append(box_object(f"env_voxel_{index:03d}", center, size))

    print(f"collision objects prepared: {len(objects)}")
    if not args.apply:
        print("dry run; pass --apply to publish")
        return 0

    rclpy.init()
    node = Node("build_environment_collision")
    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    for client in (apply_client, scene_client):
        while not client.wait_for_service(timeout_sec=1.0):
            pass

    scene = PlanningScene()
    scene.is_diff = True
    for obj in objects:
        scene.world.collision_objects.append(obj)
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = apply_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    result = future.result()
    print("apply:", bool(result and result.success))

    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    future = scene_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    scene_state = future.result()
    ids = [obj.id for obj in scene_state.scene.world.collision_objects] if scene_state else []
    print(f"scene objects: {len(ids)}")
    print("non-voxel ids:", [i for i in ids if not i.startswith("env_voxel_")])

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
