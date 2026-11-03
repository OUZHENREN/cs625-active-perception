#!/usr/bin/env python3
"""Publish the measured environment (model.json collision voxels) into MoveIt.

Why this exists
---------------
The senior's ``environment_collision_generator`` voxelised the scanned cell and
published the boxes into the planning scene, SUBTRACTING the slot and box mesh
regions so the targets never block their own grasp.  This repository already
builds the same voxel boxes offline: ``scripts/model_environment.py`` stores
them as ``collisions`` in model.json -- 2.5-D boxes whose TOP FACE IS FLUSH
with the fitted table surface and whose body hangs below it, so free space
above a table is never claimed (that is this repo's answer to the senior's
"collision scene blocks the grasp" problem).  What was missing until now is the
runtime side: publishing those boxes into the real MoveIt scene, with the same
subtraction.

Subtraction regions (owner 2026-11-02: padding = 15 mm, the senior's value)
----------------------------------------------------------------------------
* fixture:   config/cs625_real_cell_scene.yaml fixture bbox (model frame) at
             the fixture pose.  The fixture mesh is already published separately
             by apply_real_cell_scene.py, so environment boxes there would
             double-count.
* workpiece: the same config's module bbox (CAD frame) at the RUNTIME measured
             S3 (--workpiece-xyz/--workpiece-quat).  The workpiece is
             deliberately NOT published (the grasp needs that space free) --
             this is the senior's enable_box_subtraction purpose.

Design note (verified 2026-11-02): the model's boxes are the TABLE-SURFACE
BANDS only (z in [top-0.02, top]), and both the fixture body and the workpiece
body sit ABOVE their table surface, so the subtraction regions normally remove
ZERO boxes.  That is correct, not a bug: the targets never appear inside the
band, and the real table surface under a target must stay (the part rests on
it).  The machinery is kept for the general case and for any future full-3D
environment voxels.

Padding applies in XY only.  Padding Z upward would claim free space above the
table, which the 2.5-D design exists to avoid; the fixture's own volume comes
from its separately published mesh.

Frame: model.json boxes are in the base frame; on the real cell the planning
frame ``world`` coincides with base (apply_real_cell_scene publishes in world).

Usage
-----
    python3 scripts/apply_environment_collision.py --dry-run
    python3 scripts/apply_environment_collision.py \
        --workpiece-xyz 0.6146 0.5763 0.3542 --workpiece-quat 0.0062 -0.0026 0.7346 0.6785
    python3 scripts/apply_environment_collision.py --remove
    python3 scripts/apply_environment_collision.py --keep-workpiece ...

Exit codes: 0 applied; 1 no model / no boxes; 2 MoveIt services unavailable.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (CollisionObject, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import SolidPrimitive

sys.stdout.reconfigure(line_buffering=True)
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from apply_real_cell_scene import load_scene, rpy_to_quaternion  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parent.parent
MODEL_JSON = REPO / "runtime" / "environment" / "latest" / "model.json"
SCENE_CONFIG = REPO / "config" / "cs625_real_cell_scene.yaml"
WORLD = "world"
PREFIX = "env_voxel_"


def region_aabb(bbox_min, bbox_max, xyz, quat_xyzw,
                pad_xy: float) -> tuple[np.ndarray, np.ndarray]:
    """AABB of an oriented bbox transformed to world and padded in XY."""
    corners = np.array([[x, y, z]
                        for x in (bbox_min[0], bbox_max[0])
                        for y in (bbox_min[1], bbox_max[1])
                        for z in (bbox_min[2], bbox_max[2])], dtype=float)
    rotation = R.from_quat(quat_xyzw).as_matrix()
    world = corners @ rotation.T + np.array(xyz, dtype=float)
    lo, hi = world.min(axis=0), world.max(axis=0)
    lo[0] -= pad_xy
    lo[1] -= pad_xy
    hi[0] += pad_xy
    hi[1] += pad_xy
    return lo, hi


def boxes_intersect(lo_a: np.ndarray, hi_a: np.ndarray,
                    lo_b: np.ndarray, hi_b: np.ndarray) -> bool:
    return bool(np.all(np.minimum(hi_a, hi_b) > np.maximum(lo_a, lo_b)))


def env_box_aabb(box: dict) -> tuple[np.ndarray, np.ndarray]:
    centre = np.array(box["center"], dtype=float)
    half = np.array(box["size"], dtype=float) / 2.0
    return centre - half, centre + half


def fixture_region(scene: dict, pad_xy: float) -> tuple[np.ndarray, np.ndarray]:
    fixture = scene.get("fixture") or {}
    pose = fixture.get("pose_world")
    bbox_min = fixture.get("bbox_min_m")
    bbox_max = fixture.get("bbox_max_m")
    if not pose or not bbox_min or not bbox_max:
        raise SystemExit("config has no fixture pose/bbox; cannot subtract the fixture")
    quat = rpy_to_quaternion(*(float(v) for v in pose[3:6]))
    return region_aabb([float(v) for v in bbox_min], [float(v) for v in bbox_max],
                       [float(v) for v in pose[:3]], quat, pad_xy)


def workpiece_region(scene: dict, xyz, quat_xyzw,
                     pad_xy: float) -> tuple[np.ndarray, np.ndarray]:
    module = scene.get("module") or {}
    bbox_min = module.get("bbox_min_m")
    bbox_max = module.get("bbox_max_m")
    if not bbox_min or not bbox_max:
        raise SystemExit("config has no module bbox; cannot subtract the workpiece")
    return region_aabb([float(v) for v in bbox_min], [float(v) for v in bbox_max],
                       xyz, quat_xyzw, pad_xy)


def filter_boxes(collisions: list[dict],
                 regions: list[tuple[str, np.ndarray, np.ndarray]]):
    """Split env boxes into (kept, removed) with the reason per removed box."""
    kept: list[dict] = []
    removed: dict[str, int] = {}
    for box in collisions:
        lo, hi = env_box_aabb(box)
        dropped = None
        for name, region_lo, region_hi in regions:
            if boxes_intersect(lo, hi, region_lo, region_hi):
                dropped = name
                break
        if dropped:
            removed[dropped] = removed.get(dropped, 0) + 1
        else:
            kept.append(box)
    return kept, removed


def box_object(object_id: str, box: dict) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = WORLD
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(v) for v in box["size"]]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in box["center"])
    pose.orientation.w = 1.0
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


def remove_object(object_id: str) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = WORLD
    obj.operation = CollisionObject.REMOVE
    return obj


def existing_env_ids(node: Node) -> list[str]:
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not client.wait_for_service(timeout_sec=40):
        print("NO /get_planning_scene")
        raise SystemExit(2)
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    future = client.call_async(request)
    deadline = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while not future.done() and node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    scene = future.result().scene
    return [obj.id for obj in scene.world.collision_objects
            if obj.id.startswith(PREFIX)]


def apply(objects: list[CollisionObject]) -> None:
    node = rclpy.create_node("apply_environment_collision")
    try:
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=40):
            print("NO /apply_planning_scene")
            raise SystemExit(2)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        deadline = node.get_clock().now().nanoseconds / 1e9 + 10.0
        while not future.done() and node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        response = future.result()
        print(f"ApplyPlanningScene: {'success' if response and response.success else 'FAILED'}")
    finally:
        node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default=str(MODEL_JSON))
    parser.add_argument("--config", default=str(SCENE_CONFIG))
    parser.add_argument("--padding", type=float, default=0.015,
                        help="XY padding around the subtracted regions (senior's 15 mm)")
    parser.add_argument("--workpiece-xyz", nargs=3, type=float, default=None,
                        help="runtime measured S3 position (metres)")
    parser.add_argument("--workpiece-quat", nargs=4, type=float, default=None,
                        help="runtime measured S3 quaternion (xyzw)")
    parser.add_argument("--keep-workpiece", action="store_true",
                        help="do not subtract the workpiece region")
    parser.add_argument("--prefix", default=PREFIX)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--remove", action="store_true",
                        help="only remove the previously published env boxes")
    args = parser.parse_args()

    model_path = pathlib.Path(args.model)
    if not model_path.is_file():
        print(f"NO_MODEL: {model_path}")
        return 1
    model = json.loads(model_path.read_text())
    collisions = model.get("collisions") or []
    print(f"model: {model_path}")
    print(f"env boxes in model: {len(collisions)}")

    if args.remove:
        print("remove only: clearing previously published env boxes")
        rclpy.init()
        node = rclpy.create_node("apply_environment_collision_remove")
        try:
            ids = existing_env_ids(node)
            print(f"  found {len(ids)} published env boxes")
            apply_objects = [remove_object(obj_id) for obj_id in ids]
        finally:
            node.destroy_node()
        apply(apply_objects)
        rclpy.shutdown()
        return 0

    if not collisions:
        print("NO_BOXES: model has no collision voxels; nothing to publish")
        return 1

    scene = load_scene(pathlib.Path(args.config))
    regions: list[tuple[str, np.ndarray, np.ndarray]] = []
    fixture_lo, fixture_hi = fixture_region(scene, args.padding)
    regions.append(("fixture", fixture_lo, fixture_hi))
    print(f"subtract fixture: {np.round(fixture_lo, 3)} .. {np.round(fixture_hi, 3)}")
    if not args.keep_workpiece:
        if args.workpiece_xyz and args.workpiece_quat:
            work_lo, work_hi = workpiece_region(scene, args.workpiece_xyz,
                                                args.workpiece_quat, args.padding)
            regions.append(("workpiece", work_lo, work_hi))
            print(f"subtract workpiece: {np.round(work_lo, 3)} .. {np.round(work_hi, 3)}")
        else:
            print("WARN: no --workpiece-xyz/quat and subtraction enabled; "
                  "only the fixture region is subtracted")

    kept, removed = filter_boxes(collisions, regions)
    for name, count in removed.items():
        print(f"  removed by {name}: {count}")
    print(f"publish: {len(kept)} boxes (of {len(collisions)})")
    if args.dry_run:
        print("dry run: nothing published")
        return 0

    rclpy.init()
    node = rclpy.create_node("apply_environment_collision_probe")
    try:
        old_ids = existing_env_ids(node)
    finally:
        node.destroy_node()
    objects = [remove_object(obj_id) for obj_id in old_ids]
    objects += [box_object(f"{args.prefix}{box['name']}", box) for box in kept]
    apply(objects)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
