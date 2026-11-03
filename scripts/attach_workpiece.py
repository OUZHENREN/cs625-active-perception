#!/usr/bin/env python3
"""Attach / detach the carried workpiece in the MoveIt planning scene (专项 2).

Modelled on the senior's ``collision_object_publisher`` (grasp_confirmed /
release_confirmed semantics) minus its event bus: this is a callable action for
the teaching sequence.

Conventions (mirroring the senior's, cross-checked against tcp_server):
    attach pose in the TCP link frame  T_tcp_box = inv(T_b_tcp)
    release world pose                 T_world_box = T_world_tcp @ inv(T_b_tcp)
    touch_links                        [my_end_effector_link]

The attached object follows the robot state, so MoveIt collision-checks the
carried part against the environment scene automatically during lift / transit
/ staging (the offline carried-box gate stays as a second layer).

Usage
-----
    python3 scripts/attach_workpiece.py --attach
    python3 scripts/attach_workpiece.py --detach \
        --place-xyz X Y Z --place-quat QX QY QZ QW
    python3 scripts/attach_workpiece.py --reset
    python3 scripts/attach_workpiece.py --attach --dry-run

Exit codes: 0 applied; 1 bad arguments; 2 MoveIt services unavailable.
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import Mesh, MeshTriangle

sys.stdout.reconfigure(line_buffering=True)
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from apply_real_cell_scene import mesh_path  # noqa: E402
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent / "teaching"))
from pose_math import T_B_TCP  # noqa: E402

ATTACHED_LINK = "my_end_effector_link"
ATTACHED_ID = "shielding_module_attached"
PLACED_ID = "shielding_module_placed"
MODEL_URI = "model://cs625_task/shielding_module"
REPO = pathlib.Path(__file__).resolve().parent.parent


def t_tcp_box() -> np.ndarray:
    """The carried box pose in the TCP link frame = inv(T_b_tcp)."""
    return np.linalg.inv(T_B_TCP)


def attach_pose() -> tuple[list[float], list[float]]:
    matrix = t_tcp_box()
    xyz = [float(v) for v in matrix[:3, 3]]
    quat = [float(v) for v in R.from_matrix(matrix[:3, :3]).as_quat()]
    return xyz, quat


def release_world_pose(tcp_xyz, tcp_quat) -> tuple[list[float], list[float]]:
    """T_world_box = T_world_tcp @ inv(T_b_tcp) at the release TCP."""
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(tcp_quat).as_matrix()
    matrix[:3, 3] = np.array(tcp_xyz, dtype=float)
    world_box = matrix @ t_tcp_box()
    xyz = [float(v) for v in world_box[:3, 3]]
    quat = [float(v) for v in R.from_matrix(world_box[:3, :3]).as_quat()]
    return xyz, quat


def load_mesh_stl(stl: pathlib.Path) -> Mesh:
    """Binary STL -> shape_msgs Mesh (the module mesh is small and binary)."""
    from geometry_msgs.msg import Point
    raw = stl.read_bytes()
    triangles = struct.unpack("<I", raw[80:84])[0]
    mesh = Mesh()
    for index in range(triangles):
        offset = 84 + index * 50 + 12
        for vertex in range(3):
            x, y, z = struct.unpack("<fff", raw[offset + vertex * 12:
                                               offset + vertex * 12 + 12])
            mesh.vertices.append(Point(x=float(x), y=float(y), z=float(z)))
        tri = MeshTriangle()
        tri.vertex_indices = [index * 3, index * 3 + 1, index * 3 + 2]
        mesh.triangles.append(tri)
    return mesh


def pose_msg(xyz, quat) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
        (float(v) for v in quat)
    return pose


def attached_object(mesh: Mesh) -> AttachedCollisionObject:
    attached = AttachedCollisionObject()
    attached.link_name = ATTACHED_LINK
    obj = CollisionObject()
    obj.id = ATTACHED_ID
    obj.header.frame_id = ATTACHED_LINK
    obj.meshes.append(mesh)
    obj.mesh_poses.append(pose_msg(*attach_pose()))
    obj.operation = CollisionObject.ADD
    attached.object = obj
    attached.touch_links = [ATTACHED_LINK]
    return attached


def detach_object() -> AttachedCollisionObject:
    attached = AttachedCollisionObject()
    attached.link_name = ATTACHED_LINK
    obj = CollisionObject()
    obj.id = ATTACHED_ID
    obj.header.frame_id = ATTACHED_LINK
    obj.operation = CollisionObject.REMOVE
    attached.object = obj
    attached.touch_links = [ATTACHED_LINK]
    return attached


def world_placed_object(mesh: Mesh, xyz, quat) -> CollisionObject:
    obj = CollisionObject()
    obj.id = PLACED_ID
    obj.header.frame_id = "world"
    obj.meshes.append(mesh)
    obj.mesh_poses.append(pose_msg(xyz, quat))
    obj.operation = CollisionObject.ADD
    return obj


def world_remove_object() -> CollisionObject:
    obj = CollisionObject()
    obj.id = PLACED_ID
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.REMOVE
    return obj


def world_remove_attached_id() -> CollisionObject:
    """A stray world copy of the attached id can appear after aborted diffs;
    remove it from the world list too (the id may live in either place)."""
    obj = CollisionObject()
    obj.id = ATTACHED_ID
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.REMOVE
    return obj


def apply(world: list[CollisionObject], attached: list[AttachedCollisionObject]) -> None:
    node = rclpy.create_node("attach_workpiece")
    try:
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        probe = node.create_client(GetPlanningScene, "/get_planning_scene")
        existing_world: set[str] = set()
        existing_attached: set[str] = set()
        if probe.wait_for_service(timeout_sec=40):
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            future = probe.call_async(request)
            deadline = node.get_clock().now().nanoseconds / 1e9 + 10.0
            while not future.done() and node.get_clock().now().nanoseconds / 1e9 < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if future.done() and future.result() is not None:
                scene = future.result().scene
                existing_world = {obj.id for obj in scene.world.collision_objects}
                existing_attached = {obj.object.id
                                     for obj in scene.robot_state.attached_collision_objects}
        # A REMOVE for an id that exists nowhere makes ApplyPlanningScene answer
        # success=False and can reject the whole diff; send removals only for ids
        # that actually exist (the senior FORCE-REMOVE pattern hit this too).
        world = [obj for obj in world
                 if obj.operation != CollisionObject.REMOVE
                 or obj.id in existing_world or obj.id in existing_attached]
        attached = [obj for obj in attached
                    if obj.object.operation != CollisionObject.REMOVE
                    or obj.object.id in existing_attached]
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=40):
            print("NO /apply_planning_scene")
            raise SystemExit(2)
        scene = PlanningScene()
        scene.is_diff = True
        # 2026-11-02: robot_state.is_diff must be set as well -- without it the
        # attached REMOVE is treated as a full robot-state overwrite and the
        # detach silently does nothing (the object stayed attached while
        # ApplyPlanningScene reported success).
        scene.robot_state.is_diff = True
        scene.world.collision_objects = world
        scene.robot_state.attached_collision_objects = attached
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        deadline = node.get_clock().now().nanoseconds / 1e9 + 10.0
        while not future.done() and node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        response = future.result()
        print(f"ApplyPlanningScene: {'success' if response and response.success else 'FAILED'}"
              f" (world {len(world)} objects, attached {len(attached)} objects)")
    finally:
        node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--attach", action="store_true")
    group.add_argument("--detach", action="store_true")
    group.add_argument("--reset", action="store_true")
    parser.add_argument("--place-xyz", nargs=3, type=float, default=None,
                        help="release TCP pose (metres); used with --detach to put "
                             "the part back in the world scene")
    parser.add_argument("--place-quat", nargs=4, type=float, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.detach and bool(args.place_xyz) != bool(args.place_quat):
        print("--place-xyz and --place-quat must be given together")
        return 1

    xyz, quat = attach_pose()
    print(f"attach pose in {ATTACHED_LINK}: "
          f"xyz={[round(v, 4) for v in xyz]} quat={[round(v, 4) for v in quat]}")

    if args.attach:
        print("action: attach the carried workpiece")
        if args.dry_run:
            print("dry run: nothing published")
            return 0
        rclpy.init()
        apply([world_remove_object(), world_remove_attached_id()],
              [attached_object(load_mesh_stl(mesh_path(MODEL_URI)))])
        rclpy.shutdown()
        return 0

    if args.detach:
        print("action: detach the carried workpiece"
              + (" + place in the world scene" if args.place_xyz else ""))
        place_xyz = place_quat = None
        if args.place_xyz:
            place_xyz, place_quat = release_world_pose(args.place_xyz, args.place_quat)
            print(f"  release world pose: xyz={[round(v, 4) for v in place_xyz]} "
                  f"quat={[round(v, 4) for v in place_quat]}")
        if args.dry_run:
            print("dry run: nothing published")
            return 0
        world = [world_remove_attached_id()]
        if place_xyz is not None:
            world.append(world_placed_object(load_mesh_stl(mesh_path(MODEL_URI)),
                                             place_xyz, place_quat))
        rclpy.init()
        apply(world, [detach_object()])
        rclpy.shutdown()
        return 0

    print("action: reset (remove attached and any placed copy)")
    if args.dry_run:
        print("dry run: nothing published")
        return 0
    rclpy.init()
    apply([world_remove_object(), world_remove_attached_id()], [detach_object()])
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
