#!/usr/bin/env python3
"""Mirror the CS625 insertion task geometry into MoveIt's planning scene.

The fixture is static world geometry that Gazebo knows about and MoveIt does
not, so without this the planner will happily route the arm straight through the
slot fixture.  This is the same problem ``p7_apply_fixture_scene.py`` solves for
the P7 worlds, but the task fixture is a wedge whose convex hull is only 57% of
its bounding box, so a primitive decomposition would be both loose and wrong.
The collision object therefore carries the mesh itself.

The planning frame is ``world``: the SRDF pins ``base_link`` to ``world`` with a
fixed virtual joint, and ``sim_moveit.launch.py`` sets ``octomap_frame: world``.

Poses come from ``cs625_bringup/config/cs625_task_scene.yaml``, which
``test/contract_checks.py`` also cross-checks against the Gazebo world.  Nothing
here restates a dimension that already lives in the config.

    ros2 run cs625_bringup apply_task_scene.py --dry-run
    ros2 run cs625_bringup apply_task_scene.py --seated-module
"""

from __future__ import annotations

import argparse
import math
import pathlib
import struct
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle


# This script is installed with cs625_bringup and started from a launch file, so
# it deliberately imports nothing from test/.  The reader below is the same
# binary-STL parse used by test/inspect_stl_mass_properties.py, kept local so the
# installed launcher carries no dependency on the test tree.
#
# Paths resolve against a source checkout when one is reachable and against the
# ament share directories otherwise, so the same file works from `src/` during
# development and from `install/` when launched.
METADATA_PROBE = pathlib.Path("src/cs625_simulation/assets/cs625_task")

PLANNING_FRAME = "world"
TASK_SCENE_CONFIG = pathlib.Path("config/cs625_task_scene.yaml")
TASK_MODEL_SUBDIR = pathlib.Path("assets/cs625_task")
APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"

FIXTURE_OBJECT_ID = "cs625_task_slot_fixture"
MODULE_SEATED_OBJECT_ID = "cs625_task_shielding_module_seated"


def source_checkout_root() -> pathlib.Path | None:
    """Return the repository root if this file runs from a source checkout."""

    here = pathlib.Path(__file__).resolve()
    for candidate in (here.parent, *here.parents):
        if (candidate / METADATA_PROBE).is_dir():
            return candidate
    return None


def _package_share(package: str) -> pathlib.Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:  # pragma: no cover - only when the environment is broken
        return None
    try:
        return pathlib.Path(get_package_share_directory(package))
    except Exception:
        return None


def task_scene_config_path(override: str | None = None) -> pathlib.Path:
    if override:
        return pathlib.Path(override)
    root = source_checkout_root()
    if root is not None:
        return root / "src/cs625_bringup" / TASK_SCENE_CONFIG
    share = _package_share("cs625_bringup")
    if share is None:
        raise FileNotFoundError("cannot locate cs625_task_scene.yaml")
    return share / TASK_SCENE_CONFIG


def task_model_dir() -> pathlib.Path:
    root = source_checkout_root()
    if root is not None:
        return root / "src/cs625_simulation" / TASK_MODEL_SUBDIR
    share = _package_share("cs625_simulation")
    if share is None:
        raise FileNotFoundError("cannot locate the cs625_task model directory")
    return share / TASK_MODEL_SUBDIR


def read_binary_stl(path: pathlib.Path) -> np.ndarray:
    """Return an (n, 3, 3) float array of triangle vertices from a binary STL."""

    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"{path}: too short to be a binary STL")
    expected = struct.unpack_from("<I", payload, 80)[0]
    body = payload[84:]
    if len(body) != expected * 50:
        raise ValueError(
            f"{path}: header claims {expected} triangles ({expected * 50} bytes) "
            f"but the body is {len(body)} bytes; this is probably an ASCII STL"
        )
    records = np.frombuffer(body, dtype=np.dtype([("d", "<12f4"), ("attr", "<u2")]))
    return records["d"][:, 3:12].reshape(-1, 3, 3).astype(np.float64)


def compose(parent: list[float], child: list[float]) -> list[float]:
    """World pose of a child given in the parent's frame; both are SDF poses.

    Kept in step with scripts/sync_task_world.py, which derives the seated pose the
    same way.  test_task_scene_applier.py asserts the two agree, so the duplication
    cannot drift.
    """

    rotation = Rotation.from_euler("xyz", parent[3:6]).as_matrix()
    child_rotation = Rotation.from_euler("xyz", child[3:6]).as_matrix()
    product = rotation @ child_rotation
    pitch = -math.asin(max(-1.0, min(1.0, float(product[2, 0]))))
    rpy = (
        math.atan2(float(product[2, 1]), float(product[2, 2])),
        pitch,
        math.atan2(float(product[1, 0]), float(product[0, 0])),
    )
    return [float(v) for v in rotation @ np.array(child[:3]) + np.array(parent[:3])] + list(rpy)


def derive_seated_pose(parameters: dict) -> list[float]:
    insertion = parameters["insertion"]
    assembly = list(insertion["assembly_seated_position_m"]) + list(
        insertion["assembly_seated_rpy_rad"]
    )
    return compose(parameters["fixture"]["pose_world"], assembly)


def seated_pose_is_stale(parameters: dict) -> tuple[bool, list[float]]:
    expected = derive_seated_pose(parameters)
    recorded = parameters["insertion"]["seated_pose_world"]
    return (
        any(abs(a - b) > 1e-9 for a, b in zip(expected, recorded)),
        expected,
    )


def load_task_parameters(config_path: pathlib.Path | None = None) -> dict:
    """Read the task scene config, which is the authority for the geometry."""

    path = config_path or task_scene_config_path()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["cs625_task_scene"]["ros__parameters"]


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert an SDF roll/pitch/yaw triple to an xyzw quaternion.

    SDF composes the rotation as R = Rz(yaw) * Ry(pitch) * Rx(roll); this is the
    matching intrinsic Z-Y-X conversion, kept explicit because getting the order
    wrong is invisible until the fixture ends up in the wrong place.
    """

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_from_sdf(pose: list[float]) -> Pose:
    """Build a geometry_msgs/Pose from a six-element SDF pose."""

    message = Pose()
    message.position.x, message.position.y, message.position.z = (
        float(value) for value in pose[:3]
    )
    (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    ) = rpy_to_quaternion(*(float(value) for value in pose[3:6]))
    return message


def mesh_from_stl(mesh_path: pathlib.Path) -> Mesh:
    """Convert a binary STL into a shape_msgs/Mesh with shared vertices."""

    mesh = Mesh()
    index_of: dict[tuple[float, float, float], int] = {}
    for triangle in read_binary_stl(mesh_path):
        indices = []
        for vertex in triangle:
            key = (float(vertex[0]), float(vertex[1]), float(vertex[2]))
            index = index_of.get(key)
            if index is None:
                index = len(index_of)
                index_of[key] = index
                point = Point()
                point.x, point.y, point.z = key
                mesh.vertices.append(point)
            indices.append(index)
        face = MeshTriangle()
        face.vertex_indices = indices
        mesh.triangles.append(face)
    return mesh


def _collision_object(
    object_id: str, mesh_path: pathlib.Path, pose: list[float]
) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = PLANNING_FRAME
    collision_object.id = object_id
    collision_object.operation = CollisionObject.ADD
    collision_object.meshes.append(mesh_from_stl(mesh_path))
    collision_object.mesh_poses.append(pose_from_sdf(pose))
    return collision_object


def fixture_collision_object(parameters: dict) -> CollisionObject:
    """The static wedge fixture, at the pose the Gazebo world uses."""

    return _collision_object(
        FIXTURE_OBJECT_ID,
        task_model_dir() / "slot_fixture" / "meshes" / "slot_fixture.stl",
        parameters["fixture"]["pose_world"],
    )


def seated_module_collision_object(parameters: dict) -> CollisionObject:
    """The module at its seated pose, so the insert can be planned against it.

    This is the goal state, not the start state: the module at its home pose is a
    separate free body that the planner must not treat as fixed.
    """

    return _collision_object(
        MODULE_SEATED_OBJECT_ID,
        task_model_dir() / "shielding_module" / "meshes" / "shielding_module.stl",
        parameters["insertion"]["seated_pose_world"],
    )


def build_planning_scene(parameters: dict, include_seated_module: bool) -> PlanningScene:
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(fixture_collision_object(parameters))
    if include_seated_module:
        scene.world.collision_objects.append(seated_module_collision_object(parameters))
    return scene


def describe(parameters: dict, include_seated_module: bool) -> list[str]:
    """Return a human-readable summary, used by --dry-run and by the tests."""

    lines = []
    scene = build_planning_scene(parameters, include_seated_module)
    for collision_object in scene.world.collision_objects:
        pose = collision_object.mesh_poses[0]
        mesh = collision_object.meshes[0]
        lines.append(
            f"{collision_object.id}: frame={collision_object.header.frame_id} "
            f"position=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) "
            f"quaternion=({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
            f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f}) "
            f"vertices={len(mesh.vertices)} triangles={len(mesh.triangles)}"
        )
    return lines


def apply_scene(node, scene: PlanningScene, timeout_sec: float = 10.0) -> bool:
    client = node.create_client(ApplyPlanningScene, APPLY_PLANNING_SCENE_SERVICE)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(
            f"{APPLY_PLANNING_SCENE_SERVICE} is not available; is move_group running?"
        )
        return False
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        node.get_logger().error("apply_planning_scene did not answer")
        return False
    return bool(future.result().success)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task-scene-config",
        default=None,
        help="override the cs625_task_scene.yaml path",
    )
    parser.add_argument("--dry-run", action="store_true", help="print the scene and exit")
    parser.add_argument(
        "--seated-module",
        action="store_true",
        help="also add the module at its seated pose as a collision object",
    )
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--allow-stale-seated",
        action="store_true",
        help="publish even if insertion.seated_pose_world is out of date",
    )
    arguments = parser.parse_args()

    parameters = load_task_parameters(
        pathlib.Path(arguments.task_scene_config) if arguments.task_scene_config else None
    )

    if arguments.seated_module:
        stale, expected = seated_pose_is_stale(parameters)
        if stale and not arguments.allow_stale_seated:
            print(
                "REFUSING to publish a stale seated pose.\n"
                "  config insertion.seated_pose_world is out of date with the\n"
                "  fixture placement, so the module would be collided against a\n"
                "  fixture that is no longer where it was.\n"
                "  fix: python3 scripts/sync_task_world.py\n"
                "  or pass --allow-stale-seated if that is genuinely intended.",
                file=sys.stderr,
            )
            return 2

    if arguments.dry_run:
        for line in describe(parameters, arguments.seated_module):
            print(line)
        return 0

    rclpy.init()
    node = rclpy.create_node("cs625_task_fixture_scene")
    try:
        time.sleep(0.5)
        success = apply_scene(
            node,
            build_planning_scene(parameters, arguments.seated_module),
            timeout_sec=arguments.timeout_sec,
        )
        for line in describe(parameters, arguments.seated_module):
            node.get_logger().info(line)
        return 0 if success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
