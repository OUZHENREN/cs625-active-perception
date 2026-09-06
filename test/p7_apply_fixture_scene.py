#!/usr/bin/env python3
"""Apply the tracked P7 Gazebo fixture geometry to MoveIt's planning scene.

This simulation-only helper mirrors the collision geometry in one explicitly
selected P7 world profile.  It does not move the robot or claim contact.  Its
purpose is to prevent MoveIt from approving paths that pass through Gazebo's
ground, occluder, or pre-attachment target proxy.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject,
    PlanningScene, PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from cs625_task_orchestrator.p7_grasp_geometry import target_pose_relative_to_grasp
from p7_generate_grasp_candidates import generate_candidates, pose_record


PLANNING_FRAME = "base_link"
FINGER_LINKS = ("gripper_left_finger_link", "gripper_right_finger_link")


# Values are copied from the named, tracked SDF worlds.  Keep the historical
# light profile as the default because the existing P7.5 runner invokes this
# helper without a profile argument.  P7.4 severe-v5 evidence must select the
# matching profile explicitly rather than planning against the light fixture.
FIXTURE_PROFILES = {
    "light": {
        "source_world": "p7_ycb_tomato_light.sdf",
        "ground": {
            "dimensions_m": (10.0, 10.0, 0.1),
            "position_m": (0.0, 0.0, -0.05),
            "yaw_rad": 0.0,
        },
        "occluder": {
            "dimensions_m": (0.10, 0.18, 0.44),
            "position_m": (0.60, 0.30, 0.22),
            "yaw_rad": 0.18,
        },
    },
    "severe_v5": {
        "source_world": "p7_ycb_tomato_occlusion_severe_v5.sdf",
        "ground": {
            "dimensions_m": (10.0, 10.0, 0.1),
            "position_m": (0.0, 0.0, -0.05),
            "yaw_rad": 0.0,
        },
        "occluder": {
            "dimensions_m": (0.055, 0.100, 0.175),
            "position_m": (0.650, 0.055, 0.0875),
            "yaw_rad": 0.0,
        },
    },
}

TARGET_PROXY = {
    "radius_m": 0.0340,
    "height_m": 0.101855,
    "position_m": (0.72 - 0.0091685, 0.0 + 0.0840170, 0.005 + 0.0510065),
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=tuple(FIXTURE_PROFILES),
        default="light",
        help="tracked Gazebo fixture geometry to mirror (default: light)",
    )
    parser.add_argument(
        "--mode",
        choices=("full", "approach", "attached"),
        default="full",
        help=(
            "full adds all tracked fixture bodies; approach keeps the static "
            "ground/occluder/target and allows only finger-target contact; attached replaces "
            "the world target with a conservative carried-object proxy"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="optional atomic JSON receipt path for the applied scene transition",
    )
    parser.add_argument("--model-pose-file", type=Path)
    parser.add_argument("--target-estimate-file", type=Path,
                        help="accepted estimator CAD-centre pose; no model-origin offset")
    parser.add_argument("--grasp-pose-file", type=Path)
    parser.add_argument(
        "--target-local-center-m", nargs=3, type=float, metavar=("X", "Y", "Z")
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the selected geometry without contacting ROS or MoveIt",
    )
    arguments = parser.parse_args()
    if arguments.target_estimate_file and (
        arguments.model_pose_file or arguments.target_local_center_m is not None
    ):
        parser.error("estimated centre cannot be combined with model origin/local offset")
    if arguments.mode == "attached" and arguments.target_estimate_file:
        if arguments.grasp_pose_file is None:
            parser.error("estimated attached mode requires --grasp-pose-file in estimate frame")
    elif arguments.mode == "attached" and any(
        value is None
        for value in (
            arguments.model_pose_file,
            arguments.grasp_pose_file,
            arguments.target_local_center_m,
        )
    ):
        parser.error(
            "attached mode requires model/grasp pose files and target local centre"
        )
    return arguments


def fixture_geometry_record(profile: str, mode: str) -> dict:
    """Return a JSON-serializable statement of the selected scene geometry."""

    selected = FIXTURE_PROFILES[profile]
    target_operation = "REMOVE" if mode == "attached" else "ADD"
    world_objects = [
        {
            "id": "p7_ground",
            "operation": "ADD",
            "shape": "box",
            **selected["ground"],
        },
        {
            "id": "p7_occluder",
            "operation": "ADD",
            "shape": "box",
            **selected["occluder"],
        },
    ]
    if mode != "attached":
        world_objects.append(
            {
                "id": "p7_target_contact_proxy",
                "operation": target_operation,
                "shape": "cylinder",
                **TARGET_PROXY,
            }
        )
    return {
        "profile": profile,
        "source_world": selected["source_world"],
        "planning_frame": PLANNING_FRAME,
        "mode": mode,
        "world_collision_objects": world_objects,
        "attached_target_proxy": mode == "attached",
        "allowed_finger_target_contacts": list(FINGER_LINKS) if mode != "full" else [],
    }


def world_objects_for_profile(profile: str) -> tuple[list[CollisionObject], CollisionObject]:
    """Build the persistent objects and target proxy for one tracked profile."""

    selected = FIXTURE_PROFILES[profile]
    ground = selected["ground"]
    occluder = selected["occluder"]
    static_objects = [
        box_object(
            "p7_ground",
            ground["dimensions_m"],
            ground["position_m"],
            ground["yaw_rad"],
        ),
        box_object(
            "p7_occluder",
            occluder["dimensions_m"],
            occluder["position_m"],
            occluder["yaw_rad"],
        ),
    ]
    target = cylinder_object(
        "p7_target_contact_proxy",
        TARGET_PROXY["radius_m"],
        TARGET_PROXY["height_m"],
        TARGET_PROXY["position_m"],
    )
    return static_objects, target


def box_object(
    object_id: str,
    dimensions: tuple[float, float, float],
    xyz: tuple[float, float, float],
    yaw: float = 0.0,
) -> CollisionObject:
    collision = CollisionObject()
    collision.header.frame_id = PLANNING_FRAME
    collision.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(dimensions)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    collision.primitives = [primitive]
    collision.primitive_poses = [pose]
    collision.operation = CollisionObject.ADD
    return collision


def cylinder_object(
    object_id: str,
    radius: float,
    height: float,
    xyz: tuple[float, float, float],
) -> CollisionObject:
    collision = CollisionObject()
    collision.header.frame_id = PLANNING_FRAME
    collision.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [height, radius]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    collision.primitives = [primitive]
    collision.primitive_poses = [pose]
    collision.operation = CollisionObject.ADD
    return collision


def attached_target_proxy(
    operation: int, relative_pose: dict | None = None
) -> AttachedCollisionObject:
    """Return the Gazebo cylinder proxy expressed in the grasp-centre link."""

    attached = AttachedCollisionObject()
    attached.link_name = "p7_grasp_center_link"
    attached.touch_links = [
        "p7_grasp_center_link",
        *FINGER_LINKS,
    ]
    attached.object.header.frame_id = attached.link_name
    attached.object.id = "p7_target_contact_proxy"
    attached.object.operation = operation
    if operation == CollisionObject.ADD:
        if relative_pose is None:
            raise ValueError("attached target ADD needs an observed relative pose")
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.101855, 0.0340]
        pose = Pose()
        pose.position.x = relative_pose["position"]["x"]
        pose.position.y = relative_pose["position"]["y"]
        pose.position.z = relative_pose["position"]["z"]
        pose.orientation.x = relative_pose["orientation"]["x"]
        pose.orientation.y = relative_pose["orientation"]["y"]
        pose.orientation.z = relative_pose["orientation"]["z"]
        pose.orientation.w = relative_pose["orientation"]["w"]
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
    return attached


def attached_objects_for_mode(
    mode: str, relative_pose: dict | None
) -> list[AttachedCollisionObject]:
    """Return no fake REMOVE messages before the one real attachment transition."""

    if mode != "attached":
        return []
    return [attached_target_proxy(CollisionObject.ADD, relative_pose)]


def estimated_center_pose(record: dict) -> tuple[str, dict]:
    """Reuse the estimator handoff checks; do not add a CAD origin offset."""
    frame = record.get("pose_frame", "")
    generate_candidates(record, target_frame=frame)
    pose = record["pose"]
    return frame, pose_record(pose["position_m"], pose["orientation_xyzw"], frame)


def set_finger_target_policy(matrix, allow: bool):
    """Preserve existing ACM entries and update only two finger-target pairs."""
    target = "p7_target_contact_proxy"
    for name in (target, *FINGER_LINKS):
        if name not in matrix.entry_names:
            old_size = len(matrix.entry_names)
            for row in matrix.entry_values:
                row.enabled = [*row.enabled, False]
            matrix.entry_names = [*matrix.entry_names, name]
            row = AllowedCollisionEntry()
            row.enabled = [False] * (old_size + 1)
            matrix.entry_values = [*matrix.entry_values, row]
    target_index = matrix.entry_names.index(target)
    for name in FINGER_LINKS:
        finger_index = matrix.entry_names.index(name)
        matrix.entry_values[target_index].enabled[finger_index] = allow
        matrix.entry_values[finger_index].enabled[target_index] = allow
    return matrix


def read_current_acm(node):
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError("MoveIt /get_planning_scene unavailable")
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("MoveIt allowed-collision matrix unavailable")
    return future.result().scene.allowed_collision_matrix


def read_current_world_object_ids(node) -> set[str]:
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError("MoveIt /get_planning_scene unavailable")
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("MoveIt world object names unavailable")
    return {item.id for item in future.result().scene.world.collision_objects}


def main() -> None:
    arguments = parse_arguments()
    geometry_record = fixture_geometry_record(arguments.profile, arguments.mode)
    estimate_frame, estimate_pose = None, None
    if arguments.target_estimate_file:
        estimate_frame, estimate_pose = estimated_center_pose(
            json.loads(arguments.target_estimate_file.read_text(encoding="utf-8"))
        )
        for item in geometry_record["world_collision_objects"]:
            if item["id"] == "p7_target_contact_proxy":
                item["position_m"] = [estimate_pose["position"][a] for a in "xyz"]
                item["orientation_xyzw"] = [estimate_pose["orientation"][a] for a in "xyzw"]
                item["frame_id"] = estimate_frame
        geometry_record["target_pose_source"] = str(arguments.target_estimate_file)
    if arguments.dry_run:
        record = {
            "capture_schema": "p7_planning_scene_dry_run_v1",
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "applied": False,
            "geometry": geometry_record,
            "claim_boundary": "dry-run geometry inspection; no ROS or MoveIt call was made",
        }
        print(json.dumps(record, sort_keys=True))
        return
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    rclpy.init()
    node = rclpy.create_node("p7_apply_fixture_scene")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError("MoveIt /apply_planning_scene service unavailable")

    # The target cylinder includes the model pose plus the collision-local AABB
    # centre.  It is intentionally conservative and removed/allowed only by a
    # later, explicit attachment transition.
    static_objects, target = world_objects_for_profile(arguments.profile)
    if estimate_pose:
        target.header.frame_id = estimate_frame
        pose = target.primitive_poses[0]
        for axis in "xyz":
            setattr(pose.position, axis, estimate_pose["position"][axis])
        for axis in "xyzw":
            setattr(pose.orientation, axis, estimate_pose["orientation"][axis])
    world_object_ids_before_apply: list[str] = []
    world_target_remove_requested = False
    if arguments.mode in ("full", "approach"):
        objects = [*static_objects, target]
    else:
        # The world target may have been absent from MoveIt's diff scene even
        # though Gazebo still owns the real object.  Remove it only when the
        # planning scene currently reports the object id; an absent REMOVE makes
        # ApplyPlanningScene fail on this Jazzy stack.
        world_object_ids_before_apply = sorted(read_current_world_object_ids(node))
        objects = [*static_objects]
        if target.id in world_object_ids_before_apply:
            target.operation = CollisionObject.REMOVE
            objects.append(target)
            world_target_remove_requested = True
    attached_relative_pose = None
    source_identity = None
    if arguments.mode == "attached":
        grasp_record = json.loads(arguments.grasp_pose_file.read_text(encoding="utf-8"))
        if estimate_pose:
            if grasp_record.get("parent_frame") != estimate_frame:
                raise ValueError("grasp TF and estimated target must share an explicit frame")
            model_pose = estimate_pose
            local_center = (0.0, 0.0, 0.0)
            pose_source = arguments.target_estimate_file
        else:
            model_record = json.loads(arguments.model_pose_file.read_text(encoding="utf-8"))
            model_pose = model_record.get("pose", model_record)
            local_center = tuple(arguments.target_local_center_m)
            pose_source = arguments.model_pose_file
        attached_relative_pose = target_pose_relative_to_grasp(
            model_pose, grasp_record.get("pose", grasp_record), local_center)
        source_identity = {
            "model_pose_file": {
                "path": str(pose_source),
                "is_estimated_cad_center": estimate_pose is not None,
            },
            "grasp_pose_file": {
                "path": str(arguments.grasp_pose_file),
                "sha256": hashlib.sha256(arguments.grasp_pose_file.read_bytes()).hexdigest(),
            },
        }

    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = set_finger_target_policy(
        read_current_acm(node), arguments.mode != "full")
    scene.world.collision_objects = objects
    scene.robot_state.is_diff = True
    # A fresh episode has no MoveIt attached body in ``full`` or ``approach``.
    # Sending REMOVE for an absent body is harmless to motion but emits a
    # misleading RobotState error, so only the real attachment transition
    # carries an AttachedCollisionObject message.
    scene.robot_state.attached_collision_objects = attached_objects_for_mode(
        arguments.mode, attached_relative_pose
    )
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    deadline = time.monotonic() + 12.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    response = future.result() if future.done() else None
    if response is None or not response.success:
        raise RuntimeError("P7 planning-scene application failed")
    if arguments.mode == "full":
        detail = "objects=p7_ground,p7_occluder,p7_target_contact_proxy attached=none"
    elif arguments.mode == "approach":
        detail = "objects=p7_ground,p7_occluder,p7_target_contact_proxy allowed=two_fingers_only attached=none"
    else:
        detail = (
            "objects=p7_ground,p7_occluder "
            f"removed=p7_target_contact_proxy:{world_target_remove_requested} "
            "attached=p7_target_contact_proxy cylinder_radius_m=0.034 "
            "cylinder_height_m=0.101855"
        )
    record = {
        "capture_schema": "p7_planning_scene_transition_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "success": True,
        "profile": arguments.profile,
        "source_world": FIXTURE_PROFILES[arguments.profile]["source_world"],
        "mode": arguments.mode,
        "planning_frame": PLANNING_FRAME,
        "geometry": geometry_record,
        "detail": detail,
        "attached_relative_pose": attached_relative_pose,
        "world_object_ids_before_apply": world_object_ids_before_apply,
        "world_target_remove_requested": world_target_remove_requested,
        "source_identity": source_identity,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
    }
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
