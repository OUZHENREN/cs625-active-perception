#!/usr/bin/env python3
"""Check a planned trajectory's TOOL ENVELOPE against the measured tables.

Why this exists (2026-10-30 accident)
------------------------------------
The descent to the pre-grasp pose hit the slot table with the gripper's motor
protection seat.  A check that only looked at the TCP point could not catch it,
because the tool extends sideways and backwards from the TCP:

    flange -> TCP                     353 mm     (controller setting, verified to 0.0 mm)
    flange -> tool body front face    150 mm     (measured arms, cs625_parallel_gripper.xacro)
    vendor collision mesh in the TCP frame:
        x in [-0.004,  0.173]
        y in [-0.120,  0.120]
        z in [-0.360, -0.146]

So while the TCP travels over the workpiece table, the tool's corners can sweep
over the slot table and dip below its surface, especially mid-trajectory when the
tool axis is not vertical.

What it does
------------
For every sampled trajectory point: FK -> T_world_tcp, transform the tool box
corners, and measure the separation from each measured table box read straight
from :file:`config/cs625_real_cell_scene.yaml` (the tables are deliberately NOT
published into the MoveIt scene, so the scene cannot be used as the source).

Exit codes: 0 all clear, 1 a violation, 2 could not evaluate.

Usage
-----
    python3 scripts/check_tool_envelope.py /tmp/pre_grasp.json
    python3 scripts/check_tool_envelope.py '/tmp/teach_*.json' --margin 0.03
"""

from __future__ import annotations

import argparse
import glob
import json
import pathlib
import sys
import time

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parent.parent
SCENE_CONFIG = REPO / "config" / "cs625_real_cell_scene.yaml"
TASK_CONFIG = REPO / "config" / "tasks" / "common.yaml"
TIP_LINK = "my_end_effector_link"

# Tool collision envelope in the TCP frame (metres), UNION of:
#   * the repo's authoritative SolidWorks body gripper_0920.stl (cs625_active_perception
#     .urdf.xacro tool0 collision): flange frame X +/-147.285, Y +/-42.5, Z [-7, +207] mm,
#     mounted with rpy (0 0 +90deg) -> TCP frame x [+3.5, +88.5], y +/-147.3, z [-360, -146] mm.
#     The motor protection seat is PART of this rigid body (owner, 2026-10-31).
#   * the vendor tool0_collision.STL: TCP frame x [-4, +173], y +/-120, z [-359.5, -146] mm.
TOOL_BOX_MIN = np.array([-0.004, -0.1473, -0.360])
TOOL_BOX_MAX = np.array([0.173, 0.1473, -0.146])

# Carried-workpiece envelope (2026-11-02 audit, hole 4): once the jaws hold the
# part, the real swept volume is tool + part.  Same numbers as
# scripts/check_carried_clearance.py (T_LINK_BOX = grasp TCP -> box frame;
# CARRIED_BOX_MIN/MAX = the box local bbox).
T_LINK_BOX = np.array([
    [-1.0, 0.0, 0.0, -0.009],
    [0.0, 1.0, 0.0, 0.215],
    [0.0, 0.0, -1.0, -0.411],
    [0.0, 0.0, 0.0, 1.0],
])
CARRIED_BOX_MIN = np.array([-0.038, -0.4355, -0.459])
CARRIED_BOX_MAX = np.array([0.076, 0.0055, 0.006])


def load_tables() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """name -> (box_min, box_max) in the world frame, from the measured scene."""
    scene = yaml.safe_load(SCENE_CONFIG.read_text())["cs625_real_cell_scene"]["ros__parameters"]
    tables: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name, table in (scene.get("tables") or {}).items():
        centre = np.array(table["box_centre"], dtype=float)
        size = np.array(table["box_size"], dtype=float)
        tables[name] = (centre - size / 2.0, centre + size / 2.0)
    return tables


def cross_check_config() -> str:
    """Compare the mesh box with the conservative 150 mm figure in the config."""
    task = yaml.safe_load(TASK_CONFIG.read_text())
    flange_to_tcp = float(task.get("tool_flange_to_tcp_mm", 353.0)) / 1000.0
    body_forward = float(task.get("tool_body_forward_from_flange_mm", 150.0)) / 1000.0
    front_from_tcp = -(flange_to_tcp - body_forward)
    return (f"tool front face: mesh {-TOOL_BOX_MAX[2] * 1000:.0f} mm behind TCP, "
            f"measured arms {(-front_from_tcp) * 1000:.0f} mm "
            f"(conservative = the mesh value); seat is part of gripper_0920.stl")


def tool_corners(tcp_xyz: np.ndarray, tcp_quat: np.ndarray) -> np.ndarray:
    rot = R.from_quat(tcp_quat).as_matrix()
    corners = []
    for x in (TOOL_BOX_MIN[0], TOOL_BOX_MAX[0]):
        for y in (TOOL_BOX_MIN[1], TOOL_BOX_MAX[1]):
            for z in (TOOL_BOX_MIN[2], TOOL_BOX_MAX[2]):
                corners.append(rot @ np.array([x, y, z]) + tcp_xyz)
    return np.array(corners)


def carried_corners(tcp_xyz: np.ndarray, tcp_quat: np.ndarray) -> np.ndarray:
    """Corners of the carried workpiece box at the given TCP pose."""
    world_tcp = np.eye(4)
    world_tcp[:3, :3] = R.from_quat(tcp_quat).as_matrix()
    world_tcp[:3, 3] = tcp_xyz
    world_box = world_tcp @ T_LINK_BOX
    rot, origin = world_box[:3, :3], world_box[:3, 3]
    corners = []
    for x in (CARRIED_BOX_MIN[0], CARRIED_BOX_MAX[0]):
        for y in (CARRIED_BOX_MIN[1], CARRIED_BOX_MAX[1]):
            for z in (CARRIED_BOX_MIN[2], CARRIED_BOX_MAX[2]):
                corners.append(rot @ np.array([x, y, z]) + origin)
    return np.array(corners)


def separation(corners: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    """Gap between the corner cloud's AABB and a table box; negative = overlap."""
    overlap = np.minimum(corners.max(axis=0), hi) - np.maximum(corners.min(axis=0), lo)
    if np.all(overlap > 0):
        return -float(overlap.min())
    gaps = np.maximum(lo - corners.max(axis=0), corners.min(axis=0) - hi)
    return float(gaps.max())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pattern", help="trajectory JSON (joint_names + points q/t)")
    parser.add_argument("--sample", type=int, default=3,
                        help="check every Nth point (the last point is always checked)")
    parser.add_argument("--margin", type=float, default=0.02,
                        help="required minimum clearance in metres")
    parser.add_argument("--carried", action="store_true",
                        help="also check the carried-workpiece box (for lift/transit "
                             "steps after the grasp)")
    parser.add_argument("--carried-margin", type=float, default=None,
                        help="margin applied to the carried box only; defaults to "
                             "--margin.  The lift-off starts with the part resting on "
                             "the table, so the lift uses a tiny margin (0.005) while "
                             "lateral transport keeps the full margin.")
    args = parser.parse_args()
    carried_margin = args.margin if args.carried_margin is None else args.carried_margin

    files = sorted(glob.glob(args.pattern))
    if not files:
        print("NO_TRAJECTORY")
        return 2
    tables = load_tables()
    print(f"tables: {', '.join(f'{n}' for n in tables)}")
    print(cross_check_config())
    if args.carried:
        print("carried box: included in the envelope")

    rclpy.init()
    node = rclpy.create_node("check_tool_envelope")
    try:
        fk = node.create_client(GetPositionFK, "/compute_fk")
        if not fk.wait_for_service(timeout_sec=40):
            print("NO /compute_fk")
            return 2
        worst = None
        failures = 0
        for path in files:
            data = json.loads(pathlib.Path(path).read_text())
            names, points = data["joint_names"], data["points"]
            indices = list(range(0, len(points), max(1, args.sample)))
            if indices[-1] != len(points) - 1:
                indices.append(len(points) - 1)
            for index in indices:
                # Carried checks skip the departure point: at the grasp pose the
                # part is still resting on the table BY DESIGN; what must clear
                # is the path and the goal once the lift starts.
                if args.carried and index == indices[0]:
                    continue
                request = GetPositionFK.Request()
                request.header.frame_id = "world"
                request.fk_link_names = [TIP_LINK]
                state = RobotState()
                joint_state = JointState()
                joint_state.name = names
                joint_state.position = list(points[index]["q"])
                state.joint_state = joint_state
                request.robot_state = state
                future = fk.call_async(request)
                deadline = time.time() + 5.0
                while not future.done() and time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                response = future.result()
                if response is None or response.error_code.val != 1 or not response.pose_stamped:
                    print(f"{pathlib.Path(path).name}[{index}]: FK failed")
                    failures += 1
                    continue
                pose = response.pose_stamped[0].pose
                xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
                quat = np.array([pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w])
                corners = tool_corners(xyz, quat)
                margin = args.margin
                if args.carried:
                    corners = np.vstack([corners, carried_corners(xyz, quat)])
                    margin = carried_margin
                for name, (lo, hi) in tables.items():
                    gap = separation(corners, lo, hi)
                    if gap < margin:
                        failures += 1
                        print(f"{pathlib.Path(path).name}[{index}]: "
                              f"{'INTERSECTS' if gap < 0 else 'TOO CLOSE'} {name} "
                              f"({gap * 1000:.0f} mm) TCP z={xyz[2]:.3f}")
                    if worst is None or gap < worst[0]:
                        worst = (gap, name, pathlib.Path(path).name, index, xyz[2])
        if worst is None:
            print("NOTHING CHECKED")
            return 2
        gap, name, filename, index, z = worst
        print(f"worst clearance {gap * 1000:.0f} mm to {name} "
              f"({filename}[{index}], TCP z={z:.3f})")
        if failures:
            print(f"ENVELOPE_CHECK_FAILED ({failures} violations, margin {args.margin * 1000:.0f} mm)")
            return 1
        print("ENVELOPE_CHECK_OK")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
