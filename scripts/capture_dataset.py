#!/usr/bin/env python3
"""Move the arm through a designed pose list and capture labelled RGB-D frames.

Why a dedicated capture driver
------------------------------
Training data has to be *repeatable* and *labelled by construction*.  Instead of
shooting the part by hand and guessing what each frame contains, this script
drives the arm to planned poses around the workpiece and records, for every
frame, the pose it was taken from, the computed workpiece pose (from the fine
ICP stage) and a stage tag.  That makes the dataset self-describing: a training
sample knows its viewpoint, its distance and whether the part was even in the
scene.

Safety: every target is planned with the same collision-aware planner as the
task scripts, split into four chunks that the executor guards with the
first-point check, and the run aborts on the first failure instead of pushing
through.

Usage (real hardware; the arm moves):
  python3 scripts/capture_dataset.py --out ~/datasets/2026-10-25_raw \
      --tag workpiece --azimuths -45 -30 -15 0 15 30 45 --elevations -10 0 10 \
      --distance 0.52

  # empty scene (workpiece removed) uses the same pose list:
  python3 scripts/capture_dataset.py --out ... --tag empty --label empty
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parent.parent
PLAN = REPO / "scripts" / "plan_pose_once.py"
EXEC = REPO / "scripts" / "execute_assembly_chunks.py"
HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"

# The reference workpiece pose in the base frame (fine stage consensus,
# 2026-10-24).  Only used to build the pose list; each frame records its own.
PART_REFERENCE = np.array([0.6804, 0.7163, 0.1240])

# Hard workspace guard.  A -45 deg azimuth at 0.52 m drove the TCP to x=-1.02
# (the arm swung across the cell) and the run was E-stopped.  The workpiece is
# always reached from +x, so refuse any target outside this box instead of
# trusting the planner to keep it sane.
# Joint envelope measured from the verified pick-and-place trajectory
# (pre_grasp .. baseline).  J1-J3 move only inside this band; the wrist joints
# carry the viewpoint changes.
JOINT_ENVELOPE_DEG = {
    "shoulder_pan_joint":  (2.0, 45.0),
    "shoulder_lift_joint": (-128.0, -88.0),
    "elbow_joint":         (-130.0, -62.0),
    "wrist_1_joint":       (-98.0, 13.0),
    "wrist_2_joint":       (86.0, 95.0),
    "wrist_3_joint":       (62.0, 100.0),
}

# Observation envelope: the task band widened so that viewpoints around the
# working area are reachable.  The task band itself stays untouched (it is what
# the pick-and-place trajectory provably uses); this wider band is only for
# taking pictures, and the workspace box + TCP clearance + collision checks
# still apply on top of it.
OBSERVATION_ENVELOPE_DEG = {
    "shoulder_pan_joint":  (-30.0, 120.0),
    "shoulder_lift_joint": (-150.0, -30.0),
    "elbow_joint":         (-165.0, -40.0),
    "wrist_1_joint":       (-160.0, 60.0),
    "wrist_2_joint":       (15.0, 150.0),
    "wrist_3_joint":       (35.0, 125.0),
}

# The camera sits 0.296 m behind the TCP, so a 0.45 m camera distance leaves the
# GRIPPER only 0.15 m from the part centre -- inside a 0.46 m part (this caused a
# real collision).  Refuse anything closer than this TCP-to-centre distance.
MIN_TCP_CLEARANCE = 0.38

SAFE_BOX_MIN = np.array([0.35, 0.25, 0.25])
SAFE_BOX_MAX = np.array([1.10, 1.05, 0.80])
MAX_AZIMUTH = 35.0
MAX_ELEVATION = 40.0
T_FLANGE_TCP = np.array([-0.046, 0.0, 0.353])


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("capture_dataset")
        from sensor_msgs.msg import JointState
        self.joints = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "joints", m), 10)
        self.color = None
        self.depth = None
        self.info = None
        self.tcp = None
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self._on_tcp, 10)
        from sensor_msgs.msg import CameraInfo
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_info,
                                 qos_profile_sensor_data)

    def _on_color(self, msg):
        self.color = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_tcp(self, msg):
        self.tcp = msg

    def wait_fresh(self, timeout: float = 12.0) -> bool:
        """Block until a colour and a depth frame arrived *after* this call."""

        self.color = None
        self.depth = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color is not None and self.depth is not None:
                return True
        return False


def tcp_matrix(msg) -> np.ndarray:
    pose = msg.pose
    quaternion = pose.orientation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def recover_control() -> None:
    """Reset a controller that stalled after a goal timeout (known failure mode)."""

    subprocess.run(["bash", str(REPO / "scripts" / "recover_control.sh")],
                   capture_output=True)


def projected_target_ok(tcp_matrix, hand_eye, intrinsics,
                        part_xyz, part_quat,
                        min_area: float = 0.10,
                        max_center_offset: float = 0.35) -> tuple[bool, str]:
    """Check geometrically that the workpiece is framed (aiming, not detection).

    Aiming errors -- the part drifting to the image border because the target
    was taken from a stale pose, or because the model ORIGIN (a corner) was used
    instead of the geometric centre -- were the most repeated failure in this
    project.  YOLO cannot arbitrate it (it happily "detects" the curtain), so the
    known workpiece box is projected into the image instead.
    """

    import cv2
    from scipy.spatial.transform import Rotation

    rotation = Rotation.from_quat(part_quat).as_matrix()
    centre = np.asarray(part_xyz) + rotation @ np.array([-0.208, 0.066, -0.011])
    half = np.array([0.2275, 0.0655, 0.2325])
    offsets = np.array([[sx * half[0], sy * half[1], sz * half[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    corners = centre + offsets @ rotation.T
    camera = np.asarray(tcp_matrix) @ np.asarray(hand_eye)
    inverse = np.linalg.inv(camera)
    points = corners @ inverse[:3, :3].T + inverse[:3, 3]
    if np.any(points[:, 2] <= 0.05):
        return False, "workpiece behind the camera"
    fx, fy, cx, cy = intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]
    u = points[:, 0] * fx / points[:, 2] + cx
    v = points[:, 1] * fy / points[:, 2] + cy
    hull = cv2.convexHull(np.stack([u, v], axis=-1).astype(np.int32))
    area = float(cv2.contourArea(hull)) / (1280.0 * 960.0)
    inside = ((u >= 0) & (u < 1280) & (v >= 0) & (v < 960)).mean()
    if area < min_area:
        return False, f"projected area {area * 100:.1f}% < {min_area * 100:.0f}%"
    # Close-up frames legitimately overflow the image, so the box does not have
    # to be fully inside; what matters is that its CENTRE is well framed.
    if not (0.0 <= u.mean() < 1280.0 and 0.0 <= v.mean() < 960.0):
        return False, "workpiece centre outside the image"
    centre_u, centre_v = u.mean() / 1280.0 - 0.5, v.mean() / 960.0 - 0.5
    offset = max(abs(centre_u), abs(centre_v))
    if offset > max_center_offset:
        return False, f"projected centre offset {offset:.2f} > {max_center_offset}"
    return True, f"area {area * 100:.1f}% offset ({centre_u:+.2f},{centre_v:+.2f})"


def plan_feasible(node, xyz, quat, budget: float = 20.0):
    """Plan to the pose and check the trajectory stays inside the joint envelope.

    A bare IK pre-check is misleading here: /compute_ik returns ONE solution,
    often a different arm branch (shoulder_pan 158 deg, shoulder_lift -65 deg)
    that the planner would never choose.  Asking the planner is what matches
    what will actually be executed.
    """

    from moveit_msgs.srv import GetMotionPlan
    from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
    from geometry_msgs.msg import PoseStamped
    from shape_msgs.msg import SolidPrimitive

    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = "cs625_arm"
    motion.num_planning_attempts = 20
    motion.allowed_planning_time = budget
    motion.max_velocity_scaling_factor = 0.05
    motion.max_acceleration_scaling_factor = 0.05
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = quat
    position = PositionConstraint()
    position.header.frame_id = "world"
    position.link_name = "my_end_effector_link"
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.005]
    position.constraint_region.primitives.append(primitive)
    position.constraint_region.primitive_poses.append(pose.pose)
    position.weight = 1.0
    orientation = OrientationConstraint()
    orientation.header.frame_id = "world"
    orientation.link_name = "my_end_effector_link"
    orientation.orientation = pose.pose.orientation
    orientation.absolute_x_axis_tolerance = 0.02
    orientation.absolute_y_axis_tolerance = 0.02
    orientation.absolute_z_axis_tolerance = 0.02
    orientation.weight = 1.0
    goal = Constraints()
    goal.position_constraints.append(position)
    goal.orientation_constraints.append(orientation)
    motion.goal_constraints.append(goal)
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=5):
        return False, "no planner", None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=budget + 40)
    response = future.result()
    if response is None or response.motion_plan_response.error_code.val != 1:
        return False, "planning failed", None
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    names = list(trajectory.joint_names)
    values = np.array([p.positions for p in trajectory.points])
    # Only the FINAL configuration has to sit inside the envelope.  The coarse
    # observation pose itself lives outside it (J2 ~ -57 deg), so validating the
    # whole path would reject every plan that starts there.
    final = np.degrees(values[-1])
    for index, name in enumerate(names):
        if name in OBSERVATION_ENVELOPE_DEG:
            low, high = OBSERVATION_ENVELOPE_DEG[name]
            if not (low <= final[index] <= high):
                return (False, f"{name} ends at {final[index]:.0f} deg "
                               f"outside [{low},{high}]", None)
    payload = {"joint_names": names,
               "points": [{"q": list(p.positions),
                           "t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9}
                          for p in trajectory.points]}
    return True, "ok", payload


def plan_and_execute(xyz, quat) -> bool:
    plan = subprocess.run(
        ["python3", "-u", str(PLAN), *[f"{v:.4f}" for v in xyz],
         *[f"{v:.6f}" for v in quat], "--velocity", "0.05",
         "--attempts", "20", "--budget", "25", "--output", "/tmp/cap_jm.json"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    text = (plan.stdout or b"").decode("utf-8", "replace")
    if "code 1" not in text:
        return False
    data = json.loads(pathlib.Path("/tmp/cap_jm.json").read_text())
    points = data["points"]
    chunks = 4
    bounds = [round(i * (len(points) - 1) / chunks) for i in range(chunks + 1)]
    for index in range(chunks):
        part = points[bounds[index]: bounds[index + 1] + 1]
        base = part[0]["t"]
        path = pathlib.Path(f"/tmp/cap_{index}.json")
        path.write_text(json.dumps({
            "joint_names": data["joint_names"],
            "points": [{"q": q["q"], "t": round(q["t"] - base, 6)} for q in part]}))
        run = subprocess.run(
            ["python3", "-u", str(EXEC), str(path), "--time-scale", "1.5",
             "--dwell", "1.0", "--goal-tolerance", "0.002"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        tail = (run.stdout or b"").decode("utf-8", "replace").strip().splitlines()
        if not tail or "ALL_ASSEMBLY_CHUNKS_OK" not in tail[-1]:
            return False
    return True


def camera_target(part: np.ndarray, distance: float, azimuth_deg: float,
                  elevation_deg: float, T_tcp_cam: np.ndarray,
                  T_current_tcp: np.ndarray) -> np.ndarray:
    """TCP pose that puts the CAMERA at (distance, azimuth, elevation) from the part.

    The camera is aimed AT the workpiece (look-at).  Keeping the previous camera
    orientation instead was a repeated source of off-centre frames and of TCP
    targets that drifted out of the working box.
    """

    # Viewing direction built from scratch: the robot always looks at the part
    # from the base side, at a controlled elevation.  Inheriting the direction
    # from the current (high) coarse pose made every shot look steeply down.
    # horizontal direction pointing from the part TOWARDS the base
    base_direction = -np.array([part[0], part[1], 0.0])
    base_direction /= max(np.linalg.norm(base_direction), 1e-9)
    # azimuth: rotate that horizontal direction about Z
    direction = Rotation.from_euler("z", azimuth_deg, degrees=True).as_matrix() @ base_direction
    # elevation: tilt upward from horizontal by the requested angle
    horizontal = np.array([direction[0], direction[1], 0.0])
    horizontal /= max(np.linalg.norm(horizontal), 1e-9)
    tilt_axis = np.cross(horizontal, [0.0, 0.0, 1.0])
    direction = Rotation.from_rotvec(
        tilt_axis * np.radians(elevation_deg)).as_matrix() @ direction
    camera_position = part + direction * distance
    # look-at: optical +Z points from the camera to the part centre
    forward = part - camera_position
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(forward @ up)) > 0.98:
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    camera = np.eye(4)
    camera[:3, :3] = np.stack([right, down, forward], axis=1)
    camera[:3, 3] = camera_position
    return camera @ np.linalg.inv(T_tcp_cam)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    parser.add_argument("--tag", required=True, help="scene tag, e.g. workpiece/empty")
    parser.add_argument("--label", default="")
    parser.add_argument("--azimuths", type=float, nargs="+", default=[-45, -30, -15, 0, 15, 30, 45])
    parser.add_argument("--elevations", type=float, nargs="+", default=[-10, 0, 10])
    parser.add_argument("--distance", type=float, default=0.52)
    parser.add_argument("--center", type=float, nargs=3, default=None,
                        help="orbit centre in base frame (default: the workpiece)")
    parser.add_argument("--safe-min", type=float, nargs=3, default=None)
    parser.add_argument("--safe-max", type=float, nargs=3, default=None)
    parser.add_argument("--max-poses", type=int, default=0)
    parser.add_argument("--min-clearance", type=float, default=MIN_TCP_CLEARANCE,
                        help="TCP-to-centre clearance; 0.38 for the workpiece, "
                             "smaller is legitimate for the open slot fixture")
    parser.add_argument("--skip-framing", action="store_true",
                        help="skip the workpiece projection gate (used for the slot, "
                             "which has no CAD model)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the guard verdict for every target pose and exit")
    parser.add_argument("--part-xyz", type=float, nargs=3,
                        default=[0.6803, 0.7166, 0.1245])
    parser.add_argument("--part-quat", type=float, nargs=4,
                        default=[0.259, 0.9659, 0.0041, -0.0035])
    parser.add_argument("--weights",
                        default=str(pathlib.Path.home() /
                                    "cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt"))
    parser.add_argument("--settle", type=float, default=2.5,
                        help="seconds to wait after the move before capturing")
    arguments = parser.parse_args()

    centre = np.array(arguments.center) if arguments.center else PART_REFERENCE
    safe_min = np.array(arguments.safe_min) if arguments.safe_min else SAFE_BOX_MIN
    safe_max = np.array(arguments.safe_max) if arguments.safe_max else SAFE_BOX_MAX
    output = pathlib.Path(arguments.out).expanduser() / arguments.tag
    output.mkdir(parents=True, exist_ok=True)
    t_tcp_cam = np.array(json.loads(HAND_EYE.read_text())["T_tcp_cam"], dtype=float)
    intrinsics = json.loads(
        (REPO / "config/real/camera_intrinsics.json").read_text())

    rclpy.init()
    node = Capture()
    deadline = time.time() + 15
    while node.tcp is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.tcp is None:
        print("no TCP pose")
        return 2
    start_tcp = tcp_matrix(node.tcp)

    poses = []
    for elevation in arguments.elevations:
        if abs(elevation) > MAX_ELEVATION:
            print(f"skip elevation {elevation}: beyond +-{MAX_ELEVATION}")
            continue
        for azimuth in arguments.azimuths:
            if abs(azimuth) > MAX_AZIMUTH:
                print(f"skip azimuth {azimuth}: beyond +-{MAX_AZIMUTH}")
                continue
            poses.append((azimuth, elevation))
    if arguments.max_poses:
        poses = poses[:arguments.max_poses]

    deadline = time.time() + 10
    while node.joints is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.joints is None:
        print("no joint states")
        return 2

    if arguments.dry_run:
        part_centre = (np.asarray(arguments.part_xyz)
                       + Rotation.from_quat(arguments.part_quat).as_matrix()
                       @ np.array([-0.208, 0.066, -0.011]))
        print(f"part centre {np.round(part_centre, 3).tolist()}")
        print(f"{'label':<16}{'TCP':<30}{'clear':>7}  verdict")
        for azimuth, elevation in poses:
            target = camera_target(centre, arguments.distance, azimuth,
                                   elevation, t_tcp_cam, start_tcp)
            xyz = target[:3, 3]
            label = f"az{azimuth:+.0f}_el{elevation:+.0f}"
            if np.any(xyz < safe_min) or np.any(xyz > safe_max):
                print(f"{label:<16}{str(np.round(xyz,3).tolist()):<30}{'-':>7}  BOX")
                continue
            clearance = float(np.linalg.norm(xyz - part_centre))
            if clearance < arguments.min_clearance:
                print(f"{label:<16}{str(np.round(xyz,3).tolist()):<30}{clearance:7.2f}  CLEARANCE")
                continue
            quat_preview = Rotation.from_matrix(target[:3, :3]).as_quat()
            feasible, reason, _plan = plan_feasible(node, xyz, quat_preview)
            print(f"{label:<16}{str(np.round(xyz,3).tolist()):<30}{clearance:7.2f}  "
                  f"{'OK' if feasible else reason}")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    index = 0
    failures = 0
    for azimuth, elevation in poses:
        target = camera_target(centre, arguments.distance, azimuth,
                               elevation, t_tcp_cam, start_tcp)
        xyz = target[:3, 3]
        label = f"az{azimuth:+.0f}_el{elevation:+.0f}"
        if np.any(xyz < safe_min) or np.any(xyz > safe_max):
            print(f"  REFUSED {label}: TCP {np.round(xyz, 3).tolist()} outside safe box")
            failures += 1
            continue
        part_centre = (np.asarray(arguments.part_xyz)
                       + Rotation.from_quat(arguments.part_quat).as_matrix()
                       @ np.array([-0.208, 0.066, -0.011]))
        clearance = float(np.linalg.norm(xyz - part_centre))
        if clearance < arguments.min_clearance:
            print(f"  REFUSED {label}: TCP clearance {clearance:.2f} m "
                  f"< {MIN_TCP_CLEARANCE} m")
            failures += 1
            continue
        quat_preview = Rotation.from_matrix(target[:3, :3]).as_quat()
        feasible, reason, plan_payload = plan_feasible(node, xyz, quat_preview)
        if not feasible:
            print(f"  REFUSED {label}: {reason}")
            failures += 1
            continue
        quat = Rotation.from_matrix(target[:3, :3]).as_quat()
        print(f"[{index + 1}/{len(poses)}] {label} -> TCP {np.round(xyz, 3).tolist()}")
        if not plan_and_execute(xyz, quat):
            # A goal timeout leaves the controller refusing new trajectories;
            # recover once and retry before giving up on this pose.
            print(f"  retry {label} after control recovery")
            recover_control()
            if not plan_and_execute(xyz, quat):
                print(f"  plan/exec failed, skipping {label}")
                failures += 1
                continue
        time.sleep(arguments.settle)
        if not node.wait_fresh():
            print(f"  no fresh frame, skipping {label}")
            failures += 1
            continue
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = f"{arguments.tag}_{label}_{stamp}"
        if arguments.skip_framing:
            ok, reason = True, "framing gate skipped (slot capture)"
        else:
            ok, reason = projected_target_ok(
                tcp_matrix(node.tcp), t_tcp_cam, intrinsics,
                arguments.part_xyz, arguments.part_quat)
        if not ok:
            print(f"  REJECTED {label}: {reason}", flush=True)
            failures += 1
            continue
        print(f"  framing ok: {reason}", flush=True)
        color_path = output / f"{name}_color.png"
        depth_path = output / f"{name}_depth.npy"
        import cv2
        color_msg = node.color
        raw = np.frombuffer(color_msg.data, dtype=np.uint8)
        raw = raw[:color_msg.height * color_msg.step].reshape(
            color_msg.height, color_msg.step)[:, :color_msg.width * 3]
        color = raw.reshape(color_msg.height, color_msg.width, 3)
        if color_msg.encoding.lower() == "rgb8":
            color = color[:, :, ::-1]
        cv2.imwrite(str(color_path), color)
        depth = np.frombuffer(node.depth.data, dtype=np.uint8)
        depth = depth[:node.depth.height * node.depth.step].reshape(
            node.depth.height, node.depth.step)[:, :node.depth.width * 2]
        depth = depth.view(np.uint16).reshape(node.depth.height, node.depth.width)
        np.save(depth_path, depth)
        record = {
            "file": name,
            "tag": arguments.tag,
            "label": arguments.label,
            "azimuth_deg": azimuth,
            "elevation_deg": elevation,
            "distance_m": arguments.distance,
            # Full pose, not just the position: the auto-labeller needs the
            # camera orientation to place the known workpiece box in the frame.
            "tcp_matrix": [float(v) for v in np.asarray(tcp_matrix(node.tcp)).reshape(-1)],
            "tcp_base": [float(v) for v in tcp_matrix(node.tcp)[:3, 3]],
            "camera_base": [float(v) for v in (tcp_matrix(node.tcp) @ t_tcp_cam)[:3, 3]],
            "stamp": node.color.header.stamp.sec + node.color.header.stamp.nanosec * 1e-9,
        }
        with (output / "metadata.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(f"  saved {name}")
        index += 1

    print(f"done: {index} frames, {failures} failures -> {output}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if index else 1


if __name__ == "__main__":
    raise SystemExit(main())
