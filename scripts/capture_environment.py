#!/usr/bin/env python3
"""Capture environment frames (curtain, both tables, room) for training.

The workpiece capture orbits the part with the camera aimed at it.  Environment
data needs the opposite: the camera has to *look away* from the workpiece so the
frames contain the background that the segmentation model currently swallows
into the workpiece mask (measured: 26 % outliers, the wooden table).

Each pose is specified as "put the camera here, aim it at that point".  The
camera orientation is built from a look-at construction in the base frame and
converted to a TCP target through the constant hand-eye chain.

Usage (always dry-run first):
  python3 scripts/capture_environment.py --dry-run
  python3 scripts/capture_environment.py --out ~/cs625_yolo_seg/datasets/2026-10-25_raw \
      --tag environment
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
from sensor_msgs.msg import CameraInfo, Image

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parent.parent
PLAN = REPO / "scripts" / "plan_pose_once.py"
EXEC = REPO / "scripts" / "execute_assembly_chunks.py"
HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"

# (name, camera position in base, look-at point in base)
VIEWS = (
    # All environment views are taken from the workpiece side of the cell.  A
    # pose that put the camera over the slot table (x ~ 1.1, y ~ 0.0) swung the
    # forearm across that table and had to be aborted, so the slot side is off
    # limits for wide shots: x <= 0.95, y >= 0.35 keeps the arm clear.
    ("wide_curtain",   (0.78, 0.44, 0.55), (0.90, 1.40, 0.72)),
    ("wide_tables",    (0.66, 0.40, 0.62), (1.05, 0.65, 0.15)),
    ("wide_wood",      (0.58, 0.48, 0.58), (0.55, 0.95, 0.05)),
    ("wide_grasp_area", (0.72, 0.55, 0.55), (0.85, 0.80, 0.15)),
)

SAFE_BOX_MIN = np.array([0.45, 0.35, 0.35])
SAFE_BOX_MAX = np.array([0.95, 0.95, 0.78])


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("capture_environment")
        self.color = None
        self.tcp = None
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self._on_tcp, 10)

    def _on_color(self, msg):
        self.color = msg

    def _on_tcp(self, msg):
        self.tcp = msg

    def wait_fresh(self, timeout: float = 12.0) -> bool:
        self.color = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color is not None:
                return True
        return False


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Camera rotation whose +Z points from `position` to `target`."""

    forward = target - position
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(forward @ up)) > 0.98:
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.stack([right, down, forward], axis=1)   # camera x=right, y=down, z=forward
    return rotation


def plan_and_execute(xyz, quat) -> bool:
    plan = subprocess.run(
        ["python3", "-u", str(PLAN), *[f"{v:.4f}" for v in xyz],
         *[f"{v:.6f}" for v in quat], "--velocity", "0.05",
         "--attempts", "20", "--budget", "25", "--output", "/tmp/env_jm.json"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    text = (plan.stdout or b"").decode("utf-8", "replace")
    if "code 1" not in text:
        return False
    data = json.loads(pathlib.Path("/tmp/env_jm.json").read_text())
    points = data["points"]
    chunks = 6
    bounds = [round(i * (len(points) - 1) / chunks) for i in range(chunks + 1)]
    for index in range(chunks):
        part = points[bounds[index]: bounds[index + 1] + 1]
        base = part[0]["t"]
        path = pathlib.Path(f"/tmp/env_{index}.json")
        path.write_text(json.dumps({
            "joint_names": data["joint_names"],
            "points": [{"q": q["q"], "t": round(q["t"] - base, 6)} for q in part]}))
        run = subprocess.run(
            ["python3", "-u", str(EXEC), str(path), "--time-scale", "2.0",
             "--dwell", "1.0", "--goal-tolerance", "0.002"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        tail = (run.stdout or b"").decode("utf-8", "replace").strip().splitlines()
        if not tail or "ALL_ASSEMBLY_CHUNKS_OK" not in tail[-1]:
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="")
    parser.add_argument("--tag", default="environment")
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()

    hand_eye = np.array(json.loads(HAND_EYE.read_text())["T_tcp_cam"], dtype=float)
    rclpy.init()
    node = Capture()
    deadline = time.time() + 15
    while node.tcp is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.tcp is None:
        print("no TCP pose")
        return 2

    if arguments.dry_run:
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import RobotState, PositionIKRequest
        from sensor_msgs.msg import JointState
        joint = {"message": None}
        node.create_subscription(JointState, "/joint_states",
                                 lambda m: joint.__setitem__("message", m), 10)
        while joint["message"] is None and time.time() < deadline + 10:
            rclpy.spin_once(node, timeout_sec=0.2)
        client = node.create_client(GetPositionIK, "/compute_ik")
        client.wait_for_service(timeout_sec=10)
        for name, position, target in VIEWS:
            camera_rotation = look_at(np.array(position), np.array(target))
            camera = np.eye(4)
            camera[:3, :3] = camera_rotation
            camera[:3, 3] = position
            tcp = camera @ np.linalg.inv(hand_eye)
            outside = (np.any(tcp[:3, 3] < SAFE_BOX_MIN)
                       or np.any(tcp[:3, 3] > SAFE_BOX_MAX))
            request = GetPositionIK.Request()
            request.ik_request.group_name = "cs625_arm"
            request.ik_request.robot_state = RobotState(joint_state=joint["message"])
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 3
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tcp[:3, 3]
            quat = Rotation.from_matrix(tcp[:3, :3]).as_quat()
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = quat
            request.ik_request.pose_stamped = pose
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            result = future.result()
            ok = result is not None and result.error_code.val == 1
            values = np.round(tcp[:3, 3], 3).tolist()
            print(f"{name:12s} TCP {values} "
                  f"IK {'OK' if ok else 'FAIL'}{'  OUTSIDE-BOX' if outside else ''}")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    output = pathlib.Path(arguments.out).expanduser() / arguments.tag
    output.mkdir(parents=True, exist_ok=True)
    for name, position, target in VIEWS:
        camera_rotation = look_at(np.array(position), np.array(target))
        camera = np.eye(4)
        camera[:3, :3] = camera_rotation
        camera[:3, 3] = position
        tcp = camera @ np.linalg.inv(hand_eye)
        xyz = tcp[:3, 3]
        if np.any(xyz < SAFE_BOX_MIN) or np.any(xyz > SAFE_BOX_MAX):
            print(f"{name}: REFUSED outside safe box {np.round(xyz, 3).tolist()}")
            continue
        quat = Rotation.from_matrix(tcp[:3, :3]).as_quat()
        print(f"{name}: TCP {np.round(xyz, 3).tolist()}")
        if not plan_and_execute(xyz, quat):
            print(f"{name}: plan/exec failed")
            continue
        time.sleep(3.0)
        if not node.wait_fresh():
            print(f"{name}: no frame")
            continue
        import cv2
        message = node.color
        raw = np.frombuffer(message.data, dtype=np.uint8)
        raw = raw[:message.height * message.step].reshape(
            message.height, message.step)[:, :message.width * 3]
        image = raw.reshape(message.height, message.width, 3)
        if message.encoding.lower() == "rgb8":
            image = image[:, :, ::-1]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = output / f"{arguments.tag}_{name}_{stamp}_color.png"
        cv2.imwrite(str(path), image)
        record = {"file": path.name[: -len("_color.png")], "tag": arguments.tag,
                  "label": name, "tcp_matrix": [float(v) for v in tcp.reshape(-1)]}
        with (output / "metadata.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(f"  saved {path.name}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
