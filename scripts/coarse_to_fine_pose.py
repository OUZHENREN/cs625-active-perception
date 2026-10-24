#!/usr/bin/env python3
"""Coarse -> fine pose pipeline (mirrors the RVS two-stage mechanism).

Stage 1 (coarse, no motion):
    detect the workpiece, take the mask centre and the *masked* depth median,
    and express that point in the robot base frame via TF.

Stage 2 (move):
    place the CAMERA at ``--standoff`` from the target along the current optical
    axis (a fine view must be closer than the current one; the flag refuses to
    back off unless told to), convert that camera pose back to a TCP pose with
    ``inv(T_tcp_cam)`` and drive there with the existing chunked executor
    (safety gates and re-planning live there, not here).  ``--standoff`` used to
    be subtracted from the TCP, which on this cell meant the camera landed at
    standoff + 296 mm and the "fine" view was twice as far as RVS's coarse one.

Stage 3 (fine):
    re-run ICP against the CAD template and report fitness / inlier RMSE.

The node deliberately shells out to the already-tested helper scripts instead of
duplicating planning code, so a failure in any stage is easy to isolate.

Usage:
  python3 scripts/coarse_to_fine_pose.py \
      --weights .../box_seg640/weights/best.pt \
      --template /tmp/workpiece_template.stl \
      --standoff 0.45 --confirm
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

sys.stdout.reconfigure(line_buffering=True)

REPO = Path(__file__).resolve().parent.parent
ICP_SCRIPT = REPO / "scripts" / "icp_pose_once.py"
PLAN_SCRIPT = REPO / "scripts" / "plan_pose_once.py"
EXEC_SCRIPT = REPO / "scripts" / "execute_assembly_chunks.py"


class Coarse(Node):
    def __init__(self, weights: str, conf: float, min_range: float,
                 max_range: float) -> None:
        super().__init__("coarse_stage")
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.conf = conf
        self.min_range = min_range
        self.max_range = max_range
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.done = False
        self.result = None
        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data)

    def latest_tcp(self) -> np.ndarray:
        """Current TCP pose in base frame from the broadcaster topic."""
        import time as _time
        latest = {}
        sub = self.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose",
            lambda m: latest.__setitem__("m", m), 10)
        deadline = _time.time() + 3.0
        while "m" not in latest and _time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        msg = latest.get("m")
        if msg is None:
            raise RuntimeError("no TCP pose")
        p, q = msg.pose.position, msg.pose.orientation
        transform = np.eye(4)
        transform[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        transform[:3, 3] = [p.x, p.y, p.z]
        return transform

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_color(self, msg: Image) -> None:
        if not self.done:
            self.color = msg
            self.run()

    def on_depth(self, msg: Image) -> None:
        if not self.done:
            self.depth = msg
            self.run()

    def run(self) -> None:
        if self.color is None or self.depth is None or self.info is None:
            return
        self.done = True
        import cv2

        color = self.bridge.imgmsg_to_cv2(self.color, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(self.depth)
        depth_m = (depth.astype(np.float32) / 1000.0 if depth.dtype == np.uint16
                   else depth.astype(np.float32))
        height, width = depth_m.shape[:2]

        result = self.model.predict(color, imgsz=640, verbose=False,
                                    conf=self.conf, device="cpu")
        if not result or result[0].masks is None:
            self.get_logger().warn("coarse: no mask")
            return
        mask = result[0].masks.data[0].cpu().numpy()
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(mask.astype(np.float32), (width, height),
                              interpolation=cv2.INTER_NEAREST)
        mask = mask > 0.5
        ys, xs = np.nonzero(mask)
        zs = depth_m[ys, xs]
        keep = np.isfinite(zs) & (zs > self.min_range) & (zs < self.max_range)
        if keep.sum() < 500:
            self.get_logger().warn(f"coarse: too few depth points {int(keep.sum())}")
            return
        # Nearest *significant* depth peak, not the median: the mask can include
        # background, and a median would land between the part and the wall.
        valid_z = zs[keep]
        histogram, edges = np.histogram(valid_z, bins=60)
        centres = 0.5 * (edges[:-1] + edges[1:])
        significant = histogram >= max(0.05 * histogram.max(), 1)
        candidates = centres[significant]
        z_median = float(candidates[0]) if len(candidates) else float(np.median(valid_z))
        u, v = float(xs.mean()), float(ys.mean())
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        point_cam = np.array([(u - cx) * z_median / fx,
                              (v - cy) * z_median / fy, z_median])
        self.result = {
            "point_cam": point_cam,
            "pixels": int(keep.sum()),
            "area_ratio": float(mask.mean()),
            "frame": self.color.header.frame_id,
        }
        self.get_logger().info(
            f"coarse: u={u:.0f} v={v:.0f} z={z_median:.2f}m area={mask.mean():.2f} "
            f"pts={int(keep.sum())}")


def camera_to_base(node: Node, frame: str) -> np.ndarray:
    import tf2_ros
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    deadline = time.time() + 15.0
    last_error: Exception | None = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = buffer.lookup_transform("base_link", frame, rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=0.5))
            break
        except Exception as exc:  # TF is invalid until the first message lands
            last_error = exc
            continue
    raise RuntimeError(f"no TF base_link <- {frame}; last error: {last_error}")
    transform = np.eye(4)
    q = tf.transform.rotation
    transform[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t = tf.transform.translation
    transform[:3, 3] = [t.x, t.y, t.z]
    return transform


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--template", required=True)
    parser.add_argument("--standoff", type=float, default=0.45,
                        help="desired CAMERA-to-target distance for the fine stage; "
                             "0.40-0.45 m frames the part at 60-80%% (RVS reference: "
                             "S2 coarse 0.65 m, S3 grasp 0.30 m)")
    parser.add_argument("--allow-back-off", action="store_true",
                        help="allow a standoff that is larger than the current "
                             "camera-target distance (the camera would retreat)")
    parser.add_argument("--max-range", type=float, default=4.5)
    parser.add_argument("--min-range", type=float, default=0.3)
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--confirm", action="store_true",
                        help="actually move the robot for the fine stage")
    parser.add_argument("--plan-only", action="store_true",
                        help="plan the fine move but do not execute it")
    parser.add_argument("--transit-velocity", type=float, default=0.05,
                        help="velocity/acceleration scaling for the move; 0.05 is the "
                             "normal speed on this cell (owner-confirmed), assembly "
                             "segments use 0.005-0.01")
    parser.add_argument("--plan-retries", type=int, default=3)
    parser.add_argument("--time-scale", type=float, default=1.5,
                        help="executor time scaling for the plan")
    args = parser.parse_args()

    rclpy.init()
    node = Coarse(args.weights, args.conf, args.min_range, args.max_range)
    deadline = time.time() + 40
    while rclpy.ok() and not node.result and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.result:
        print("COARSE_FAILED")
        return 2

    # Preferred path: TCP pose from the topic times a fixed calibrated offset.
    # The cell publishes several unconnected TF trees, so looking camera frames
    # up in TF is unreliable; the offset is rigid and only needs one calibration.
    calib_path = Path(__file__).resolve().parent.parent / "config/handeye/tcp_to_camera.json"
    if calib_path.exists():
        import json as _json
        calib = _json.loads(calib_path.read_text())
        T_tcp_cam = np.array(calib["T_tcp_cam"], dtype=float)
        T_base_cam = node.latest_tcp() @ T_tcp_cam
        print("camera pose from TCP x calibrated offset")
    else:
        T_base_cam = camera_to_base(node, node.result["frame"])
    target_base = T_base_cam[:3, :3] @ node.result["point_cam"] + T_base_cam[:3, 3]
    print("target in base:", np.round(target_base, 3).tolist())

    cam_pos = T_base_cam[:3, 3]
    axis = target_base - cam_pos
    distance = float(np.linalg.norm(axis))
    axis /= max(distance, 1e-6)

    # The standoff is a CAMERA-to-target distance, not a TCP-to-target one: the
    # camera sits t_tcp_cam behind the TCP (296 mm on this cell), so subtracting
    # the standoff from the TCP either backs the camera off or - worse - leaves
    # it further than intended.  Solve for the camera pose first, then convert:
    #     T_base_tcp = T_base_cam @ inv(T_tcp_cam)
    # A fine view must be CLOSER than the current one; RVS's own reference is
    # S2 coarse = 0.65 m, S3 grasp = 0.30 m, so 0.40-0.45 m puts the part at
    # 60-80 % of the frame (area scales as 1/d^2).
    if args.standoff >= distance and not args.allow_back_off:
        print(f"REFUSED: camera is already {distance:.2f} m from the target and the "
              f"standoff is {args.standoff:.2f} m, so this would move it AWAY. "
              "Pass --allow-back-off if that is really what you want.")
        return 5
    desired_cam = np.eye(4)
    desired_cam[:3, :3] = T_base_cam[:3, :3]      # keep the current optical axis
    desired_cam[:3, 3] = target_base - axis * args.standoff
    desired_tcp = desired_cam @ np.linalg.inv(T_tcp_cam)
    fine_tcp = desired_tcp[:3, 3]
    fine_quat = R.from_matrix(desired_tcp[:3, :3]).as_quat()
    predicted_area = node.result["area_ratio"] * (distance / args.standoff) ** 2
    print(f"current camera-target {distance:.2f} m (part {node.result['area_ratio'] * 100:.0f}% of frame)")
    print(f"fine camera-target {args.standoff:.2f} m (predicted {predicted_area * 100:.0f}%)"
          f" -> fine TCP {np.round(fine_tcp, 3).tolist()}"
          f" quat {np.round(fine_quat, 4).tolist()}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if not args.confirm and not args.plan_only:
        print("dry run: pass --confirm to move (or --plan-only to check the plan)")
        return 0

    print("== planning the fine observation move ==")
    # Observation goal: 5 mm / 0.02 rad is enough because the next stage re-derives
    # the target from the new frame.  Assembly-grade tolerances (0.5 mm / 0.001 rad)
    # made this ordinary move fail with code 99999.  5 % is this cell's NORMAL speed,
    # so the 20-50 s durations below are expected, not a slow-down; OMPL is still
    # stochastic, hence the retries.
    text = "plan not attempted"
    for attempt in range(args.plan_retries):
        plan = subprocess.run(
            ["python3", "-u", str(PLAN_SCRIPT), *[f"{v:.4f}" for v in fine_tcp],
             *[f"{v:.6f}" for v in fine_quat],
             "--velocity", str(args.transit_velocity),
             "--position-tolerance", "0.005",
             "--orientation-tolerance", "0.02",
             "--attempts", "20", "--budget", "25",
             "--output", "/tmp/jm.json"],
            capture_output=True)
        text = (plan.stdout or b"").decode("utf-8", "replace")
        line = text.strip().splitlines()[-1] if text.strip() else "plan failed"
        print(f"attempt {attempt + 1}/{args.plan_retries}: {line}")
        if "code 1" in text:
            break
    else:
        print("FINE_PLAN_FAILED")
        return 3

    if args.plan_only:
        print("plan-only: not executing")
        return 0

    import json
    data = json.loads(Path("/tmp/jm.json").read_text())
    points = data["points"]
    chunks = 4
    bounds = [round(i * (len(points) - 1) / chunks) for i in range(chunks + 1)]
    for index in range(chunks):
        part = points[bounds[index]: bounds[index + 1] + 1]
        base = part[0]["t"]
        Path(f"/tmp/c2f_{index}.json").write_text(json.dumps({
            "joint_names": data["joint_names"],
            "points": [{"q": p["q"], "t": round(p["t"] - base, 6)} for p in part],
        }))
        run = subprocess.run(["python3", "-u", str(EXEC_SCRIPT),
                              f"/tmp/c2f_{index}.json", "--time-scale", str(args.time_scale),
                              "--dwell", "1.0", "--goal-tolerance", "0.002"],
                             capture_output=True)
        print(f"chunk{index}:", (run.stdout or b"").decode("utf-8", "replace").strip().splitlines()[-1])

    print("== fine stage: ICP ==")
    icp = subprocess.run(
        ["python3", "-u", str(ICP_SCRIPT), "--weights", args.weights,
         "--template", args.template, "--max-range", str(args.max_range)],
        capture_output=True)
    for line in (icp.stdout or b"").decode("utf-8", "replace").strip().splitlines()[-2:]:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
