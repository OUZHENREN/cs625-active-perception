#!/usr/bin/env python3
"""Known-target calibration of the real camera pose chain (no robot motion).

Purpose
-------
Every base-frame target the real camera produces goes through

    base  <-  TCP pose (topic)  <-  T_tcp_cam (config/handeye/tcp_to_camera.json)
          <-  camera_color_optical_frame  <-  pixels + depth (camera intrinsics)

and any one of those links being wrong shows up as a target metres away from the
real one.  This script pins the chain down on ONE settled RGB-D frame by using
two pieces of ground truth that are already in view, so nothing has to be moved:

  1. the workpiece's own size -- its unknown pose does not matter, only the
     metric extent of its face, which is 114 x 441 x 465 mm;
  2. the table it stands on -- a horizontal plane whose height was measured
     before (the table band used by scripts/build_environment_collision.py,
     z = -0.13 .. -0.07 m).

Checks and what a failure means
-------------------------------
  depth scale   the face must measure ~0.44 m.  A 4x larger extent means the
                depth stream is publishing raw device counts (PS800-E1:
                DepthScaleUnit 0.25 mm) instead of millimetres.
  rotation      the fitted table normal must be vertical in the base frame.
                A tilted table means the camera rotation (or an axis convention
                between camera and TCP) is wrong; a hand-written extrinsic that
                disagreed with the RVS hand-eye by 135 deg showed up here as a
                48 deg table tilt.
  translation   the table must come out at the band measured before.  A constant
                offset here is a wrong TCP offset, a wrong hand-eye translation,
                or a re-mounted camera.

It also prints the workpiece centroid in the base frame, which is the number the
coarse stage of scripts/coarse_to_fine_pose.py feeds into planning.

Usage
-----
    python3 scripts/known_target_calibration.py \
        --weights ~/cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

sys.stdout.reconfigure(line_buffering=True)

ROOT = pathlib.Path(__file__).resolve().parents[1]
HAND_EYE = ROOT / "config/handeye/tcp_to_camera.json"

# Ground truth that is already in the field of view.
PART_FACE_M = 0.44          # workpiece face 441 mm (spec 114 x 441 x 465 mm)
PART_FACE_TOL_M = 0.06
TABLE_BAND_Z = (-0.13, -0.07)   # measured table surface band in the base frame
TABLE_TOL_M = 0.04
TABLE_MAX_TILT_DEG = 5.0


def transform(rotation: np.ndarray, translation) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


class Frame(Node):
    """One settled colour + depth frame plus the matching TCP pose."""

    def __init__(self) -> None:
        super().__init__("known_target_calibration")
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.tcp = None
        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self.on_tcp, 10)

    def on_color(self, msg):
        self.color = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.info = msg

    def on_tcp(self, msg):
        self.tcp = msg

    def ready(self) -> bool:
        return all(v is not None for v in (self.color, self.depth, self.info, self.tcp))


def wait_frame(node: Frame, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.ready():
            return True
    return False


def tcp_matrix(msg) -> np.ndarray:
    pose = msg.pose
    quaternion = pose.orientation
    return transform(
        Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z,
                            quaternion.w]).as_matrix(),
        [pose.position.x, pose.position.y, pose.position.z])


def mask_and_cloud(color, depth_mm, info, model, conf, min_range, max_range):
    """YOLO workpiece mask plus every masked point in the working range."""

    import cv2
    result = model.predict(color, imgsz=640, verbose=False, conf=conf, device="cpu")
    if not result or result[0].masks is None:
        return None, None
    mask = result[0].masks.data[0].cpu().numpy()
    if mask.shape[:2] != depth_mm.shape[:2]:
        mask = cv2.resize(mask.astype(np.float32),
                          (depth_mm.shape[1], depth_mm.shape[0]),
                          interpolation=cv2.INTER_NEAREST)
    mask = mask > 0.5
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    if count > 1:
        mask = labels == (1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])))

    vs, us = np.nonzero(mask)
    zs = depth_mm[vs, us] / 1000.0
    valid = (zs > min_range) & (zs < max_range)
    if valid.sum() < 500:
        return None, mask
    us, vs, zs = us[valid], vs[valid], zs[valid]
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    points = np.column_stack([(us - cx) * zs / fx, (vs - cy) * zs / fy, zs])
    return points, mask


def table_plane(color, depth_mm, mask, info, min_range, max_range):
    """RANSAC plane on the table, away from the part."""

    import cv2
    depth_m = depth_mm / 1000.0
    free = cv2.dilate(mask.astype(np.uint8), np.ones((9, 9), np.uint8)) == 0
    vs, us = np.nonzero(free & (depth_m > min_range) & (depth_m < max_range))
    if vs.size < 2000:
        return None
    zs = depth_m[vs, us]
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    points = np.column_stack([(us - cx) * zs / fx, (vs - cy) * zs / fy, zs])

    # 2560x1920 depth gives millions of free points; RANSAC on all of them is
    # minutes of work and the SVD refinement below would ask numpy for an
    # N x N matrix (451 GiB at 246k points).  Subsample for the fit.
    if len(points) > 150_000:
        points = points[np.random.default_rng(0).choice(len(points), 150_000, replace=False)]
    rng = np.random.default_rng(0)
    best = (0, None, None)
    for _ in range(800):
        p0, p1, p2 = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        inliers = int((np.abs((points - p0) @ normal) < 0.008).sum())
        if inliers > best[0]:
            best = (inliers, normal, p0)
    if best[1] is None:
        return None
    inliers = np.abs((points - best[2]) @ best[1]) < 0.008
    cloud = points[inliers]
    centre = cloud.mean(axis=0)
    # Smallest-eigenvector of the covariance IS the plane normal; the SVD form
    # needs full_matrices=False or it allocates len(cloud)^2.
    covariance = (cloud - centre).T @ (cloud - centre)
    normal = np.linalg.eigh(covariance)[1][:, 0]
    rms = float(np.abs((cloud - centre) @ normal).std())
    return {"normal": normal, "point": centre, "inliers": int(inliers.sum()),
            "total": len(points), "rms_m": rms}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=30.0)
    arguments = parser.parse_args()

    chain = json.loads(HAND_EYE.read_text(encoding="utf-8"))
    t_tcp_cam = np.array(chain["T_tcp_cam"], dtype=float)

    from ultralytics import YOLO
    model = YOLO(arguments.weights)

    rclpy.init()
    node = Frame()
    if not wait_frame(node, arguments.timeout):
        print("FAIL: no settled colour+depth frame (is the camera publishing?)")
        return 2

    color = node.bridge.imgmsg_to_cv2(node.color, desired_encoding="bgr8")
    depth = node.bridge.imgmsg_to_cv2(node.depth)
    if depth.dtype != np.uint16:
        print(f"FAIL: depth encoding is {depth.dtype}, expected uint16 millimetres")
        return 2

    points, mask = mask_and_cloud(color, depth.astype(np.float32), node.info, model,
                                  arguments.conf, arguments.min_range,
                                  arguments.max_range)
    if points is None:
        print("FAIL: no workpiece points in this frame")
        return 2
    plane = table_plane(color, depth.astype(np.float32), mask, node.info,
                        arguments.min_range, arguments.max_range)
    if plane is None:
        print("FAIL: no table plane in this frame")
        return 2
    # The trained mask also swallows the table under the part, so the measured
    # extent is only the part once the table plane's points are taken out.
    off_plane = np.abs((points - plane["point"]) @ plane["normal"]) > 0.015
    part = points[off_plane]
    if len(part) < 500:
        print("FAIL: too few workpiece points after removing the table")
        return 2

    t_base_cam = tcp_matrix(node.tcp) @ t_tcp_cam

    # 1. depth scale: principal axis lengths against the known 114x441x465 mm
    # box.  The part stands at an unknown angle, so projections onto the camera
    # axes are not comparable; the spread ALONG each principal axis is, and for
    # a box std = extent / sqrt(12).
    centred = part - part.mean(axis=0)
    eigenvalues, _ = np.linalg.eigh(centred.T @ centred / max(len(centred) - 1, 1))
    axes = np.sqrt(eigenvalues[::-1] * 12.0)
    face = float(axes[1])                    # 441 mm, the middle principal axis
    scale_ok = abs(face - PART_FACE_M) <= PART_FACE_TOL_M

    # 2/3. table in the base frame: verticality and height.
    normal_base = t_base_cam[:3, :3] @ plane["normal"]
    if normal_base[2] < 0:
        normal_base = -normal_base
    tilt = math.degrees(math.acos(min(1.0, abs(float(normal_base[2])))))
    table_point_base = t_base_cam[:3, :3] @ plane["point"] + t_base_cam[:3, 3]
    table_z = float(table_point_base[2]
                    - (normal_base[0] * table_point_base[0]
                       + normal_base[1] * table_point_base[1]) / normal_base[2])
    tilt_ok = tilt <= TABLE_MAX_TILT_DEG
    height_ok = abs(table_z - 0.5 * sum(TABLE_BAND_Z)) <= TABLE_TOL_M

    # target the coarse stage would feed into planning
    centroid_cam = part.mean(axis=0)
    centroid_base = t_base_cam[:3, :3] @ centroid_cam + t_base_cam[:3, 3]

    print("== known-target calibration ==")
    print(f"camera in base     {np.round(t_base_cam[:3, 3], 3).tolist()}"
          f"   (camera->TCP {np.linalg.norm(t_tcp_cam[:3, 3]) * 1000:.0f} mm)")
    print(f"tcp in base        {np.round(tcp_matrix(node.tcp)[:3, 3], 3).tolist()}")
    print(f"workpiece axes     {np.round(axes, 3).tolist()} m"
          f"   vs known 0.114/0.441/0.465"
          f"   middle {face:.3f} m -> {'PASS' if scale_ok else 'FAIL'}")
    print(f"table plane        {plane['inliers']}/{plane['total']} inliers"
          f"  rms {plane['rms_m'] * 1000:.1f} mm")
    print(f"table normal base  {np.round(normal_base, 3).tolist()}"
          f"   tilt {tilt:.1f} deg vs <= {TABLE_MAX_TILT_DEG:.1f} deg"
          f"   -> {'PASS' if tilt_ok else 'FAIL'}")
    print(f"table height       {table_z:+.3f} m"
          f"   expected {TABLE_BAND_Z[0]:+.2f}..{TABLE_BAND_Z[1]:+.2f} m"
          f"   -> {'PASS' if height_ok else 'FAIL'}")
    print(f"workpiece centroid {np.round(centroid_base, 3).tolist()} m (base frame)")

    node.destroy_node()
    rclpy.shutdown()
    ok = scale_ok and tilt_ok and height_ok
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
