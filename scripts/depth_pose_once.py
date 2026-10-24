#!/usr/bin/env python3
"""Phase-2: single-shot depth capture -> masked point cloud -> 6D pose.

Pipeline (ROS-side, no RVS):

    colour frame + aligned depth frame + camera_info
      -> YOLO-seg mask (same model as phase 1)
      -> back-project masked pixels with the depth intrinsics
      -> centroid + PCA axes  => coarse 6D pose
      -> publish pose + quality

This is the static "stop and shoot" beat: depth on this sensor is ~0.85 fps, so
the node grabs one settled frame pair instead of streaming.

Published
---------
/perception/target_pose      geometry_msgs/PoseStamped  (world frame via TF later)
/perception/pose_quality     std_msgs/Float32MultiArray
                             [points, extent_x, extent_y, extent_z, rms_mm]
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

sys.stdout.reconfigure(line_buffering=True)


class DepthPoseNode(Node):
    def __init__(self, weights: str, conf: float, dump_dir: str = "") -> None:
        super().__init__("depth_pose")
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.conf = conf
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.done = False
        self.results = []
        self.repeat = 1
        self.min_range = 0.3
        self.max_range = 2.5
        self.dump_dir = dump_dir

        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/target_pose", 10)
        self.quality_pub = self.create_publisher(
            Float32MultiArray, "/perception/pose_quality", 10)
        self.get_logger().info("waiting for one settled colour+depth frame pair ...")

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_color(self, msg: Image) -> None:
        if self.done:
            return
        self.color = msg
        self.try_solve()

    def on_depth(self, msg: Image) -> None:
        if self.done:
            return
        self.depth = msg
        self.try_solve()

    # ------------------------------------------------------------------ solve
    def try_solve(self) -> None:
        if self.color is None or self.depth is None or self.info is None:
            return
        if len(self.results) >= self.repeat:
            return
        color = self.bridge.imgmsg_to_cv2(self.color, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(self.depth)
        height, width = color.shape[:2]

        results = self.model.predict(color, imgsz=320, verbose=False,
                                     conf=self.conf, device="cpu")
        if not results or results[0].masks is None:
            self.get_logger().warn("no mask from detector")
            return
        mask = results[0].masks.data[0].cpu().numpy()
        if mask.shape[:2] != (height, width):
            import cv2
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        # YOLO returns the mask at a reduced feature-map size (e.g. 256x320);
        # it must be rescaled to the DEPTH image size, otherwise every masked
        # pixel samples the wrong depth and the cloud is stretched.
        import cv2
        depth_h, depth_w = (depth.shape[0], depth.shape[1])
        if mask.shape[:2] != (depth_h, depth_w):
            mask = cv2.resize(mask.astype(np.float32), (depth_w, depth_h),
                              interpolation=cv2.INTER_NEAREST)
        mask = mask > 0.5
        height, width = depth_h, depth_w
        # Keep only the largest connected component: stray detections on the
        # table otherwise dominate the extent and corrupt the pose.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8)
        if count > 1:
            biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            mask = labels == biggest

        fx = self.info.k[0]
        fy = self.info.k[4]
        cx = self.info.k[2]
        cy = self.info.k[5]

        depth_m = depth.astype(np.float32) / 1000.0 if depth.dtype == np.uint16 \
            else depth.astype(np.float32)
        if depth_m.shape[:2] != (height, width):
            import cv2
            depth_m = cv2.resize(depth_m, (width, height), interpolation=cv2.INTER_NEAREST)

        vs, us = np.nonzero(mask)
        zs = depth_m[vs, us]
        # The camera also sees the wall/curtain behind the cell; those pixels can
        # be inside the predicted mask, so an explicit range gate keeps only the
        # working volume.  Defaults come from the cell geometry (part 1.0-2.2 m).
        valid = (np.isfinite(zs) & (zs > 0.05)
                 & (zs < self.max_range) & (zs > self.min_range))
        # The workpiece is an open frame: pixels inside its openings see the
        # background metres behind.  Keep only the nearest depth cluster, which
        # is the part itself, otherwise the cloud spans metres.
        if valid.sum() > 500:
            near = zs[valid]
            # Pick the dominant depth peak (the workpiece surface) and keep a
            # narrow band around it; the open frame shows background through its
            # openings, which would otherwise stretch the cloud by metres.
            histogram, edges = np.histogram(near, bins=40)
            peak = 0.5 * (edges[int(np.argmax(histogram))]
                          + edges[int(np.argmax(histogram)) + 1])
            valid &= np.abs(zs - peak) <= 0.08
        us, vs, zs = us[valid], vs[valid], zs[valid]
        if zs.size < 200:
            self.get_logger().warn(f"too few valid depth points: {zs.size}")
            return

        xs = (us - cx) * zs / fx
        ys = (vs - cy) * zs / fy
        points = np.column_stack([xs, ys, zs])

        # statistical outlier removal around the median point
        median = np.median(points, axis=0)
        distances = np.linalg.norm(points - median, axis=1)
        keep = distances <= 2.0 * np.median(distances) + 0.05
        points = points[keep]
        if len(points) < 200:
            self.get_logger().warn("too few points after outlier removal")
            return
        centroid = points.mean(axis=0)
        centred = points - centroid
        cov = centred.T @ centred / max(len(centred) - 1, 1)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        order = np.argsort(eigenvalues)[::-1]
        axes = eigenvectors[:, order]
        if np.linalg.det(axes) < 0:
            axes[:, 2] *= -1.0

        self.results.append(centroid.copy())
        if len(self.results) < self.repeat:
            self.color = None
            self.depth = None
            return
        points_all = np.array(self.results)
        spread = points_all.std(axis=0)
        self.get_logger().info(
            "repeat=" + str(len(points_all))
            + " std_mm=" + str(np.round(spread * 1000.0, 2).tolist())
            + " mean=" + str(np.round(points_all.mean(axis=0), 4).tolist()))
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.color.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = centroid
        quat = R.from_matrix(axes).as_quat()
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quat
        self.pose_pub.publish(pose)

        if self.dump_dir:
            import os
            import cv2
            os.makedirs(self.dump_dir, exist_ok=True)
            overlay = color.copy()
            contours, _ = cv2.findContours(mask.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(self.dump_dir, "mask_overlay.png"), overlay)
            keep = mask & (depth_m > 0.05) & (depth_m < 3.0)
            vs2, us2 = np.nonzero(keep)
            zs2 = depth_m[vs2, us2]
            xs2 = (us2 - cx) * zs2 / fx
            ys2 = (vs2 - cy) * zs2 / fy
            ply = os.path.join(self.dump_dir, "masked_cloud.ply")
            with open(ply, "w", encoding="ascii") as handle:
                handle.write("ply\nformat ascii 1.0\n")
                handle.write(f"element vertex {len(zs2)}\n")
                handle.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for x_, y_, z_ in zip(xs2, ys2, zs2):
                    handle.write(f"{x_:.4f} {y_:.4f} {z_:.4f}\n")
            self.get_logger().info(f"dumped mask overlay + {len(zs2)} points to {self.dump_dir}")

        extent = points.max(axis=0) - points.min(axis=0)
        residual = centred - (centred @ axes[:, :2]) @ axes[:, :2].T
        rms_mm = float(np.sqrt((residual ** 2).sum(axis=1).mean()) * 1000.0)
        quality = Float32MultiArray()
        quality.data = [float(len(points)), *[float(v) for v in extent], rms_mm]
        self.quality_pub.publish(quality)

        self.get_logger().info(
            "pose xyz=" + str(np.round(centroid, 4).tolist())
            + " quat=" + str(np.round(quat, 4).tolist())
            + f" points={len(points)} extent={np.round(extent, 3).tolist()} rms={rms_mm:.1f}mm")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--dump", default="", help="directory for mask overlay + PLY")
    parser.add_argument("--min-range", type=float, default=0.3,
                        help="metres; ignore depths closer than this")
    parser.add_argument("--max-range", type=float, default=2.5,
                        help="metres; ignore depths beyond this (background)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="capture N frames in one run and report the std")
    args = parser.parse_args()

    rclpy.init()
    node = DepthPoseNode(args.weights, args.conf, args.dump)
    node.repeat = max(1, args.repeat)
    node.min_range = args.min_range
    node.max_range = args.max_range
    deadline = time.time() + args.timeout
    while rclpy.ok() and not node.done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
