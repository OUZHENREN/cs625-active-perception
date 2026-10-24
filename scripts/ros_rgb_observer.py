#!/usr/bin/env python3
"""Phase-1 real-time RGB observer on the ROS side (no RVS).

Purpose
-------
Provide the front-end signal for active re-observation:

    /camera/color/image_raw  ->  detection  ->  geometry + quality

Published
---------
/perception/target_2d            geometry_msgs/Pose2D      (normalised u,v + angle)
/perception/observation_quality  std_msgs/Float32MultiArray
                                 [score, confidence, area_ratio, margin_ratio, fps]

Detectors
---------
``--detector template``  OpenCV template matching against ``--template`` PNG.
``--detector onnx``      cv2.dnn on an exported ONNX detector (preferred later).
``--detector none``      no detection; only publishes frame statistics, useful to
                         verify the transport and the scoring pipeline.

The scoring follows the owner's criterion: RVS score AND target completeness in
the field of view.  ``score`` combines normalised confidence, how much of the
frame the target covers and how far it is from the frame border.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

sys.stdout.reconfigure(line_buffering=True)


class RgbObserver(Node):
    def __init__(self, detector: str, template_path: str, onnx_path: str,
                 min_confidence: float) -> None:
        super().__init__("rgb_observer")
        self.bridge = CvBridge()
        self.detector = detector
        self.min_confidence = min_confidence
        self.template = None
        self.net = None
        self.frames = 0
        self.started = time.time()

        if detector == "template":
            import cv2
            self.cv2 = cv2
            self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if self.template is None:
                raise SystemExit(f"cannot read template: {template_path}")
            self.get_logger().info(f"template detector, size={self.template.shape}")
        elif detector == "onnx":
            import cv2
            self.cv2 = cv2
            self.net = cv2.dnn.readNetFromONNX(onnx_path)
            self.get_logger().info(f"onnx detector: {onnx_path}")
        elif detector == "yolo":
            # Ultralytics handles YOLO-seg post-processing correctly, so it is
            # used for the trained model while cv2.dnn stays for plain ONNX nets.
            from ultralytics import YOLO
            self.yolo = YOLO(onnx_path if onnx_path.endswith(('.pt', '.onnx')) else onnx_path)
            self.yolo_imgsz = 320
            self.get_logger().info(f"yolo detector: {onnx_path}")

        self.pose_pub = self.create_publisher(Pose2D, "/perception/target_2d", 10)
        self.quality_pub = self.create_publisher(
            Float32MultiArray, "/perception/observation_quality", 10)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self.on_image, 10)

    # ------------------------------------------------------------------ image
    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = frame.shape[:2]
        self.frames += 1
        elapsed = max(time.time() - self.started, 1e-6)
        fps = self.frames / elapsed

        confidence, centre_u, centre_v, angle, area_ratio, margin_ratio = self.detect(frame)
        score = self.score(confidence, area_ratio, margin_ratio)

        pose = Pose2D()
        pose.x = centre_u
        pose.y = centre_v
        pose.theta = angle
        quality = Float32MultiArray()
        quality.data = [score, confidence, area_ratio, margin_ratio, fps]
        if confidence >= self.min_confidence:
            self.pose_pub.publish(pose)
        self.quality_pub.publish(quality)

        if self.frames % 30 == 0:
            self.get_logger().info(
                f"fps={fps:.1f} conf={confidence:.2f} area={area_ratio:.2f} "
                f"margin={margin_ratio:.2f} score={score:.2f} "
                f"frame={width}x{height}")

    # --------------------------------------------------------------- detectors
    def detect(self, frame):
        """Return (confidence, u, v, angle, area_ratio, margin_ratio) in [0,1]."""
        import cv2

        height, width = frame.shape[:2]
        if self.detector == "template" and self.template is not None:
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            result = cv2.matchTemplate(grey, self.template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            th, tw = self.template.shape[:2]
            left, top = max_loc
            right, bottom = left + tw, top + th
            centre_u = (left + tw / 2) / width
            centre_v = (top + th / 2) / height
            area_ratio = (tw * th) / float(width * height)
            margin = min(left, top, width - right, height - bottom)
            margin_ratio = max(margin, 0) / float(min(width, height))
            return float(max_val), centre_u, centre_v, 0.0, area_ratio, margin_ratio
        if self.detector == "yolo":
            results = self.yolo.predict(frame, imgsz=self.yolo_imgsz, verbose=False,
                                        conf=self.min_confidence, device="cpu")
            if not results:
                return 0.0, 0.5, 0.5, 0.0, 0.0, 0.0
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return 0.0, 0.5, 0.5, 0.0, 0.0, 0.0
            confs = boxes.conf.cpu().numpy()
            best = int(np.argmax(confs))
            confidence = float(confs[best])
            x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().tolist()
            centre_u = ((x1 + x2) / 2.0) / width
            centre_v = ((y1 + y2) / 2.0) / height
            area_ratio = ((x2 - x1) * (y2 - y1)) / float(width * height)
            margin = min(x1, y1, width - x2, height - y2)
            margin_ratio = max(margin, 0.0) / float(min(width, height))
            return confidence, centre_u, centre_v, 0.0, area_ratio, margin_ratio

        if self.detector == "onnx" and self.net is not None:
            blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (640, 640),
                                         swapRB=True, crop=False)
            self.net.setInput(blob)
            output = self.net.forward()
            if output is None or output.size == 0:
                return 0.0, 0.5, 0.5, 0.0, 0.0, 0.0
            flat = output.reshape(-1, output.shape[-1]) if output.ndim == 3 else output
            best = flat[np.argmax(flat[:, 4])] if flat.shape[1] > 4 else flat[0]
            confidence = float(best[4])
            cx, cy, w, h = (float(v) for v in best[:4])
            area_ratio = (w * h) / (640.0 * 640.0)
            margin_ratio = max(0.0, min(cx - w / 2, cy - h / 2,
                                        1 - cx - w / 2, 1 - cy - h / 2))
            return confidence, cx, cy, 0.0, area_ratio, margin_ratio
        # detector == none: no target, only transport statistics
        return 0.0, 0.5, 0.5, 0.0, 0.0, 0.0

    @staticmethod
    def score(confidence: float, area_ratio: float, margin_ratio: float) -> float:
        """Owner's criterion: high detector score and good in-frame completeness."""
        completeness = min(1.0, margin_ratio / 0.15)      # fully inside at 15% margin
        size_term = min(1.0, area_ratio / 0.15)           # target neither tiny nor huge
        return float(0.5 * confidence + 0.3 * completeness + 0.2 * size_term)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--detector", default="none",
                        choices=["none", "template", "onnx", "yolo"])
    parser.add_argument("--template", default="")
    parser.add_argument("--onnx", default="")
    parser.add_argument("--min-confidence", type=float, default=0.35)
    args = parser.parse_args()

    rclpy.init()
    node = RgbObserver(args.detector, args.template, args.onnx, args.min_confidence)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
