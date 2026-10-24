#!/usr/bin/env python3
"""Record RGB-D frames continuously while another script drives the task.

The task run produces the most valuable training data: the workpiece at every
distance and angle the real pipeline actually visits (approach, grasp, lift,
transport, insert).  This recorder samples the camera at a fixed rate and stores
colour, depth, the TCP pose and the latest grasp event, so each frame can be
labelled later by *stage*, not just by pose.

Usage:
  python3 scripts/record_frames.py --out ~/datasets/2026-10-25_raw/task --rate 1.0
  # in another shell: python3 scripts/run_pick_and_place.py --confirm ...
  # stop the recorder with Ctrl-C; one metadata.jsonl entry per saved frame.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.stdout.reconfigure(line_buffering=True)


class Recorder(Node):
    def __init__(self) -> None:
        super().__init__("record_frames")
        self.color = None
        self.depth = None
        self.tcp = None
        self.event = ""
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self._on_tcp, 10)
        self.create_subscription(String, "/cs625/grasp_state_event",
                                 self._on_event, 10)

    def _on_color(self, msg):
        self.color = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_tcp(self, msg):
        self.tcp = msg

    def _on_event(self, msg):
        self.event = msg.data


def decode(image) -> np.ndarray:
    raw = np.frombuffer(image.data, dtype=np.uint8)
    channels = 3 if image.encoding.lower() in ("bgr8", "rgb8") else 2
    raw = raw[:image.height * image.step].reshape(image.height, image.step)
    raw = raw[:, :image.width * channels]
    array = raw.reshape(image.height, image.width, channels)
    if image.encoding.lower() == "rgb8":
        array = array[:, :, ::-1]
    return array


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    parser.add_argument("--tag", default="task")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="0 = run until Ctrl-C")
    arguments = parser.parse_args()

    output = pathlib.Path(arguments.out).expanduser() / arguments.tag
    output.mkdir(parents=True, exist_ok=True)
    period = 1.0 / max(arguments.rate, 0.05)

    rclpy.init()
    node = Recorder()
    start = time.time()
    last_save = 0.0
    saved = 0
    print(f"recording -> {output} ({arguments.rate:.1f} Hz)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if arguments.duration and now - start > arguments.duration:
                break
            if now - last_save < period:
                continue
            if node.color is None or node.depth is None or node.tcp is None:
                continue
            last_save = now
            stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int((now % 1) * 1000):03d}"
            name = f"{arguments.tag}_{stamp}"
            cv2.imwrite(str(output / f"{name}_color.png"), decode(node.color))
            np.save(output / f"{name}_depth.npy", decode(node.depth))
            pose = node.tcp.pose
            quaternion = [pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w]
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
            matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            record = {"file": name, "tag": arguments.tag,
                      "label": node.event or "task",
                      "tcp_matrix": [float(v) for v in matrix.reshape(-1)],
                      "elapsed_s": round(now - start, 2)}
            with (output / "metadata.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            saved += 1
            if saved % 20 == 0:
                print(f"  {saved} frames, event={node.event or '-'}", flush=True)
    except KeyboardInterrupt:
        pass
    print(f"stopped: {saved} frames -> {output}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
