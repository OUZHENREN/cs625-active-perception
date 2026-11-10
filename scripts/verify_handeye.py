#!/usr/bin/env python3
"""验明手眼口径：把 base 系的已知点投到真实图像上，看哪条链路落在物体上。

背景
----
文档里长期挂着一条"手眼链口径（T_tcp_cam vs T_flange_cam）未验证，差约 0.16 m"。
2026-11-10 用两件事把它解开了：

  1. TF：`base -> tool0` 与 `base -> tcp_pose` 平移相差 **0.356 m**（≈ 法兰→TCP 的 353 mm），
     说明 `/tcp_pose_broadcaster/pose` 报的是 **TCP**，`tool0` 才是法兰；
  2. 投影：用 `T_base_cam = T_base_tcp @ T_tcp_cam` 把 model.json 的工件中心投到实时图像上，
     落点正好在工件表面上；把同一份标定当成 `T_flange_cam` 用（错误口径）则落到画面外。

本脚本把这两件事做成可复现的检查（不动机械臂，只读 TF/话题/图像）。

用法
----
    source scripts/source_dev_env.sh --full
    python3 scripts/verify_handeye.py            # 打印两条链路的投影对比
    python3 scripts/verify_handeye.py --draw     # 另存标注图 runtime/handeye_projection.png
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
# base 系的已知点（来源：runtime/environment/latest/model.json）
KNOWN_POINTS = {
    "workpiece_center": [0.7206, 0.6182, 0.1335],
    "slot_center": [1.2194, 0.1389, 0.4444],
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--draw", action="store_true", help="保存标注图")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from scipy.spatial.transform import Rotation as R
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("verify_handeye")
    packets: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: packets.__setitem__("tcp", m), 10)
    if args.draw:
        node.create_subscription(Image, "/camera/color/image_raw",
                                 lambda m: packets.__setitem__("img", m), 10)
    deadline = time.time() + args.timeout
    while time.time() < deadline and ("tcp" not in packets or (args.draw and "img" not in packets)):
        rclpy.spin_once(node, timeout_sec=0.2)
    if "tcp" not in packets:
        print("读不到 /tcp_pose_broadcaster/pose：驱动栈没起？", file=sys.stderr)
        return 2

    pose = packets["tcp"].pose
    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w]).as_matrix()
    T_base_tcp[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    handeye = json.loads(HANDEYE.read_text())
    chains = {
        "A: T_base_tcp @ T_tcp_cam": T_base_tcp @ np.array(handeye["T_tcp_cam"]),
        "B: T_base_tcp @ T_flange_cam": T_base_tcp @ np.array(
            handeye["T_flange_cam_color_optical"]),
    }
    k = [931.473, 648.695, 931.595, 449.319]      # 1280×960（见 pose_conventions.md）
    width, height = 1280, 960
    print(f"TCP(base): {np.round(T_base_tcp[:3, 3], 4)}")
    for name, matrix in chains.items():
        print(f"\n{name}\n  相机位置(base): {np.round(matrix[:3, 3], 4)}")
        for label, point in KNOWN_POINTS.items():
            camera = matrix[:3, :3].T @ (np.array(point) - matrix[:3, 3])
            if camera[2] <= 0:
                print(f"  {label:<18} 相机背后")
                continue
            u = k[0] * camera[0] / camera[2] + k[1]
            v = k[2] * camera[1] / camera[2] + k[3]
            inside = 0 <= u < width and 0 <= v < height
            print(f"  {label:<18} 像素 ({u:6.0f},{v:6.0f})  {'画面内' if inside else '画面外'}"
                  f"  深度 {camera[2]:.3f} m")

    if args.draw and "img" in packets:
        image = np.frombuffer(bytes(packets["img"].data), np.uint8).reshape(
            packets["img"].height, packets["img"].width, 3).copy()
        colours = {"workpiece_center": (0, 255, 0), "slot_center": (255, 0, 255)}
        matrix = chains["A: T_base_tcp @ T_tcp_cam"]
        for label, point in KNOWN_POINTS.items():
            camera = matrix[:3, :3].T @ (np.array(point) - matrix[:3, 3])
            if camera[2] <= 0:
                continue
            u = int(k[0] * camera[0] / camera[2] + k[1])
            v = int(k[2] * camera[1] / camera[2] + k[3])
            colour = colours[label]
            cv2.drawMarker(image, (u, v), colour, cv2.MARKER_CROSS, 40, 3)
            cv2.putText(image, label, (u + 12, v - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, colour, 2)
        out = REPO / "runtime" / "handeye_projection.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), image)
        print(f"\n标注图：{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
