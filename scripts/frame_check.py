#!/usr/bin/env python3
"""拍摄前检查：这个观测位能不能把目标完整框进画面、深度窗该取多少。

为什么需要它
------------
今天踩过两次同样的坑：
  · 固定观测位对新工件位置失效 → 拍到的是背景红布；
  · 近距离观测把工件框爆 → 边缘被裁、分割退化。
两者都会让下游"看起来有结果、其实是垃圾"。所以在按快门前先算一遍几何。

做法
----
把目标的 CAD 包围盒 8 个角点用"观测位 → 相机"的变换投到图像上：
  · 8 个角有几落在画面内；
  · 目标的深度范围（决定深度窗）；
  · 目标中心到相机的距离。

用法
----
    # 插槽：目标位姿取 config 的 slot_origin + slot_origin_q
    python3 scripts/frame_check.py --preset slot --observe slot_coarse

    # 工件：目标位姿从最近一次测量结果读
    python3 scripts/frame_check.py --preset workpiece --observe workpiece_coarse \
        --object-pose runtime/measurements/workpiece_main.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp                                          # noqa: E402

TEMPLATES = {
    "workpiece": pathlib.Path.home() / "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL",
    "slot": REPO / "src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl",
}
K = [931.473, 648.695, 931.595, 449.319]
WIDTH, HEIGHT = 1280, 960


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preset", choices=list(TEMPLATES), required=True)
    parser.add_argument("--observe", required=True, help="config/tasks/common.yaml 里的观测位名字")
    parser.add_argument("--object-pose", default=None,
                        help="工件目标位姿（measure_object 的输出 JSON）；插槽不需要")
    parser.add_argument("--margin", type=float, default=40.0, help="画面边缘容差（像素）")
    args = parser.parse_args()

    config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
    if args.observe not in config["observe"]:
        print(f"config 里没有观测位 {args.observe}", file=sys.stderr)
        return 2
    observe = config["observe"][args.observe]
    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = R.from_quat(observe["quat"]).as_matrix()
    T_base_tcp[:3, 3] = observe["xyz"]
    T_base_cam = T_base_tcp @ np.array(handeye["T_tcp_cam"])

    template = pathlib.Path(TEMPLATES[args.preset]).expanduser()
    bbox_min, bbox_max = cp.load_template_bbox(template)
    corners = np.array([[bbox_max[i] if (index >> i) & 1 else bbox_min[i] for i in range(3)]
                        for index in range(8)])

    if args.preset == "slot":
        T_base_object = np.eye(4)
        T_base_object[:3, :3] = R.from_quat(config["slot_origin_q"]).as_matrix()
        T_base_object[:3, 3] = config["slot_origin"]
    else:
        if not args.object_pose:
            print("工件需要 --object-pose（measure_object 的输出）", file=sys.stderr)
            return 2
        data = json.loads(pathlib.Path(args.object_pose).read_text())
        group = data["per_group"][0]
        if "pose_base" in group:
            T_base_object = np.eye(4)
            T_base_object[:3, :3] = R.from_quat(group["pose_base"]["quaternion"]).as_matrix()
            T_base_object[:3, 3] = group["pose_base"]["translation"]
        else:
            print("measure_object 输出里没有 pose_base", file=sys.stderr)
            return 2

    world_corners = (T_base_object[:3, :3] @ corners.T).T + T_base_object[:3, 3]
    centre = (bbox_min + bbox_max) / 2
    centre_world = T_base_object[:3, :3] @ centre + T_base_object[:3, 3]
    distance = float(np.linalg.norm(centre_world - T_base_cam[:3, 3]))

    camera_points = (T_base_cam[:3, :3].T @ (world_corners - T_base_cam[:3, 3]).T).T
    depth_min, depth_max = camera_points[:, 2].min(), camera_points[:, 2].max()
    u = K[0] * camera_points[:, 0] / camera_points[:, 2] + K[1]
    v = K[2] * camera_points[:, 1] / camera_points[:, 2] + K[3]
    inside = int(((u > -args.margin) & (u < WIDTH + args.margin)
                  & (v > -args.margin) & (v < HEIGHT + args.margin)).sum())

    print(f"目标 {args.preset} @ {args.observe}")
    print(f"  相机在 base {np.round(T_base_cam[:3, 3], 3)}，到目标中心 {distance:.2f} m")
    print(f"  目标深度范围 {depth_min:.2f}–{depth_max:.2f} m → 建议深度窗 "
          f"{max(0.05, depth_min - 0.10):.2f} {depth_max + 0.10:.2f}")
    print(f"  8 个包围盒角中 {inside}/8 落在画面内（容差 {args.margin:.0f} px）")
    print(f"  像素范围 u {u.min():.0f}–{u.max():.0f}，v {v.min():.0f}–{v.max():.0f}")
    verdict = "OK" if inside == 8 else ("PARTIAL（有个别角出画）" if inside >= 6 else "BAD")
    print(f"  取景判定：{verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
