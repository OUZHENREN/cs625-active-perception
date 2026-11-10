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


def check(preset: str, observe_name: str, object_pose: str | None = None,
          margin: float = 40.0) -> dict:
    """返回取景检查结果（供 measure_object 等复用）。"""
    config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
    if observe_name not in config["observe"]:
        raise KeyError(f"config 里没有观测位 {observe_name}")
    observe = config["observe"][observe_name]
    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = R.from_quat(observe["quat"]).as_matrix()
    T_base_tcp[:3, 3] = observe["xyz"]
    T_base_cam = T_base_tcp @ np.array(handeye["T_tcp_cam"])

    template = pathlib.Path(TEMPLATES[preset]).expanduser()
    bbox_min, bbox_max = cp.load_template_bbox(template)
    corners = np.array([[bbox_max[i] if (index >> i) & 1 else bbox_min[i] for i in range(3)]
                        for index in range(8)])

    if preset == "slot":
        T_base_object = np.eye(4)
        T_base_object[:3, :3] = R.from_quat(config["slot_origin_q"]).as_matrix()
        T_base_object[:3, 3] = config["slot_origin"]
    else:
        if not object_pose:
            raise ValueError("工件需要 object_pose（measure_object 的输出）")
        data = json.loads(pathlib.Path(object_pose).read_text())
        group = data["per_group"][0]
        T_base_object = np.eye(4)
        T_base_object[:3, :3] = R.from_quat(group["pose_base"]["quaternion"]).as_matrix()
        T_base_object[:3, 3] = group["pose_base"]["translation"]

    world_corners = (T_base_object[:3, :3] @ corners.T).T + T_base_object[:3, 3]
    centre = (bbox_min + bbox_max) / 2
    centre_world = T_base_object[:3, :3] @ centre + T_base_object[:3, 3]
    distance = float(np.linalg.norm(centre_world - T_base_cam[:3, 3]))
    camera_points = (T_base_cam[:3, :3].T @ (world_corners - T_base_cam[:3, 3]).T).T
    depth_min, depth_max = float(camera_points[:, 2].min()), float(camera_points[:, 2].max())
    u = K[0] * camera_points[:, 0] / camera_points[:, 2] + K[1]
    v = K[2] * camera_points[:, 1] / camera_points[:, 2] + K[3]
    inside = int(((u > -margin) & (u < WIDTH + margin)
                  & (v > -margin) & (v < HEIGHT + margin)).sum())
    verdict = ("OK" if inside == 8 else ("PARTIAL" if inside >= 6 else "BAD"))
    return {
        "preset": preset, "observe": observe_name,
        "camera_base": [round(float(x), 4) for x in T_base_cam[:3, 3]],
        "distance_m": round(distance, 3),
        "depth_range_m": [round(depth_min, 3), round(depth_max, 3)],
        "suggested_window_m": [round(max(0.05, depth_min - 0.10), 2),
                               round(depth_max + 0.10, 2)],
        "corners_inside": inside,
        "pixel_u": [round(float(u.min())), round(float(u.max()))],
        "pixel_v": [round(float(v.min())), round(float(v.max()))],
        "verdict": verdict,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preset", choices=list(TEMPLATES), required=True)
    parser.add_argument("--observe", required=True, help="config/tasks/common.yaml 里的观测位名字")
    parser.add_argument("--object-pose", default=None,
                        help="工件目标位姿（measure_object 的输出 JSON）；插槽不需要")
    parser.add_argument("--margin", type=float, default=40.0, help="画面边缘容差（像素）")
    args = parser.parse_args()
    result = check(args.preset, args.observe, args.object_pose, args.margin)
    print(f"目标 {result['preset']} @ {result['observe']}")
    print(f"  相机在 base {result['camera_base']}，到目标中心 {result['distance_m']:.2f} m")
    print(f"  目标深度范围 {result['depth_range_m'][0]:.2f}–{result['depth_range_m'][1]:.2f} m"
          f" → 建议深度窗 {result['suggested_window_m'][0]:.2f} {result['suggested_window_m'][1]:.2f}")
    print(f"  8 个包围盒角中 {result['corners_inside']}/8 落在画面内（容差 {args.margin:.0f} px）")
    print(f"  像素范围 u {result['pixel_u'][0]}–{result['pixel_u'][1]}，"
          f"v {result['pixel_v'][0]}–{result['pixel_v'][1]}")
    print(f"  取景判定：{result['verdict']}")
    return 0 if result["verdict"] != "BAD" else 2


if __name__ == "__main__":
    raise SystemExit(main())
