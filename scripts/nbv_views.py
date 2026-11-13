#!/usr/bin/env python3
"""NBV 视角评分（统一入口，离线可用）：用 z-buffer 的"侧面像素占比"给候选视角排序。

为什么换掉原来的余弦模型（2026-11-13 复审结论）
----------------------------------------------
原来的打分是 |cos(法向, 视线)| × 面积 × 权重，它衡量的是"看得见多少面积"，
而精观测失败的真正原因是【偏航散布】——偏航只能靠侧面约束。
工件 455×131×465 mm：±y 是大面（0.211 m²，手柄所在），±x 是侧面（0.061 m²），
±z 是顶/底（0.060 m²）。
所以评分必须是"侧面的可见像素占比"，而不是总面积。

视角的选取
----------
* 候选：绕工件方位 ±180°、俯角 {10,20,30,40}°、距离 {0.65,0.8}；
* 每个候选先用 view_scan.predict() 算带遮挡的逐面像素数；
* 得分 = 侧面像素占比（±x 取较大者），并要求大面（±y）像素 ≥ 8% 以免看不到手柄；
* 输出排名；--execute 交给 nbv_lateral 执行（本脚本不移动机械臂）。
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp  # noqa: E402
import view_scan  # noqa: E402

TEMPLATE = pathlib.Path.home() / "cs625_rvs_archive" / "models_20261023" / "弹仓_装配导出.STL"


def load_pose(path: pathlib.Path, group: str = "g00_m") -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [item for item in rows if item.get("group") == group][0]
    pose = np.eye(4)
    pose[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    pose[:3, 3] = row["pose_base"]["translation"]
    return pose


def score_view(T_world_object: np.ndarray, T_world_cam: np.ndarray, bbox, cad) -> dict:
    """侧面像素占比 + 大面可见性门槛。"""
    result = view_scan.predict(T_world_object, T_world_cam, bbox, cad)
    pixels = result["visible_px"]
    total = max(1, sum(pixels.values()))
    side = max(pixels["-x"], pixels["+x"]) / total          # 侧面（约束偏航）
    big = max(pixels["-y"], pixels["+y"]) / total           # 大面（手柄）
    top = max(pixels["-z"], pixels["+z"]) / total           # 顶/底
    usable = big >= 0.08 and top >= 0.02 and side >= 0.01
    # 主项是侧面，其次给大面一点权重，保证手柄在画面里
    score = side + 0.25 * big if usable else -1.0
    return {"score": round(float(score), 4), "side": round(float(side), 4),
            "big": round(float(big), 4), "top": round(float(top), 4),
            "usable": bool(usable), "pixels": pixels}


def camera_pose(centre: np.ndarray, azimuth_deg: float, elevation_deg: float,
                distance: float) -> np.ndarray:
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    direction = np.array([math.cos(el) * math.cos(az),
                          math.cos(el) * math.sin(az),
                          math.sin(el)])
    position = centre + direction * distance
    z_axis = -direction / np.linalg.norm(direction)          # 光轴指向工件
    world_up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(world_up, z_axis)
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1.0, 0.0, 0.0])
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    pose[:3, 3] = position
    return pose


def select_known_view(yaw_deg: float, tolerance: float = 12.0):
    """按工件偏转角从查表里取"已验证可用"的精观测位。

    为什么优先查表（2026-11-13 结论）：用 CAD 盒做 z-buffer 打分对"哪个视角真的能测准"
    几乎没有分辨力（实测三个视角的好/差，得分 0.2704 / 0.2704 / 0.2715）。
    已经验证通过的位姿比任何几何打分都可靠，而且不用搜索、不会撞。
    """
    import yaml
    config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
    table = config.get("workpiece_views") or {}
    best = None
    for key, entry in table.items():
        if not isinstance(entry, dict) or "tcp" not in entry:
            continue
        try:
            angle = float(key)
        except ValueError:
            continue
        error = abs(angle - yaw_deg)
        if error <= tolerance and (best is None or error < best[0]):
            best = (error, key, entry)
    return best


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--measurements", required=True)
    parser.add_argument("--group", default="g00_m")
    parser.add_argument("--top", type=int, default=8)
    parser.add_argument("--yaw-deg", type=float, default=None,
                        help="工件偏转角；给出时优先查已验证视角表，查不到再排名")
    parser.add_argument("--distance", default="0.65,0.80")
    parser.add_argument("--elevations", default="10,20,30,40")
    parser.add_argument("--step", type=float, default=20.0)
    parser.add_argument("--cad-points", type=int, default=20000)
    args = parser.parse_args()

    if args.yaw_deg is not None:
        found = select_known_view(args.yaw_deg)
        if found is not None:
            error, key, entry = found
            print(f"偏转 {args.yaw_deg:+.1f}° 命中查表 {key}°（差 {error:.1f}°）："
                  f"TCP {np.round(entry['tcp'], 4)}")
            print("→ 直接使用该已验证位姿，不搜索")
            return 0
        print(f"偏转 {args.yaw_deg:+.1f}° 未命中查表，改用几何排名（可靠性有限）")

    pose = load_pose(pathlib.Path(args.measurements), args.group)
    bbox = cp.load_template_bbox(TEMPLATE)
    import open3d as o3d
    cad = cp.sample_mesh(o3d.io.read_triangle_mesh(str(TEMPLATE)), args.cad_points, seed=7)
    centre = pose[:3, :3] @ ((bbox[0] + bbox[1]) / 2) + pose[:3, 3]
    print(f"工件中心 {np.round(centre, 3)}，模板尺寸 {np.round(bbox[1] - bbox[0], 3)}")

    distances = [float(v) for v in args.distance.split(",")]
    elevations = [float(v) for v in args.elevations.split(",")]
    rows = []
    for step in np.arange(-180, 180, args.step):
        for elevation in elevations:
            for distance in distances:
                T_cam = camera_pose(centre, float(step), elevation, distance)
                info = score_view(pose, T_cam, bbox, cad)
                rows.append((info["score"], float(step), elevation, distance, info, T_cam))
    rows.sort(key=lambda item: -item[0])
    print("排名（得分 = 侧面像素占比 + 0.25×大面占比，要求大面 ≥8%）：")
    for score, step, elevation, distance, info, T_cam in rows[: args.top]:
        print(f"  方位偏移 {step:+5.0f}° 俯角 {elevation:4.0f}° 距离 {distance:.2f} m  "
              f"侧面 {info['side']:.3f}  大面 {info['big']:.3f}  顶面 {info['top']:.3f}  "
              f"得分 {score:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
