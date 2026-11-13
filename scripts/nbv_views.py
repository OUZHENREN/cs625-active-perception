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
    return pose_from_payload(payload, group)


def pose_from_payload(payload, group: str = "g00_m") -> np.ndarray:
    """measure_object 输出（per_group 结构）→ 4x4 base 系位姿。"""
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [item for item in rows if item.get("group") == group][0]
    pose = np.eye(4)
    pose[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    pose[:3, 3] = row["pose_base"]["translation"]
    return pose


def yaw_degrees_between(ref_pose: np.ndarray, cur_pose: np.ndarray) -> float:
    """工件绕竖直轴（世界 z）的转角：cur 相对 ref 的角度，符号与 measure_object 一致。

    约定用 2026-11-13 真机实测文件核对过：
      workpiece_zero.json vs workpiece_minus30.json → −29.57°（measure_object 报 −29.6°）
      workpiece_zero.json vs workpiece_coarse.json → +11.26°（交接文档报 +11.2°）
    """
    rel = cur_pose[:3, :3] @ ref_pose[:3, :3].T
    return float(np.degrees(np.arctan2(rel[1, 0], rel[0, 0])))


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


def select_known_view(yaw_deg: float, tolerance: float | None = None):
    """按工件偏转角从查表里取"已验证可用"的精观测位。

    为什么优先查表（2026-11-13 结论）：用 CAD 盒做 z-buffer 打分对"哪个视角真的能测准"
    几乎没有分辨力（实测三个视角的好/差，得分 0.2704 / 0.2704 / 0.2715）。
    已经验证通过的位姿比任何几何打分都可靠，而且不用搜索、不会撞。

    tolerance 缺省时取查表里的 tolerance_deg（默认 12.0°）。
    """
    import yaml
    config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
    table = config.get("workpiece_views") or {}
    if tolerance is None:
        tolerance = float(table.get("tolerance_deg", 12.0))
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


def station_plan(views: dict, ref_pose: np.ndarray, cur_pose: np.ndarray,
                 T_tcp_cam: np.ndarray, family_gate_mm: float = 40.0) -> dict:
    """方案 A 选站：按偏转角选最近的预教站位，并与"0° 相对视角推导"交叉核对。

    纯数学、不碰 ROS。数据流：
      粗观测工件位姿（cur_pose，base 系）→ yaw → 查表选站（workpiece_views）
      → 0° 标定视角的相对变换跟到当前工件位姿（derived，诊断用）
      → 站位绝对位姿（stored，已验证）是执行目标。

    安全语义：
      · 查表无覆盖（|yaw − key| 超过容差）→ verdict "no_station"，拒绝自动换位；
      · 站位属于 relative_zero 族（由 0° 推导而来，默认族）且推导结果与存储位姿
        相差 > family_gate_mm（默认 40 mm）→ verdict "station_moved"：工件位置
        与示教时不同，存储位姿已过期，拒绝自动换位（宁可原地精测/人工摆位）；
      · family == "taught"（所有者手摆的独立站位，如 +11/+11_left8）不做此门
        ——它本来就与 0° 族不同视角，derived 差值只作参考信息。

    返回字段：yaw_deg / station (key, entry, error_deg) 或 None /
    derived_tcp / derived_quat / stored_tcp / stored_quat /
    delta_mm / delta_deg / verdict ∈ {station, no_station, station_moved}
    """
    yaw = yaw_degrees_between(ref_pose, cur_pose)
    tolerance = float(views.get("tolerance_deg", 12.0))
    best = None
    for key, entry in views.items():
        if not isinstance(entry, dict) or "tcp" not in entry:
            continue
        try:
            angle = float(key)
        except ValueError:
            continue
        error = abs(angle - yaw)
        if error <= tolerance and (best is None or error < best[0]):
            best = (error, key, entry)

    # 0° 相对视角推导（与 nbv_from_reference.py 同一套数学），只用于交叉核对。
    anchor = views.get("0")
    if not isinstance(anchor, dict) or "tcp" not in anchor:
        raise ValueError("workpiece_views 缺少 '0' 基准站位，无法做相对视角推导")
    T_cam0 = np.eye(4)
    T_cam0[:3, :3] = R.from_quat(anchor["quat"]).as_matrix()
    T_cam0[:3, 3] = anchor["tcp"]
    T_obj_cam0 = np.linalg.inv(ref_pose) @ (T_cam0 @ T_tcp_cam)
    derived = cur_pose @ T_obj_cam0 @ np.linalg.inv(T_tcp_cam)
    derived_tcp = derived[:3, 3]
    derived_quat = R.from_matrix(derived[:3, :3]).as_quat()

    if best is None:
        return {"yaw_deg": yaw, "station": None,
                "derived_tcp": derived_tcp, "derived_quat": derived_quat,
                "stored_tcp": None, "stored_quat": None,
                "delta_mm": None, "delta_deg": None, "verdict": "no_station"}

    error, key, entry = best
    stored_tcp = np.asarray(entry["tcp"], dtype=float)
    stored_quat = np.asarray(entry["quat"], dtype=float)
    delta_mm = float(np.linalg.norm(stored_tcp - derived_tcp) * 1000.0)
    delta_deg = float(np.degrees(
        (R.from_quat(stored_quat) * R.from_quat(derived_quat).inv()).magnitude()))
    family = entry.get("family", "relative_zero")
    verdict = "station"
    if family == "relative_zero" and delta_mm > family_gate_mm:
        verdict = "station_moved"
    return {"yaw_deg": yaw, "station": (error, key, entry),
            "derived_tcp": derived_tcp, "derived_quat": derived_quat,
            "stored_tcp": stored_tcp, "stored_quat": stored_quat,
            "delta_mm": delta_mm, "delta_deg": delta_deg, "verdict": verdict}


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
