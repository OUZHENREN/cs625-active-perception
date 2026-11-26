#!/usr/bin/env python3
"""角点锚定插槽位姿（离线原型，2026-11-26）。

为什么需要它
------------
插槽以大平面为主，点-面 ICP 在面内几乎没有约束：同一片点云两条配准路径差 **66 mm**
（手册第 68 条），实测跨天差异 ~40 mm。**多测取平均不会收敛**——必须让每个自由度
都由几何锚点定义。本脚本用「三平面求交 → 角点」构造插槽坐标系：

    左内壁面 ∩ 右内壁面 → 一条竖直棱（给出 y 与 yaw）
    该棱 ∩ 底面（或楔形斜面）→ 角点（给出 z / x 的一个锚点）
    两个角点 + 面法向 → 完整 6 DOF 插槽坐标系

数据来源（离线，不动真机）
--------------------------
每个采集组有 `metadata.jsonl`（T_base_tcp）+ 手眼 `config/handeye/tcp_to_camera.json`
（T_tcp_cam）→ `T_base_cam = T_base_tcp @ T_tcp_cam`，把 camera 系点云变换到 base 系，
于是不同组/不同天的结果可以直接比较。

用法
----
    python3 scripts/slot_corner_anchor.py --dataset runtime/vision_dataset/measure_20261125_191925
    python3 scripts/slot_corner_anchor.py --dataset <dir> --roi-centre 1.17 -0.05 0.19 --save-json out.json
退出码：0 成功；2 数据缺失；4 平面提取失败。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
from dataclasses import dataclass

import numpy as np

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
PLANE_DIST_M = 0.004          # RANSAC 内点距离
MIN_PLANE_POINTS = 2000
ROI_HALF = (0.30, 0.30, 0.30)


@dataclass
class Plane:
    normal: np.ndarray
    offset: float
    inliers: int
    centroid: np.ndarray

    def signed_dist(self, pts: np.ndarray) -> np.ndarray:
        return pts @ self.normal + self.offset


def load_handeye() -> np.ndarray:
    payload = json.loads(HANDEYE.read_text(encoding="utf-8"))
    return np.array(payload["T_tcp_cam"], dtype=float)


def group_transform(group: pathlib.Path) -> np.ndarray | None:
    meta = group / "metadata.jsonl"
    if not meta.exists():
        return None
    first = json.loads(meta.read_text(encoding="utf-8").splitlines()[0])
    return np.array(first["tcp_matrix"], dtype=float).reshape(4, 4)


def load_cloud_base(group: pathlib.Path, t_base_cam: np.ndarray) -> np.ndarray:
    cloud = np.load(group / "cloud.npy").astype(np.float64)
    return cloud @ t_base_cam[:3, :3].T + t_base_cam[:3, 3]


def fit_planes(points: np.ndarray, max_planes: int = 8) -> list[Plane]:
    """迭代 RANSAC 提取主平面（不依赖 open3d，纯 numpy 实现，便于离线复现）。"""
    remaining = points.copy()
    planes: list[Plane] = []
    rng = np.random.default_rng(0)
    for _ in range(max_planes):
        if len(remaining) < MIN_PLANE_POINTS:
            break
        best = None
        for _ in range(300):
            idx = rng.choice(len(remaining), 3, replace=False)
            p0, p1, p2 = remaining[idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal = normal / norm
            offset = -normal @ p0
            dist = np.abs(remaining @ normal + offset)
            inliers = int((dist < PLANE_DIST_M).sum())
            if best is None or inliers > best[0]:
                best = (inliers, normal, offset)
        if best is None or best[0] < MIN_PLANE_POINTS:
            break
        inliers, normal, offset = best
        mask = np.abs(remaining @ normal + offset) < PLANE_DIST_M
        pts = remaining[mask]
        # 用内点做最小二乘精修
        centroid = pts.mean(axis=0)
        _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
        normal = vh[-1]
        if normal @ centroid + offset < 0:      # 统一朝向：让 offset 的符号一致
            normal, offset = -normal, -offset
        offset = -normal @ centroid
        planes.append(Plane(normal=normal, offset=offset, inliers=inliers, centroid=centroid))
        remaining = remaining[~mask]
    return planes


def classify(planes: list[Plane]) -> dict:
    """把平面分成三类锚点（2026-11-26 实测结论驱动）。

    通道沿 **y** 延伸、宽度沿 **x**：
      · 侧壁（法向 ±x）→ 锚定 **x 与 yaw**
      · 端面（法向 ±y）→ 锚定 **y**（这是唯一能压住长轴滑移的锚点）
      · 底面/顶面（法向 ±z）→ 锚定 **z**
    """
    side = [p for p in planes if abs(p.normal[0]) > 0.9 and abs(p.normal[2]) < 0.35]
    ends = [p for p in planes if abs(p.normal[1]) > 0.9 and abs(p.normal[2]) < 0.5]
    horiz = [p for p in planes if abs(p.normal[2]) > 0.6]
    side.sort(key=lambda p: -p.inliers)
    ends.sort(key=lambda p: -p.inliers)
    horiz.sort(key=lambda p: -p.inliers)
    left = right = None
    if len(side) >= 2:
        a, b = side[0], side[1]
        left, right = (a, b) if a.centroid[0] < b.centroid[0] else (b, a)
    return {"left": left, "right": right, "end": ends[0] if ends else None,
            "floor": horiz[0] if horiz else None}


def analyse_group(group: pathlib.Path, t_tcp_cam: np.ndarray,
                  roi_centre: np.ndarray | None) -> dict | None:
    t_base_tcp = group_transform(group)
    if t_base_tcp is None:
        return None
    t_base_cam = t_base_tcp @ t_tcp_cam
    pts = load_cloud_base(group, t_base_cam)
    centre = roi_centre if roi_centre is not None else np.array([1.17, -0.05, 0.25])
    pts = pts[np.all(np.abs(pts - centre) < np.array(ROI_HALF), axis=1)]
    if len(pts) < MIN_PLANE_POINTS:
        return {"group": group.name, "error": f"ROI 内点太少（{len(pts)}）"}
    planes = fit_planes(pts)
    cls = classify(planes)
    out = {"group": group.name, "roi_points": int(len(pts)), "planes": len(planes),
           "plane_summary": [{"n": np.round(p.normal, 3).tolist(), "inliers": p.inliers,
                              "centroid": np.round(p.centroid, 4).tolist()} for p in planes[:6]]}
    left, right, end, floor = cls["left"], cls["right"], cls["end"], cls["floor"]
    if left is None or right is None:
        out["error"] = "没有找到两面侧壁（法向 ±x）"
        return out
    width = abs(float(left.offset) - float(right.offset))
    out["wall_width_m"] = round(width, 4)
    out["wall_parallel_deg"] = round(float(np.degrees(np.arccos(
        np.clip(abs(left.normal @ right.normal), -1, 1)))), 2)
    # y 锚点：端面
    if end is not None:
        out["end_face"] = True
        out["end_y_m"] = round(float(-end.offset / (end.normal @ np.array([0, 1, 0]))), 4)
        out["end_normal"] = np.round(end.normal, 3).tolist()
    else:
        out["end_face"] = False
        out["error_y"] = "没有找到端面（法向 ±y）→ y 无法锚定（这是长轴滑移的根源）"
    if floor is not None:
        out["floor_normal"] = np.round(floor.normal, 3).tolist()
        out["floor_z_m"] = round(float(-floor.offset / floor.normal[2]), 4)
    # 综合锚定位姿：x 取两壁中点、y 取端面、z 取底面
    anchor = {}
    if left is not None and right is not None:
        anchor["x_m"] = round(float(-(left.offset + right.offset) / (
            left.normal[0] + right.normal[0]) if (left.normal[0] + right.normal[0]) else
            (left.centroid[0] + right.centroid[0]) / 2), 4)
    if end is not None:
        anchor["y_m"] = out["end_y_m"]
    if floor is not None:
        anchor["z_m"] = out["floor_z_m"]
    out["anchor"] = anchor
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", required=True, type=pathlib.Path)
    parser.add_argument("--roi-centre", nargs=3, type=float, default=None,
                        help="ROI 中心（base 系，米）；默认 [1.17,-0.05,0.25]")
    parser.add_argument("--save-json", type=pathlib.Path, default=None)
    args = parser.parse_args()
    if not args.dataset.is_dir():
        print(f"数据集不存在：{args.dataset}")
        return 2
    t_tcp_cam = load_handeye()
    roi = np.array(args.roi_centre) if args.roi_centre else None
    groups = sorted(g for g in args.dataset.glob("g*") if g.is_dir())
    print(f"数据集 {args.dataset.name}：{len(groups)} 组；ROI 半边长 {ROI_HALF} m")
    results = []
    for group in groups:
        res = analyse_group(group, t_tcp_cam, roi)
        if res is None:
            print(f"  {group.name}: 缺 metadata.jsonl，跳过")
            continue
        results.append(res)
        if "error" in res:
            print(f"  {group.name}: {res['error']}（ROI {res.get('roi_points')} 点，"
                  f"{res.get('planes')} 平面）")
            continue
        a = res.get("anchor", {})
        y_txt = f"{a['y_m']*1000:8.1f}" if "y_m" in a else "   无  "
        x_txt = f"{a['x_m']*1000:8.1f}" if "x_m" in a else "   无  "
        z_txt = f"{a['z_m']*1000:8.1f}" if "z_m" in a else "   无  "
        print(f"  {group.name}: 壁间距 {res['wall_width_m']*1000:6.1f} mm｜平行度 "
              f"{res['wall_parallel_deg']:5.2f}°｜锚点 x{x_txt} y{y_txt} z{z_txt} mm"
              + ("" if res.get("end_face") else "  ← 端面缺失，y 未锚定"))
    if len(results) >= 2:
        mids = [np.array([r["anchor"].get(k, np.nan) for k in ("x_m", "y_m", "z_m")])
                for r in results if "anchor" in r]
        widths = [r["wall_width_m"] for r in results if "wall_width_m" in r]
        if len(mids) >= 2:
            arr = np.array(mids, dtype=float)
            with np.errstate(invalid="ignore"):
                spread = np.nanmax(np.abs(arr - np.nanmean(arr, axis=0)), axis=0) * 1000
            print(f"\n组间锚点最大偏差: x {spread[0]:.1f} / y {spread[1]:.1f} / z {spread[2]:.1f} mm"
                  "   ← 关键看 y（通道长轴）")
        if widths:
            print(f"壁间距: 均值 {np.mean(widths)*1000:.1f} mm，极差 {(max(widths)-min(widths))*1000:.1f} mm")
    if args.save_json:
        args.save_json.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"结果已写：{args.save_json}")
    return 0 if any("anchor" in r and r["anchor"] for r in results) else 4


if __name__ == "__main__":
    raise SystemExit(main())
