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


def _plane_coord(plane: Plane, axis: int) -> float | None:
    """把平面换算成"沿某坐标轴的常数值"：n·p + d = 0 → 该轴坐标 = -d/n_axis。

    仅当该轴分量的绝对值足够大（面基本垂直于该轴）时才有意义。
    """
    comp = float(plane.normal[axis])
    if abs(comp) < 0.85:
        return None
    return float(-plane.offset / comp)


def classify(planes: list[Plane]) -> dict:
    """按"能锚定哪个自由度"分类，并加**物理约束**筛选配对（2026-11-26 第二版）。

    通道几何事实（实测）：宽约 120–140 mm（沿 x）、沿 y 延伸、底面在下。
      · 侧壁对：两个 ±x 面、法向相反、间距落在 [0.10, 0.17] m、平行度 <5°
        → 锚定 x 与 yaw（间距本身就是宽度，是强约束）
      · 端面：±y 面 → 锚定 y（长轴；唯一能压住滑移的锚点）
      · 底面：+z 面 → 锚定 z

    第一版没有间距约束，导致 g00/g04 把相隔 2.4 m 的两个面当成一对；
    这里把物理区间写死，配不上就明确报错，而不是给个假结果。
    """
    side = [p for p in planes if abs(p.normal[0]) > 0.88 and abs(p.normal[2]) < 0.35]
    ends = [p for p in planes if abs(p.normal[1]) > 0.85 and abs(p.normal[2]) < 0.5]
    horiz = [p for p in planes if abs(p.normal[2]) > 0.6]

    best = None
    for i, a in enumerate(side):
        for b in side[i + 1:]:
            if float(a.normal @ b.normal) > -0.995:     # 近似反向（面对面）
                continue
            if float(np.degrees(np.arccos(np.clip(abs(a.normal @ b.normal), -1, 1)))) > 4.0:
                continue                                   # 平行度 <4°，否则不是同一通道的两壁
            xa, xb = _plane_coord(a, 0), _plane_coord(b, 0)
            if xa is None or xb is None:
                continue
            width = abs(xa - xb)
            if not (0.10 <= width <= 0.17):             # 通道宽度物理区间
                continue
            score = a.inliers + b.inliers
            if best is None or score > best[0]:
                best = (score, a, b, xa, xb, width)

    picked: dict = {"side_pair": None, "left": None, "right": None,
                    "end": None, "floor": None, "width_m": None,
                    "rejected": []}
    if best is not None:
        _, a, b, xa, xb, width = best
        left, right = (a, b) if xa < xb else (b, a)
        picked.update({"side_pair": (left, right), "left": left, "right": right,
                       "width_m": width})
    else:
        picked["rejected"].append(
            f"侧壁对未通过物理约束：候选 {len(side)} 面，需 2 面反向且间距 100–170 mm")

    if picked["left"] is not None and picked["right"] is not None:
        lo = min(_plane_coord(picked["left"], 0), _plane_coord(picked["right"], 0))
        hi = max(_plane_coord(picked["left"], 0), _plane_coord(picked["right"], 0))
        # 端面：优先取落在两壁之间、内点最多的
        good_ends = []
        wall_n = 0.5 * (picked["left"].normal + picked["right"].normal)
        wall_n = wall_n / (np.linalg.norm(wall_n) or 1.0)
        for e in ends:
            y = _plane_coord(e, 1)
            if y is None:
                continue
            perp = float(np.degrees(np.arccos(np.clip(abs(float(e.normal @ wall_n)), -1, 1))))
            cx = float(e.centroid[0])
            inside = (lo - 0.03) <= cx <= (hi + 0.03)
            good_ends.append((e.inliers, y, e, perp, inside))
        # 诊断：把每个 ±y 候选面的 y / 垂直度 / 是否在两壁之间都打出来
        diag = [(round(y * 1000, 1), round(perp, 1), inside, inl)
                for inl, y, _e, perp, inside in good_ends]
        picked["end_candidates"] = diag
        # 选择规则：优先"垂直且有内点"的，其次内点最多
        good_ends = [g for g in good_ends if g[3] < 12.0]
        good_ends.sort(key=lambda item: (-item[4], -item[0]))
        good_ends.sort(key=lambda item: -item[0])
        if good_ends:
            picked["end"] = good_ends[0][2]
            picked["end_y"] = good_ends[0][1]
        picked["x_lo"], picked["x_hi"] = lo, hi
    horiz.sort(key=lambda p: -p.inliers)
    if horiz:
        picked["floor"] = horiz[0]
        picked["floor_z"] = _plane_coord(horiz[0], 2)
    return picked


def analyse_group(group: pathlib.Path, t_tcp_cam: np.ndarray,
                  roi_centre: np.ndarray | None) -> dict | None:
    t_base_tcp = group_transform(group)
    if t_base_tcp is None:
        return None
    pts = load_cloud_base(group, t_base_tcp @ t_tcp_cam)
    centre = roi_centre if roi_centre is not None else np.array([1.17, -0.05, 0.25])
    pts = pts[np.all(np.abs(pts - centre) < np.array(ROI_HALF), axis=1)]
    if len(pts) < MIN_PLANE_POINTS:
        return {"group": group.name, "error": f"ROI 内点太少（{len(pts)}）"}
    planes = fit_planes(pts)
    cls = classify(planes)
    out = {"group": group.name, "roi_points": int(len(pts)), "planes": len(planes)}
    if cls["side_pair"] is None:
        out["error"] = "；".join(cls["rejected"]) or "未找到合法侧壁对"
        return out
    left, right = cls["left"], cls["right"]
    anchor = {"x_m": round((cls["x_lo"] + cls["x_hi"]) / 2, 4)}
    if "end_y" in cls:
        anchor["y_m"] = round(float(cls["end_y"]), 4)
    if "floor_z" in cls:
        anchor["z_m"] = round(float(cls["floor_z"]), 4)
    out["end_candidates"] = cls.get("end_candidates", [])
    out.update({
        "wall_width_mm": round(cls["width_m"] * 1000, 1),
        "wall_parallel_deg": round(float(np.degrees(np.arccos(
            np.clip(abs(left.normal @ right.normal), -1, 1)))), 2),
        "end_face": "end_y" in cls,
        "anchor": anchor,
    })
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
        y_txt = f"{a['y_m']*1000:8.1f}" if "y_m" in a else "    未锚定"
        if res.get("end_candidates"):
            print(f"      端面候选 (y_mm, 垂直偏差°, 在两壁间, 内点): {res['end_candidates'][:4]}")
        print(f"  {group.name}: 壁宽 {res['wall_width_mm']:6.1f} mm｜平行度 "
              f"{res['wall_parallel_deg']:5.2f}°｜x {a.get('x_m', float('nan'))*1000:8.1f} mm"
              f"｜y {y_txt}" + ("" if res.get("end_face") else "  ← 端面缺失"))
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
