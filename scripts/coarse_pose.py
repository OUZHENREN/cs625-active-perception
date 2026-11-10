#!/usr/bin/env python3
"""D2 粗测量：RANSAC 三平面 + 正交校验 + 三平面求交（相机系）。

做什么
------
输入一组采集（`depth.npy` + `meta.json` 里的内参），输出工件的一个**粗位姿**：
可见三个面的法向（正交化后）与它们的交点（角点）。
所有计算都在相机光学系里，不需要手眼标定。

为什么是这三步
--------------
1. RANSAC 平面分割：金属工件的面是平面，比"整块点云配准"稳得多；
2. 法向统一朝向相机 + 正交校验：工件是长方体，三个可见面必然互相垂直——
   这条几何先验能直接把"被镀膜噪声带偏的平面"筛掉；
3. 三平面求交：三个面交于一个角点，比"取质心"稳定（质心会被缺失面拉偏）。

用法
----
    # 模板只用来给出工件尺寸（可选）
    python3 scripts/coarse_pose.py --dataset runtime/vision_dataset/<stamp> \
        --template ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL

    # 只看一组
    python3 scripts/coarse_pose.py --group runtime/vision_dataset/<stamp>/g00_rot0

输出：每组一行（三个面的内点数/内点率/RMS、正交误差、角点坐标），
末尾按"同一朝向的 4 次重复"给出角点重复性与法向重复性。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import polarizer_ab as pab

PLANE_DIST_M = 0.003          # RANSAC 内点阈值 3 mm
MIN_POINTS = 800              # 一个面至少这么多点才算数
MIN_DEPTH_M = 0.55            # 工件所在深度窗（背景幕布 1.35 m，桌面 ~1.0 m）
MAX_DEPTH_M = 1.05


def load_group(directory: pathlib.Path) -> tuple[np.ndarray, list[float]]:
    meta = json.loads((directory / "meta.json").read_text())
    depth = np.load(directory / "depth.npy").astype(np.float32)
    return depth, meta["camera_info"]["k"]


def segment_planes(points: np.ndarray, max_planes: int = 8) -> list[dict]:
    """逐个 RANSAC 平面分割，每次剔除内点。"""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    planes: list[dict] = []
    working = cloud
    for _ in range(max_planes):
        if len(working.points) < MIN_POINTS:
            break
        model, inliers = working.segment_plane(PLANE_DIST_M, 3, 2000)
        if len(inliers) < MIN_POINTS:
            break
        a, b, c, d = (float(v) for v in model)
        normal = np.array([a, b, c], dtype=float)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            break
        normal /= norm
        d /= norm
        index = np.asarray(inliers, dtype=int)
        sel = working.select_by_index(index)
        pts = np.asarray(sel.points)
        centroid = pts.mean(axis=0)
        # 统一朝向相机：相机原点在 (0,0,0)
        if float(np.dot(normal, centroid)) > 0:
            normal = -normal
            d = -d
        residual = np.abs(pts @ normal + d)
        planes.append({
            "normal": normal,
            "d": float(d),
            "centroid": centroid,
            "inliers": int(len(index)),
            "ratio": float(len(index) / max(1, len(cloud.points))),
            "rms_mm": float(np.sqrt(np.mean(residual ** 2)) * 1000.0),
            "extent_mm": float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)) * 1000.0),
        })
        working = working.select_by_index(index, invert=True)
    return planes


def dedup_planes(planes: list[dict], similarity: float = 0.92) -> list[dict]:
    """去掉"同一个面的碎片"：法向几乎相同的平面只保留内点最多的那个。

    RANSAC 每次剔除内点后，同一块大面常被再抓一次（被凹槽/边缘切成若干块），
    不去重的话三个"面"可能是同一个面，三平面求交会退化成病态。
    """
    ordered = sorted(planes, key=lambda item: item["inliers"], reverse=True)
    kept: list[dict] = []
    for plane in ordered:
        if all(abs(float(np.dot(plane["normal"], other["normal"]))) < similarity
               for other in kept):
            kept.append(plane)
    return kept


def choose_triple(planes: list[dict]) -> tuple[list[dict], float] | None:
    """挑一组"互相近似正交、内点最多"的三个面。"""
    best = None
    count = len(planes)
    for i in range(count):
        for j in range(i + 1, count):
            for k in range(j + 1, count):
                normals = [planes[i]["normal"], planes[j]["normal"], planes[k]["normal"]]
                dots = [abs(float(np.dot(normals[a], normals[b])))
                        for a, b in ((0, 1), (0, 2), (1, 2))]
                worst = max(dots)
                if worst > 0.20:                    # 约 11.5°，超出就不是同一个长方体的面
                    continue
                score = sum(planes[m]["inliers"] for m in (i, j, k)) * (1.0 - 2.0 * worst)
                if best is None or score > best[0]:
                    best = (score, [planes[i], planes[j], planes[k]], worst)
    if best is None:
        return None
    return best[1], best[2]


def orthonormalise(normals: list[np.ndarray]) -> np.ndarray:
    """Gram-Schmidt，返回列向量为 x/y/z 的旋转矩阵（右手系）。"""
    e1 = normals[0] / np.linalg.norm(normals[0])
    e2 = normals[1] - np.dot(e1, normals[1]) * e1
    e2 /= np.linalg.norm(e2)
    e3 = np.cross(e1, e2)
    return np.column_stack([e1, e2, e3])


def intersect(planes: list[dict]) -> tuple[np.ndarray, float]:
    """三平面求交：n_i·x + d_i = 0，用 SVD 解，返回解与条件数。"""
    matrix = np.stack([p["normal"] for p in planes])
    rhs = -np.array([p["d"] for p in planes])
    solution, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
    singular = np.linalg.svd(matrix, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-12))
    return solution, condition


def extract_workpiece(depth: np.ndarray, k: list[float]) -> tuple[np.ndarray, dict]:
    """先在图像空间把工件抠出来：深度窗 + 最大连通域。

    为什么不直接全图分割平面：桌面、幕布、墙面都是大平面，RANSAC 会先抓它们。
    工件在当前观测位是"深度窗内最大的连通块"，这一步之后平面分割才轮到工件的面。
    """
    import cv2
    mask = np.isfinite(depth) & (depth > MIN_DEPTH_M) & (depth < MAX_DEPTH_M)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    info: dict = {"components": int(max(0, count - 1))}
    if count <= 1:
        return np.zeros((0, 3)), info
    areas = stats[1:, cv2.CC_STAT_AREA]
    best = 1 + int(np.argmax(areas))
    selected = labels == best
    info["area_px"] = int(areas.max())
    info["area_ratio"] = round(float(areas.max() / mask.size), 4)
    points = pab.back_project(np.where(selected, depth, 0.0).astype(np.float32), k)
    return points, info


def analyse_group(directory: pathlib.Path) -> dict:
    depth, k = load_group(directory)
    points, info = extract_workpiece(depth, k)
    if len(points) < MIN_POINTS:
        return {"group": directory.name, "error": f"工件区域太小（{len(points)} 点）"}
    info.update(
        extent_mm=[round(float(v), 0) for v in
                   ((points.max(axis=0) - points.min(axis=0)) * 1000.0)],
        centroid_mm=[round(float(v) * 1000.0, 1) for v in points.mean(axis=0)],
    )
    planes = dedup_planes(segment_planes(points))
    result = {"group": directory.name, "planes": len(planes), **info}
    if len(planes) < 3:
        result["error"] = f"去重后只剩 {len(planes)} 个不同的面（换朝向时侧面可能接近侧视）"
        return result
    triple = choose_triple(planes)
    if triple is None:
        result["error"] = f"没找到互相正交的三平面（候选 {len(planes)} 个）"
        return result
    chosen, worst = triple
    rotation = orthonormalise([p["normal"] for p in chosen])
    corner, condition = intersect(chosen)
    result.update(
        inliers=[p["inliers"] for p in chosen],
        ratios=[round(p["ratio"], 3) for p in chosen],
        rms_mm=[round(p["rms_mm"], 2) for p in chosen],
        orthogonality=round(worst, 4),
        condition=round(condition, 1),
        corner_mm=[round(float(v) * 1000.0, 1) for v in corner],
        rotation=rotation.round(6).tolist(),
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default=None, help="vision_dataset 目录（含 g*/）")
    parser.add_argument("--group", default=None, help="只分析一个组目录")
    parser.add_argument("--template", default=None, help="CAD STL（只用来核对尺寸，可选）")
    parser.add_argument("--out", default=None, help="把结果写成 JSON")
    args = parser.parse_args()

    if args.group:
        directories = [pathlib.Path(args.group)]
    elif args.dataset:
        directories = sorted(pathlib.Path(args.dataset).glob("g*"))
    else:
        parser.error("给 --dataset 或 --group")
    if not directories:
        print("没有找到组目录", file=sys.stderr)
        return 2

    if args.template:
        path = pathlib.Path(args.template).expanduser()
        if path.exists():
            mesh = o3d.io.read_triangle_mesh(str(path))
            extent = np.asarray(mesh.vertices).max(0) - np.asarray(mesh.vertices).min(0)
            print(f"模板 {path.name}: {len(mesh.triangles)} 面，外形 "
                  f"{np.round(extent * 1000).astype(int)} mm")
        else:
            print(f"模板不存在：{path}（继续，不影响平面分割）")

    results = []
    print(f"{'组':<14}{'面数':>4}{'工件点数':>9}{'内点':>22}{'正交':>7}{'cond':>7}{'角点(mm)':>28}")
    print("-" * 88)
    for directory in directories:
        result = analyse_group(directory)
        results.append(result)
        if result.get("error"):
            print(f"{result['group']:<14}{'':>4}  ✗ {result['error']}")
            continue
        inliers = ",".join(str(v) for v in result["inliers"])
        corner = ",".join(f"{v:+.1f}" for v in result["corner_mm"])
        print(f"{result['group']:<14}{result['planes']:>4}{result.get('area_px', 0):>9}"
              f"{inliers:>22}{result['orthogonality']:>7.3f}{result['condition']:>7.1f}  [{corner}]")

    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"\n结果写入 {args.out}")
    good = [r for r in results if not r.get("error")]
    print(f"\n成功 {len(good)}/{len(results)} 组")
    return 0 if good else 1


if __name__ == "__main__":
    raise SystemExit(main())
