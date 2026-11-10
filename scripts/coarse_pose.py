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


def reject_with_pose(points: np.ndarray, transformation: np.ndarray,
                     cad_points: np.ndarray, threshold_m: float = 0.025) -> np.ndarray:
    """用当前位姿把"不像工件"的点剔掉：点到 CAD 的最近距离超过阈值的丢掉。

    rotp90 那类朝向里，深度窗内的最大连通域会和桌面/夹具连成一片（extent 917 mm，
    而工件只有 455 mm）。用 CAD 反过来筛点，比继续调深度窗更稳。
    """
    from scipy.spatial import cKDTree
    tree = cKDTree(cad_points)
    points_obj = (points - transformation[:3, 3]) @ transformation[:3, :3]
    distance, _ = tree.query(points_obj, k=1)
    return points[distance < threshold_m]


def analyse_group(directory: pathlib.Path, template: tuple | None = None,
                  cad_points: np.ndarray | None = None) -> dict:
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
    if template is not None:
        pose = estimate_pose(chosen, points, template[0], template[1], corner)
        result["pose_score"] = round(pose["score"], 3)
        result["pose"] = {
            "rotation": pose["rotation"].round(6).tolist(),
            "translation": [round(float(v), 5) for v in pose["translation"]],
            "order": list(pose["order"]),
            "signs": list(pose["signs"]),
        }
        if cad_points is not None and pose.get("candidates"):
            refined = cad_register(points, pose["candidates"], cad_points)
            for _ in range(2):                      # 用位姿筛点、重拟合、再配准（两轮收敛）
                if not refined:
                    break
                kept = reject_with_pose(points, refined["transformation"], cad_points)
                if len(kept) < MIN_POINTS:
                    break
                planes2 = dedup_planes(segment_planes(kept))
                if len(planes2) < 3:
                    break
                triple2 = choose_triple(planes2)
                if triple2 is None:
                    break
                chosen2 = triple2[0]
                corner2, _ = intersect(chosen2)
                pose2 = estimate_pose(chosen2, kept, template[0], template[1], corner2)
                if not pose2.get("candidates"):
                    break
                refined2 = cad_register(kept, pose2["candidates"], cad_points)
                if not refined2:
                    break
                refined = refined2
                points = kept
            matrix = refined["transformation"]
            result["pose"] = {
                "rotation": np.asarray(matrix[:3, :3]).round(6).tolist(),
                "translation": [round(float(v), 5) for v in matrix[:3, 3]],
                "source": "cad_icp",
            }
            result["pose_fitness"] = round(refined["fitness"], 3)
            result["pose_rmse_mm"] = round(refined["rmse_mm"], 2)
            result["points_kept"] = int(len(points))
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


def load_template_bbox(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray]:
    mesh = o3d.io.read_triangle_mesh(str(path))
    vertices = np.asarray(mesh.vertices)
    return vertices.min(axis=0), vertices.max(axis=0)


def estimate_pose(planes: list[dict], cloud: np.ndarray, bbox_min: np.ndarray,
                  bbox_max: np.ndarray, corner_cam: np.ndarray,
                  margin_m: float = 0.006) -> dict:
    """把"三个可见面 + 角点"变成 T_camera_object。

    三平面只给出姿态和交点；交点对应 CAD bbox 的哪一个角，取决于哪三个面可见。
    这里穷举 3! 种轴指派 × 2^3 种正负号（只保留右手系），对每种组合：
      1. 旋转 R 由三个面法向按指派方向排列；
      2. 平移 t = 角点 - R·bbox 对应角点；
      3. 打分 = 工件点云落在 bbox（外扩 margin）内的比例。
    取分数最高的组合。手套形状/扶手带来的 90° 歧义（455 vs 465 mm 很接近）会被记录下来。
    """
    from itertools import permutations, product
    best = None
    candidates: list[dict] = []
    for order in permutations(range(3)):
        for signs in product((1, -1), repeat=3):
            columns = [planes[order[i]]["normal"] * signs[i] for i in range(3)]
            rotation = np.column_stack(columns)
            if np.linalg.det(rotation) < 0:
                continue
            corner_obj = np.array([bbox_max[i] if signs[i] > 0 else bbox_min[i]
                                   for i in range(3)])
            translation = corner_cam - rotation @ corner_obj
            points_obj = (cloud - translation) @ rotation          # 行向量形式
            inside = np.all(points_obj >= bbox_min - margin_m, axis=1) & \
                     np.all(points_obj <= bbox_max + margin_m, axis=1)
            # 视角约束：相机看到的是"离镜头最近的那个角"。把 8 个 bbox 角变换到相机系，
            # 真正的解必须让观测角点正好是最靠近相机原点的那个——这条能消掉 90°/180° 翻转。
            corners = np.array([[bbox_max[0] if bx else bbox_min[0],
                                 bbox_max[1] if by else bbox_min[1],
                                 bbox_max[2] if bz else bbox_min[2]]
                                for bx in (0, 1) for by in (0, 1) for bz in (0, 1)])
            corners_cam = corners @ rotation.T + translation
            nearest = int(np.argmin(np.linalg.norm(corners_cam, axis=1)))
            observed_index = int(np.argmin(np.linalg.norm(corners - corner_obj, axis=1)))
            view_ok = nearest == observed_index
            score = float(inside.mean()) * (1.0 if view_ok else 0.2)
            candidate = {"score": score, "order": order, "signs": signs,
                         "rotation": rotation, "translation": translation,
                         "corner_obj": corner_obj}
            candidates.append(candidate)
            if best is None or score > best["score"]:
                best = candidate
    candidates.sort(key=lambda item: item["score"], reverse=True)
    if best is not None:
        best["candidates"] = candidates[:8]
    return best


def cad_register(cloud: np.ndarray, candidates: list[dict], cad_points: np.ndarray) -> dict:
    """用 CAD 形状（含扶手）把 bbox 拟合的歧义解开。

    纯 bbox 拟合对"近立方的工件"天生有 24 种等价解（455/465 mm 只差 2%）；
    扶手的凸起是唯一能把它们分开的形状信息，所以这里对每个候选做一次 ICP，
    用"观测点云到 CAD 的距离"挑最好的那个。
    """
    observed = o3d.geometry.PointCloud()
    observed.points = o3d.utility.Vector3dVector(cloud)
    observed = observed.voxel_down_sample(0.006)
    observed.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(cad_points)

    best = None
    for candidate in candidates:
        init = np.eye(4)
        init[:3, :3] = candidate["rotation"]
        init[:3, 3] = candidate["translation"]
        try:
            result = o3d.pipelines.registration.registration_icp(
                template, observed, 0.03, init,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
        except Exception:                       # noqa: BLE001
            continue
        # 选择指标要看"观测点云有多少落在 CAD 上"（覆盖率），而不是反过来：
        # 只看到 3 个面时，CAD→观测的 fitness 上限本来就只有 ~0.5。
        # evaluate_registration 把 transformation 作用在 source 上：
        # source=观测（相机系）、target=CAD（对象系），所以要传 T_object_camera = inv(T_camera_object)
        evaluation = o3d.pipelines.registration.evaluate_registration(
            observed, template, 0.010, np.linalg.inv(result.transformation))
        entry = {"transformation": result.transformation,
                 "fitness": float(evaluation.fitness),
                 "rmse_mm": float(evaluation.inlier_rmse) * 1000.0,
                 "bbox_score": candidate["score"]}
        if best is None or (entry["fitness"], -entry["rmse_mm"]) > (best["fitness"], -best["rmse_mm"]):
            best = entry
    return best or {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default=None, help="vision_dataset 目录（含 g*/）")
    parser.add_argument("--group", default=None, help="只分析一个组目录")
    parser.add_argument("--template", default=None, help="CAD STL（用于尺寸约束与形状消歧）")
    parser.add_argument("--cad-points", type=int, default=0,
                        help="用 CAD 采样这么多点做 ICP 消歧（0 = 只用 bbox）")
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

    template = None
    cad_points = None
    if args.template:
        path = pathlib.Path(args.template).expanduser()
        if path.exists():
            bbox_min, bbox_max = load_template_bbox(path)
            template = (bbox_min, bbox_max)
            if args.cad_points:
                mesh = o3d.io.read_triangle_mesh(str(path))
                cad_points = np.asarray(
                    mesh.sample_points_uniformly(args.cad_points).points)
            print(f"模板 {path.name}: 外形 {np.round((bbox_max - bbox_min) * 1000).astype(int)} mm；"
                  f"原点相对 bbox 中心 {np.round((bbox_min + bbox_max) / 2 * 1000, 1)} mm")
        else:
            print(f"模板不存在：{path}（继续，只做三平面，不出位姿）")

    results = []
    print(f"{'组':<14}{'面数':>4}{'工件点数':>9}{'内点':>22}{'正交':>7}{'cond':>7}"
          f"{'打分':>8}{'角点(mm)':>26}")
    print("-" * 88)
    for directory in directories:
        result = analyse_group(directory, template, cad_points)
        results.append(result)
        if result.get("error"):
            print(f"{result['group']:<14}{'':>4}  ✗ {result['error']}")
            continue
        inliers = ",".join(str(v) for v in result["inliers"])
        corner = ",".join(f"{v:+.1f}" for v in result["corner_mm"])
        if "pose_fitness" in result:
            score = f"{result['pose_fitness']:.2f}/{result['pose_rmse_mm']:.1f}"
        elif "pose_score" in result:
            score = f"{result['pose_score']:.3f}"
        else:
            score = "  -  "
        print(f"{result['group']:<14}{result['planes']:>4}{result.get('area_px', 0):>9}"
              f"{inliers:>22}{result['orthogonality']:>7.3f}{result['condition']:>7.1f}"
              f"{score:>8}  [{corner}]")

    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"\n结果写入 {args.out}")
    good = [r for r in results if not r.get("error")]
    print(f"\n成功 {len(good)}/{len(results)} 组")
    return 0 if good else 1


if __name__ == "__main__":
    raise SystemExit(main())
