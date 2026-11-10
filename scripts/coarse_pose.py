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
import time

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import polarizer_ab as pab

PLANE_DIST_M = 0.003          # RANSAC 内点阈值 3 mm
MIN_POINTS = 800              # 一个面至少这么多点才算数
POSE_MIN_COVERAGE = 0.70      # 位姿置信门槛：观测点落在 CAD 附近的比例（低于它标"不可信"）
CORNER_ANCHOR_MIN_INLIERS = 5000  # 最弱面少于这么多点时不拿交点锚定位置（交点会抖 10+ mm）
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


def focus_crop(points: np.ndarray, T_base_cam: np.ndarray,
               centre: np.ndarray, radius: float = 0.35) -> np.ndarray:
    """只保留"已知工件中心附近"的点（base 系球裁）。

    近距观测时深度窗里会同时出现桌面、墙、夹具，连通域和平面剔除都分不干净；
    但我们其实知道工件大致在哪（上一帧的测量结果），按球裁最省事、最可靠。
    """
    base_points = (T_base_cam[:3, :3] @ points.T).T + T_base_cam[:3, 3]
    keep = np.linalg.norm(base_points - np.asarray(centre), axis=1) < radius
    if keep.sum() >= MIN_POINTS:
        print(f"   [focus] 球裁 {np.round(centre,2)} r={radius} m：保留 {int(keep.sum())} 点", flush=True)
        return points[keep]
    print("   [focus] 球内点太少，跳过", flush=True)
    return points


def remove_table(points: np.ndarray, T_base_cam: np.ndarray,
                 max_angle_deg: float = 15.0, thickness_m: float = 0.015,
                 attempts: int = 6) -> np.ndarray:
    """在 base 系里剔掉桌面：找法向接近竖直的大平面，删掉它附近的点。

    为什么必须在 base 系做：相机是斜着装的，"桌面"在相机系里没有简单的法向特征；
    但在 base 系里桌面法向就是 ±z。这一步依赖已经验明的手眼（2026-11-10）。

    注意：RANSAC 第一次通常抓到的是墙/幕布（竖直大平面），不能一看到"不是桌面"就退出，
    要继续往下找（实测：面0 是墙 34 万点，面1 才是桌面 15.7 万点、高度 −0.110 m）。
    """
    if len(points) < MIN_POINTS:
        return points
    base_points = (T_base_cam[:3, :3] @ points.T).T + T_base_cam[:3, 3]
    remaining = base_points.copy()
    for _ in range(attempts):
        if len(remaining) < MIN_POINTS:
            break
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(remaining)
        model, inliers = cloud.segment_plane(distance_threshold=0.010,
                                             ransac_n=3, num_iterations=600)
        normal = np.array(model[:3])
        normal /= np.linalg.norm(normal)
        plane_points = remaining[inliers]
        if abs(normal[2]) >= np.cos(np.radians(max_angle_deg)) and len(inliers) >= 20000:
            height = float(np.mean(plane_points[:, 2]))
            distance = np.abs(base_points[:, 2] - height)
            keep = distance > thickness_m
            print(f"   [table] 桌面在 base z={height:.3f} m，剔除 {int((~keep).sum())} 点"
                  f"（剩 {int(keep.sum())}）", flush=True)
            return points[keep]
        remaining = np.delete(remaining, inliers, axis=0)
    print("   [table] 没找到近水平的大平面，跳过剔除", flush=True)
    return points


def filter_planes(planes: list[dict], centre: np.ndarray, max_distance_m: float = 0.40,
                  min_inliers: int = 1500, max_rms_mm: float = 3.0) -> list[dict]:
    """只留下"像工件外表面"的面。

    rotp60 的失败原因：加工件和夹具在深度图里连成一片，夹具的大平面也被当成候选面，
    于是两平面定向搜索选了一个错误的组合。判据很朴素——面要够大、够平、且离工件
    中心足够近（工件最大边长 455 mm，半对角线约 0.33 m）。
    """
    kept = []
    for plane in planes:
        if plane["inliers"] < min_inliers:
            continue
        if plane["rms_mm"] > max_rms_mm:
            continue
        if float(np.linalg.norm(plane["centroid"] - centre)) > max_distance_m:
            continue
        kept.append(plane)
    return kept


def _coverage(points: np.ndarray, cad_points: np.ndarray, transformation: np.ndarray,
              threshold_m: float = 0.010) -> float:
    observed = _prepared_cloud(points, 0.003)
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(cad_points)
    evaluation = o3d.pipelines.registration.evaluate_registration(
        observed, template, threshold_m, np.linalg.inv(transformation))
    return float(evaluation.fitness)


def anchor_on_corner(transformation: np.ndarray, planes: list[dict],
                    corner_cam: np.ndarray, bbox_min: np.ndarray,
                    bbox_max: np.ndarray) -> tuple[np.ndarray, int]:
    """用三平面角点锚定位置，ICP 只保留姿态。

    为什么：ICP 在近立方体上会把平移推进不同的浅局部极小（实测 rot0/rotp30 重复性 11/25 mm），
    而三平面交点的重复性是 2–7 mm。所以姿态用 ICP 的，平移用角点反解。
    返回 (新的 4×4, 所用角点在 bbox 里的编号)。
    """
    from itertools import permutations, product
    rotation = transformation[:3, :3]
    best = None
    for order in permutations(range(3)):
        for signs in product((1, -1), repeat=3):
            columns = [planes[order[i]]["normal"] * signs[i] for i in range(3)]
            candidate_rotation = np.column_stack(columns)
            if np.linalg.det(candidate_rotation) < 0:
                continue
            trace = float(np.trace(rotation.T @ candidate_rotation))
            corner_obj = np.array([bbox_max[i] if signs[i] > 0 else bbox_min[i]
                                   for i in range(3)])
            if best is None or trace > best[0]:
                best = (trace, corner_obj, signs)
    if best is None:
        return transformation, -1
    _, corner_obj, signs = best
    result = transformation.copy()
    result[:3, 3] = corner_cam - rotation @ corner_obj
    index = int(np.sum([1 << i for i in range(3) if signs[i] > 0]))
    return result, index


def refine_rotation(transformation: np.ndarray, planes: list[dict],
                    bbox_min: np.ndarray, bbox_max: np.ndarray) -> np.ndarray:
    """用三个面的法向把姿态"拧正"：把观测法向对齐到对象轴上。

    ICP 会让姿态在浅局部极小里差几度；平面法向是更硬的约束（三面互orthogonal），
    所以最后用它做一次小角度修正。
    """
    rotation = transformation[:3, :3].copy()
    corrections = []
    for plane in planes:
        normal_obj = rotation.T @ plane["normal"]
        axis = int(np.argmax(np.abs(normal_obj)))
        target = np.zeros(3)
        target[axis] = np.sign(normal_obj[axis]) or 1.0
        # 观测法向 -> 目标轴 的最小旋转
        current = normal_obj / np.linalg.norm(normal_obj)
        cross = np.cross(current, target)
        dot = float(np.clip(np.dot(current, target), -1.0, 1.0))
        angle = np.arctan2(np.linalg.norm(cross), dot)
        if np.linalg.norm(cross) > 1e-9:
            axis_cam = rotation @ (cross / np.linalg.norm(cross))
            corrections.append(R.from_rotvec(axis_cam * angle).as_matrix())
    if not corrections:
        return transformation
    average = np.mean(corrections, axis=0)
    # 重新正交化
    u, _, vt = np.linalg.svd(average @ rotation)
    fixed = u @ vt
    result = transformation.copy()
    result[:3, :3] = fixed
    return result


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


def extract_workpiece(depth: np.ndarray, k: list[float],
                      min_depth: float = MIN_DEPTH_M,
                      max_depth: float = MAX_DEPTH_M,
                      expected_extent=None) -> tuple[np.ndarray, dict]:
    """先在图像空间把工件抠出来：深度窗 + 最大连通域。

    为什么不直接全图分割平面：桌面、幕布、墙面都是大平面，RANSAC 会先抓它们。
    工件在当前观测位是"深度窗内最大的连通块"，这一步之后平面分割才轮到工件的面。
    """
    import cv2
    mask = np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    info: dict = {"components": int(max(0, count - 1))}
    if count <= 1:
        return np.zeros((0, 3)), info
    areas = stats[1:, cv2.CC_STAT_AREA]
    best = 1 + int(np.argmax(areas))       # 用最大连通域，再由 --remove-table 剔除桌面
    selected = labels == best
    info["area_px"] = int(stats[best, cv2.CC_STAT_AREA])
    info["area_ratio"] = round(float(stats[best, cv2.CC_STAT_AREA] / mask.size), 4)
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
                  cad_points: np.ndarray | None = None,
                  depth_window: tuple[float, float] | None = None,
                  T_base_cam: np.ndarray | None = None,
                  focus: tuple[np.ndarray, float] | None = None) -> dict:
    depth, k = load_group(directory)
    expected = None
    if template is not None:
        expected = np.asarray(template[1]) - np.asarray(template[0])
    points, info = extract_workpiece(depth, k, *(depth_window or (MIN_DEPTH_M, MAX_DEPTH_M)),
                                     expected_extent=expected)
    if T_base_cam is not None and focus is not None:
        points = focus_crop(points, T_base_cam, focus[0], focus[1])
    if T_base_cam is not None:
        points = remove_table(points, T_base_cam)
    if len(points) < MIN_POINTS:
        return {"group": directory.name, "error": f"工件区域太小（{len(points)} 点）"}
    info.update(
        extent_mm=[round(float(v), 0) for v in
                   ((points.max(axis=0) - points.min(axis=0)) * 1000.0)],
        centroid_mm=[round(float(v) * 1000.0, 1) for v in points.mean(axis=0)],
    )
    centre = points.mean(axis=0)
    planes_all = dedup_planes(segment_planes(points))
    planes = filter_planes(planes_all, centre)
    result = {"group": directory.name, "planes": len(planes),
              "planes_raw": len(planes_all), **info}
    triple = choose_triple(planes) if len(planes) >= 3 else None
    if triple is None:
        # 退化情况（只有两个可见面，或第三个面不正交）：交给形状配准
        if cad_points is None:
            result["error"] = (f"只有 {len(planes)} 个可用的面，且没有给 CAD（--cad-points）"
                               if len(planes) < 3 else
                               f"没有互相正交的三平面（候选 {len(planes)} 个）")
            return result
        init = None
        if len(planes) >= 2 and template is not None:
            init = guided_two_plane(planes, points, template[0], template[1], cad_points)
        if init is None:
            init = global_register(cad_points, points)
        if init is None:
            result["error"] = "全局配准（FPFH+RANSAC）失败"
            return result
        refined = multiscale_icp(cad_points, points, init)
        for _ in range(2):                     # 与三平面路径一致：剔点后重跑，覆盖率按剩下的点算
            kept = reject_with_pose(points, refined, cad_points)
            if len(kept) < MIN_POINTS or len(kept) >= len(points):
                break
            planes_k = filter_planes(dedup_planes(segment_planes(kept)), kept.mean(axis=0))
            init_k = None
            if len(planes_k) >= 2:
                init_k = guided_two_plane(planes_k, kept, template[0], template[1], cad_points)
            if init_k is None:
                init_k = refined
            refined = multiscale_icp(cad_points, kept, init_k)
            points = kept
        observed = _prepared_cloud(points, 0.006)
        template_cloud = o3d.geometry.PointCloud()
        template_cloud.points = o3d.utility.Vector3dVector(cad_points)
        evaluation = o3d.pipelines.registration.evaluate_registration(
            observed, template_cloud, 0.010, np.linalg.inv(refined))
        result["pose"] = {
            "rotation": np.asarray(refined[:3, :3]).round(6).tolist(),
            "translation": [round(float(v), 5) for v in refined[:3, 3]],
            "source": "global+icp",
        }
        result["pose_fitness"] = round(float(evaluation.fitness), 3)
        result["pose_rmse_mm"] = round(float(evaluation.inlier_rmse) * 1000.0, 2)
        result["fallback"] = "global"
        result["points_kept"] = int(len(points))
        if len(planes) >= 2:
            free = np.cross(planes[0]["normal"], planes[1]["normal"])
            if np.linalg.norm(free) > 1e-9:
                free /= np.linalg.norm(free)
                free_obj = np.asarray(refined[:3, :3]).T @ free
                axis = int(np.argmax(np.abs(free_obj)))
                result["unconstrained_axis_obj"] = axis
                result["view_quality"] = {
                    "planes": len(planes), "weakest_inliers": None,
                    "unconstrained_axis": axis, "verdict": "NEED_ANOTHER_VIEW",
                }
                result["position_note"] = (f"只有两个可见面：沿对象轴 {axis} 的位置由模型先验给出，"
                                           f"无观测约束（该轴不要当精确值用）")
        else:
            result["position_note"] = "可见面不足，位置全部依赖模型先验"
        result["pose_valid"] = bool(evaluation.fitness >= POSE_MIN_COVERAGE)
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
                chosen, corner = chosen2, corner2      # 后续角点锚定用最后一轮的平面
                points = kept
            weakest = min(int(item["inliers"]) for item in chosen)
            result["view_quality"] = {
                "planes": len(planes),
                "weakest_inliers": weakest,
                "unconstrained_axis": None,
                "verdict": ("OK" if weakest >= CORNER_ANCHOR_MIN_INLIERS
                            else "NEED_ANOTHER_VIEW"),
            }
            if weakest >= CORNER_ANCHOR_MIN_INLIERS:
                matrix, corner_index = anchor_on_corner(
                    refined["transformation"], chosen, corner, template[0], template[1])
                # 2026-11-10 教训：角点编号是按"旋转最接近"选的，可能对到错误的 bbox 角，
                # 结果位姿一致但整体偏一个边长（实测覆盖率从 0.97 掉到 0.26）。
                # 所以锚定之后必须用数据复核，掉太多就退回 ICP 的平移。
                anchored_fit = _coverage(points, cad_points, matrix)
                icp_fit = _coverage(points, cad_points, refined["transformation"])
                if anchored_fit + 0.15 < icp_fit:
                    matrix = refined["transformation"]
                    result["position_note"] = (f"角点锚定被数据否决（覆盖 {anchored_fit:.2f} < "
                                               f"ICP {icp_fit:.2f}），改用 ICP 平移")
                else:
                    result["corner_index"] = corner_index
                    result["position_note"] = f"位置由三平面角点锚定（最弱面 {weakest} 点）"
            else:
                # 第三面太弱时，交点本身抖 10+ mm（实测 rot0），不如用 ICP 的姿态+平移
                matrix = refined["transformation"]
                result["position_note"] = (f"最弱面只有 {weakest} 点（<{CORNER_ANCHOR_MIN_INLIERS}），"
                                           f"不用角点锚定；位置置信度低")
            result["pose"] = {
                "rotation": np.asarray(matrix[:3, :3]).round(6).tolist(),
                "translation": [round(float(v), 5) for v in matrix[:3, 3]],
                "source": "cad_icp",
            }
            result["pose_fitness"] = round(refined["fitness"], 3)
            result["pose_rmse_mm"] = round(refined["rmse_mm"], 2)
            result["points_kept"] = int(len(points))
            result["pose_valid"] = bool(refined["fitness"] >= POSE_MIN_COVERAGE)
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


def sample_mesh(mesh: o3d.geometry.TriangleMesh, count: int, seed: int = 7) -> np.ndarray:
    """按面积均匀采样网格表面，结果可复现（Open3D 的 sample_points_uniformly 不接受 seed）。"""
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    a, b, c = vertices[triangles[:, 0]], vertices[triangles[:, 1]], vertices[triangles[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    rng = np.random.default_rng(seed)
    picks = rng.choice(len(areas), size=count, p=areas / areas.sum())
    u = rng.random(count)[:, None]
    v = rng.random(count)[:, None]
    flip = (u + v) > 1.0
    u[flip] = 1.0 - u[flip]
    v[flip] = 1.0 - v[flip]
    return a[picks] + u * (b[picks] - a[picks]) + v * (c[picks] - a[picks])


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


def _prepared_cloud(points: np.ndarray, voxel: float) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud = cloud.voxel_down_sample(voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=30))
    return cloud


def multiscale_icp(template_points: np.ndarray, observed_points: np.ndarray,
                   init: np.ndarray, scales=(0.03, 0.015, 0.008)) -> np.ndarray:
    """从粗到细的 ICP：每级用更小的对应距离继续收敛，避免停在近立方体的浅局部极小。"""
    transformation = init.copy()
    observed = _prepared_cloud(observed_points, 0.004)
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(template_points)
    template = template.voxel_down_sample(0.004)
    template.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.016, max_nn=30))
    for threshold in scales:
        result = o3d.pipelines.registration.registration_icp(
            template, observed, threshold, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
        transformation = result.transformation
    return transformation


def guided_two_plane(planes: list[dict], points: np.ndarray, bbox_min: np.ndarray,
                    bbox_max: np.ndarray, cad_points: np.ndarray,
                    lam_range=0.35, lam_steps=9) -> np.ndarray | None:
    """只有两个可见面时的定向搜索：两个面固定两条轴，第三条轴用叉积，沿自由方向扫平移。

    两平面能给出：哪两条对象轴、各自的符号（共 3 种"缺哪条轴" × 2 种指派 × 4 种符号组合），
    以及三个平移分量中的两个；剩下一个沿两法向的叉积方向自由。
    这里把这个自由度扫一遍，用"观测点落在 CAD 附近的比例"粗筛，再交给多尺度 ICP。
    """
    from itertools import product
    from scipy.spatial import cKDTree
    if len(planes) < 2:
        return None
    coarse_cad = cad_points[::max(1, len(cad_points) // 6000)]
    tree = cKDTree(coarse_cad)
    observed = points[::max(1, len(points) // 12000)]

    best = None
    axes = [0, 1, 2]
    for missing in axes:
        others = [axis for axis in axes if axis != missing]
        for flip in (0, 1):
            i1, i2 = others if flip == 0 else others[::-1]
            for s1, s2 in product((1, -1), repeat=2):
                n1 = planes[0]["normal"] * s1
                n2 = planes[1]["normal"] * s2
                if abs(float(np.dot(n1, n2))) > 0.20:
                    continue
                e1 = n1 / np.linalg.norm(n1)
                e2 = n2 - np.dot(e1, n2) * e1
                if np.linalg.norm(e2) < 1e-9:
                    continue
                e2 /= np.linalg.norm(e2)
                e3 = np.cross(e1, e2)
                rotation = np.zeros((3, 3))
                rotation[:, i1], rotation[:, i2], rotation[:, i3 if False else missing] = e1, e2, e3
                if np.linalg.det(rotation) < 0:
                    continue
                c1 = bbox_max[i1] if s1 > 0 else bbox_min[i1]
                c2 = bbox_max[i2] if s2 > 0 else bbox_min[i2]
                matrix = np.stack([n1, n2])
                rhs = np.array([-planes[0]["d"] - s1 * c1, -planes[1]["d"] - s2 * c2])
                base, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
                direction = np.cross(n1, n2)
                norm = np.linalg.norm(direction)
                if norm < 1e-9:
                    continue
                direction /= norm
                for step in range(-lam_steps, lam_steps + 1):
                    translation = base + direction * (lam_range * step / lam_steps)
                    points_obj = (observed - translation) @ rotation
                    distance, _ = tree.query(points_obj, k=1)
                    score = float((distance < 0.02).mean())
                    if best is None or score > best[0]:
                        best = (score, rotation, translation)
    if best is None:
        return None
    init = np.eye(4)
    init[:3, :3] = best[1]
    init[:3, 3] = best[2]
    return init


def global_register(template_points: np.ndarray, observed_points: np.ndarray) -> np.ndarray | None:
    """三平面不够时（只有两个可见面）用 FPFH + RANSAC 做全局配准，再交给多尺度 ICP。

    为什么需要它：工件转到某些角度只剩两个面可见，三平面求交退化，
    bbox 指派也无从谈起；此时只能靠形状特征（扶手、凹槽）把位姿找回来。
    """
    observed = _prepared_cloud(observed_points, 0.006)
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(template_points)
    template = template.voxel_down_sample(0.006)
    template.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.024, max_nn=30))
    radius = 0.02
    observed_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        observed, o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=100))
    template_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        template, o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=100))
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        template, observed, template_fpfh, observed_fpfh, True, 0.02,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.02)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999))
    if result.fitness <= 0.0:
        return None
    return result.transformation


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
            refined = multiscale_icp(cad_points, cloud, init)
        except Exception:                       # noqa: BLE001
            continue
        # 选择指标要看"观测点云有多少落在 CAD 上"（覆盖率），而不是反过来：
        # 只看到 3 个面时，CAD→观测的 fitness 上限本来就只有 ~0.5。
        # evaluate_registration 把 transformation 作用在 source 上：
        # source=观测（相机系）、target=CAD（对象系），所以要传 T_object_camera = inv(T_camera_object)
        evaluation = o3d.pipelines.registration.evaluate_registration(
            observed, template, 0.010, np.linalg.inv(refined))
        entry = {"transformation": refined,
                 "fitness": float(evaluation.fitness),
                 "rmse_mm": float(evaluation.inlier_rmse) * 1000.0,
                 "bbox_score": candidate["score"]}
        if best is None or (entry["fitness"], -entry["rmse_mm"]) > (best["fitness"], -best["rmse_mm"]):
            best = entry
    return best or {}


def _read_base_camera() -> np.ndarray | None:
    """从实时话题读 TCP，并用已验明的手眼链算出 T_base_cam。"""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from scipy.spatial.transform import Rotation as RotationTool
    except Exception:                              # noqa: BLE001
        return None
    rclpy.init()
    node = rclpy.create_node("coarse_pose_tcp")
    holder: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda msg: holder.__setitem__("tcp", msg), 10)
    deadline = time.time() + 8.0
    while time.time() < deadline and "tcp" not in holder:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "tcp" not in holder:
        return None
    pose = holder["tcp"].pose
    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = RotationTool.from_quat([pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w]).as_matrix()
    T_base_tcp[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    handeye = json.loads((pathlib.Path(__file__).resolve().parents[1] /
                          "config" / "handeye" / "tcp_to_camera.json").read_text())
    return T_base_tcp @ np.array(handeye["T_tcp_cam"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default=None, help="vision_dataset 目录（含 g*/）")
    parser.add_argument("--group", default=None, help="只分析一个组目录")
    parser.add_argument("--template", default=None, help="CAD STL（用于尺寸约束与形状消歧）")
    parser.add_argument("--focus", type=float, nargs=4, default=None,
                        metavar=("X", "Y", "Z", "R"),
                        help="只保留 base 系里以 (X,Y,Z) 为心、半径 R 的点（近距观测很有效）")
    parser.add_argument("--remove-table", action="store_true",
                        help="用当前 TCP 位姿+手眼在 base 系剔掉桌面（需驱动栈在线）")
    parser.add_argument("--depth-window", type=float, nargs=2, default=None,
                        metavar=("MIN_M", "MAX_M"),
                        help="工件所在深度窗（默认 0.55–1.05；精观测 0.45 m 时应改成 0.2–0.8）")
    parser.add_argument("--cad-points", type=int, default=0,
                        help="用 CAD 采样这么多点做 ICP 消歧（0 = 只用 bbox）")
    parser.add_argument("--out", default=None, help="把结果写成 JSON")
    args = parser.parse_args()

    base_cam = None
    if args.remove_table:
        base_cam = _read_base_camera()
        if base_cam is None:
            print("读不到 /tcp_pose_broadcaster/pose，无法剔桌面", file=sys.stderr)
            return 2
        print(f"手眼：相机在 base {np.round(base_cam[:3, 3], 3)}")

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
                cad_points = sample_mesh(mesh, args.cad_points, seed=7)
            print(f"模板 {path.name}: 外形 {np.round((bbox_max - bbox_min) * 1000).astype(int)} mm；"
                  f"原点相对 bbox 中心 {np.round((bbox_min + bbox_max) / 2 * 1000, 1)} mm")
        else:
            print(f"模板不存在：{path}（继续，只做三平面，不出位姿）")

    results = []
    print(f"{'组':<14}{'面数':>4}{'工件点数':>9}{'内点':>22}{'正交':>7}{'cond':>7}"
          f"{'打分':>8}{'角点(mm)':>26}")
    print("-" * 88)
    for directory in directories:
        window = tuple(args.depth_window) if args.depth_window else None
        focus = (np.array(args.focus[:3]), args.focus[3]) if args.focus else None
        result = analyse_group(directory, template, cad_points, window, base_cam, focus)
        results.append(result)
        if result.get("error"):
            print(f"{result['group']:<14}{'':>4}  ✗ {result['error']}")
            continue
        inliers = ",".join(str(v) for v in result.get("inliers", [])) or "（全局配准）"
        corner = ",".join(f"{v:+.1f}" for v in result.get("corner_mm", [])) or "-"
        if "pose_fitness" in result:
            score = f"{result['pose_fitness']:.2f}/{result['pose_rmse_mm']:.1f}"
        elif "pose_score" in result:
            score = f"{result['pose_score']:.3f}"
        else:
            score = "  -  "
        orth = result.get("orthogonality")
        cond = result.get("condition")
        orth_text = f"{orth:.3f}" if orth is not None else "  -- "
        cond_text = f"{cond:.1f}" if cond is not None else "  --  "
        print(f"{result['group']:<14}{result['planes']:>4}{result.get('area_px', 0):>9}"
              f"{inliers:>22}{orth_text:>7}{cond_text:>7}"
              f"{score:>8}{'' if result.get('pose_valid', True) else '?'}  [{corner}]")

    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"\n结果写入 {args.out}")
    good = [r for r in results if not r.get("error")]
    print(f"\n成功 {len(good)}/{len(results)} 组")
    return 0 if good else 1


if __name__ == "__main__":
    raise SystemExit(main())
