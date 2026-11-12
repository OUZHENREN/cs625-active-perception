#!/usr/bin/env python3
"""D3 精配准：在粗位姿上做细尺度 ICP + 180° 翻转消歧 + 角点验证。

三步对应计划里的 D3：
  1. 细尺度 ICP：体素 2 mm、对应阈值 8→4→2 mm、point-to-plane；
  2. 翻转消歧：工件沿中轴线镜面对称（R_flip = diag(1, −1, −1)），
     把"候选翻转解"也精配准一次，比较两者的覆盖率与 RMSE，差距不足 5% 时报警；
  3. 角点验证：粗测量的三平面交点与"模型角点经精位姿变换后"的距离，
     超过 2 mm 判为不可信（文档里定的阈值）。

输出每组的：精位姿、覆盖率、RMSE、翻转判定、角点距离。不动机械臂、只读数据集。

用法：
    python3 scripts/fine_pose.py --dataset runtime/vision_dataset/<stamp> \
        --template ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL --cad-points 60000
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import coarse_pose as cp

FLIP = np.diag([1.0, -1.0, -1.0, 1.0])      # 沿对象 x 轴转 180°（文档里的 R_flip）
CORNER_TOL_M = 0.002                        # 角点验证阈值 2 mm
FLIP_MARGIN = 0.05                          # 翻转解与正解覆盖率差小于 5% 视为"有歧义"


def fine_icp(cloud: np.ndarray, cad_points: np.ndarray, init: np.ndarray) -> np.ndarray:
    """细尺度 ICP（2 mm 体素，多级阈值）。"""
    observed = cp._prepared_cloud(cloud, 0.002)
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(cad_points)
    template = template.voxel_down_sample(0.002)
    template.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.008, max_nn=30))
    transformation = init.copy()
    for threshold in (0.006, 0.003, 0.0015):
        result = o3d.pipelines.registration.registration_icp(
            template, observed, threshold, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        transformation = result.transformation
    return transformation


def coverage(cloud: np.ndarray, cad_points: np.ndarray, transformation: np.ndarray) -> tuple[float, float]:
    observed = cp._prepared_cloud(cloud, 0.003)
    template = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(cad_points)
    evaluation = o3d.pipelines.registration.evaluate_registration(
        observed, template, 0.005, np.linalg.inv(transformation))
    return float(evaluation.fitness), float(evaluation.inlier_rmse) * 1000.0


def analyse(directory: pathlib.Path, cad_points: np.ndarray, template: tuple) -> dict:
    coarse = cp.analyse_group(directory, template, cad_points)
    if coarse.get("error") or "pose" not in coarse:
        return {"group": directory.name, "error": coarse.get("error", "粗配准没出位姿")}
    rotation = np.array(coarse["pose"]["rotation"])
    translation = np.array(coarse["pose"]["translation"])
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, translation
    cloud = cp.extract_workpiece(*cp.load_group(directory))[0]
    cloud = cp.reject_with_pose(cloud, pose, cad_points, threshold_m=0.02)

    refined = fine_icp(cloud, cad_points, pose)
    # 精配准只应当"微调"。实测（2026-11-10）细尺度 ICP 会被桌面上离工件底面很近的点拖走 50 mm，
    # 覆盖率反而从 0.97 跌到 0.87；所以这里加一道守卫：位移 >10 mm 或转角 >2° 就拒绝它。
    shift_mm = float(np.linalg.norm(refined[:3, 3] - pose[:3, 3]) * 1000.0)
    angle_deg = float(np.degrees((R.from_matrix(np.array(refined[:3, :3]))
                                  * R.from_matrix(np.array(pose[:3, :3])).inv()).magnitude()))
    # 守卫（2026-11-12 修订）：原来的"位移 >10 mm 就拒绝"会误杀正确的精配准——
    # 实测两组分别移动 8.7 / 11.2 mm，但覆盖率与 RMSE 都变好、组间分散从 3.44 mm 掉到 0.08 mm。
    # 所以判据改成"位移上限放宽到 20 mm / 2°，并要求覆盖率确实提升"，以覆盖率为准。
    fit_before, rmse_before = coverage(cloud, cad_points, pose)
    fit_after, _rmse_after = coverage(cloud, cad_points, refined)
    refine_rejected = (shift_mm > 20.0 or angle_deg > 2.0
                       or fit_after + 1e-6 < fit_before)
    if refine_rejected:
        refined = pose
    flipped_init = refined @ FLIP
    flipped = fine_icp(cloud, cad_points, flipped_init)

    fit_ok, rmse_ok = coverage(cloud, cad_points, refined)
    fit_flip, rmse_flip = coverage(cloud, cad_points, flipped)
    margin = abs(fit_ok - fit_flip)
    ambiguous = margin < FLIP_MARGIN

    # 角点验证：粗测量的三平面交点 vs 模型角点经精位姿变换
    corner_error = None
    if "corner_index" in coarse and "pose" in coarse:
        # corner_index 的位序：bit0=x, bit1=y, bit2=z（与 anchor_on_corner 一致）
        index = int(coarse["corner_index"])
        model_corner = np.array([template[1][axis] if (index >> axis) & 1
                                 else template[0][axis] for axis in range(3)])
        moved = refined[:3, :3] @ model_corner + refined[:3, 3]
        observed_corner = np.array(coarse.get("corner_mm", [0, 0, 0])) / 1000.0
        corner_error = float(np.linalg.norm(moved - observed_corner) * 1000.0)

    return {
        "group": directory.name,
        "coverage": round(fit_ok, 4),
        "rmse_mm": round(rmse_ok, 2),
        "flip_coverage": round(fit_flip, 4),
        "flip_margin": round(margin, 4),
        "flip_ambiguous": bool(ambiguous),
        "refine_shift_mm": round(shift_mm, 2),
        "refine_angle_deg": round(angle_deg, 2),
        "refine_rejected": bool(refine_rejected),
        "chosen": "normal" if fit_ok >= fit_flip else "flipped",
        "corner_error_mm": None if corner_error is None else round(corner_error, 2),
        "corner_valid": None if corner_error is None else bool(corner_error <= CORNER_TOL_M * 1000),
        "pose": {"rotation": refined[:3, :3].round(6).tolist(),
                 "translation": [round(float(v), 6) for v in refined[:3, 3]]},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--group", default=None)
    parser.add_argument("--template", required=True)
    parser.add_argument("--cad-points", type=int, default=60000)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    directories = ([pathlib.Path(args.group)] if args.group
                   else sorted(pathlib.Path(args.dataset).glob("g*")))
    if not directories:
        print("没有组目录", file=sys.stderr)
        return 2
    path = pathlib.Path(args.template).expanduser()
    mesh = o3d.io.read_triangle_mesh(str(path))
    bbox_min, bbox_max = cp.load_template_bbox(path)
    cad_points = cp.sample_mesh(mesh, args.cad_points, seed=7)

    results = []
    print(f"{'组':<14}{'覆盖率':>8}{'RMSE':>8}{'翻转覆盖':>9}{'判定':>8}{'角点误差':>10}{'精修位移':>10}")
    print("-" * 60)
    for directory in directories:
        result = analyse(directory, cad_points, (bbox_min, bbox_max))
        results.append(result)
        if result.get("error"):
            print(f"{result['group']:<14}  ✗ {result['error']}")
            continue
        corner = ("  -  " if result["corner_error_mm"] is None
                  else f"{result['corner_error_mm']:.2f}mm")
        mark = "拒绝" if result.get("refine_rejected") else "采纳"
        print(f"{result['group']:<14}{result['coverage']:>8.3f}{result['rmse_mm']:>8.2f}"
              f"{result['flip_coverage']:>9.3f}{('有歧义' if result['flip_ambiguous'] else result['chosen']):>8}"
              f"{corner:>10}{result.get('refine_shift_mm', 0):>7.1f}{mark:>4}")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"\n结果写入 {args.out}")
    good = [r for r in results if not r.get("error")]
    print(f"\n成功 {len(good)}/{len(results)} 组")
    return 0 if good else 1


if __name__ == "__main__":
    raise SystemExit(main())
