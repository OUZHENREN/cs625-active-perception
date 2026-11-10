#!/usr/bin/env python3
"""从粗位姿算"精观测位"，并离线验证它是否能把工件的三个面都看到。

为什么需要这一步（RVS 的两级视点位）
------------------------------------
粗观测给出工件的大致位姿；由它就能算出"从哪里看、看多近"的精观测位，
移动过去重采一帧，再做精配准。单帧精配准无法创造数据里没有的面——
rot0 的第三个面只有 4700 点就是例子，只能靠换视角补。

这里先做"算 + 验"，不动机械臂：
  fine_view_pose()   从 T_camera_object 算出精观测位（相机在工件系里的位姿）
  face_visibility()  把 CAD 采样点投到该视角，统计六个面各有多少比例可见
                     （可见 = 面朝相机 + 落在图像内 + 没有被点云挡住）

用法
----
    python3 scripts/fine_view.py --dataset runtime/vision_dataset/<stamp> \
        --template ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL

输出每组：粗视角 vs 精视角的六面可见比例，以及"三面是否都够用"的判定。
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

DEFAULT_DISTANCE_M = 0.45          # 精观测距离（与 observe.workpiece_fine 一致）
DEFAULT_AZ_DEG = 25.0
DEFAULT_EL_DEG = 30.0
MIN_VISIBLE_FRACTION = 0.05        # 一个面至少 5% 的点可见才算"看得见"


def fine_view_pose(pose_camera_object: np.ndarray, bbox_min: np.ndarray, bbox_max: np.ndarray,
                   distance_m: float = DEFAULT_DISTANCE_M,
                   az_deg: float = DEFAULT_AZ_DEG,
                   el_deg: float = DEFAULT_EL_DEG) -> np.ndarray:
    """由粗位姿算精观测位，返回 T_camera_object 形式的"精观测相机位姿"。

    相机放在"工件的体对角线方向"上、距离 distance_m 处，光轴指向 bbox 中心：
      · 体对角线方向 = 从薄面（131 mm）斜看，同时带上两个侧面的分量；
        正对近正方形大面会造成四重对称歧义（2026-10-26 的教训），所以要斜看。
    """
    centre = (bbox_min + bbox_max) / 2.0
    az, el = np.radians(az_deg), np.radians(el_deg)
    # 工件系里的观察方向（单位向量）：以厚度轴 y 为主，按方位/俯仰偏出去
    direction = np.array([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])
    direction /= np.linalg.norm(direction)

    camera_z = -direction                       # 相机 z 轴指向工件（光学惯例）
    # 相机 x 轴取工件 x 轴在像面上的投影，保证图像与工件面大致对齐
    camera_x = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], camera_z) * camera_z
    camera_x /= np.linalg.norm(camera_x)
    camera_y = np.cross(camera_z, camera_x)

    rotation = np.column_stack([camera_x, camera_y, camera_z])     # 相机轴在工件系下的表示
    translation = centre + direction * distance_m                   # 相机位置（工件系）
    # 上面两个量构成 T_object_camera；对外要的是 T_camera_object（对象→相机），取逆
    fine = np.eye(4)
    fine[:3, :3] = rotation.T
    fine[:3, 3] = -rotation.T @ translation
    return fine


def face_visibility(pose_camera_object: np.ndarray, bbox_min: np.ndarray, bbox_max: np.ndarray,
                    cad_points: np.ndarray, k: list[float], image_size=(960, 1280)) -> dict:
    """把 CAD 采样点投到该视角，统计 6 个 bbox 面各有多少比例可见。"""
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    height, width = image_size
    # 点属于哪个 bbox 面（取最近的那个面）
    distances = np.stack([np.abs(cad_points[:, i] - bbox_min[i]) for i in range(3)] +
                         [np.abs(cad_points[:, i] - bbox_max[i]) for i in range(3)], axis=1)
    face = np.argmin(distances, axis=1)
    tolerance = 0.004
    on_surface = distances[np.arange(len(cad_points)), face] < tolerance

    points_cam = (pose_camera_object[:3, :3] @ cad_points.T).T + pose_camera_object[:3, 3]
    z = points_cam[:, 2]
    front = z > 0.05
    u = fx * points_cam[:, 0] / np.maximum(z, 1e-6) + cx
    v = fy * points_cam[:, 1] / np.maximum(z, 1e-6) + cy
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height) & front

    # 面朝相机：法向（对象系）在相机方向上的投影 > 0
    camera_in_object = -(pose_camera_object[:3, :3].T @ pose_camera_object[:3, 3])
    normals = []
    for index in range(3):
        for limit in (-1.0, 1.0):
            normal = np.zeros(3)
            normal[index] = limit
            normals.append(normal)
    facing = np.zeros(len(cad_points), dtype=bool)
    for index, normal in enumerate(normals):
        selected = face == index
        facing[selected] = np.dot(cad_points[selected] - camera_in_object, normal) > 0

    result = {}
    names = ["-x", "+x", "-y", "+y", "-z", "+z"]
    for index, name in enumerate(names):
        members = on_surface & (face == index)
        total = int(members.sum())
        visible = int((members & facing & inside).sum())
        result[name] = {"points": total,
                        "visible": visible,
                        "fraction": round(float(visible / total), 3) if total else 0.0}
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--group", default=None)
    parser.add_argument("--template", required=True)
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_M)
    parser.add_argument("--az", type=float, default=DEFAULT_AZ_DEG)
    parser.add_argument("--el", type=float, default=DEFAULT_EL_DEG)
    parser.add_argument("--cad-points", type=int, default=60000)
    args = parser.parse_args()

    directories = ([pathlib.Path(args.group)] if args.group
                   else sorted(pathlib.Path(args.dataset).glob("g*")))
    path = pathlib.Path(args.template).expanduser()
    mesh = o3d.io.read_triangle_mesh(str(path))
    bbox = cp.load_template_bbox(path)
    cad = cp.sample_mesh(mesh, args.cad_points, seed=7)

    print(f"{'组':<14}{'视角':>6}  {'-x':>6}{'+x':>6}{'-y':>6}{'+y':>6}{'-z':>6}{'+z':>6}   三个主要面")
    print("-" * 84)
    for directory in directories:
        coarse = cp.analyse_group(directory, bbox, cad)
        if coarse.get("error") or "pose" not in coarse:
            print(f"{directory.name:<14}  ✗ 粗配准失败")
            continue
        pose = np.eye(4)
        pose[:3, :3] = np.array(coarse["pose"]["rotation"])
        pose[:3, 3] = np.array(coarse["pose"]["translation"])
        k = cp.load_group(directory)[1]
        current = face_visibility(pose, bbox[0], bbox[1], cad, k)
        fine_pose = fine_view_pose(pose, bbox[0], bbox[1], args.distance, args.az, args.el)
        fine = face_visibility(fine_pose, bbox[0], bbox[1], cad, k)
        for label, table in (("粗", current), ("精", fine)):
            values = "".join(f"{table[name]['fraction']:>6.2f}" for name in
                             ["-x", "+x", "-y", "+y", "-z", "+z"])
            best = sorted((value for value in table.values()),
                          key=lambda item: item["fraction"], reverse=True)[:3]
            ok = sum(1 for item in best if item["fraction"] >= MIN_VISIBLE_FRACTION)
            print(f"{directory.name if label == '粗' else '':<14}{label:>6}  {values}   {ok}/3 面可用")
        # 记录精视角的位姿（工件系），后续真正移动时用
        (directory / "fine_view.json").write_text(json.dumps({
            "T_camera_object": fine_pose.round(6).tolist(),
            "distance_m": args.distance, "az_deg": args.az, "el_deg": args.el,
        }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
