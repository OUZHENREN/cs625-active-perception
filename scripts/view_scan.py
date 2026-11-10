#!/usr/bin/env python3
"""绕工件择视角：先用 CAD 预测"从哪个方向看能得到三个面"，再只对有希望的方位做规划/移动。

为什么不能"绕着转一圈就拍"
---------------------------
1. 机械臂可达范围有限：绕工件 360° 里大部分方位根本规划不到（实测 8 个方位只有 5 个可达，
   且都在机器人一侧）；
2. 拍了才知道好不好，代价是每个方位一次真实移动；
3. 真正要判断的是"三个面是否都看得见、最小面是否够大"，这个**用 CAD 就能离线算**。

所以流程是：候选位姿（方位×俯仰×距离）→ CAD 可见性预测 → 排序 → 规划校验 → 只移动最优的。

用法
----
    # 1) 只预测（不动机械臂）
    python3 scripts/view_scan.py --object-pose /tmp/object_pose_base.npy --predict

    # 2) 对预测出的前 N 个做规划校验（仍不动）
    python3 scripts/view_scan.py --object-pose /tmp/object_pose_base.npy --plan-top 5

    # 3) 移动到最优可达视角并拍摄
    python3 scripts/view_scan.py --object-pose /tmp/object_pose_base.npy --execute-best
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp
import fine_view as fv

HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
K = [931.473, 0.0, 648.695, 0.0, 931.595, 449.319, 0.0, 0.0, 1.0]   # 9 元 K（与 camera_info 同序）
TEMPLATE = pathlib.Path.home() / "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL"
FACES = ["-x", "+x", "-y", "+y", "-z", "+z"]
# 每根轴取一个面，组成"三个互相垂直的可见面"
FACE_COMBOS = [("-x", "-y", "-z"), ("-x", "-y", "+z"), ("-x", "+y", "-z"), ("-x", "+y", "+z"),
               ("+x", "-y", "-z"), ("+x", "-y", "+z"), ("+x", "+y", "-z"), ("+x", "+y", "+z")]


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    return np.column_stack([right, np.cross(right, forward), forward])


def candidates(centre: np.ndarray, distance: float, azimuths, elevations) -> list[dict]:
    result = []
    for elevation_deg in elevations:
        elevation = np.radians(elevation_deg)
        for azimuth_deg in azimuths:
            azimuth = np.radians(azimuth_deg)
            direction = np.array([np.cos(elevation) * np.cos(azimuth),
                                  np.cos(elevation) * np.sin(azimuth),
                                  np.sin(elevation)])
            position = centre + direction * distance
            T_world_cam = np.eye(4)
            T_world_cam[:3, :3] = look_at(position, centre)
            T_world_cam[:3, 3] = position
            result.append({"azimuth_deg": float(azimuth_deg),
                           "elevation_deg": float(elevation_deg),
                           "T_world_cam": T_world_cam})
    return result


def visible_pixels(T_cam_object: np.ndarray, bbox, cad: np.ndarray,
                   width: int = 1280, height: int = 960) -> dict:
    """z-buffer 统计每个 bbox 面真正可见的像素数（带遮挡）。

    之前用"面朝相机就算可见"，结果是几乎所有候选都拿满分、排不出差别；
    真实情况是：面不仅要有朝向，还得**不被别的面挡住**、并且投影面积够大。
    """
    fx, _, cx, _, fy, cy = K[0], K[1], K[2], K[3], K[4], K[5]
    distances = np.stack([np.abs(cad[:, i] - bbox[0][i]) for i in range(3)] +
                         [np.abs(cad[:, i] - bbox[1][i]) for i in range(3)], axis=1)
    face = np.argmin(distances, axis=1)
    on_surface = distances[np.arange(len(cad)), face] < 0.004

    points_cam = (T_cam_object[:3, :3] @ cad.T).T + T_cam_object[:3, 3]
    z = points_cam[:, 2]
    front = z > 0.05
    u = (fx * points_cam[:, 0] / np.maximum(z, 1e-6) + cx).astype(int)
    v = (fy * points_cam[:, 1] / np.maximum(z, 1e-6) + cy).astype(int)
    inside = front & (u >= 0) & (u < width) & (v >= 0) & (v < height) & on_surface

    pixel = v * width + u
    depth = np.where(inside, z, np.inf)
    order = np.argsort(depth)
    pixel_sorted = pixel[order]
    _, first = np.unique(pixel_sorted, return_index=True)   # 每个像素最近的那个点
    nearest = order[first]
    counts = np.zeros(6, dtype=int)
    for index in nearest:
        counts[face[index]] += 1
    names = ["-x", "+x", "-y", "+y", "-z", "+z"]
    return {name: int(counts[i]) for i, name in enumerate(names)}


def predict(T_world_object: np.ndarray, T_world_cam: np.ndarray, bbox, cad) -> dict:
    T_cam_object = np.linalg.inv(T_world_cam) @ T_world_object
    pixels = visible_pixels(T_cam_object, bbox, cad)
    total = max(1, sum(pixels.values()))
    best = None
    for combo in FACE_COMBOS:
        values = [pixels[name] / total for name in combo]
        worst = min(values)
        if best is None or worst > best[0]:
            best = (worst, combo, values)
    return {"visible_px": pixels,
            "best_combo": list(best[1]), "worst_face": round(best[0], 3),
            "score": round(best[0], 3)}


def tcp_of(T_world_cam: np.ndarray) -> np.ndarray:
    handeye = json.loads(HANDEYE.read_text())
    return T_world_cam @ np.linalg.inv(np.array(handeye["T_tcp_cam"]))


def plan(tag: str, T_world_tcp: np.ndarray) -> tuple[bool, str]:
    quat = R.from_matrix(T_world_tcp[:3, :3]).as_quat()
    command = [sys.executable, "-u", str(REPO / "scripts" / "plan_pose_once.py"),
               *[f"{v:.6f}" for v in T_world_tcp[:3, 3]],
               *[f"{v:.6f}" for v in quat], "--output", f"/tmp/scan_{tag}.json"]
    run = subprocess.run(command, capture_output=True, text=True, timeout=240)
    stdout = run.stdout or ""
    ok = "code 1 " in stdout
    return ok, stdout.strip().splitlines()[-1] if stdout.strip() else "(no output)"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--object-pose", required=True, help="T_world_object 的 .npy（近似即可）")
    parser.add_argument("--distance", type=float, default=0.70)
    parser.add_argument("--azimuths", default=",".join(str(v) for v in range(0, 360, 30)))
    parser.add_argument("--elevations", default="15,30,45,60")
    parser.add_argument("--predict", action="store_true")
    parser.add_argument("--plan-top", type=int, default=0)
    parser.add_argument("--execute-best", action="store_true")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    T_world_object = np.load(args.object_pose)
    bbox = cp.load_template_bbox(TEMPLATE)
    mesh = o3d.io.read_triangle_mesh(str(TEMPLATE))
    cad = cp.sample_mesh(mesh, 30000, seed=7)
    centre = (bbox[0] + bbox[1]) / 2
    centre_world = T_world_object[:3, :3] @ centre + T_world_object[:3, 3]

    azimuths = [float(v) for v in args.azimuths.split(",") if v.strip()]
    elevations = [float(v) for v in args.elevations.split(",") if v.strip()]
    rows = []
    for candidate in candidates(centre_world, args.distance, azimuths, elevations):
        prediction = predict(T_world_object, candidate["T_world_cam"], bbox, cad)
        T_world_tcp = tcp_of(candidate["T_world_cam"])
        rows.append({**candidate, **prediction,
                     "tcp": [float(v) for v in T_world_tcp[:3, 3]],
                     "T_world_tcp": T_world_tcp})
    rows.sort(key=lambda row: row["score"], reverse=True)

    print(f"{'方位':>6}{'俯仰':>6}{'最弱面':>8}{'得分':>7}   可见面组合            TCP(base)")
    for row in rows[:15]:
        print(f"{row['azimuth_deg']:>6.0f}{row['elevation_deg']:>6.0f}{row['worst_face']:>8.2f}"
              f"{row['score']:>7.2f}   {str(row['best_combo']):<20} {np.round(row['tcp'], 3)}")

    if args.plan_top:
        print("\n规划校验（不动机械臂）：")
        for index, row in enumerate(rows[:args.plan_top]):
            ok, summary = plan(f"p{index}", row["T_world_tcp"])
            row["reachable"] = ok
            print(f"  #{index} az{row['azimuth_deg']:.0f}/el{row['elevation_deg']:.0f} "
                  f"{'可达' if ok else '不可达'}  {summary}")

    if args.execute_best:
        for index, row in enumerate(rows):
            ok, summary = plan(f"best{index}", row["T_world_tcp"])
            print(f"尝试 az{row['azimuth_deg']:.0f}/el{row['elevation_deg']:.0f}: "
                  f"{'可达' if ok else '不可达'} {summary}")
            if not ok:
                continue
            run = subprocess.run([sys.executable, "-u", str(REPO / "scripts" / "execute_assembly_chunks.py"),
                                  f"/tmp/scan_best{index}.json", "--time-scale", "2.0",
                                  "--dwell", "0.5", "--goal-tolerance", "0.003"],
                                 capture_output=True, text=True, timeout=300)
            if "ALL_ASSEMBLY_CHUNKS_OK" in (run.stdout or ""):
                print(f"已移动到 az{row['azimuth_deg']:.0f}/el{row['elevation_deg']:.0f}")
                break
            print("执行失败，试下一个")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(
            [{k: v for k, v in row.items() if k != "T_world_tcp"} for row in rows],
            ensure_ascii=False, indent=2))
        print(f"\n结果写入 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
