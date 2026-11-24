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
* 输出排名；本脚本**不移动机械臂**。真机换位由
  `scripts/nbv_from_reference.py --auto-station [--execute]` 负责（旧的 nbv_lateral 已于
  2026-11-13 归档到 docs/archive/2026-11-13-scripts-legacy/six-mode/）。
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

    tolerance 缺省时取查表里的 tolerance_deg（2026-11-14 起 15.0°）；
    一致性门默认 200 mm（2026-11-14 所有者从 40 放宽）；超出 near_tolerance_deg 判 station_far（告警）
    （放行 + 告警，由精观测质量门兜底）。
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
        if entry.get("validated", True) is False:
            continue   # 未通过精度门的站位不参与自动选站（-45° 即此类）
        try:
            angle = float(key)
        except ValueError:
            continue
        error = abs(angle - yaw_deg)
        if error <= tolerance + 1e-6 and (best is None or error < best[0]):
            best = (error, key, entry)
    return best


# 摆放约束（与 nbv_from_reference 同一组常数，2026-11-19 所有者定）
PLACEMENT_MAX_MM = 30.0
PLACEMENT_MAX_DEG = 3.5

# 手摆（taught）站位在偏转角差小于这个值时优先选用（它是为那个偏转角手摆的）
TAUGHT_PREFERENCE_DEG = 5.0


def placement_gap(entry: dict, cur_pose: np.ndarray):
    """工件相对该站位参考测量位置的 ``(平移 mm, 偏航差 °)``；没有参考返回 None。"""
    reference = entry.get("reference_measurement")
    if not reference:
        return None
    path = pathlib.Path(reference)
    if not path.is_absolute():
        path = REPO / path
    try:
        reference_pose = pose_from_payload(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, json.JSONDecodeError, KeyError, IndexError):
        return None
    offset = cur_pose[:3, 3] - reference_pose[:3, 3]
    return (float(np.linalg.norm(offset)) * 1000.0,
            abs(float(yaw_degrees_between(reference_pose, cur_pose))))


def rank_known_views(views: dict, yaw_deg: float,
                     cur_pose: np.ndarray | None = None,
                     tolerance: float | None = None) -> list[dict]:
    """容差内候选按"摆放一致性"排序（2026-11-19 所有者定）。

    为什么改：只按偏转角最近选站，11-18 时 −16（差 3.46°）以 0.09° 击败 −23（差 3.54°），
    但工件已偏离 −16 验证时的位置 80 mm（第三面只剩 2 个面），离 −23 的只有 11 mm。
    yaw 误差几乎相同时，应该选"工件离它验证位置更近"的那个站位。

    排序键（越小越优先）：
      ① 有参考测量的站位优先（group 0），没有的排后面（group 1）——有参考才能判断摆放；
      ② 归一化摆放偏差 max(平移/30 mm, yaw/3.5°)；
      ③ 偏转角误差。
    """
    if tolerance is None:
        tolerance = float(views.get("tolerance_deg", 12.0))
    ranked = []
    for key, entry in views.items():
        if not isinstance(entry, dict) or "tcp" not in entry:
            continue
        if entry.get("validated", True) is False:
            continue
        try:
            angle = float(key)
        except ValueError:
            continue
        error = abs(angle - yaw_deg)
        if error > tolerance + 1e-6:
            continue
        gap = placement_gap(entry, cur_pose) if cur_pose is not None else None
        if gap is None:
            ranked.append({"key": key, "entry": entry, "error_deg": error,
                           "gap_mm": None, "gap_deg": None,
                           "score": float("inf"), "group": 1})
        else:
            gap_mm, gap_deg = gap
            ranked.append({"key": key, "entry": entry, "error_deg": error,
                           "gap_mm": round(gap_mm, 2), "gap_deg": round(gap_deg, 2),
                           "score": round(max(gap_mm / PLACEMENT_MAX_MM,
                                              gap_deg / PLACEMENT_MAX_DEG), 4),
                           "group": 0})
    ranked.sort(key=lambda item: (item["group"], item["score"], item["error_deg"]))
    return ranked


def station_plan(views: dict, ref_pose: np.ndarray, cur_pose: np.ndarray,
                 T_tcp_cam: np.ndarray, family_gate_mm: float = 200.0) -> dict:
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

    # 0° 相对视角推导（与 nbv_from_reference.py 同一套数学）。放在选站之前：
    # 选站要按"站位存储位姿与当前推导位姿的差"排序，而不是只看偏转角最近。
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

    # 候选排序（2026-11-19 所有者定）：容差内不再只取 yaw 最近。
    #   主键 delta_mm = |站位存储位姿 − 当前推导位姿|：越小说明该站位的视角越贴合"当前
    #   这件工件"。11-18 的实测：−16 差 80.2 mm、−23 只差 10.9 mm，而两者 yaw 误差几乎相同
    #   （3.46° vs 3.54°）；按 yaw 选会选到 −16，第三面只剩 2 个面。
    #   次键：摆放检查超限的候选排后面（它的精测大概率只看到两个面）。
    nearest = None            # 无论是否在容差内，最近的站位（用于失败诊断）
    skipped_unvalidated = []  # 在容差内但未过精度门的站位
    candidates = []
    for key, entry in views.items():
        if not isinstance(entry, dict) or "tcp" not in entry:
            continue
        # 设计偏角：优先读 entry.yaw_deg（新拍站位可能与已有角度撞名，如 '+30@20261120'），
        # 没有该字段时退回解析 key。
        if entry.get("yaw_deg") is not None:
            angle = float(entry["yaw_deg"])
        else:
            try:
                angle = float(key)
            except ValueError:
                continue
        error = abs(angle - yaw)
        if nearest is None or error < nearest[0]:
            nearest = (error, key)
        if entry.get("validated", True) is False:
            if error <= tolerance:
                skipped_unvalidated.append((key, error))
            continue   # 未通过精度门的站位不参与自动选站（-45° 即此类）
        if error > tolerance + 1e-6:
            continue
        delta_mm_i = float(np.linalg.norm(
            np.asarray(entry["tcp"], dtype=float) - derived_tcp) * 1000.0)
        gap = placement_gap(entry, cur_pose)
        gap_bad = bool(gap and (gap[0] > PLACEMENT_MAX_MM + 1e-6
                                or gap[1] > PLACEMENT_MAX_DEG + 1e-6))
        candidates.append({
            "key": key,
            "entry": entry,
            "error_deg": round(error, 2),
            "delta_mm": round(delta_mm_i, 1),
            "gap_mm": round(gap[0], 1) if gap else None,
            "gap_deg": round(gap[1], 2) if gap else None,
            "placement_ok": None if gap is None else (not gap_bad),
            "blocked": gap_bad,
            "family": entry.get("family", "relative_zero"),
            "coverage": (float(entry["empirical_coverage"])
                         if entry.get("empirical_coverage") is not None else None),
            # 优先级：tier 1 = 新拍的首选站位；缺省 tier 2 = 现有站位（兜底）
            "tier": int(entry.get("tier", 2)),
        })
    # 选站判据（2026-11-19 所有者定，实测数据驱动）：
    #   **主判据 = 该站位实测的 ICP 覆盖率（empirical_coverage）**，偏转角只是过滤条件。
    #   为什么：11-19 真机实测，yaw 只差 1.2° 的 −23 覆盖率 0.88–0.93（拒），
    #   而差 5.8° 的 −16 覆盖率 0.9725（通过）。"偏转角更近"不等于"测得更好"。
    #   没有实测覆盖率的站位（如 −23）排在有数据的后面，按偏转角误差兜底。
    def rank_key(item):
        coverage = item["coverage"]
        has_coverage = 0 if coverage is not None else 1
        # 2026-11-24 修正：摆放检查超限（blocked）必须排在覆盖率之前。
        # 以前 blocked 只算出来参与诊断、没进排序键，导致 yaw −9.2° 时选中了
        # 0@20261120（工件离它参考位置 67 mm / 9.45°，精测必然只看到 2 个面），
        # 而真正匹配的 −9@20261120（6.2 mm / 0.38°）因为覆盖率 0.968 略低于
        # 0.975 被排到第 3 位。摆放检查不过 = 这个站位对当前工件就是错的，
        # 覆盖率再高也没有意义。
        return (item["tier"],            # ① 新拍的首选站位优先，现有站位兜底
                has_coverage,            # ② 有实测覆盖率的优先
                1 if item["blocked"] else 0,   # ③ 摆放检查超限的排后面
                -float(coverage) if coverage is not None else 0.0,   # ④ 覆盖率高的优先
                item["error_deg"],       # ⑤ 偏转角误差小的
                item["delta_mm"])        # ⑥ 与当前工件匹配度高的

    candidates.sort(key=rank_key)
    best = None
    if candidates:
        chosen = candidates[0]
        best = (chosen["error_deg"], chosen["key"], chosen["entry"])

    diagnostics = {
        "nearest_key": nearest[1] if nearest else None,
        "nearest_error_deg": round(nearest[0], 2) if nearest else None,
        "skipped_unvalidated": [(key, round(error, 2))
                                for key, error in skipped_unvalidated],
        "ranked": [{k: item[k] for k in ("key", "error_deg", "delta_mm", "coverage",
                                          "tier", "gap_mm", "gap_deg", "placement_ok")}
                   for item in candidates[:5]],
    }
    if best is None:
        return {"yaw_deg": yaw, "station": None,
                "derived_tcp": derived_tcp, "derived_quat": derived_quat,
                "stored_tcp": None, "stored_quat": None,
                "delta_mm": None, "delta_deg": None, "verdict": "no_station",
                **diagnostics}

    error, key, entry = best
    stored_tcp = np.asarray(entry["tcp"], dtype=float)
    stored_quat = np.asarray(entry["quat"], dtype=float)
    delta_mm = float(np.linalg.norm(stored_tcp - derived_tcp) * 1000.0)
    delta_deg = float(np.degrees(
        (R.from_quat(stored_quat) * R.from_quat(derived_quat).inv()).magnitude()))
    family = entry.get("family", "relative_zero")
    near_tolerance = float(views.get("near_tolerance_deg", tolerance))
    verdict = "station"
    if error > near_tolerance:
        # 远站位（偏差 > near_tolerance）：只作告警。门本身仍然适用——
        # 2026-11-14 所有者把门从 40 mm 放宽到 200 mm，因为相机绕工件转 12-15°
        # 本来就会产生 90-120 mm 的几何位移；真被搬走 200 mm 以上仍会被拦住。
        verdict = "station_far"
    if family == "relative_zero" and delta_mm > family_gate_mm:
        verdict = "station_moved"      # 覆盖 far：远站位也要过门
    return {"yaw_deg": yaw, "station": (error, key, entry),
            "derived_tcp": derived_tcp, "derived_quat": derived_quat,
            "stored_tcp": stored_tcp, "stored_quat": stored_quat,
            "delta_mm": delta_mm, "delta_deg": delta_deg, "verdict": verdict,
            "near_tolerance_deg": near_tolerance,
            "far_by_deg": round(max(0.0, error - near_tolerance), 2),
            **diagnostics}


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
