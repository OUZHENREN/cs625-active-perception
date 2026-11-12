#!/usr/bin/env python3
"""一条命令完成"观测 → 测量 → 质量判定 → base 系位姿"（效率模式观测环节的替代接口）。

它做的事
--------
    采集 N 组 → 三平面 + CAD 配准（含翻转消歧）→ base 系位姿 → 重复性与质量判定

质量门槛（不通过就以退出码 2 拒绝输出，下游不得使用）
    · 每组都要出位姿（三平面成功或兜底成功）
    · 覆盖率中位数 ≥ 0.95
    · 无翻转歧义（flip_check.ambiguous 全为 False）
    · 组间中心重复性 ≤ 5 mm、姿态重复性 ≤ 2°

用法
----
    source scripts/source_dev_env.sh --full
    python3 scripts/measure_object.py \
        --template ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL \
        --depth-window 0.5 1.2 --groups 4 --out runtime/measurements/workpiece.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp                                        # noqa: E402

MIN_COVERAGE = 0.95
# 精观测门（2026-11-12 收紧）：扶手与工件之间的插入间隙每侧只有 2 mm，
# 5 mm / 2° 的"重复性"根本保证不了能插进去。历史实测能做到 0.3–0.5 mm，
# 所以精观测要求 ≤1.0 mm / ≤0.5°；达不到就应当换视角重测，而不是带着误差去抓。
MAX_TRANSLATION_MM = 1.0
MAX_ROTATION_DEG = 0.5


def refine_pose_base(row: dict, directory, cad, bbox, base_cam,
                     depth_window=(0.41, 1.10)) -> dict:
    """在 D2 位姿上做 D3 细配准，并把结果写回 pose_base。"""
    import fine_pose as fp
    from scipy.spatial.transform import Rotation as _R
    if base_cam is None:
        return row
    pose = np.eye(4)
    pose[:3, :3] = _R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    pose[:3, 3] = row["pose_base"]["translation"]
    depth, k = cp.load_group(directory)
    points, _ = cp.extract_workpiece(depth, k, *depth_window)
    pose_cam = np.linalg.inv(base_cam) @ pose
    points = cp.reject_with_pose(points, pose_cam, cad, threshold_m=0.02)
    refined_cam = fp.fine_icp(points, cad, pose_cam)
    refined = base_cam @ refined_cam
    fit_before, _ = fp.coverage(points, cad, pose_cam)
    fit_after, rmse_after = fp.coverage(points, cad, refined_cam)
    shift = float(np.linalg.norm(refined_cam[:3, 3] - pose_cam[:3, 3]) * 1000.0)
    if shift > 20.0 or fit_after + 1e-6 < fit_before:
        row["refine"] = {"accepted": False, "shift_mm": round(shift, 2),
                         "coverage_before": round(fit_before, 4),
                         "coverage_after": round(fit_after, 4)}
        return row
    centre_obj = (bbox[0] + bbox[1]) / 2
    row["pose_base"] = {
        "translation": [round(float(v), 5) for v in refined[:3, 3]],
        "quaternion": [round(float(v), 6) for v in
                       _R.from_matrix(np.array(refined[:3, :3])).as_quat()],
        "centre_mm": [round(float(v) * 1000, 1) for v in
                      (refined[:3, :3] @ centre_obj + refined[:3, 3])],
    }
    row["refine"] = {"accepted": True, "shift_mm": round(shift, 2),
                     "coverage_before": round(fit_before, 4),
                     "coverage_after": round(fit_after, 4),
                     "rmse_mm": round(rmse_after, 2)}
    return row


def publish_stage(node, stage: str, payload: dict) -> None:
    """把视觉流程的每一步发到 /vision_stage（面板订阅它，把"黑盒"摊开）。"""
    try:
        from std_msgs.msg import String
        if getattr(node, "vision_pub", None) is None:
            node.vision_pub = node.create_publisher(String, "/vision_stage", 10)
        payload = dict(payload)
        payload["stage"] = stage
        payload["stamp"] = time.time()
        node.vision_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
    except Exception:  # noqa: BLE001 - 进度推送不能影响测量
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preset", choices=["workpiece", "slot"], default="workpiece",
                        help="workpiece=弹仓（用扶手判据）；slot=插槽夹具（用整体覆盖率判据）")
    parser.add_argument("--template", default=None)
    parser.add_argument("--no-refine", dest="refine", action="store_false",
                        help="fine 阶段不做 D3 细配准（默认做）")
    parser.add_argument("--stage", choices=["coarse", "fine"], default="fine",
                        help="coarse=只要能给出一个'够用来算精观测位'的粗略位姿（5–20 mm 可接受）；"
                             "fine=严格质量门（给最终位姿）")
    parser.add_argument("--groups", type=int, default=4, help="采集组数（每组 3 帧取中位）")
    parser.add_argument("--depth-window", type=float, nargs=2, default=None,
                        metavar=("MIN_M", "MAX_M"),
                        help="默认用 preset 的建议深度窗（工件 0.41–1.10 / 插槽 0.48–1.35）")
    parser.add_argument("--observe-pose", default=None,
                        help="观测位名字（config 里）；给了就先做取景检查，BAD 直接拒绝采集")
    parser.add_argument("--object-pose", default=None,
                        help="工件的上一帧测量结果（取景检查用）；插槽不需要")
    parser.add_argument("--cad-points", type=int, default=40000)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    presets = {
        "workpiece": {
            "template": str(pathlib.Path.home()
                            / "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL"),
            "window": (0.41, 1.10),      # frame_check(workpiece_fine) 建议值
        },
        "slot": {
            "template": str(REPO / "src/cs625_simulation/assets/cs625_real_cell/slot_fixture"
                            / "meshes/slot_fixture_collision.stl"),
            "window": (0.48, 1.35),      # frame_check(slot_coarse) 建议值
        },
    }
    template_arg = args.template or presets[args.preset]["template"]
    use_feature = args.preset == "workpiece"
    if args.template is None:
        print(f"preset {args.preset} → 模板 {template_arg}，特征判据 {'用' if use_feature else '不用'}")

    if args.observe_pose:
        import frame_check
        try:
            frame = frame_check.check(args.preset, args.observe_pose, args.object_pose)
        except (KeyError, ValueError) as error:
            print(f"取景检查跳过：{error}")
            frame = None
        if frame:
            print(f"取景检查：{frame['verdict']}，{frame['corners_inside']}/8 角入画，"
                  f"深度 {frame['depth_range_m'][0]:.2f}–{frame['depth_range_m'][1]:.2f} m，"
                  f"建议窗 {frame['suggested_window_m']}")
            if frame["verdict"] == "BAD":
                print("取景判定 BAD：先调整观测位，不采集", file=sys.stderr)
                return 2

    stamp = time.strftime("%Y%m%d_%H%M%S")
    dataset = REPO / "runtime" / "vision_dataset" / f"measure_{stamp}"
    capture = subprocess.run(
        [sys.executable, "-u", str(REPO / "scripts" / "vision_capture.py"),
         "--groups", str(args.groups), "--frames", "3", "--tag", "m",
         "--out", str(dataset)], capture_output=True, text=True, timeout=600)
    if "产物" not in (capture.stdout or ""):
        print("采集失败：\n" + (capture.stdout or "")[-800:], file=sys.stderr)
        return 2
    print(f"采集完成：{dataset}")

    template_path = pathlib.Path(template_arg).expanduser()
    bbox = cp.load_template_bbox(template_path)
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(template_path))
    cad = cp.sample_mesh(mesh, args.cad_points, seed=7)
    base_cam = cp._read_base_camera()
    if base_cam is None:
        print("读不到 TCP，无法换算到 base 系", file=sys.stderr)
        return 2

    depth_window = tuple(args.depth_window or presets[args.preset]["window"])
    focus = None
    prior_pose_base = None
    if args.preset == "workpiece":
        # 工件侧也可能出现"桌面/背景与工件连成一片"（实测 738k 点的连通域覆盖整张桌子）。
        # 在已知工件附近做球裁最稳；中心/半径由 config 的 workpiece_focus 提供。
        import yaml
        config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text()) or {}
        spec = config.get("workpiece_focus")
        if spec and len(spec) >= 4:
            focus = (np.array(spec[:3], dtype=float), float(spec[3]))
            print(f"工件球裁：中心 {np.round(focus[0], 3)}，半径 {focus[1]:.2f} m")
    if args.preset == "slot":
        # 夹具比工件大得多，深度窗内的最大连通域往往是桌面/背景，必须按已知位置球裁。
        # 夹具模型原点就是 config 的 slot_origin，姿态是 slot_origin_q。
        import yaml
        from scipy.spatial.transform import Rotation as _Rotation
        config = yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
        bbox_centre = (bbox[0] + bbox[1]) / 2
        T_base_slot = np.eye(4)
        T_base_slot[:3, :3] = _Rotation.from_quat(config["slot_origin_q"]).as_matrix()
        T_base_slot[:3, 3] = config["slot_origin"]
        centre = T_base_slot[:3, :3] @ bbox_centre + T_base_slot[:3, 3]
        radius = float(np.linalg.norm(bbox[1] - bbox[0]) / 2 + 0.06)
        focus = (centre, radius)
        prior_pose_base = T_base_slot.copy()      # 夹具位置已知 → 走先验引导配准
        print(f"夹具先验：中心 {np.round(centre, 3)}，球裁半径 {radius:.2f} m")
    print(f"深度窗：{depth_window[0]:.2f}–{depth_window[1]:.2f} m")
    # 视觉流程进度：有 rclpy 就发 /vision_stage，没有就只打印
    coarse_stage = args.stage == "coarse"
    stage_node = None
    try:
        import rclpy
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()
        stage_node = rclpy.create_node("measure_object_stages")
    except Exception:  # noqa: BLE001
        stage_node = None

    def progress(stage: str, payload: dict) -> None:
        if stage_node is not None:
            publish_stage(stage_node, stage, payload)

    results = []
    table_heights: list[float] = []
    for index, directory in enumerate(sorted(dataset.glob("g*"))):
        prior = None if not table_heights else float(np.median(table_heights))
        row = cp.analyse_group(directory, bbox, cad, depth_window, base_cam,
                               table_prior=prior,
                                        focus=focus, use_feature=use_feature,
                               prior_pose_base=prior_pose_base,
                               progress=progress)
        # 记录采集时的相机位：没有它，事后无法在离线数据上复算（机械臂一动就不可复现）
        if base_cam is not None:
            row["T_base_cam"] = [[round(float(v), 6) for v in line] for line in base_cam]
        # 精观测：在 D2 位姿上再做一次细配准（D3）。
        # 实测组间分散 3.44 mm → 0.08 mm（同一批数据），是达到 1.0 mm/0.5° 门的关键。
        if (not coarse_stage) and args.refine and "pose_base" in row:
            row = refine_pose_base(row, directory, cad, bbox, base_cam,
                                   depth_window=depth_window)
        height = cp.LAST_TABLE_HEIGHT[0]
        if height is not None:
            table_heights.append(float(height))
            row["table_height_m"] = round(float(height), 4)
        results.append(row)
    good = [row for row in results if "pose_base" in row]
    if not good:
        print("所有组都没出位姿", file=sys.stderr)
        return 2

    # 只统计"可信组"：位姿有效 + 对称解未判不可用。
    # 退化组（例如只有两个可见面）的解本身是错的，若把它算进重复性，
    # 会把一次本来合格的测量判成不合格（实测：3 组正解 0.1°，1 组退化把重复性拉到 3.9°）。
    def credible(row) -> bool:
        check = row.get("symmetry_check") or {}
        if coarse_stage:
            # 粗观测只要求"位姿大致可信"：覆盖率够、没有离谱的退化即可，
            # 扶手判据（需要扶手清晰可见）在这里不作硬性要求。
            return float(row.get("pose_fitness", 0.0)) >= 0.70
        return bool(row.get("pose_valid", True)) and bool(check.get("valid", True))

    credible_rows = [row for row in good if credible(row)]
    dropped = len(good) - len(credible_rows)
    # 粗观测只用来"定视角"，两组可信就够；精观测必须要三组，否则重复性没意义。
    min_credible = 2 if coarse_stage else 3
    if len(credible_rows) < min_credible:
        summary_reason = (f"可信组只有 {len(credible_rows)} 组（需要 ≥{min_credible}），"
                          f"其余 {dropped} 组退化")
        print(summary_reason, file=sys.stderr)
        if stage_node is not None:
            publish_stage(stage_node, "gate", {
                "verdict": "REFUSE", "coverage": 0.0, "translation_mm": 0.0,
                "rotation_deg": 0.0, "credible": len(credible_rows),
                "group_total": len(results), "reasons": [summary_reason]})
        return 2

    centres = np.array([row["pose_base"]["centre_mm"] for row in credible_rows])
    from scipy.spatial.transform import Rotation as R
    # 姿态重复性必须用【最终位姿】pose_base——D3 只更新 pose_base，
    # 用相机系的 pose（D2 结果）会把已修正的差异当成"重复性差"（实测报 1.31°，
    # 而最终位姿之间只有 0.05–0.08°）。
    rotations = [R.from_quat(row["pose_base"]["quaternion"]) for row in credible_rows]
    translation_spread = float(np.linalg.norm(centres.max(axis=0) - centres.min(axis=0)))
    rotation_spread = max(
        float(np.degrees((rotations[i].inv() * rotations[j]).magnitude()))
        for i in range(len(rotations)) for j in range(len(rotations)))
    coverage = float(np.median([row.get("pose_fitness", 0.0) for row in credible_rows]))
    ambiguous = sum(1 for row in credible_rows if row.get("symmetry_check", {}).get("ambiguous"))

    verdict = "OK"
    reasons = []
    if len(good) < args.groups:
        verdict = "REFUSE"
        reasons.append(f"只有 {len(good)}/{args.groups} 组出位姿")
    min_coverage = 0.70 if coarse_stage else MIN_COVERAGE
    max_translation = 20.0 if coarse_stage else MAX_TRANSLATION_MM
    max_rotation = 20.0 if coarse_stage else MAX_ROTATION_DEG
    if coverage < min_coverage:
        verdict = "REFUSE"
        reasons.append(f"覆盖率中位 {coverage:.3f} < {min_coverage}")
    if ambiguous:
        verdict = "REFUSE"
        reasons.append(f"{ambiguous} 组存在翻转歧义")
    if translation_spread > max_translation:
        verdict = "REFUSE"
        reasons.append(f"中心重复性 {translation_spread:.1f} mm > {max_translation}")
    if rotation_spread > max_rotation:
        verdict = "REFUSE"
        reasons.append(f"姿态重复性 {rotation_spread:.2f}° > {max_rotation}")

    summary = {
        "stamp": stamp,
        "dataset": str(dataset),
        "groups": len(results),
        "pose_base_centre_mm": [round(float(v), 1) for v in centres.mean(axis=0)],
        "repeatability": {"translation_mm": round(translation_spread, 2),
                          "rotation_deg": round(rotation_spread, 3)},
        "coverage_median": round(coverage, 4),
        "flip_ambiguous_groups": ambiguous,
        "degenerate_groups_dropped": dropped,
        "stage": args.stage,
        "T_base_cam": ([[round(float(v), 6) for v in line] for line in base_cam]
                       if base_cam is not None else None),
        "verdict": verdict,
        "reasons": reasons,
        "per_group": [{k: row.get(k) for k in
                       ("group", "planes", "pose_fitness", "pose_rmse_mm", "pose_base", "symmetry_check")}
                      for row in results],
    }
    print(json.dumps({k: summary[k] for k in
                      ("pose_base_centre_mm", "repeatability", "coverage_median",
                       "flip_ambiguous_groups", "degenerate_groups_dropped",
                       "verdict", "reasons")},
                     ensure_ascii=False, indent=2))
    if stage_node is not None:
        publish_stage(stage_node, "gate", {
            "verdict": verdict, "coverage": round(coverage, 3),
            "translation_mm": round(translation_spread, 2),
            "rotation_deg": round(rotation_spread, 2),
            "credible": len(credible_rows), "group_total": len(results),
            "reasons": reasons})
    if args.out:
        pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.out).write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print(f"写入 {args.out}")
    return 0 if verdict == "OK" else 2


if __name__ == "__main__":
    raise SystemExit(main())
