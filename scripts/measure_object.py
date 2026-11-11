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
MAX_TRANSLATION_MM = 5.0
MAX_ROTATION_DEG = 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preset", choices=["workpiece", "slot"], default="workpiece",
                        help="workpiece=弹仓（用扶手判据）；slot=插槽夹具（用整体覆盖率判据）")
    parser.add_argument("--template", default=None)
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
    results = []
    for directory in sorted(dataset.glob("g*")):
        results.append(cp.analyse_group(directory, bbox, cad, depth_window, base_cam,
                                        focus=focus, use_feature=use_feature,
                                        prior_pose_base=prior_pose_base))
    good = [row for row in results if "pose_base" in row]
    if not good:
        print("所有组都没出位姿", file=sys.stderr)
        return 2

    centres = np.array([row["pose_base"]["centre_mm"] for row in good])
    from scipy.spatial.transform import Rotation as R
    rotations = [R.from_matrix(np.array(row["pose"]["rotation"])) for row in good]
    translation_spread = float(np.linalg.norm(centres.max(axis=0) - centres.min(axis=0)))
    rotation_spread = max(
        float(np.degrees((rotations[i].inv() * rotations[j]).magnitude()))
        for i in range(len(rotations)) for j in range(len(rotations)))
    coverage = float(np.median([row.get("pose_fitness", 0.0) for row in good]))
    ambiguous = sum(1 for row in good if row.get("symmetry_check", {}).get("ambiguous"))

    verdict = "OK"
    reasons = []
    if len(good) < args.groups:
        verdict = "REFUSE"
        reasons.append(f"只有 {len(good)}/{args.groups} 组出位姿")
    if coverage < MIN_COVERAGE:
        verdict = "REFUSE"
        reasons.append(f"覆盖率中位 {coverage:.3f} < {MIN_COVERAGE}")
    if ambiguous:
        verdict = "REFUSE"
        reasons.append(f"{ambiguous} 组存在翻转歧义")
    if translation_spread > MAX_TRANSLATION_MM:
        verdict = "REFUSE"
        reasons.append(f"中心重复性 {translation_spread:.1f} mm > {MAX_TRANSLATION_MM}")
    if rotation_spread > MAX_ROTATION_DEG:
        verdict = "REFUSE"
        reasons.append(f"姿态重复性 {rotation_spread:.2f}° > {MAX_ROTATION_DEG}")

    summary = {
        "stamp": stamp,
        "dataset": str(dataset),
        "groups": len(results),
        "pose_base_centre_mm": [round(float(v), 1) for v in centres.mean(axis=0)],
        "repeatability": {"translation_mm": round(translation_spread, 2),
                          "rotation_deg": round(rotation_spread, 3)},
        "coverage_median": round(coverage, 4),
        "flip_ambiguous_groups": ambiguous,
        "verdict": verdict,
        "reasons": reasons,
        "per_group": [{k: row.get(k) for k in
                       ("group", "planes", "pose_fitness", "pose_rmse_mm", "pose_base", "symmetry_check")}
                      for row in results],
    }
    print(json.dumps({k: summary[k] for k in
                      ("pose_base_centre_mm", "repeatability", "coverage_median",
                       "flip_ambiguous_groups", "verdict", "reasons")},
                     ensure_ascii=False, indent=2))
    if args.out:
        pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.out).write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print(f"写入 {args.out}")
    return 0 if verdict == "OK" else 2


if __name__ == "__main__":
    raise SystemExit(main())
