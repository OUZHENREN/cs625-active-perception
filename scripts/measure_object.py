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
    parser.add_argument("--template", required=True)
    parser.add_argument("--groups", type=int, default=4, help="采集组数（每组 3 帧取中位）")
    parser.add_argument("--depth-window", type=float, nargs=2, default=(0.5, 1.2),
                        metavar=("MIN_M", "MAX_M"))
    parser.add_argument("--cad-points", type=int, default=40000)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

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

    template_path = pathlib.Path(args.template).expanduser()
    bbox = cp.load_template_bbox(template_path)
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(template_path))
    cad = cp.sample_mesh(mesh, args.cad_points, seed=7)
    base_cam = cp._read_base_camera()
    if base_cam is None:
        print("读不到 TCP，无法换算到 base 系", file=sys.stderr)
        return 2

    results = []
    for directory in sorted(dataset.glob("g*")):
        results.append(cp.analyse_group(directory, bbox, cad, tuple(args.depth_window), base_cam))
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
    ambiguous = sum(1 for row in good if row.get("flip_check", {}).get("ambiguous"))

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
                       ("group", "planes", "pose_fitness", "pose_rmse_mm", "pose_base", "flip_check")}
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
