#!/usr/bin/env python3
"""把本次插槽测量与历史基线对比，判断"插槽是否还在老位置"（2026-11-24 所有者要求）。

为什么需要它
------------
插入动作用的是固定参考位姿（runtime/slot_reference_verified.json / config
teaching_slot_reference），插槽一旦被碰动，照固定位姿插就会保护停止。
所以 observe:slot_fine 阶段在移动到精观测位后跑这个检查：
拿 slot_coarse 阶段写出的 runtime/measurements/slot_latest.json 与
"上一次确认过的测量基线"（runtime/slot_measurement_baseline.json）比：
  · 差距 ≤ warn（默认 3 mm / 1°）        → PASS
  · warn < 差距 ≤ tol（默认 10 mm / 2°） → PASS，但打印警告（插槽可能被碰过）
  · 差距 > tol                            → REFUSE（插入链路不应继续）

为什么用"测量基线自比"而不是直接比 slot_reference_verified.json
--------------------------------------------------------------
那份参考位姿是示教链反算出来的插入目标，坐标系/原点约定与 measure_object 的
CAD 模板不一定相同（2026-11-24 实测：同一插槽，测量值比参考位姿差 x≈36 mm 的
常数）。同一条测量管线自比（今天的 slot_latest vs 上次的 slot_latest）不受
约定影响，才能真正回答"插槽动了没有"。参考位姿的差距只打印、不参与判定。

基线怎么来
----------
第一次使用（或插槽被重新固定后）由**所有者在场确认**"插槽确实在标准位置"，
然后执行：
    python3 scripts/check_slot_reference.py --record
把当时的 slot_latest.json 存成基线。基线文件带 stamp 与来源说明。

用法
----
    python3 scripts/check_slot_reference.py \\
        --measurement runtime/measurements/slot_latest.json
    python3 scripts/check_slot_reference.py --record   # 把当前测量存为基线

退出码：0 = 在容差内（可能带 WARN）；2 = 超出容差 REFUSE；3 = 输入缺失/无基线。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
BASELINE = REPO / "runtime" / "slot_measurement_baseline.json"
REFERENCE = REPO / "runtime" / "slot_reference_verified.json"


def read_pose(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray] | None:
    """读测量 JSON 的 per_group[0].pose_base，返回 (translation, quat_xyzw)。"""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    rows = payload.get("per_group") or []
    if not rows:
        return None
    pose = rows[0].get("pose_base") or {}
    translation = pose.get("translation")
    quaternion = pose.get("quaternion")
    if translation is None or quaternion is None or len(translation) != 3 \
            or len(quaternion) != 4:
        return None
    return np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)


def gap_mm_deg(a: tuple[np.ndarray, np.ndarray],
               b: tuple[np.ndarray, np.ndarray]) -> tuple[float, float]:
    position_mm = float(np.linalg.norm(a[0] - b[0])) * 1000.0
    angle = float(np.abs((R.from_quat(a[1]) * R.from_quat(b[1]).inv()).magnitude()))
    return position_mm, np.degrees(angle)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--measurement", default=str(REPO / "runtime" / "measurements"
                                                    / "slot_latest.json"))
    parser.add_argument("--record", action="store_true",
                        help="把当前测量存为基线（所有者确认插槽在标准位置后执行）")
    parser.add_argument("--tol-mm", type=float, default=10.0)
    parser.add_argument("--tol-deg", type=float, default=2.0)
    parser.add_argument("--warn-mm", type=float, default=3.0)
    parser.add_argument("--warn-deg", type=float, default=1.0)
    args = parser.parse_args()

    measurement = read_pose(pathlib.Path(args.measurement))
    if measurement is None:
        print(f"读不到测量 {args.measurement}（或 per_group[0].pose_base 缺失）")
        return 3

    if args.record:
        payload = {"per_group": [{"group": "baseline",
                                  "pose_base": {"translation": measurement[0].tolist(),
                                                "quaternion": measurement[1].tolist()}}],
                   "stamp": time.strftime("%Y%m%d_%H%M%S"),
                   "source": args.measurement,
                   "note": "插槽测量基线：所有者确认插槽在标准位置后由 "
                           "scripts/check_slot_reference.py --record 写入"}
        BASELINE.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        print("已记录插槽测量基线：")
        print(f"  translation {np.round(measurement[0], 4).tolist()}")
        print(f"  quaternion  {np.round(measurement[1], 4).tolist()}")
        return 0

    baseline = read_pose(BASELINE)
    if baseline is None:
        print("没有基线（runtime/slot_measurement_baseline.json）：无法判断插槽是否移动。")
        print("所有者确认插槽在标准位置后执行：")
        print("  python3 scripts/check_slot_reference.py --record")
        return 3

    position_mm, angle_deg = gap_mm_deg(measurement, baseline)
    print(f"插槽 vs 基线（{BASELINE.name}）：平移 {position_mm:.1f} mm / "
          f"姿态 {angle_deg:.2f}°")

    # 参考位姿只打印、不参与判定（坐标系约定不同，见 docstring）
    reference = read_pose(REFERENCE)
    if reference is not None:
        ref_mm, ref_deg = gap_mm_deg(measurement, reference)
        print(f"  （信息）vs 插入参考位姿 slot_reference_verified.json："
              f"{ref_mm:.1f} mm / {ref_deg:.2f}° —— 仅供参考，不参与判定")

    if position_mm > args.tol_mm or angle_deg > args.tol_deg:
        print(f"REFUSE：插槽位置差 {position_mm:.1f} mm / {angle_deg:.2f}° 超过容差 "
              f"（{args.tol_mm:.0f} mm / {args.tol_deg:.0f}°）。插槽可能被碰动，"
              "插入链路不应继续；请人工检查后再 --record 新基线。")
        return 2
    if position_mm > args.warn_mm or angle_deg > args.warn_deg:
        print(f"WARN：插槽位置差 {position_mm:.1f} mm / {angle_deg:.2f}° 在容差内"
              f"但超过提醒线（{args.warn_mm:.0f} mm / {args.warn_deg:.0f}°）。")
        return 0
    print("PASS：插槽还在老位置。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
