#!/usr/bin/env python3
"""从"已验证站位"推导当前工件角度下的相机位姿（完整刚体变换，2026-11-20）。

为什么需要它
------------
工件在桌上绕某个角点旋转时，**实际会带一点倾斜（roll/pitch）**。之前的做法只取
两次测量的 yaw 标量差、绕竖直轴做纯旋转，丢掉了相对位姿里的倾斜与残余平移，
于是每拍一次就偏一点，越拍越差。

这里改用**完整 SE(3) 相对变换**：

    T_cam_now = T_obj_now · inv(T_obj_base) · T_cam_base

其中
    T_obj_base / T_cam_base  来自某个已验证站位（tier 1 优先）拍摄时的
                             工件位姿 + 相机位姿（同一次测量，同一时刻）
    T_obj_now                当前工件的实测位姿（粗观测即可）

几何含义：把"相机相对工件的那个视角"整体搬到当前工件上——倾斜、平移、旋转一起跟随。
工件绕角点还是绕中心转**不影响结果**，因为两次用的都是同一个参考点（测量出的 CAD 原点），
刚体变换会把这个选择消掉。

用法
----
    # 只打印推导结果
    python3 scripts/derive_from_station.py \
        --current runtime/measurements/workpiece_at_m35_coarse.json

    # 指定参考站位
    python3 scripts/derive_from_station.py \
        --current ...json --station -29@20261120

    # 直接规划（不动臂）
    python3 scripts/derive_from_station.py --current ...json --plan
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import nbv_views  # noqa: E402

COMMON = REPO / "config" / "tasks" / "common.yaml"
PLANNER = REPO / "scripts" / "plan_pose_ik.py"


def station_entries(views: dict) -> dict:
    return {key: value for key, value in views.items()
            if isinstance(value, dict) and "tcp" in value
            and value.get("reference_measurement")}


def pose_from_station(entry: dict) -> np.ndarray:
    """站位记录的相机位姿（TCP）→ 4x4。"""
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(entry["quat"]).as_matrix()
    matrix[:3, 3] = np.asarray(entry["tcp"], dtype=float)
    return matrix


def derive(views: dict, current_measurement: pathlib.Path,
           station: str | None = None,
           args_strategy: str = "best") -> tuple[np.ndarray, dict, str]:
    """返回 (T_cam_now 4x4, 使用的站位条目, 站位键)。"""
    entries = station_entries(views)
    if station:
        if station not in entries:
            raise SystemExit(f"站位 {station} 不存在或缺少 reference_measurement")
        key = station
    else:
        # 默认用**质量最好的锚点**推导，而不是"上一个站位"：
        # 链式继承（−19→−29→−36→…）会把每一跳的小误差累加起来，越拍越偏；
        # 固定锚点每次都从同一个已知良好视角出发，误差不累积。
        # 加 --strategy nearest 可退回"最近角度"的旧行为。
        ranked = []
        for name, entry in entries.items():
            tier = entry.get("tier", 2)
            coverage = float(entry.get("empirical_coverage") or 0.0)
            if args_strategy == "nearest":
                yaw = entry.get("yaw_deg")
                if yaw is None:
                    continue
                payload = json.loads(
                    pathlib.Path(current_measurement).read_text(encoding="utf-8"))
                yaw_now = payload.get("workpiece_yaw_deg")
                if yaw_now is None:
                    yaw_now = payload["per_group"][0].get("workpiece_yaw_deg")
                ranked.append((abs(float(yaw) - float(yaw_now)), tier, -coverage, name))
            else:
                # best：tier 1 优先，再看覆盖率
                ranked.append((tier, -coverage, 0.0, name))
        if not ranked:
            raise SystemExit("没有可用站位")
        ranked.sort()
        key = ranked[0][3]
    entry = entries[key]

    T_obj_base = nbv_views.load_pose(REPO / entry["reference_measurement"])
    T_obj_now = nbv_views.load_pose(pathlib.Path(current_measurement))
    T_cam_base = pose_from_station(entry)
    T_cam_now = T_obj_now @ np.linalg.inv(T_obj_base) @ T_cam_base
    return T_cam_now, entry, key


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--current", required=True, help="当前工件的测量结果 JSON")
    parser.add_argument("--station", help="参考站位键（默认自动选最近的一个）")
    parser.add_argument("--strategy", choices=["best", "nearest"], default="best",
                        help="best=从质量最好的锚点推导（默认，误差不累积）；nearest=从角度最近的站位推导")
    parser.add_argument("--plan", action="store_true", help="顺便做一次规划可达性检查")
    parser.add_argument("--output", default="/tmp/derived_from_station.json")
    args = parser.parse_args()

    config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    views = config["workpiece_views"]
    T_cam_now, entry, key = derive(views, pathlib.Path(args.current), args.station,
                                   args.strategy)

    tcp = T_cam_now[:3, 3]
    quat = R.from_matrix(T_cam_now[:3, :3]).as_quat()
    print(f"参考站位：{key}（yaw {entry.get('yaw_deg')}°，tier {entry.get('tier', 2)}，"
          f"覆盖率 {entry.get('empirical_coverage')}）")
    print(f"参考工件位姿：{entry['reference_measurement']}")
    print(f"推导相机 tcp : {[round(float(v), 5) for v in tcp]}")
    print(f"推导相机 quat: {[round(float(v), 6) for v in quat]}")

    if not args.plan:
        return 0
    command = [sys.executable, str(PLANNER),
               *[f"{float(v):.5f}" for v in tcp],
               *[f"{float(v):.6f}" for v in quat],
               "--output", args.output]
    result = subprocess.run(command, capture_output=True, text=True, timeout=300)
    for line in (result.stdout or "").strip().splitlines()[-6:]:
        print("  " + line)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
