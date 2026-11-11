#!/usr/bin/env python3
"""标定"测量用的 CAD 系"与"示教用的 box 系"之间的固定变换（workpiece_frame_offset）。

为什么需要它
------------
- `measure_object` 输出的是 T_base_object（**CAD 系**：原点在模型导出原点）；
- 示教常数 `grasp/pre_grasp` 是按 **box 系**（RVS 那套）标定的；
- 两者相差一个固定变换。没标定时抓取点只能用常数；标定后抓取点就能跟着实测位姿走。

标定前提（重要）
----------------
工件必须处在**示教时的那一位姿**（即 `workpiece_reference` 描述的那个位姿），
否则算出来的是"当前位姿与参考位姿的差"，不是坐标系之间的固定变换。

用法
----
    # 1) 工件摆回示教位姿 → 在 workpiece_fine 观测位跑一次测量
    python3 scripts/measure_object.py --preset workpiece --observe-pose workpiece_fine \
        --object-pose runtime/measurements/workpiece_latest.json --groups 3 \
        --out runtime/measurements/workpiece_calib.json

    # 2) 确认工件确实在示教位姿后，打印要写进配置的 offset
    python3 scripts/calibrate_workpiece_frame.py \
        --measurement runtime/measurements/workpiece_calib.json --confirm

输出不会自动写配置：请把打印出来的 `workpiece_frame_offset` 复制到
`config/tasks/common.yaml`（或模式文件）里，再跑一次 plan-only 复核。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--measurement", required=True, help="measure_object 的输出 JSON")
    parser.add_argument("--config", default=str(REPO / "config/tasks/common.yaml"))
    parser.add_argument("--confirm", action="store_true",
                        help="确认工件此刻就在示教位姿（不确认则拒绝计算）")
    args = parser.parse_args()

    if not args.confirm:
        print("拒绝计算：请先确认工件处于示教位姿，再加 --confirm", file=sys.stderr)
        return 2

    data = json.loads(pathlib.Path(args.measurement).read_text(encoding="utf-8"))
    group = (data.get("per_group") or [{}])[0]
    place = group.get("pose_base") or {}
    translation = place.get("translation")
    quaternion = place.get("quaternion")
    if translation is None or quaternion is None:
        print("测量文件里没有 pose_base（需要用 --preset workpiece 且通过质量门）", file=sys.stderr)
        return 2

    config = yaml.safe_load(pathlib.Path(args.config).read_text(encoding="utf-8"))
    reference = config.get("workpiece_reference") or {}
    if not reference.get("xyz"):
        print("配置里没有 workpiece_reference，无法标定", file=sys.stderr)
        return 2

    measured = np.eye(4)
    measured[:3, :3] = R.from_quat(quaternion).as_matrix()
    measured[:3, 3] = translation
    box = np.eye(4)
    box[:3, :3] = R.from_quat(reference["quat"]).as_matrix()
    box[:3, 3] = reference["xyz"]
    offset = np.linalg.inv(measured) @ box

    translation_mm = np.round(offset[:3, 3] * 1000.0, 1).tolist()
    quat = R.from_matrix(offset[:3, :3]).as_quat()
    angle = float(np.degrees(np.linalg.norm(R.from_matrix(offset[:3, :3]).as_rotvec())))
    print("测量位姿 (CAD 系, base):", np.round(measured[:3, 3], 4))
    print("参考位姿 (box 系, base):", np.round(box[:3, 3], 4))
    print(f"两者距离 {np.linalg.norm(measured[:3, 3] - box[:3, 3]) * 1000:.1f} mm，"
          f"姿态差 {angle:.2f}°")
    print()
    print("把下面这段写进 config/tasks/common.yaml（顶层 key 即可）：")
    print(f"workpiece_frame_offset: [{offset[0,3]:.6f}, {offset[1,3]:.6f}, {offset[2,3]:.6f}, "
          f"{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
    print(f"# 平移 {translation_mm} mm，旋转 {angle:.2f}°")
    print()
    print("写好后跑一次 plan-only 复核：")
    print("  python3 scripts/run_task.py --scope load --mode efficient --world real --plan-only")
    print("  日志里 [grasp target] 应显示“已按实测位姿平移 …”，且平移量应在几十毫米以内。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
