#!/usr/bin/env python3
"""一键到"插槽精观测位"（= 示教 S0，2026-11-26 所有者定）。

为什么需要它
------------
插槽观测位在我们仓库里曾有两套值：
  · 效率模式旧常量 [0.7036, −0.0945, 0.8146] —— 离插槽 0.651 m（六模式时代遗留）
  · **教学 S0** [1.084404, 0.04897, 0.614883] —— 离插槽 0.252 m（真正的精观测位）
所有者 2026-11-26 定：**精观测位 = S0**，粗观测位不变（手册第 79 条）。

本脚本从**当前位姿**规划到 S0 并执行；执行时带
`--phase observe:slot_fine`，该名字在 `SAFE_TRANSIT_PHASES` 白名单里
（所有者 2026-11-26 批准"粗观测位→精观测位不分段"），因此：
  · 不按 25° 自动分段（一次走完）；
  · 但**所有安全闸照旧**：工作空间/关节绝对区间/碰撞/翻转拒绝/在线看门狗/到位复核。

用法
----
    python3 scripts/goto_slot_fine.py              # 只规划，打印关节行程
    python3 scripts/goto_slot_fine.py --execute    # 真正动臂
退出码：0 成功；2 配置/规划失败；4 执行失败。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
COMMON = REPO / "config" / "tasks" / "common.yaml"
PLANNER = REPO / "scripts" / "plan_pose_ik.py"
EXECUTOR = REPO / "scripts" / "execute_assembly_chunks.py"
PLAN_FILE = REPO / "runtime" / "plan_goto_slot_fine.json"
PHASE = "observe:slot_fine"


def slot_fine_pose() -> tuple[list[float], list[float]]:
    import yaml
    config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    pose = (config.get("observe") or {}).get("slot_fine") or {}
    if not pose.get("xyz") or not pose.get("quat"):
        raise SystemExit("config/tasks/common.yaml 缺少 observe.slot_fine")
    return [float(v) for v in pose["xyz"]], [float(v) for v in pose["quat"]]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--execute", action="store_true", help="真正移动机械臂（默认只规划）")
    args = parser.parse_args()

    xyz, quat = slot_fine_pose()
    print(f"目标：插槽精观测位 S0 = {[round(v, 4) for v in xyz]}")
    print(f"      姿态 = {[round(v, 4) for v in quat]}")
    plan = subprocess.run(
        [sys.executable, str(PLANNER), *[f"{v:.6f}" for v in xyz + quat],
         "--allow-outside-tcp-box", "--ik-seeds", "24", "--output", str(PLAN_FILE)],
        capture_output=True, text=True, timeout=300)
    for line in (plan.stdout or "").strip().splitlines():
        print("  " + line)
    if plan.returncode != 0:
        print(f"规划失败（exit {plan.returncode}）：不移动")
        return 2
    if not args.execute:
        print(f"\n计划已写出：{PLAN_FILE}\n（未执行；确认后加 --execute）")
        return 0
    run = subprocess.run(
        [sys.executable, str(EXECUTOR), str(PLAN_FILE),
         "--time-scale", "1.0", "--goal-tolerance", "0.001", "--phase", PHASE],
        capture_output=True, text=True, timeout=1800)
    for line in (run.stdout or "").strip().splitlines()[-14:]:
        print("  " + line)
    if run.returncode != 0:
        print(f"执行失败（exit {run.returncode}）")
        return 4
    print("\n✅ 已到插槽精观测位（安全区间内不分段）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
