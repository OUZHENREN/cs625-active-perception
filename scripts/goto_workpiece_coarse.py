#!/usr/bin/env python3
"""一键回到"工件粗观测位"（效率模式起点，2026-11-24 所有者要求，参照 goto_park_pose.py）。

目标位姿
--------
config/tasks/common.yaml 的 observe.workpiece_coarse_efficient
（= efficient_start_pose，2026-11-13 所有者确认的效率模式起点）：
TCP (0.6238, 0.37375, 0.50762)，quat (-0.383818, 0.748045, 0.213452, 0.497544)。
它是 TCP 位姿而不是六轴位姿，所以规划走 scripts/plan_pose_ik.py（IK + 绝对安全区间
+ 腕部翻转/远离已验证姿态拒绝 + 轨迹抽样碰撞校验），执行走 execute_assembly_chunks。

与 goto_park_pose.py 的对应关系
--------------------------------
  · 停车位姿 = 断电安全的六轴位姿 → plan_joint_locked 关节直线；
  · 粗观测位 = 带姿态的 TCP 位姿 → plan_pose_ik（与 nbv_from_reference --execute 同一条
    规划链），因此同样享受 plan_pose_ik 的全套安全闸。

速度与分段（所有者 2026-11-24 批准）
------------------------------------
换位段走白名单 "goto_workpiece_coarse"：0.20 rad/s（11.5°/s）、单段执行、不自动分段；
在线关节区间看门狗（JOINT_OUT_OF_RANGE）与整段到位复核仍然生效。接触/装配段不受影响。

安全
----
  · 默认**只规划不动臂**；要动臂必须显式加 --execute；
  · 腕部翻转解默认拒绝（plan_pose_ik 行为）：被拒绝时打印六个关节角，
    所有者确认后加 --allow-flip 再跑。
  · **从停车位过来的标准路径必然需要 --allow-flip**：停车位就是"其余五轴留在
    粗观测位附近、只把 J4 转到 −80.5°"设计的，所以两个位姿之间 wrist_1 天生差
    94°，超过 90° 的翻转阈值。VS Code 任务 ⑬ 因此内置 --allow-flip，并在执行前
    打印六个关节角与行程（所有者 2026-11-25 确认这是设计好的标准路径）。

用法
----
    python3 scripts/goto_workpiece_coarse.py                # 只规划，打印关节行程
    python3 scripts/goto_workpiece_coarse.py --execute      # 真机执行（VS Code 任务 ⑬）
    python3 scripts/goto_workpiece_coarse.py --allow-flip   # 允许腕部翻转解（需确认）
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
COMMON = REPO / "config" / "tasks" / "common.yaml"
PLANNER = REPO / "scripts" / "plan_pose_ik.py"
EXECUTOR = REPO / "scripts" / "execute_assembly_chunks.py"
PLAN_FILE = pathlib.Path("/tmp/goto_workpiece_coarse_plan.json")

WAYPOINT = "workpiece_coarse_efficient"


def load_target() -> tuple[list[float], list[float]]:
    """读 config 的 observe.workpiece_coarse_efficient，返回 (xyz, quat)。"""
    config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    pose = (config.get("observe") or {}).get(WAYPOINT)
    if not pose or not pose.get("xyz") or not pose.get("quat"):
        raise SystemExit(f"config 里没有 observe.{WAYPOINT}（xyz/quat），无法回位")
    xyz = [float(v) for v in pose["xyz"]]
    quat = [float(v) for v in pose["quat"]]
    return xyz, quat


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--execute", action="store_true",
                        help="真正移动机械臂（默认只规划）")
    parser.add_argument("--allow-flip", action="store_true",
                        help="允许腕部翻转解（plan_pose_ik 默认拒绝；需所有者确认后使用）")
    args = parser.parse_args()

    xyz, quat = load_target()
    print(f"目标：{WAYPOINT} TCP {[round(v, 4) for v in xyz]} "
          f"quat {[round(v, 4) for v in quat]}")

    plan_args = [sys.executable, "-u", str(PLANNER),
                 *[f"{v:.6f}" for v in (*xyz, *quat)],
                 "--output", str(PLAN_FILE)]
    if args.allow_flip:
        plan_args.append("--allow-flip")
    plan = subprocess.run(plan_args, capture_output=True, text=True, timeout=300)
    for line in (plan.stdout or "").strip().splitlines():
        print("  " + line)
    for line in (plan.stderr or "").strip().splitlines():
        print("  [stderr] " + line)
    if plan.returncode != 0:
        print(f"规划失败（exit {plan.returncode}）：不移动。"
              "若因腕部翻转被拒绝，确认六个关节角后加 --allow-flip 重跑。")
        return 4

    if args.allow_flip:
        # 2026-11-25 所有者：停车位 ↔ 粗观测位 之间 wrist_1 必然要转 94°。
        # 停车位就是这样设计的（"其余五轴保持粗观测位附近，只把 J4 转到 −80.5°"），
        # 所以这一对位姿之间的单轴 94° 旋转是**所有者设计并批准的标准路径**，
        # 不是危险的重构。仍然每次都把六个关节角打出来，让所有者一眼看到会怎么动。
        try:
            data = json.loads(PLAN_FILE.read_text(encoding="utf-8"))
            names = data["joint_names"]
            first = data["points"][0]["q"]
            last = data["points"][-1]["q"]
            order = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            index = [names.index(n) for n in order]
            print("\n[腕部翻转放行] 停车位 ↔ 粗观测位 的标准路径（所有者设计）：")
            print(f"  {'关节':<22}{'当前':>10}{'目标':>10}{'行程':>10}")
            for j, name in enumerate(order):
                now = first[index[j]]
                end = last[index[j]]
                print(f"  {name:<22}{now*57.2958:9.2f}°{end*57.2958:9.2f}°"
                      f"{(end-now)*57.2958:+9.2f}°")
            wr = abs(last[index[3]] - first[index[3]]) * 57.2958
            print(f"  → wrist_1 单轴旋转 {wr:.1f}°（停车位与粗观测位在设计上就差这么多），"
                  f"其余五轴几乎不动")
        except Exception:  # noqa: BLE001 - 打印失败不影响后续判断
            pass

    if not args.execute:
        print(f"\n计划已写出：{PLAN_FILE}")
        print("（未执行；确认后加 --execute）")
        return 0

    # 换位白名单段：0.20 rad/s、单段执行；在线关节区间看门狗 + 到位复核仍生效
    run = subprocess.run(
        [sys.executable, "-u", str(EXECUTOR), str(PLAN_FILE),
         "--time-scale", "1.0", "--goal-tolerance", "0.002",
         "--verify-final", "0.004", "--fault-poll", "0.1",
         "--phase", "goto_workpiece_coarse", "--max-joint-speed", "0.20"],
        capture_output=True, text=True, timeout=900)
    output = run.stdout or ""
    for line in output.strip().splitlines():
        if any(key in line for key in ("speed governor", "安全换位段", "status=",
                                       "ALL_ASSEMBLY", "REFUSED", "FAILED", "取消")):
            print("  " + line)
    if run.returncode != 0 or "ALL_ASSEMBLY_CHUNKS_OK" not in output:
        tail = output.strip().splitlines()
        print(f"执行失败（rc={run.returncode}）：" + (tail[-1] if tail else "无输出"))
        return 5
    print("已回到工件粗观测位。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
