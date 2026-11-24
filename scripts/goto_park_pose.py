#!/usr/bin/env python3
"""一键回到"J4 停车位姿"（2026-11-20 所有者定）。

为什么需要它
------------
J4 抱闸失效（2026-11-13 碰撞导致线束磨损）。断电后 J4 会自由摆动，末端可能甩到桌面/
夹具上。所有者找到一个"停车位姿"：其余五轴保持粗观测位附近，只把 J4 转到 −80.5°，
此时 J4 的转轴与重力方向平行，断电后重力对 J4 不产生力矩 —— 不会摆动。

这个脚本做两件事：
  1. 把当前六轴位姿记录进 config/tasks/common.yaml 的 j4_park_pose（--record）；
  2. 一键回到记录的那个位姿（默认动作），用于"下次开工/收工"的快速归位。

安全
----
  · 只走关节空间直线，目标位姿来自已验证记录，不做位姿搜索；
  · 规划交给 scripts/teaching/plan_joint_locked.py（它会校验关节限位与行程）；
  · 执行交给 scripts/execute_assembly_chunks.py（自动分段 + 速度治理 + 逐段复核）；
  · 默认 **只规划不动臂**；要动臂必须显式加 --execute。

用法
----
    # 记录当前位姿为停车位姿（所有者确认位置后）
    python3 scripts/goto_park_pose.py --record

    # 查看会怎么动（只规划，不动臂）
    python3 scripts/goto_park_pose.py

    # 真正执行
    python3 scripts/goto_park_pose.py --execute
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
PLANNER = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTOR = REPO / "scripts" / "execute_assembly_chunks.py"
PLAN_FILE = pathlib.Path("/tmp/j4_park_pose_plan.json")

JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# 与 plan_pose_ik 的绝对安全区间一致（2026-11-24 所有者批准：肩部 −130、肘部 −60）
ABSOLUTE_LIMITS_DEG = {
    "shoulder_pan_joint": (-100.0, 100.0),
    "shoulder_lift_joint": (-130.0, 10.0),
    "elbow_joint": (-160.0, -60.0),
    "wrist_1_joint": (-150.0, 150.0),
    "wrist_2_joint": (-150.0, 150.0),
    "wrist_3_joint": (-150.0, 150.0),
}


def read_joints_and_tcp(timeout: float = 15.0):
    """读当前六轴（度）与 TCP；读不到返回 (None, None)。"""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception:  # noqa: BLE001
        return None, None
    import time as _time
    try:
        rclpy.init()
    except Exception:  # noqa: BLE001 - 已初始化
        pass
    node = Node("goto_park_pose_reader")
    joints: dict = {}
    tcp: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: joints.update(dict(zip(m.name, m.position))), 10)
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: tcp.setdefault("m", m), 10)
    deadline = _time.time() + timeout
    while (len([k for k in JOINT_ORDER if k in joints]) < 6 or "m" not in tcp) \
            and _time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    if len([k for k in JOINT_ORDER if k in joints]) < 6:
        return None, None
    degrees = {name: joints[name] * 180.0 / 3.141592653589793 for name in JOINT_ORDER}
    message = tcp.get("m")
    tcp_values = None
    if message is not None:
        p, q = message.pose.position, message.pose.orientation
        tcp_values = ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    return degrees, tcp_values


def record_pose() -> int:
    """把当前位姿写进 config 的 j4_park_pose（保留其他配置与注释）。"""
    degrees, tcp_values = read_joints_and_tcp()
    if degrees is None:
        print("读不到 /joint_states，无法记录")
        return 2
    for name, (low, high) in ABSOLUTE_LIMITS_DEG.items():
        value = degrees[name]
        if not (low <= value <= high):
            print(f"拒绝记录：{name} = {value:.1f}° 不在安全区间 [{low}, {high}]")
            return 3

    values = [round(degrees[name], 2) for name in JOINT_ORDER]
    tcp_text = ""
    if tcp_values is not None:
        position, quaternion = tcp_values
        tcp_text = ("  tcp:\n" + "".join(f"  - {v:.5f}\n" for v in position)
                    + "  quat:\n" + "".join(f"  - {v:.6f}\n" for v in quaternion))

    text = COMMON.read_text(encoding="utf-8")
    if "j4_park_pose:" not in text:
        print("config 里没有 j4_park_pose 段，拒绝自动插入（请先手工加占位）")
        return 4
    start = text.index("j4_park_pose:")
    # 找到该段结束（下一个顶格 key）
    end = len(text)
    for index in range(start + len("j4_park_pose:"), len(text)):
        if text[index] == "\n" and index + 1 < len(text) and text[index + 1] not in " \n\t#":
            end = index + 1
            break
    block = ("j4_park_pose:\n"
             "  recorded_at: \"" + __import__("datetime").datetime.now().isoformat(
                 timespec="minutes") + "\"\n"
             f"  j4_deg: {values[3]}\n"
             "  joints_deg:\n"
             + "".join(f"  - {value}      # {name}\n"
                       for value, name in zip(values, JOINT_ORDER))
             + tcp_text
             + "  note: J4 停车位姿：J4 转轴与重力平行，断电后重力不会带动 J4；\n"
               "    一键回位见 scripts/goto_park_pose.py（VS Code 任务 ⑪）。\n\n")
    COMMON.write_text(text[:start] + block + text[end:], encoding="utf-8")

    reloaded = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    check = reloaded.get("j4_park_pose") or {}
    if check.get("joints_deg") != values:
        print("写入后回读校验失败")
        return 5
    print("已记录 j4_park_pose：")
    for name, value in zip(JOINT_ORDER, values):
        print(f"  {name:<22}{value:9.2f}°")
    if tcp_values is not None:
        print(f"  TCP {[round(v, 4) for v in tcp_values[0]]}")
    return 0


def load_target_deg() -> list[float]:
    config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    pose = config.get("j4_park_pose") or {}
    values = pose.get("joints_deg")
    if not values or len(values) != 6:
        raise SystemExit("config 的 j4_park_pose.joints_deg 不是 6 个数，先用 --record 记录")
    return [float(v) for v in values]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--record", action="store_true",
                        help="把当前六轴位姿记录为停车位姿（不改动机械臂）")
    parser.add_argument("--execute", action="store_true",
                        help="真正移动机械臂（默认只规划）")
    args = parser.parse_args()

    if args.record:
        return record_pose()

    target = load_target_deg()
    current, _ = read_joints_and_tcp()
    if current is None:
        print("读不到 /joint_states，无法回位")
        return 2

    travel = {name: target[i] - current[name] for i, name in enumerate(JOINT_ORDER)}
    worst = max(travel.items(), key=lambda kv: abs(kv[1]))
    print("目标停车位姿 vs 当前：")
    for name in JOINT_ORDER:
        print(f"  {name:<22}{current[name]:9.2f}° → {target[JOINT_ORDER.index(name)]:9.2f}°"
              f"  (Δ {travel[name]:+7.2f}°)")
    if max(abs(v) for v in travel.values()) < 0.5:
        print("\n当前已经在停车位姿附近（所有关节 <0.5°），无需移动。")
        return 0
    print(f"\n最大关节行程：{worst[0]} {worst[1]:+.1f}°"
          + ("（会被自动分段）" if abs(worst[1]) > 25 else ""))

    joints_arg = ",".join(f"{v:.3f}" for v in target)
    plan = subprocess.run(
        [sys.executable, str(PLANNER), "--joints", joints_arg,
         "--output", str(PLAN_FILE)],
        capture_output=True, text=True, timeout=300)
    for line in (plan.stdout or "").strip().splitlines():
        print("  " + line)
    if plan.returncode != 0:
        print(f"规划失败（exit {plan.returncode}）：不移动")
        return 4

    if not args.execute:
        print(f"\n计划已写出：{PLAN_FILE}")
        print("（未执行；确认后加 --execute）")
        return 0

    run = subprocess.run(
        [sys.executable, str(EXECUTOR), str(PLAN_FILE),
         "--time-scale", "1.0", "--goal-tolerance", "0.001",
         "--verify-final", "0.004", "--fault-poll", "0.1",
         "--phase", "goto_park_pose", "--max-joint-speed", "0.20"],
        capture_output=True, text=True, timeout=900)
    output = run.stdout or ""
    for line in output.strip().splitlines():
        if any(key in line for key in ("speed governor", "安全换位段", "status=",
                                       "ALL_ASSEMBLY", "REFUSED", "FAILED", "取消")):
            print("  " + line.strip())
    if run.returncode != 0 or "ALL_ASSEMBLY_CHUNKS_OK" not in output:
        print(f"执行失败（rc={run.returncode}）："
              + (output.strip().splitlines()[-1] if output.strip() else "?"))
        return 5
    print("\n已回到停车位姿。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
