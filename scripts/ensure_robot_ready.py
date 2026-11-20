#!/usr/bin/env python3
"""快速把机器人挂回"运动中"（RUNNING），而不是干等。

背景（所有者 2026-11-12）
--------------------------
示教器接管过之后，控制器会停在 IDLE/未使能状态，流程要等很久才发现。
这里先读 (safety_mode, robot_mode)，只有在 safety=NORMAL 且 robot≠RUNNING 时才
重发外部控制脚本（resend_external_script）+ 恢复速度滑条，然后最多等 5 s 复查，
而不是固定等 45 s。

2026-11-18 补充（真机事故后）：`robot_mode == 7` **不能证明机器人侧的外部控制程序
还在跑**。一次轨迹中止（PATH_TOLERANCE_VIOLATED）之后，机器人可能仍报 RUNNING、
驱动端口也仍 ESTAB，但外部控制脚本已经停止——此时动作目标会被"接受"却永不执行
（手册第 28 条）。所以新增 `--force`：即使已经是 RUNNING，也重发外部控制脚本
+ 恢复速度滑条。**任何一次运动被中止/保护停止之后，下一次动臂前都应当用 --force。**

退出码：0=已就绪；3=安全模式异常（需要人去示教器复位）；4=重挂失败。
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import time

import rclpy
from eli_common_interface.msg import RobotMode, SafetyMode
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def read_modes(node: Node, timeout: float = 8.0) -> tuple[int | None, int | None]:
    seen: dict = {}
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                             lambda m: seen.__setitem__("s", m.mode), qos)
    node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                             lambda m: seen.__setitem__("r", m.mode), qos)
    # 2026-11-19：节点多时 DDS 发现可能超过 2 s，读到 (None, None) 会被误判成
    # "安全模式不是 NORMAL"，把本来正常的机器人挡在门外。默认 8 s，且不足两条就再等。
    deadline = time.time() + timeout
    while time.time() < deadline and len(seen) < 2:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(seen) < 2:
        deadline = time.time() + 5.0
        while time.time() < deadline and len(seen) < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
    return seen.get("s"), seen.get("r")


FORCE_DRIFT_LIMIT_DEG = 2.0
LAST_COMMANDED = pathlib.Path("runtime/last_commanded_pose.json")


def measured_vs_last_commanded(node):
    """当前实测关节角与"最后一条轨迹终点"的最大偏差（度），以及最差的关节名。

    复用调用方已经建好的 node（重复 rclpy.init() 会抛异常）。
    读不到任一来源时返回 None（不做判断，交给操作者）。
    """
    import math

    try:
        payload = json.loads(LAST_COMMANDED.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    names = payload.get("joint_names") or []
    positions = payload.get("positions") or []
    if not names or len(names) != len(positions):
        return None

    measured: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: measured.update(dict(zip(m.name, m.position))), 10)
    deadline = time.time() + 8.0
    while len([n for n in names if n in measured]) < len(names) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    worst_joint, worst = None, 0.0
    for name, target in zip(names, positions):
        if name not in measured:
            return None
        delta = abs(math.degrees(measured[name] - float(target)))
        if delta > worst:
            worst_joint, worst = name, delta
    return (worst, worst_joint)


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--force", action="store_true",
                        help="即使 robot_mode 已是 RUNNING，也重发外部控制脚本"
                             "（运动被中止/保护停止之后必须这样做）")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("ensure_robot_ready")
    safety, robot = read_modes(node)
    print(f"[ready] safety_mode={safety} robot_mode={robot}")
    if safety != 1:
        print("[ready] 安全模式不是 NORMAL：请在示教器上复位（急停/保护停止）后重试")
        return 3
    # 2026-11-20 事故硬闸：重挂外部控制脚本会让控制器把**最后一条轨迹的终点**
    # 重新下发。如果机械臂被人手拖动过（当前位姿 ≠ 最后指令位姿），重挂就会让
    # 机械臂"抽搐着快速跳回旧位置"——这是绝对不允许的。
    if args.force:
        drift = measured_vs_last_commanded(node)
        if drift is not None and drift[0] > FORCE_DRIFT_LIMIT_DEG:
            print(f"[ready] 拒绝 --force：机械臂被手动移动过（{drift[1]} 偏了 "
                  f"{drift[0]:.1f}°，限 {FORCE_DRIFT_LIMIT_DEG:.1f}°）。")
            print("        重挂会让控制器把最后一条轨迹的终点重新下发 → 机械臂会快速跳回旧位置。")
            print("        正确做法：不要重挂，直接下发一条**从当前实测位姿出发**的新轨迹")
            print("        （execute_assembly_chunks 的 FIRST_POINT_FAR 会保证起点与实测一致）。")
            print("        若确实需要重挂，请先手动把机械臂摆回最后指令位姿附近。")
            return 5

    if robot == 7 and not args.force:
        print("[ready] 已经是 RUNNING，无需重挂"
              "（若上一次运动被中止或触发过保护停止，请加 --force）")
        return 0

    # 重挂：重发外部控制脚本 + 速度滑条
    subprocess.run(["bash", str(__import__("pathlib").Path(__file__).resolve().parents[1]
                                 / "scripts" / "recover_control.sh")],
                   capture_output=True, text=True, timeout=120)
    for _ in range(5):
        time.sleep(1.0)
        safety, robot = read_modes(node, timeout=1.0)
        print(f"[ready] 复查 safety_mode={safety} robot_mode={robot}")
        if robot == 7:
            print("[ready] 已重挂为 RUNNING")
            return 0
    print("[ready] 重挂失败：机器人仍未进入 RUNNING，请检查示教器状态")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
