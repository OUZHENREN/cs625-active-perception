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

import subprocess
import sys
import time

import rclpy
from eli_common_interface.msg import RobotMode, SafetyMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def read_modes(node: Node, timeout: float = 2.0) -> tuple[int | None, int | None]:
    seen: dict = {}
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                             lambda m: seen.__setitem__("s", m.mode), qos)
    node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                             lambda m: seen.__setitem__("r", m.mode), qos)
    deadline = time.time() + timeout
    while time.time() < deadline and len(seen) < 2:
        rclpy.spin_once(node, timeout_sec=0.1)
    return seen.get("s"), seen.get("r")


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
