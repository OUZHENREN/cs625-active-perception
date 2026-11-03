#!/usr/bin/env python3
"""Pre-motion safety gate (2026-11-03 collision incident).

Before ANY commanded move, all of the following must hold; otherwise the caller
must not send a trajectory:

  * the effort stream is ALIVE and plausible -- after today's controller switches
    the effort interface read ~0 N across the board while the arm was carrying a
    6.8 kg part (a dead stream: any contact detector silently sees nothing);
  * the three reverse-control ports are ESTAB (the channel dropped six times on
    2026-11-03 and the controller then accepts goals it never executes);
  * arm_controller is active and safety_mode=1 / robot_mode=7.

Usage:
    python3 scripts/preflight_motion.py [--min-effort-max 1.0] [--min-effort-sum 2.0]

Exit codes: 0 pass; 1 a gate failed; 2 could not evaluate (also refuse).
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def reverse_ports_up() -> bool:
    try:
        out = subprocess.run(["ss", "-tn"], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return any(f":{port}" in line and "ESTAB" in line
               for line in out.splitlines()
               for port in ("50001", "50003", "50004"))


def controller_active(node: Node) -> bool:
    from controller_manager_msgs.srv import ListControllers
    client = node.create_client(ListControllers, "/eli_ros2_control_node/list_controllers")
    if not client.wait_for_service(timeout_sec=10.0):
        print("FAIL: controller manager unreachable")
        return False
    future = client.call_async(ListControllers.Request())
    deadline = time.time() + 10.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = future.result()
    if result is None:
        print("FAIL: list_controllers timed out")
        return False
    for controller in result.controller:
        if controller.name == "arm_controller":
            if controller.state != "active":
                print(f"FAIL: arm_controller is {controller.state}")
                return False
            return True
    print("FAIL: arm_controller not loaded")
    return False


def modes_ok(node: Node, timeout: float = 6.0) -> bool:
    from eli_common_interface.msg import RobotMode, SafetyMode
    modes: dict = {}
    node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                             lambda m: modes.__setitem__("safety", m.mode), 10)
    node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                             lambda m: modes.__setitem__("robot", m.mode), 10)
    deadline = time.time() + timeout
    while len(modes) < 2 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if modes.get("safety") != 1:
        print(f"FAIL: safety_mode={modes.get('safety')} (need 1 NORMAL)")
        return False
    if modes.get("robot") != 7:
        print(f"FAIL: robot_mode={modes.get('robot')} (need 7 RUNNING)")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--min-effort-max", type=float, default=1.0,
                        help="max |effort| across joints must exceed this (dead stream reads ~0)")
    parser.add_argument("--min-effort-sum", type=float, default=2.0,
                        help="sum |effort| must exceed this as a second signature")
    args = parser.parse_args()

    if not reverse_ports_up():
        print("FAIL: reverse-control ports (50001/50003/50004) are not ESTAB")
        return 1

    rclpy.init()
    node = Node("preflight_motion")
    samples: list[list[float]] = []

    def on_state(msg: JointState) -> None:
        effort = dict(zip(msg.name, msg.effort))
        if all(k in effort for k in ORDER):
            samples.append([effort[k] for k in ORDER])

    node.create_subscription(JointState, "/joint_states", on_state, 10)
    deadline = time.time() + 1.2
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not controller_active(node) or not modes_ok(node):
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if len(samples) < 20:
        print("FAIL: joint_states not flowing")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    matrix = np.array(samples)
    finite = np.all(np.isfinite(matrix)) and np.all(np.abs(matrix) < 1.0e4)
    max_effort = float(np.abs(matrix).max())
    sum_effort = float(np.abs(matrix).mean(axis=0).sum())
    print(f"effort stream: n={len(samples)} finite={finite} "
          f"max|effort|={max_effort:.3f} sum|effort|={sum_effort:.3f}")
    if not finite:
        print("FAIL: effort values are non-finite or absurd")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if max_effort < args.min_effort_max or sum_effort < args.min_effort_sum:
        print(f"FAIL: effort stream looks DEAD (max {max_effort:.3f} < "
              f"{args.min_effort_max}, sum {sum_effort:.3f} < {args.min_effort_sum})")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("PREFLIGHT_OK")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
