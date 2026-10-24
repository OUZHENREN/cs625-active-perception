#!/usr/bin/env python3
"""Fast controller health check / auto-recovery for the real CS625 stack.

Checks (all sub-second except optional fix):
  * reverse-control sockets 50001/50003/50004 ESTAB
  * arm_controller active
  * /joint_states still flowing

Usage:
  python3 scripts/control_health.py         # exit 0 if healthy
  python3 scripts/control_health.py --fix   # recover when unhealthy
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

SOCKETS = ("50001", "50003", "50004")


def sockets_ok() -> bool:
    try:
        out = subprocess.run(["ss", "-tn"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return False
    # Only look at the local listener ports; the robot address stays in config.
    return all(
        any(line.split()[3].endswith(f":{port}") and "ESTAB" in line
            for line in out.splitlines() if len(line.split()) >= 4)
        for port in SOCKETS
    )


def controller_active() -> bool:
    try:
        out = subprocess.run(
            ["ros2", "control", "list_controllers",
             "--controller-manager", "/eli_ros2_control_node"],
            capture_output=True, text=True, timeout=8,
        ).stdout
    except Exception:
        return False
    for line in out.splitlines():
        if line.startswith("arm_controller") and "active" in line:
            return True
    return False


def joints_flowing(timeout_sec: float = 2.0) -> bool:
    """Count two /joint_states messages within the timeout using rclpy."""
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except Exception:
        return False
    seen = {"n": 0}
    rclpy.init(args=None)
    node = rclpy.create_node("control_health_probe")
    try:
        node.create_subscription(JointState, "/joint_states",
                                 lambda _m: seen.__setitem__("n", seen["n"] + 1), 10)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and seen["n"] < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return seen["n"] >= 2


def recover() -> None:
    subprocess.run(["bash", "scripts/recover_control.sh"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fix", action="store_true")
    args = parser.parse_args()

    checks = {
        "sockets": sockets_ok(),
        "controller": controller_active(),
        "joint_states": joints_flowing(),
    }
    bad = [name for name, ok in checks.items() if not ok]
    print("health:", checks)
    if not bad:
        return 0
    print("unhealthy:", bad)
    if not args.fix:
        return 1
    print("recovering ...")
    recover()
    checks = {
        "sockets": sockets_ok(),
        "controller": controller_active(),
        "joint_states": joints_flowing(),
    }
    print("after recovery:", checks)
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    sys.exit(main())
