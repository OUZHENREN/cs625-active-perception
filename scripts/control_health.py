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
import os
import pathlib
import subprocess
import sys
import time

SOCKETS = ("50001", "50003", "50004")
REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def _setting(name: str, default: str) -> str:
    """Controller/socket names are machine configuration, not source code.

    ``~/.cs625_local.env`` (or the environment) may override them; the defaults
    are the names this cell has always used, so nothing changes on this machine.
    """
    value = os.environ.get(name, "").strip()
    if value:
        return value
    try:
        for line in pathlib.Path("~/.cs625_local.env").expanduser().read_text(
                encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith(f"{name}="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    except OSError:
        pass
    return default


CONTROLLER_MANAGER = _setting("CS625_CONTROLLER_MANAGER", "/eli_ros2_control_node")
ARM_CONTROLLER = _setting("CS625_ARM_CONTROLLER", "arm_controller")


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


def controller_state(text: str) -> str | None:
    """Return the state column of the arm controller, or None when absent.

    ``ros2 control list_controllers`` prints ``name type state`` (optionally with
    a ``[*]`` marker in front of the active row).  The old check was
    ``"active" in line``, which is also true for ``inactive`` -- on 2026-10-26 the
    cell reported ``controller: True`` while ``arm_controller`` was inactive, so
    the runner happily sent goals to a controller that could not execute them.
    Compare the state token exactly instead.
    """
    for line in text.splitlines():
        columns = [column for column in line.replace("[*]", " ").split() if column]
        if len(columns) >= 3 and columns[0] == ARM_CONTROLLER:
            return columns[-1]
    return None


def controller_active() -> bool:
    try:
        out = subprocess.run(
            ["ros2", "control", "list_controllers",
             "--controller-manager", CONTROLLER_MANAGER],
            capture_output=True, text=True, timeout=8,
        ).stdout
    except Exception:
        return False
    return controller_state(out) == "active"


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


def recover() -> bool:
    """Run the recovery script from the repository root and report what it said.

    The old version threw the output away, so a failed recovery looked exactly
    like a successful one and the caller only saw "still unhealthy".
    """
    script = REPO_ROOT / "scripts" / "recover_control.sh"
    result = subprocess.run(["bash", str(script)], cwd=str(REPO_ROOT),
                            capture_output=True, text=True, check=False)
    if result.returncode != 0:
        tail = ((result.stdout or "") + (result.stderr or "")).strip().splitlines()
        print("recovery failed (rc=%d): %s" % (result.returncode,
                                               " | ".join(tail[-4:] or ["no output"])))
        return False
    return True


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
    if not checks["controller"]:
        print(f"  {ARM_CONTROLLER} is not active (manager {CONTROLLER_MANAGER}); "
              "activate it with: ros2 control switch_controllers "
              f"--controller-manager {CONTROLLER_MANAGER} --activate {ARM_CONTROLLER} --strict")
    if not args.fix:
        return 1
    print("recovering ...")
    recovered = recover()
    checks = {
        "sockets": sockets_ok(),
        "controller": controller_active(),
        "joint_states": joints_flowing(),
    }
    print("after recovery:", checks)
    return 0 if recovered and all(checks.values()) else 2


if __name__ == "__main__":
    sys.exit(main())
