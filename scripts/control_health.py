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


def controller_state_live(timeout_sec: float = 12.0) -> str | None:
    """Ask controller_manager directly for the arm controller's state.

    The ``ros2 control list_controllers`` CLI is not reliable while the camera
    publishes a 900k-point cloud: measured 2026-10-26 it sometimes answered in
    1.6 s and sometimes timed out three times at 10 s each, which made the start
    task report "controller always fails to activate" while the controller had
    been active the whole time.  A direct service call from a short-lived rclpy
    node is what the task runner itself uses, and it is bounded here.
    """
    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
    except Exception:  # noqa: BLE001 - fall back to the CLI
        return None
    try:
        rclpy.init(args=None)
    except Exception:  # noqa: BLE001 - already initialised by the caller
        pass
    node = rclpy.create_node("control_health_controller_probe")
    try:
        client = node.create_client(ListControllers, f"{CONTROLLER_MANAGER}/list_controllers")
        if not client.wait_for_service(timeout_sec=timeout_sec / 3.0):
            return None
        future = client.call_async(ListControllers.Request())
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        response = future.result() if future.done() else None
        if response is None:
            return None
        for controller in response.controller:
            if controller.name == ARM_CONTROLLER:
                return controller.state
        return None
    except Exception:  # noqa: BLE001 - never let a probe crash the health check
        return None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def controller_active() -> tuple[bool, str | None]:
    """(is the arm controller active, its state name).

    Service first; the CLI is only a fallback for environments where the service
    python bindings are unavailable.
    """
    state = controller_state_live()
    if state is None:
        try:
            out = subprocess.run(
                ["ros2", "control", "list_controllers",
                 "--controller-manager", CONTROLLER_MANAGER],
                capture_output=True, text=True, timeout=8,
            ).stdout
            state = controller_state(out)
        except Exception:  # noqa: BLE001
            state = None
    return state == "active", state


def joints_flowing(timeout_sec: float = 4.0, attempts: int = 2) -> bool:
    """Count two /joint_states messages within the timeout using rclpy.

    Two attempts and a 4 s window, because a freshly created probe node needs a
    moment to discover a 125 Hz topic and because controller_manager is busy right
    after a controller switch.  Measured 2026-10-26: immediately after activating
    arm_controller the first probe saw zero messages and reported an unhealthy
    control chain although /joint_states was running at 125 Hz.  A false alarm
    here makes the operator think the arm is broken.
    """
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except Exception:
        return False
    for attempt in range(attempts):
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
        if seen["n"] >= 2:
            return True
        if attempt + 1 < attempts:
            time.sleep(1.0)
    return False


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


def activate_controller(timeout_sec: float = 15.0) -> tuple[bool, str]:
    """Activate the arm controller through controller_manager, if it is not active.

    Returns (ok, message).  The ``ros2 control switch_controllers`` CLI is not used:
    with the camera cloud running it took three 10 s timeouts and then "failed"
    although the controller was already active.  Asking the state first and
    calling the service directly keeps this bounded to a few seconds.
    """
    active, state = controller_active()
    if active:
        return True, f"{ARM_CONTROLLER} is already active"
    try:
        import rclpy
        from controller_manager_msgs.srv import SwitchController
    except Exception as exc:  # noqa: BLE001
        return False, f"cannot import controller_manager_msgs: {exc}"
    try:
        rclpy.init(args=None)
    except Exception:  # noqa: BLE001
        pass
    node = rclpy.create_node("control_health_activate")
    try:
        client = node.create_client(SwitchController, f"{CONTROLLER_MANAGER}/switch_controller")
        if not client.wait_for_service(timeout_sec=timeout_sec / 3.0):
            return False, f"service {CONTROLLER_MANAGER}/switch_controller is not available"
        request = SwitchController.Request()
        request.activate_controllers = [ARM_CONTROLLER]
        request.strictness = SwitchController.Request.STRICT
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            return False, "switch_controller call timed out"
        response = future.result()
        if response is None or not response.ok:
            return False, "controller_manager refused the activation"
    except Exception as exc:  # noqa: BLE001
        return False, f"activation error: {exc}"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    active, state = controller_active()
    return active, f"{ARM_CONTROLLER}: {state or 'unknown'}"


def inspect() -> tuple[dict, str | None]:
    active, state = controller_active()
    checks = {
        "sockets": sockets_ok(),
        "controller": active,
        "joint_states": joints_flowing(),
    }
    return checks, state


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fix", action="store_true")
    parser.add_argument("--activate", action="store_true",
                        help="activate the arm controller first (service call, not the CLI)")
    args = parser.parse_args()

    if args.activate:
        ok, message = activate_controller()
        print(("activated: " if ok else "activation failed: ") + message)
        if not ok:
            return 1

    checks, state = inspect()
    bad = [name for name, ok in checks.items() if not ok]
    print("health:", checks)
    print(f"  {ARM_CONTROLLER}: {state or 'unknown'}")
    if not bad:
        return 0
    print("unhealthy:", bad)
    if not checks["controller"]:
        # Point at this script, not the ros2 control CLI: under camera-cloud load
        # that CLI timed out three times at 10 s and made an active controller look
        # broken.
        print(f"  {ARM_CONTROLLER} is not active (manager {CONTROLLER_MANAGER}); "
              "activate it with: python3 scripts/control_health.py --activate")
    if not args.fix:
        return 1
    print("recovering ...")
    recovered = recover()
    checks, state = inspect()
    print("after recovery:", checks)
    print(f"  {ARM_CONTROLLER}: {state or 'unknown'}")
    return 0 if recovered and all(checks.values()) else 2


if __name__ == "__main__":
    sys.exit(main())
