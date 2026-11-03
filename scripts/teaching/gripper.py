#!/usr/bin/env python3
"""Gripper wrapper for the teaching chain (HITBOT EFG80 through the Windows bridge).

Conventions proven on this cell (2026-10-27):

    init   -> jaws go to their referenced (closed) position, ~0 mm opening
    grip N -> move the jaws toward N mm opening; 0 is closed, 50 is fully open
              because this is an EXPANDING grasp, "grip" at the configured width
              is what actually holds the workpiece handles

So the safe order is:

    enter the grasp pose with the jaws CLOSED (right after init), then open them
    to the configured grip width, then lift.  Entering with the jaws already open
    knocks the handles over; that mistake caused the 2026-10-27 emergency stop.

The bridge is a Windows PowerShell script, therefore this wrapper is Windows-only
and refuses to run when the platform cannot reach it.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

BRIDGE = r"C:\Users\Public\hitbot_cmd.ps1"
GRIP_WIDTH_MM = 50.0     # config/tasks/common.yaml: grip_width_mm
GRIP_CURRENT_A = 0.1     # config/tasks/common.yaml: grip_current


def call(command: str, value: float | None = None,
         current: float | None = None, retries: int = 3) -> dict:
    args = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", BRIDGE, "-Cmd", command]
    if value is not None:
        args += ["-Value", str(value)]
    if current is not None:
        args += ["-Current", str(current)]
    last = ""
    for _ in range(retries):
        done = subprocess.run(args, capture_output=True, timeout=40)
        text = (done.stdout or b"").decode("utf-8", "replace").strip()
        lines = [line for line in text.splitlines() if line.strip()]
        if lines:
            try:
                data = json.loads(lines[-1])
                if data.get("ok"):
                    return data
                last = lines[-1]
            except json.JSONDecodeError:
                last = lines[-1]
        time.sleep(1.0)
    raise RuntimeError(f"gripper {command} failed: {last}")


def status() -> dict:
    return call("status")


def initialise(timeout_sec: float = 30.0) -> dict:
    """Always calibrate: the init flag survives a stop and does not prove the jaws."""
    call("init")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        current = status()
        if current.get("init") == 5:
            return current
        time.sleep(0.8)
    raise RuntimeError("gripper init did not finish")


def open_to_grip_width(width_mm: float = GRIP_WIDTH_MM,
                       current_a: float = GRIP_CURRENT_A,
                       timeout_sec: float = 20.0) -> dict:
    """Expand the jaws onto the handles; this is the action that holds the part."""
    call("grip", width_mm, current_a)
    deadline = time.time() + timeout_sec
    latest = status()
    while time.time() < deadline:
        latest = status()
        position = latest.get("position_mm")
        if latest.get("state") == 0 and position is not None:
            return latest
        time.sleep(0.6)
    return latest


def release() -> dict:
    """Release the expanding grasp: close the jaws back to the reference.

    The grip is EXPANDING (closed entry, then open onto the handles), so the
    release is a closure — the jaws come off the handles and the part drops.
    This is exactly the ``init`` action, which returns the jaws to ~0 mm.
    """
    return initialise()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["status", "init", "grip", "release"])
    parser.add_argument("--width", type=float, default=GRIP_WIDTH_MM)
    parser.add_argument("--current", type=float, default=GRIP_CURRENT_A)
    args = parser.parse_args()

    if args.command == "status":
        print(json.dumps(status(), ensure_ascii=False))
    elif args.command == "init":
        print(json.dumps(initialise(), ensure_ascii=False))
    elif args.command == "release":
        print(json.dumps(release(), ensure_ascii=False))
    else:
        print(json.dumps(open_to_grip_width(args.width, args.current),
                         ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
