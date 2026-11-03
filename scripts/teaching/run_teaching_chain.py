#!/usr/bin/env python3
"""Teaching chain: RVS poses -> robot motion, kept separate from the six modes.

This is the executable record of the path that actually worked on the real cell on
2026-10-27.  It deliberately does **not** import ``scripts/run_task.py`` or the six
``config/tasks/<scope>_<mode>.yaml`` files, and it does not modify the senior's
packages: it only talks to them over ROS topics (``tcp_server``) and to RVS over
its command port.

Data flow
---------
    RVS (Windows)                         this chain (WSL)                     robot
    -------------                         ----------------                     -----
    T2/T3 command  <-- pose of tool0 <--  rvs_pose_client.flange_pose()
        |                                        |
        +--> /box_target_pose  -------------->  read pose (S3)
        |    /box_grasp_tcp_pose  ----------->  read grasp TCP
        |
    move  -->  MoveIt plan (scripts/plan_pose_once.py)
          -->  one-shot execution (scripts/execute_assembly_chunks.py, tail slow)
    grip  -->  scripts/teaching/gripper.py  (init -> enter closed -> open to width)

Step order that is proven safe
------------------------------
    1. pre-grasp        jaws already initialised (closed)
    2. init gripper     jaws reference to ~0 mm
    3. enter grasp      jaws CLOSED, low speed, verified to 5 mm
    4. open to width    grip_width_mm (50 mm) -> expanding grasp on the handles
    5. lift 0.15 m      the only proof the part is held

Never enter the workpiece with open jaws: that knocked the handles over and
tripped the emergency stop on 2026-10-27.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "plan_pose_once.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
GRIPPER = REPO / "scripts" / "teaching" / "gripper.py"
RVS_CLIENT = REPO / "scripts" / "teaching" / "rvs_pose_client.py"

# T_b_tcp from the senior's tcp_bridge: box frame -> grasp TCP.
# Rotation only; the translation is already applied by tcp_server when it publishes
# /box_grasp_tcp_pose.  Kept here so the grasp orientation can be derived from S3
# without copying numbers by hand.
BOX_TO_TCP_ROTATION = [[-1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, -1.0]]


def run(args: list[str], timeout: float = 180.0) -> tuple[int, str]:
    done = subprocess.run(args, capture_output=True, timeout=timeout)
    text = ((done.stdout or b"") + (done.stderr or b"")).decode("utf-8", "replace")
    return done.returncode, text.strip()


def plan_to(xyz, quat, velocity: float, output: Path) -> bool:
    code, text = run([sys.executable, "-u", str(PLAN),
                      *[f"{v:.5f}" for v in (*xyz, *quat)],
                      "--velocity", str(velocity), "--output", str(output)])
    print(text.splitlines()[-1] if text else f"plan exit {code}")
    return "code 1" in text


def execute_plan(path: Path, time_scale: float, tail_fraction: float,
                 tail_scale: float, verify_final: float) -> bool:
    code, text = run([sys.executable, "-u", str(EXECUTE), str(path),
                      "--time-scale", str(time_scale),
                      "--tail-fraction", str(tail_fraction),
                      "--tail-scale", str(tail_scale),
                      "--verify-final", str(verify_final)], timeout=240.0)
    for line in text.splitlines()[-3:]:
        print(line)
    return "ALL_ASSEMBLY_CHUNKS_OK" in text


def gripper(command: str) -> dict:
    code, text = run([sys.executable, "-u", str(GRIPPER), command], timeout=90.0)
    try:
        return json.loads(text.splitlines()[-1])
    except Exception:
        print(text)
        return {}


def capture_pose(command: str) -> dict | None:
    code, text = run([sys.executable, "-u", str(RVS_CLIENT), command,
                      "--timeout", "300"], timeout=360.0)
    for line in text.splitlines():
        if line.startswith("answer:"):
            return {"raw": line}
    print(text)
    return None


def grasp_orientation_from_s3(quat: list[float]) -> list[float]:
    """R_grasp = R_S3 * diag(-1, 1, -1).

    The workpiece on this cell is an open frame whose face was rotated relative to
    the 2026-10-22 reference (7.71 deg about the vertical on 2026-10-27), so the
    grasp orientation must be derived from the live S3 instead of a constant.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    rotation = R.from_quat(quat).as_matrix() @ np.array(BOX_TO_TCP_ROTATION, float)
    return [float(v) for v in R.from_matrix(rotation).as_quat()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", choices=["observe", "pre-grasp", "enter", "grip",
                                         "lift"])
    parser.add_argument("--grasp-xyz", nargs=3, type=float, default=None,
                        help="defaults are the 2026-10-27 values and are almost "
                             "certainly STALE -- pass today's numbers explicitly")
    parser.add_argument("--pre-grasp-xyz", nargs=3, type=float, default=None)
    parser.add_argument("--s3-quat", nargs=4, type=float, default=None,
                        help="default is the 2026-10-27 S3 quaternion (STALE)")
    parser.add_argument("--lift", type=float, default=0.15)
    args = parser.parse_args()

    if args.grasp_xyz is None:
        args.grasp_xyz = [0.788, 0.662, -0.058]
        print("WARN: --grasp-xyz not given; using the 2026-10-27 default "
              f"{args.grasp_xyz} -- the workpiece may have moved since")
    if args.pre_grasp_xyz is None:
        args.pre_grasp_xyz = [0.739, 0.553, -0.059]
        print("WARN: --pre-grasp-xyz not given; using the 2026-10-27 default "
              f"{args.pre_grasp_xyz}")
    if args.s3_quat is None:
        args.s3_quat = [0.0069, -0.0022, 0.5412, 0.8408]
        print("WARN: --s3-quat not given; using the 2026-10-27 default")

    quat = grasp_orientation_from_s3(args.s3_quat)
    print("grasp quat from S3:", [round(v, 4) for v in quat])

    if args.step == "observe":
        result = capture_pose("T3")
        print(result or "no S3 received")
        return 0 if result else 3

    if args.step == "pre-grasp":
        if not plan_to(args.pre_grasp_xyz, quat, 0.05, Path("/tmp/teach_pg.json")):
            return 2
        return 0 if execute_plan(Path("/tmp/teach_pg.json"), 2.0, 0.2, 2.0, 0.01) else 4

    if args.step == "enter":
        state = gripper("init")
        print("gripper after init:", state)
        if not plan_to(args.grasp_xyz, quat, 0.02, Path("/tmp/teach_gr.json")):
            return 2
        return 0 if execute_plan(Path("/tmp/teach_gr.json"), 5.0, 0.5, 3.0, 0.005) else 4

    if args.step == "grip":
        print("open to grip width:", gripper("grip"))
        return 0

    lifted = list(args.grasp_xyz)
    lifted[2] += args.lift
    if not plan_to(lifted, quat, 0.03, Path("/tmp/teach_lift.json")):
        return 2
    return 0 if execute_plan(Path("/tmp/teach_lift.json"), 4.0, 0.3, 2.5, 0.008) else 4


if __name__ == "__main__":
    sys.exit(main())
