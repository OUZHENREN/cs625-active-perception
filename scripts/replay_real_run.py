#!/usr/bin/env python3
"""Play back the trajectories the REAL cell actually executed, inside Gazebo.

Why this exists
---------------
The real cell completed the whole task on 2026-10-22 and every move was recorded
under ``~/cs625_legacy_snapshot_20261020/runs/<run>/``.  Re-planning the same target
in simulation throws that evidence away: the planner may pick a different IK branch,
different timing, or a path the real controller never took, and then a simulation
failure says nothing about the real process.

This tool instead sends the recorded joint trajectory straight to the simulated
controller, phase by phase, and reports what happened.  If the simulation cannot
follow a trajectory the real arm executed, the difference is in the model, not in
the task.

Usage
-----
    # list what is available
    python3 scripts/replay_real_run.py --list

    # replay one phase
    python3 scripts/replay_real_run.py --run 20261022_pre_grasp_exec --speed 1.0

    # replay a whole ordered sequence (files given in order)
    python3 scripts/replay_real_run.py --files \\
      20261022_s2_candidate/s2_candidate_plan.json \\
      20261022_pre_grasp_exec/pre_grasp_plan.json \\
      20261022_grasp_joint_partial/grasp_joint_plan.json

Environment: the trajectories are joint-space, so this works in Gazebo and on the
real cell alike; the action name is chosen by --action (default: the simulation's
joint_trajectory_controller).
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

RUNS = pathlib.Path.home() / "cs625_legacy_snapshot_20261020" / "runs"
EXECUTOR = pathlib.Path(__file__).resolve().parent / "execute_assembly_chunks.py"


def discover(limit: int = 200) -> list[tuple[str, str, int, float]]:
    """(run, file, points, duration) for every recorded joint trajectory."""
    found = []
    for run_dir in sorted(RUNS.iterdir()):
        if not run_dir.is_dir():
            continue
        for path in sorted(run_dir.glob("*.json")):
            try:
                doc = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            points = doc.get("points") if isinstance(doc, dict) else None
            if not isinstance(points, list) or not points or "q" not in points[0]:
                continue
            found.append((run_dir.name, path.name, len(points),
                          float(points[-1].get("t", 0.0))))
    return found[:limit]


def resolve(spec: str) -> pathlib.Path:
    """Accept 'run/file.json' or a plain path."""
    candidate = pathlib.Path(spec)
    if candidate.is_file():
        return candidate
    candidate = RUNS / spec
    if candidate.is_file():
        return candidate
    raise SystemExit(f"找不到轨迹文件：{spec}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="列出所有真机轨迹记录")
    parser.add_argument("--run", help="回放某个 run 目录里的全部轨迹（按文件名顺序）")
    parser.add_argument("--files", nargs="*", default=[], help="按给定顺序回放这些轨迹")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="时间缩放：>1 更慢（仿真里常用 1.5~2.0）")
    parser.add_argument("--action",
                        default="/joint_trajectory_controller/follow_joint_trajectory")
    parser.add_argument("--goal-tolerance", type=float, default=0.02)
    parser.add_argument("--stop-on-failure", action="store_true", default=True)
    parser.add_argument("--keep-going", dest="stop_on_failure", action="store_false")
    args = parser.parse_args(argv)

    if args.list:
        rows = discover()
        print(f"{'run':36s} {'file':28s} {'pts':>4s} {'dur':>7s}")
        for run, name, points, duration in rows:
            print(f"{run:36s} {name:28s} {points:4d} {duration:7.2f}")
        print(f"\n{len(rows)} 条轨迹，来自 {RUNS}")
        return 0

    if args.run:
        run_dir = RUNS / args.run
        if not run_dir.is_dir():
            raise SystemExit(f"没有这个 run：{run_dir}")
        files = [p for p in sorted(run_dir.glob("*.json"))
                 if (doc := json.loads(p.read_text())) and isinstance(doc, dict)
                 and isinstance(doc.get("points"), list) and doc["points"]
                 and "q" in doc["points"][0]]
    elif args.files:
        files = [resolve(spec) for spec in args.files]
    else:
        raise SystemExit("需要 --list / --run / --files 之一")

    print(f"回放 {len(files)} 条真机轨迹，time-scale={args.speed}\n")
    failures = 0
    for path in files:
        doc = json.loads(path.read_text())
        duration = float(doc["points"][-1].get("t", 0.0))
        label = f"{path.parent.name}/{path.name}"
        print(f"[{label}] {len(doc['points'])} 点, {duration:.2f} s")
        command = [sys.executable, "-u", str(EXECUTOR), str(path),
                   "--time-scale", str(args.speed),
                   "--action", args.action,
                   "--goal-tolerance", str(args.goal_tolerance),
                   "--wait-scale", "2.0"]
        result = subprocess.run(command, capture_output=True, text=True, timeout=1800)
        output = (result.stdout or "") + (result.stderr or "")
        ok = "ALL_ASSEMBLY_CHUNKS_OK" in output
        tail = [line for line in output.strip().splitlines() if line.strip()][-3:]
        print("  " + ("OK  " if ok else "FAIL") + "  " + " | ".join(tail))
        if not ok:
            failures += 1
            if args.stop_on_failure:
                print("\n第一条失败就停下（--keep-going 可继续）")
                return 1
    print(f"\n完成：{len(files) - failures}/{len(files)} 条成功")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
