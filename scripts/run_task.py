#!/usr/bin/env python3
"""Single entry point for the six task modes (3 scopes x 2 modes).

    scope  load    = grasp the workpiece on its table and insert it into the slot
    scope  unload  = take the workpiece out of the slot and place it back
    scope  full    = load, then (after an explicit handover confirmation) unload

    mode   safe      = conservative: many segments, a dwell after each one
    mode   efficient = one segment per phase, no dwell, fast cruise, and the
                       same conservative speed inside the slot

This script owns no motion logic.  It composes the existing pieces:

    config/tasks/common.yaml + config/tasks/<scope>_<mode>.yaml
        -> shallow merge -> runtime/effective_<scope>_<mode>.yaml
    runtime/environment/latest.json  (environment model + gate)
    scripts/run_pick_and_place.py    (the phase runner, --flow pick_place|unload)

Every flow must be preceded by a fresh environment model: the gate compares the
model against the thresholds in ``environment.thresholds`` and refuses to run
when the slot moved, the workpiece moved, the cloud is too thin, or the model is
older than ``max_age_h`` hours.

Usage:
  python3 scripts/run_task.py --scope load --mode efficient --dry-run
  python3 scripts/run_task.py --scope load --mode efficient --plan-only
  python3 scripts/run_task.py --scope full --mode safe --confirm
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib
import subprocess
import sys
import time
from datetime import datetime, timezone

import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent
TASK_DIR = REPO / "config" / "tasks"
RUNTIME = REPO / "runtime"
RUNNER = REPO / "scripts" / "run_pick_and_place.py"
MODEL_ENV = REPO / "scripts" / "model_environment.py"

SCOPES = ("load", "unload", "full")
MODES = ("safe", "efficient")

# Phase lists are owned by the runner; these are used for reporting and for the
# full-flow handover point.  Keep them in sync with phase_plan() / run_unload().
LOAD_PHASES = [
    "pre_grasp", "gripper_init", "grip", "grasp", "lift", "transit", "staging",
    "pre_insert", "insert", "straighten", "release", "withdraw", "baseline",
]
UNLOAD_PHASES = [
    "d1_staging", "d2_init_before_slot", "d2a_pre_insert",
    "d2b_entrance_tilted", "d2c_straighten", "d3a_gripper_init", "d3b_grip",
    "d3c_pull_test", "d3d_pull_check", "d3e_return", "d4_tilt_for_removal",
    "d5_extract_*", "d6a_lift", "d6b_transit", "d7a_place_pre_grasp",
    "d7b_place_grasp", "d8_release", "d8b_init_before_retreat", "d9a_withdraw",
    "d9b_lift", "d9c_baseline",
]


# --------------------------------------------------------------------- config
def mode_path(scope: str, mode: str) -> pathlib.Path:
    return TASK_DIR / f"{scope}_{mode}.yaml"


def common_path() -> pathlib.Path:
    return TASK_DIR / "common.yaml"


def scope_steps(scope: str) -> list[str]:
    """Ordered list of runner invocations for a scope."""
    if scope == "load":
        return ["load"]
    if scope == "unload":
        return ["unload"]
    return ["load", "unload"]


def flow_of(step: str) -> str:
    return "pick_place" if step == "load" else "unload"


def phases_of(step: str) -> list[str]:
    return LOAD_PHASES if step == "load" else UNLOAD_PHASES


def merge_configs(common: dict, mode_cfg: dict) -> dict:
    """Shallow merge: the mode file wins, world constants stay in common.yaml.

    A shallow merge is enough because both files use the flat key names that
    scripts/run_pick_and_place.py reads (pre_grasp, grasp_q, phase_speeds, ...).
    ``environment`` and ``verification`` live in common.yaml and are carried
    through untouched for the gate and the runner.
    """
    merged = dict(common)
    merged.update(mode_cfg)
    return merged


def effective_for_step(scope: str, mode: str, merged: dict, step: str) -> tuple[pathlib.Path, dict]:
    """Effective config for one step of the scope.

    ``full`` runs two different phase sequences (load body, then unload body), so
    a per-step file is written with ``phases:`` set from ``phases_<step>`` when
    the mode file provides one.  Everything else is identical between the steps.
    """
    data = dict(merged)
    key = f"phases_{step}"
    if key in data:
        data["phases"] = list(data[key])
    return write_effective(f"{scope}_{mode}_{step}", "", data), data


def write_effective(scope: str, mode: str, merged: dict) -> pathlib.Path:
    RUNTIME.mkdir(parents=True, exist_ok=True)
    name = f"effective_{scope}" + (f"_{mode}" if mode else "") + ".yaml"
    out = RUNTIME / name
    out.write_text(yaml.safe_dump(merged, sort_keys=False, allow_unicode=True),
                   encoding="utf-8")
    return out


# ------------------------------------------------------------- environment
def load_gate_module():
    """Import the gate function from scripts/model_environment.py (pure python)."""
    spec = importlib.util.spec_from_file_location("model_environment", MODEL_ENV)
    module = importlib.util.module_from_spec(spec)
    sys.modules["model_environment"] = module
    spec.loader.exec_module(module)
    return module


def latest_model_path() -> pathlib.Path | None:
    pointer = RUNTIME / "environment" / "latest.json"
    if pointer.exists():
        return pointerlib_resolve(pointer)
    link = RUNTIME / "environment" / "latest"
    if link.exists():
        return link / "model.json"
    return None


def pointerlib_resolve(pointer: pathlib.Path) -> pathlib.Path:
    """latest.json holds either a path or {"model": ...}; accept both."""
    text = pointer.read_text(encoding="utf-8").strip()
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = text
    if isinstance(data, dict):
        for key in ("model", "path", "latest"):
            if key in data:
                candidate = pathlib.Path(str(data[key]))
                return candidate if candidate.is_absolute() else pointer.parent / candidate
    candidate = pathlib.Path(str(data))
    return candidate if candidate.is_absolute() else pointer.parent / candidate


def check_environment(merged: dict, model_path: pathlib.Path | None) -> tuple[bool, list[str], dict]:
    """Evaluate the environment gate; returns (ok, reasons, model)."""
    thresholds = dict((merged.get("environment") or {}).get("thresholds") or {})
    required = bool((merged.get("environment") or {}).get("required", True))
    if model_path is None or not pathlib.Path(model_path).exists():
        reason = [f"环境模型缺失：{model_path} 不存在，先运行 "
                  "python3 scripts/model_environment.py --dataset ... --out runtime/environment"]
        return (not required), reason, {}
    model = json.loads(pathlib.Path(model_path).read_text(encoding="utf-8"))
    module = load_gate_module()
    ok, reasons = module.evaluate_gate(model, thresholds, datetime.now(timezone.utc).isoformat())
    return ok, reasons, model


def print_model_summary(model: dict, model_path: pathlib.Path | None) -> None:
    if not model:
        print("environment model : MISSING")
        return
    delta = model.get("baseline_delta", {})
    print(f"environment model : {model_path}")
    print(f"  created_at      : {model.get('created_at')}")
    print(f"  frames/points   : {model.get('frames_used')} / {model.get('valid_points')}")
    print(f"  slot delta      : {delta.get('slot_mm', 0):.2f} mm / "
          f"{delta.get('slot_deg', 0):.2f} deg")
    print(f"  workpiece delta : {delta.get('workpiece_mm', 0):.2f} mm / "
          f"{delta.get('workpiece_deg', 0):.2f} deg")
    tables = model.get("tables", {})
    for name, table in tables.items():
        print(f"  {name:<16}: z={table.get('z', float('nan')):.4f} m")


# --------------------------------------------------------------------- runner
def runner_command(effective: pathlib.Path, step: str, args) -> list[str]:
    command = [sys.executable, "-u", str(RUNNER), "--config", str(effective),
               "--flow", flow_of(step),
               "--log", str(RUNTIME / f"phase_log_{step}.json")]
    if args.plan_only:
        command.append("--plan-only")
    elif args.confirm:
        command.append("--confirm")
    if args.phases:
        command += ["--phases", args.phases]
    start_joints = getattr(args, "start_joints", "")
    if args.plan_only and start_joints:
        command += ["--start-joints", start_joints]
    if getattr(args, "allow_release", False):
        command.append("--allow-release")
    return command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scope", required=True, choices=SCOPES)
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--world", default="real", choices=("real", "sim"),
                        help="real = 真机（默认）；sim = 仿真。sim 会先叠加 "
                             "config/tasks/world_sim.yaml（最小差异集：控制动作名等）")
    parser.add_argument("--confirm", action="store_true",
                        help="actually execute motion (required for a real run)")
    parser.add_argument("--plan-only", action="store_true",
                        help="plan every phase without moving the arm")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the phase list, speed profile and environment "
                             "model, then stop")
    parser.add_argument("--model", default="",
                        help="environment model.json to use (default: runtime/environment/latest)")
    parser.add_argument("--phases", default="", help="restrict a single step to these phases")
    parser.add_argument("--start-joints", default="",
                        help="plan-only: six comma-separated joint angles to chain the "
                             "plans from (default: the measured state)")
    parser.add_argument("--handover", default="confirm",
                        choices=["confirm", "auto"],
                        help="full scope: stop for confirmation between load and unload, "
                             "or continue automatically")
    parser.add_argument("--allow-release", action="store_true",
                        help="allow the gripper to release (owner approval for this run)")
    parser.add_argument("--summary", default="", help="where to write the run summary")
    parser.add_argument("--no-record", action="store_true",
                        help="do not start the experiment recorder.  By default every "
                             "real run (--confirm) is recorded: 20 Hz timeseries, phase "
                             "markers, camera frames, then trimmed and archived to "
                             "~/cs625_experiment_archive with an index line.")
    args = parser.parse_args()

    common = yaml.safe_load(common_path().read_text(encoding="utf-8"))
    mode_cfg = yaml.safe_load(mode_path(args.scope, args.mode).read_text(encoding="utf-8"))
    merged = dict(common)
    if args.world == "sim":
        overlay_path = TASK_DIR / "world_sim.yaml"
        if not overlay_path.exists():
            print(f"missing world overlay: {overlay_path}")
            return 2
        merged.update(yaml.safe_load(overlay_path.read_text(encoding="utf-8")))
    merged = merge_configs(merged, mode_cfg)
    merged["world"] = args.world
    effective = write_effective(args.scope, args.mode, merged)

    model_path = pathlib.Path(args.model) if args.model else latest_model_path()
    ok, reasons, model = check_environment(merged, model_path)

    print(f"scope={args.scope} mode={args.mode} world={args.world} "
          f"flow(s)={scope_steps(args.scope)}")
    print(f"effective config : {effective}")
    print(f"control action   : {merged.get('action', '/arm_controller/follow_joint_trajectory')}")
    print_model_summary(model, model_path)
    print(f"environment gate : {'PASS' if ok else 'REJECT'}")
    for reason in reasons:
        print(f"  - {reason}")

    steps = scope_steps(args.scope)
    summary = {"scope": args.scope, "mode": args.mode, "config": str(effective),
               "model": str(model_path) if model_path else None,
               "gate_ok": ok, "gate_reasons": reasons,
               "started_at": datetime.now(timezone.utc).isoformat(),
               "steps": []}

    if args.dry_run:
        for step in steps:
            _path, step_cfg = effective_for_step(args.scope, args.mode, merged, step)
            names = list(step_cfg.get("phases") or phases_of(step))
            print(f"\n[{step}] flow={flow_of(step)} phases={len(names)} "
                  f"config={_path.name}")
            for name in names:
                key = name.split(":", 1)[1] if name.startswith("observe:") else name
                if name.startswith("observe:"):
                    pose = (step_cfg.get("observe") or {}).get(key, {})
                    check = "check" if pose.get("check") else "no-check"
                    print(f"    {name:<28} observe ({check}) "
                          f"xyz={[round(v, 3) for v in pose.get('xyz', [])]}")
                    continue
                speed = (step_cfg.get("phase_speeds") or {}).get(name, {})
                chunks = speed.get("chunks", step_cfg.get("max_chunks", "?"))
                velocity = speed.get("velocity", step_cfg.get("velocity_scale"))
                band = speed.get("slow_band_mm")
                extra = f" band={band}" if band else ""
                print(f"    {name:<28} v={velocity:<6} chunks={chunks}{extra}")
        print("\ndry run: no motion, no planning")
        _write_summary(summary, args.summary)
        return 0

    if not ok:
        print("\nrefusing to run: the environment model did not pass the gate")
        _write_summary(summary, args.summary)
        return 2

    # After the environment model is accepted the camera can run in its fast
    # profile; every observation check switches back to the modeling profile
    # (depth is what the gate and the pose estimation need).
    if args.confirm and not args.plan_only:
        camera_script = pathlib.Path(__file__).resolve().parent / "camera_mode.py"
        if camera_script.exists():
            subprocess.run([sys.executable, "-u", str(camera_script), "fast"],
                           check=False)
            print("camera: switched to the fast profile (observation checks switch back)")

    recorder_started = False
    if args.confirm and not args.no_record:
        script = pathlib.Path(__file__).resolve().parent / "experiment_recorder.py"
        if script.exists():
            subprocess.run(
                [sys.executable, "-u", str(script), "start", "--world", args.world,
                 "--scope", args.scope, "--mode", args.mode, "--images"],
                check=False)
            recorder_started = True
            print(f"experiment recorder: started ({args.world}/{args.scope}/{args.mode})")
        else:
            print("experiment recorder: script missing, run will not be recorded")

    for index, step in enumerate(steps):
        if index > 0 and args.handover == "confirm":
            print("\n" + "=" * 60)
            print("HANDOVER: load finished.  Check the workpiece is seated in the slot,")
            print("then re-measure the pose and continue with the unload flow.")
            print("Re-run with --handover auto to skip this stop.")
            print("=" * 60)
            summary["steps"].append({"step": step, "skipped": "handover stop"})
            _write_summary(summary, args.summary)
            return 0
        step_path, _ = effective_for_step(args.scope, args.mode, merged, step)
        command = runner_command(step_path, step, args)
        print(f"\n=== [{step}] {' '.join(command)}")
        started = time.time()
        completed = subprocess.run(command)
        elapsed = time.time() - started
        summary["steps"].append({"step": step, "returncode": completed.returncode,
                                 "seconds": round(elapsed, 1)})
        _write_summary(summary, args.summary)
        if completed.returncode != 0:
            print(f"[{step}] failed with code {completed.returncode}")
            return completed.returncode

    elapsed = sum(item.get("seconds", 0) for item in summary["steps"])
    print(f"\nall steps finished in {elapsed:.1f} s")
    summary["recorder"] = {"started": recorder_started, "archived": None}
    _write_summary(summary, args.summary)

    if recorder_started:
        stopped = subprocess.run(
            [sys.executable, "-u",
             str(pathlib.Path(__file__).resolve().parent / "experiment_recorder.py"),
             "stop"], check=False)
        run_dirs = sorted((RUNTIME / "experiments").glob("*"), key=lambda p: p.stat().st_mtime,
                          reverse=True) if (RUNTIME / "experiments").exists() else []
        if run_dirs and stopped.returncode == 0:
            archived = subprocess.run(
                [sys.executable, "-u",
                 str(pathlib.Path(__file__).resolve().parent / "archive_experiment.py"),
                 "--run-dir", str(run_dirs[0])], check=False)
            if archived.returncode == 0:
                print(f"experiment archived: {run_dirs[0].name}")
                summary["recorder"]["archived"] = run_dirs[0].name
                _write_summary(summary, args.summary)
    camera_script = pathlib.Path(__file__).resolve().parent / "camera_mode.py"
    if args.confirm and camera_script.exists():
        subprocess.run([sys.executable, "-u", str(camera_script), "modeling"],
                       check=False)
        print("camera: back to the modeling profile")
    return 0


def _write_summary(summary: dict, path: str) -> None:
    summary["finished_at"] = datetime.now(timezone.utc).isoformat()
    target = pathlib.Path(path) if path else (RUNTIME / "last_run_summary.json")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                      encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
