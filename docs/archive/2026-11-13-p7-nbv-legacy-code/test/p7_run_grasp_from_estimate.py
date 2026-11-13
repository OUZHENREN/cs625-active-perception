#!/usr/bin/env python3
"""Continue a live post-NBV observation using existing P7 phase executors.

Run inside the existing Jazzy environment, ROS domain and Gazebo partition:
  python3 test/p7_run_grasp_from_estimate.py --input-dir CAPTURE_DIR \
      --output-dir NEW_GRASP_DIR --view-trace VIEW_TRACE_JSON --until close

This runner performs no perception or planning itself. It composes the existing
estimator handoff, CAD template, MoveIt phase executors and Gazebo measurements.
No Gazebo attachment is commanded: lift relies on physical finger contact.
MoveIt's attached body is only the carried-object planning representation.
--resume skips only this runner's successful completed stage receipts.
"""
from __future__ import annotations

import argparse
import atexit
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parent.parent
LOCAL_CENTER = ["-0.0091685", "0.0840170", "0.0510065"]


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def now():
    return datetime.now(timezone.utc).isoformat()


def stage(args, name, script, arguments, contact_policy=None, settle_sec=0.0):
    directory = args.output_dir
    outcome_path = directory / f"{name}_stage.json"
    if outcome_path.exists():
        if args.resume and read(outcome_path).get("success") is True:
            return
        raise RuntimeError(f"stage already exists; preserve failed attempt: {outcome_path}")
    command = [sys.executable, str(ROOT / "test" / script), *map(str, arguments)]
    monitor = None
    contact_path = directory / f"{name}_contacts.json"
    outcome = {"stage": name, "script": script, "started_at_utc": now(),
               "success": False, "command": command}
    try:
        if contact_policy:
            monitor = subprocess.Popen(
                [str(args.contact_monitor_bin), "300", str(contact_path), contact_policy],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            selector = selectors.DefaultSelector()
            selector.register(monitor.stdout, selectors.EVENT_READ)
            ready = selector.select(timeout=12.0)
            line = monitor.stdout.readline().strip() if ready else ""
            selector.close()
            if line != "P7_CONTACT_MONITOR_READY":
                raise RuntimeError(f"contact monitor startup: {line}")
        outcome["command_issued_at_utc"] = now()
        with (directory / f"{name}.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=270, check=False)
        outcome["command_returned_at_utc"] = now()
        outcome["return_code"] = result.returncode
        if settle_sec:
            time.sleep(settle_sec)
        if monitor:
            outcome["contact_monitor_live_through_command"] = monitor.poll() is None
            monitor.send_signal(signal.SIGINT)
            monitor.communicate(timeout=5)
            contacts = read(contact_path)
            outcome["contact_path"] = str(contact_path)
            outcome["unexpected_environment_collision"] = contacts["unexpected_collision"]
            if not outcome["contact_monitor_live_through_command"] or contacts["unexpected_collision"]:
                raise RuntimeError(f"{name}: unexpected contact or incomplete contact recording")
        if result.returncode != 0:
            raise RuntimeError(f"{name} failed; see {directory / (name + '.log')}")
        outcome["success"] = True
    except Exception as error:
        outcome["error"] = str(error)
        raise
    finally:
        if monitor and monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
            monitor.communicate(timeout=5)
        outcome["finished_at_utc"] = now()
        write(outcome_path, outcome)
    print(f"P7_GRASP_STAGE_COMPLETE {name}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True, type=Path)
    parser.add_argument("--view-trace", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--candidate-id", default="tomato_historical_r5")
    parser.add_argument("--until", choices=("pregrasp", "approach", "close", "hold"), default="hold")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--close-position-m", type=float, default=0.022)
    parser.add_argument("--template-yaws-deg", type=float, nargs="+", default=(0.0,))
    parser.add_argument("--level-side-yaws-deg", type=float, nargs="+", default=())
    parser.add_argument("--top-grasp-yaws-deg", type=float, nargs="+", default=())
    parser.add_argument("--top-grasp-pitch-deg", type=float, default=90.0)
    parser.add_argument("--top-grasp-x-offset-m", type=float, default=0.0)
    parser.add_argument("--lift-pose-key", choices=("lift", "pregrasp"), default="lift",
                        help="candidate pose used for the post-close lift command")
    parser.add_argument("--pregrasp-branch", type=Path,
                        help="use the checked joint branch and an isolated instance of the existing adapter")
    parser.add_argument("--contact-monitor-bin", type=Path, default=Path("/tmp/p7_gazebo_contact_monitor"))
    args = parser.parse_args()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError("requires isolated P7 simulation execution environment")
    for key in ("input_dir", "output_dir", "view_trace"):
        setattr(args, key, getattr(args, key).resolve())
    sensor = read(args.input_dir / "post_move_sensor/raw/gate.json")
    evaluation = read(args.input_dir / "post_move_pose/evaluation.json")
    view = read(args.view_trace)
    if not (sensor.get("gate_pass") and evaluation.get("gate_pass") and view["receipt"].get("success")):
        raise RuntimeError("preceding live view/sensor/pose gate is not accepted")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    arm_topics = []
    if args.pregrasp_branch:
        args.pregrasp_branch = args.pregrasp_branch.resolve()
        branch = read(args.pregrasp_branch)
        if branch.get("success") is not True or branch.get("candidate_id") != args.candidate_id:
            raise ValueError("checked pregrasp branch does not match selected candidate")
        config = yaml.safe_load((ROOT / "src/cs625_bringup/config/p7_static_grasp_sim.yaml").read_text())
        parameters = config["/cs625_p7_arm_motion_adapter"]["ros__parameters"]
        parameters.update(command_topic="/p7_5/arm_motion_command", status_topic="/p7_5/arm_motion_status")
        adapter_command = ["ros2", "run", "cs625_motion_adapter", "p7_arm_motion_adapter", "--ros-args",
                           "-r", "__node:=cs625_p75_grasp_motion_adapter"]
        for name, value in parameters.items():
            adapter_command += ["-p", name + ":=" + json.dumps(value, separators=(",", ":"))]
        adapter_log = (args.output_dir / "grasp_adapter.log").open("a", encoding="utf-8")
        adapter = subprocess.Popen(adapter_command, stdout=adapter_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)

        def stop_adapter():
            if adapter.poll() is None:
                os.killpg(adapter.pid, signal.SIGINT)
                try:
                    adapter.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(adapter.pid, signal.SIGTERM)
            adapter_log.close()

        atexit.register(stop_adapter)
        arm_topics = ["--command-topic", parameters["command_topic"],
                      "--status-topic", parameters["status_topic"]]
    estimate = args.input_dir / "post_move_pose/window_01_estimate.json"
    out = args.output_dir
    stage(args, "base_from_world", "p7_capture_tf_pose.py", [
        "--parent-frame", "base_link", "--child-frame", "world", "--output", out / "base_from_world.json"])
    stage(args, "generation", "p7_generate_grasp_candidates.py", [
        "--estimate", estimate, "--frame-transform", out / "base_from_world.json",
        "--template-yaws-deg", *args.template_yaws_deg,
        *(["--level-side-yaws-deg", *args.level_side_yaws_deg] if args.level_side_yaws_deg else []),
        *(["--top-grasp-yaws-deg", *args.top_grasp_yaws_deg] if args.top_grasp_yaws_deg else []),
        "--top-grasp-pitch-deg", args.top_grasp_pitch_deg,
        "--top-grasp-x-offset-m", args.top_grasp_x_offset_m,
        "--output", out / "grasp_candidates.json"])
    generated = read(out / "grasp_candidates.json")
    candidate = next(c for c in generated["candidates"] if c["candidate_id"] == args.candidate_id)
    write(out / "grasp_source.json", {
        "schema": "p7_perception_grasp_source_v1", "input_dir": str(args.input_dir),
        "view_trace": str(args.view_trace), "candidate_id": args.candidate_id,
        "gazebo_attachment_commanded": False, "mode": "physical_contact_friction",
        "lift_pose_key": args.lift_pose_key,
        "pose_estimate_file": str(estimate), "ground_truth_used_for_grasp_generation": False,
        "runtime": {name: os.environ.get(name) for name in ("ROS_DOMAIN_ID", "GZ_PARTITION")}})
    stage(args, "scene_full", "p7_apply_fixture_scene.py", [
        "--profile", "severe_v5", "--mode", "full", "--target-estimate-file", estimate,
        "--output", out / "scene_full.json"])
    stage(args, "open", "p7_execute_gripper_capture.py", [
        "--command-id", out.name + "_open", "--target-position-m", "0.030",
        "--output", out / "open.json"], "view")

    def move(name, phase, pose):
        branch_arguments = (["--joint-goal-file", args.pregrasp_branch]
                            if phase == "pregrasp" and args.pregrasp_branch else [])
        stage(args, name, "p7_execute_pose_capture.py", [
            "--command-id", out.name + "_" + name, "--phase", phase,
            "--position", *[pose["position"][a] for a in "xyz"],
            "--orientation", *[pose["orientation"][a] for a in "xyzw"],
            "--output", out / (name + ".json"), *branch_arguments, *arm_topics],
            "view" if phase == "pregrasp" else "grasp")

    move("pregrasp", "pregrasp", candidate["pregrasp"])
    if args.until == "pregrasp":
        return
    stage(args, "scene_approach", "p7_apply_fixture_scene.py", [
        "--profile", "severe_v5", "--mode", "approach", "--target-estimate-file", estimate,
        "--output", out / "scene_approach.json"])
    move("approach", "approach", candidate["grasp"])
    if args.until == "approach":
        return
    stage(args, "close", "p7_execute_gripper_capture.py", [
        "--command-id", out.name + "_close", "--target-position-m", args.close_position_m,
        "--output", out / "close.json"], "grasp", settle_sec=0.8)
    contact = read(out / "close_contacts.json")
    if not contact["bilateral_finger_contact"]["observed"]:
        raise RuntimeError("BILATERAL_FINGER_CONTACT_NOT_OBSERVED; no lift/attachment commanded")
    if args.until == "close":
        return
    stage(args, "target_before_lift", "p7_capture_gazebo_model_pose.py", [
        "--model", "target_object", "--output", out / "target_before_lift.json"])
    stage(args, "grasp_before_lift", "p7_capture_tf_pose.py", [
        "--parent-frame", "world", "--child-frame", "p7_grasp_center_link",
        "--output", out / "grasp_before_lift.json"])
    stage(args, "scene_carried", "p7_apply_fixture_scene.py", [
        "--profile", "severe_v5", "--mode", "attached", "--target-estimate-file", estimate,
        "--grasp-pose-file", out / "grasp_before_lift.json", "--output", out / "scene_carried.json"])
    move("lift", "lift", candidate[args.lift_pose_key])
    stage(args, "target_after_lift", "p7_capture_gazebo_model_pose.py", [
        "--model", "target_object", "--output", out / "target_after_lift.json"])
    stage(args, "lift_measurement", "p7_evaluate_lift_capture.py", [
        "--before-model-pose", out / "target_before_lift.json",
        "--after-model-pose", out / "target_after_lift.json",
        "--target-local-center-m", *LOCAL_CENTER, "--minimum-height-m", "0.10",
        "--output", out / "lift_measurement.json"])
    stage(args, "hold", "p7_capture_hold_trace.py", [
        "--model", "target_object", "--duration-sec", "2.2", "--sample-period-sec", "0.2",
        "--clock-domain", "simulation",
        "--target-local-center-m", *LOCAL_CENTER, "--max-height-drift-m", "0.01",
        "--output", out / "hold.json"], "grasp")
    print("P7_PHYSICAL_GRASP_STAGES_COMPLETE; independent episode scoring still required", flush=True)


if __name__ == "__main__":
    main()
