#!/usr/bin/env python3
"""Capture reproducibility identity for one P7 Jazzy simulation run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


DEFAULT_ARTIFACTS = (
    "docs/p7_five_gate_protocol.md",
    "docs/diagnostics/p7/p7_1_static_observation_pose_20260901.json",
    "src/cs625_simulation/worlds/p7_ycb_tomato_light.sdf",
    "src/cs625_simulation/worlds/p7_1_camera_axis_diagnostic.sdf",
    "src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro",
    "src/cs625_ap_description/urdf/cs625_camera_extension.xacro",
    "src/cs625_ap_description/urdf/cs625_parallel_gripper.xacro",
    "src/cs625_bringup/config/cs625_active_perception.srdf",
    "src/cs625_bringup/config/p7_safe_initial_positions.yaml",
    "src/cs625_bringup/config/p7_1_observation_initial_positions.yaml",
    "src/cs625_bringup/config/p7_1_observation_settled_positions.yaml",
    "src/cs625_bringup/config/p7_static_grasp_sim.yaml",
    "src/cs625_bringup/config/sim.yaml",
    "src/cs625_simulation/assets/ycb/MANIFEST.md",
    "src/cs625_sensor_adapter/cs625_sensor_adapter/point_cloud_relay.py",
    "src/cs625_motion_adapter/cs625_motion_adapter/p7_arm_motion_adapter.py",
    "src/cs625_task_orchestrator/cs625_task_orchestrator/p7_attachment_adapter.py",
    "src/cs625_task_orchestrator/cs625_task_orchestrator/p7_episode_contract.py",
    "src/cs625_task_orchestrator/cs625_task_orchestrator/p7_gazebo_pose.py",
    "src/cs625_task_orchestrator/cs625_task_orchestrator/p7_grasp_geometry.py",
    "src/cs625_task_orchestrator/cs625_task_orchestrator/p7_gripper_adapter.py",
    "test/p7_apply_fixture_scene.py",
    "test/p7_assemble_episode.py",
    "test/p7_capture_episode_marker.py",
    "test/p7_capture_gazebo_model_pose.py",
    "test/p7_capture_hold_trace.py",
    "test/p7_capture_manifest.py",
    "test/p7_capture_preflight.py",
    "test/p7_capture_sensor_gate.py",
    "test/p7_capture_axis_marker_diagnostic.py",
    "test/p7_2_inspect_frozen_observation.py",
    "test/p7_2_estimate_cylinder_pose.py",
    "test/p7_2_evaluate_pose_estimates.py",
    "test/p7_static_camera_projection.py",
    "test/p7_solve_static_observation_pose.py",
    "test/p7_capture_tf_pose.py",
    "test/p7_evaluate_geometry_capture.py",
    "test/p7_evaluate_lift_capture.py",
    "test/p7_execute_attachment_capture.py",
    "test/p7_execute_gripper_capture.py",
    "test/p7_execute_pose_capture.py",
    "test/run_p7_adapter.sh",
    "test/run_p7_static_episode_capture.sh",
    "test/run_p7_1_sensor_gate_capture.sh",
    "test/run_p7_1_sensor_sim.sh",
    "test/run_p7_tipfix_sim.sh",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--artifact", action="append", default=[])
    return parser.parse_args()


def command_output(command: list[str], cwd: Path) -> dict:
    try:
        completed = subprocess.run(
            command, cwd=cwd, check=False, capture_output=True, text=True, timeout=20.0
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": command, "returncode": None, "stdout": "", "stderr": str(error)}
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    arguments = parse_arguments()
    root = arguments.repo_root.resolve()
    artifacts = [root / relative for relative in (*DEFAULT_ARTIFACTS, *arguments.artifact)]
    missing = [str(path) for path in artifacts if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"P7 manifest artifacts missing: {missing}")
    git_head = command_output(["git", "rev-parse", "HEAD"], root)
    git_status = command_output(["git", "status", "--porcelain=v1"], root)
    record = {
        "manifest_schema": "p7_jazzy_runtime_manifest_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository_root": str(root),
        "git": {
            "head": git_head,
            "status": git_status,
            "dirty": bool(git_status.get("stdout")),
        },
        "runtime": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
            "gz_sim_versions": command_output(["gz", "sim", "--versions"], root),
            "ros2_doctor": command_output(["ros2", "doctor", "--report"], root),
        },
        "artifacts": {
            str(path.relative_to(root)).replace("\\", "/"): {
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in artifacts
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))


if __name__ == "__main__":
    main()
