#!/usr/bin/env python3
"""Capture the P7 single-run readiness gate as an atomic JSON record."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
import yaml


REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_controller",
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-positions-file", required=True, type=Path)
    parser.add_argument("--expected-positions-file", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--joint-tolerance-rad", type=float, default=0.002)
    parser.add_argument("--settle-sim-sec", type=float, default=5.0)
    parser.add_argument("--stability-window-sec", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    arguments = parser.parse_args()
    if (
        arguments.joint_tolerance_rad <= 0.0
        or arguments.settle_sim_sec < 0.0
        or arguments.stability_window_sec <= 0.0
        or arguments.timeout_sec <= 0.0
    ):
        parser.error("tolerance, stability window and timeout must be positive")
    return arguments


def atomic_write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    expected_positions_file = (
        arguments.expected_positions_file or arguments.initial_positions_file
    )
    expected = yaml.safe_load(expected_positions_file.read_text(encoding="utf-8"))
    if not isinstance(expected, dict) or not expected:
        raise ValueError("initial-position profile must be a non-empty mapping")
    expected = {str(name): float(value) for name, value in expected.items()}
    if any(not math.isfinite(value) for value in expected.values()):
        raise ValueError("initial-position profile contains a non-finite value")

    rclpy.init()
    node = rclpy.create_node("p7_capture_preflight")
    measured: dict[str, float] = {}
    joint_samples: dict[str, list[float]] = {name: [] for name in expected}
    clock_samples: list[float] = []

    def on_joint_state(message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            name = str(name)
            value = float(position)
            measured[name] = value
            if name in joint_samples:
                joint_samples[name].append(value)

    def on_clock(message: Clock) -> None:
        clock_samples.append(
            message.clock.sec + message.clock.nanosec / 1_000_000_000.0
        )

    node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
    node.create_subscription(Clock, "/clock", on_clock, 10)
    controller_client = node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )
    deadline = time.monotonic() + arguments.timeout_sec
    while (
        any(name not in measured for name in expected)
        or len(clock_samples) < 10
        or not controller_client.service_is_ready()
    ) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Renderer load changes the ratio between wall time and simulation time.
    # Wait in /clock time so every run receives the same physics-settling
    # interval before stability is assessed.
    settle_start = clock_samples[-1] if clock_samples else None
    settle_target = (
        settle_start + arguments.settle_sim_sec
        if settle_start is not None
        else None
    )
    while (
        settle_target is not None
        and clock_samples[-1] < settle_target
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    # Controller activation and sensor-topic readiness can occur in the same
    # scheduler tick.  Observe a bounded post-settling window so the gate
    # records the settled state and its spread, rather than a single transient
    # joint-state sample.
    stability_deadline = min(
        deadline, time.monotonic() + arguments.stability_window_sec
    )
    for samples in joint_samples.values():
        samples.clear()
    while time.monotonic() < stability_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    controllers: dict[str, dict[str, str]] = {}
    if controller_client.service_is_ready():
        future = controller_client.call_async(ListControllers.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        response = future.result() if future.done() else None
        if response is not None:
            controllers = {
                controller.name: {
                    "state": controller.state,
                    "type": controller.type,
                }
                for controller in response.controller
            }

    errors = {
        name: abs(measured[name] - target)
        for name, target in expected.items()
        if name in measured
    }
    joint_spreads = {
        name: max(samples) - min(samples)
        for name, samples in joint_samples.items()
        if samples
    }
    missing_joints = sorted(set(expected) - set(measured))
    inactive_controllers = sorted(
        name
        for name in REQUIRED_CONTROLLERS
        if controllers.get(name, {}).get("state") != "active"
    )
    clock_regressions = sum(
        current < previous
        for previous, current in zip(clock_samples, clock_samples[1:])
    )
    failures = []
    if missing_joints:
        failures.append("INITIAL_JOINTS_UNOBSERVED")
    if errors and max(errors.values()) > arguments.joint_tolerance_rad:
        failures.append("INITIAL_JOINT_MISMATCH")
    if joint_spreads and max(joint_spreads.values()) > arguments.joint_tolerance_rad:
        failures.append("INITIAL_JOINT_UNSTABLE")
    if inactive_controllers:
        failures.append("CONTROLLERS_NOT_ACTIVE")
    if len(clock_samples) < 10:
        failures.append("CLOCK_UNOBSERVED")
    if clock_regressions:
        failures.append("CLOCK_REGRESSION")
    if settle_target is not None and clock_samples[-1] < settle_target:
        failures.append("SETTLE_SIM_TIME_INCOMPLETE")

    payload = {
        "capture_schema": "p7_preflight_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "success": not failures,
        "failure_codes": failures,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
        "controllers": controllers,
        "required_controllers": sorted(REQUIRED_CONTROLLERS),
        "inactive_controllers": inactive_controllers,
        "initial_positions_file": str(arguments.initial_positions_file),
        "expected_positions_file": str(expected_positions_file),
        "expected_joint_positions_rad": expected,
        "observed_joint_positions_rad": {
            name: measured[name] for name in expected if name in measured
        },
        "joint_errors_rad": errors,
        "joint_sample_counts": {
            name: len(samples) for name, samples in joint_samples.items()
        },
        "joint_spreads_rad": joint_spreads,
        "joint_tolerance_rad": arguments.joint_tolerance_rad,
        "settle_sim_sec": arguments.settle_sim_sec,
        "settle_sim_completed": bool(
            settle_target is not None and clock_samples[-1] >= settle_target
        ),
        "stability_window_sec": arguments.stability_window_sec,
        "clock_sample_count": len(clock_samples),
        "clock_regression_count": clock_regressions,
    }
    atomic_write(arguments.output, payload)
    print(json.dumps(payload, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
