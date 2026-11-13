#!/usr/bin/env python3
"""Capture a timed Gazebo target-pose trace for the P7 hold gate."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time

from cs625_task_orchestrator.p7_grasp_geometry import target_center_world
from cs625_task_orchestrator.p7_gazebo_pose import parse_gz_model_pose
import rclpy
from rosgraph_msgs.msg import Clock


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="target_object")
    parser.add_argument("--duration-sec", type=float, default=2.2)
    parser.add_argument("--sample-period-sec", type=float, default=0.2)
    parser.add_argument("--max-height-drift-m", type=float, default=0.01)
    parser.add_argument("--clock-domain", choices=("simulation", "wall"), default="wall")
    parser.add_argument(
        "--target-local-center-m", required=True, nargs=3, type=float,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    if arguments.duration_sec < 2.0 or arguments.sample_period_sec <= 0.0:
        parser.error("duration must be >=2 s and sample period must be positive")
    return arguments


class SimulationClock:
    def __init__(self, timeout_sec: float = 8.0) -> None:
        self.latest_sec: float | None = None
        rclpy.init()
        self.node = rclpy.create_node("p7_hold_trace_clock")
        self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        deadline = time.monotonic() + timeout_sec
        while self.latest_sec is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.latest_sec is None:
            self.close()
            raise RuntimeError("simulation /clock unavailable")

    def _on_clock(self, message: Clock) -> None:
        self.latest_sec = message.clock.sec + message.clock.nanosec / 1_000_000_000.0

    def now(self) -> float:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.latest_sec is None:
            raise RuntimeError("simulation /clock unavailable")
        return self.latest_sec

    def wait_until(self, target_sec: float, timeout_sec: float = 20.0) -> float:
        deadline = time.monotonic() + timeout_sec
        while self.now() < target_sec and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        current = self.now()
        if current < target_sec:
            raise RuntimeError("timed out waiting for simulation clock")
        return current

    def close(self) -> None:
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def evaluate_hold_window(
    elapsed_seconds: list[float],
    heights_m: list[float],
    requested_duration_sec: float,
    maximum_drift_m: float,
) -> tuple[float, float, bool]:
    """Evaluate only time covered by decoded samples, excluding query latency."""

    if not elapsed_seconds or len(elapsed_seconds) != len(heights_m):
        raise ValueError("hold evidence needs equally sized, non-empty samples")
    observed_duration = elapsed_seconds[-1] - elapsed_seconds[0]
    drift = max(heights_m) - min(heights_m)
    return (
        observed_duration,
        drift,
        observed_duration >= requested_duration_sec and drift <= maximum_drift_m,
    )


def conservative_elapsed_seconds(samples: list[dict], clock_domain: str) -> list[float]:
    if not samples:
        raise ValueError("hold evidence needs non-empty samples")
    if clock_domain == "wall":
        return [sample["elapsed_wall_sec"] for sample in samples]
    first_finish = samples[0]["query_finished_sim_sec"]
    return [0.0] + [sample["query_started_sim_sec"] - first_finish for sample in samples[1:]]


def main() -> None:
    arguments = parse_arguments()
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError(
            "set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation"
        )
    command = ["gz", "model", "-m", arguments.model, "--pose"]
    samples = []
    clock = SimulationClock() if arguments.clock_domain == "simulation" else None
    started = time.monotonic()
    started_sim = clock.now() if clock is not None else None
    next_sample = started_sim if clock is not None else started
    first_captured = None
    try:
        while True:
            if clock is None:
                now = time.monotonic()
                if now < next_sample:
                    time.sleep(next_sample - now)
                query_started_sim = None
            else:
                query_started_sim = clock.wait_until(next_sample)
            query_started_wall = time.monotonic()
            completed = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=8.0
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"gz model failed ({completed.returncode}): {completed.stderr.strip()}"
                )
            captured = time.monotonic()
            query_finished_sim = clock.now() if clock is not None else None
            if first_captured is None:
                first_captured = captured
            sample = {
                "elapsed_sec": captured - started,
                "elapsed_wall_sec": captured - started,
                "query_started_wall_sec": query_started_wall - started,
                "query_finished_wall_sec": captured - started,
                "pose": parse_gz_model_pose(completed.stdout, arguments.model),
                "raw_output": completed.stdout,
            }
            if started_sim is not None:
                sample["elapsed_sim_sec"] = query_finished_sim - started_sim
                sample["query_started_sim_sec"] = query_started_sim
                sample["query_finished_sim_sec"] = query_finished_sim
            samples.append(sample)
            evidence_elapsed = conservative_elapsed_seconds(samples, arguments.clock_domain)
            if evidence_elapsed[-1] >= arguments.duration_sec:
                break
            next_sample += arguments.sample_period_sec
    finally:
        if clock is not None:
            clock.close()

    local_center = tuple(arguments.target_local_center_m)
    centers = [target_center_world(sample["pose"], local_center) for sample in samples]
    for sample, center in zip(samples, centers):
        sample["target_center_world_m"] = list(center)
    heights = [center[2] for center in centers]
    observed_duration, drift, stable = evaluate_hold_window(
        conservative_elapsed_seconds(samples, arguments.clock_domain),
        heights,
        arguments.duration_sec,
        arguments.max_height_drift_m,
    )
    record = {
        "capture_schema": "p7_hold_trace_v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "model": arguments.model,
        "command": command,
        "target_local_center_m": list(local_center),
        "runtime": {
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "ign_partition": os.environ.get("IGN_PARTITION", ""),
        },
        "clock_domain": arguments.clock_domain,
        "duration_sec": observed_duration,
        "requested_duration_sec": arguments.duration_sec,
        "sample_count": len(samples),
        "height_min_m": min(heights),
        "height_max_m": max(heights),
        "max_height_drift_m": drift,
        "height_drift_threshold_m": arguments.max_height_drift_m,
        "stable": stable,
        "samples": samples,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(arguments.output)
    print(json.dumps(record, sort_keys=True))
    if not stable:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
