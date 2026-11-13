#!/usr/bin/env python3
"""Simulation-only 0.05 rad probe of the P7 trajectory command chain.

This intentionally bypasses MoveIt only to isolate ros2_control/Gazebo
tracking.  It keeps five joints at their measured positions and changes
``wrist_2_joint`` by +0.05 rad, then prints controller feedback and the final
measured error.  It is not a grasp or task-success test.
"""

from __future__ import annotations

import argparse
import os
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=JOINT_NAMES, default="wrist_2_joint")
    parser.add_argument("--delta", type=float, default=0.05)
    arguments = parser.parse_args()
    if not 0.0 < abs(arguments.delta) <= 0.05:
        parser.error("--delta must be non-zero and no larger than 0.05 rad")
    return arguments


def main() -> None:
    arguments = parse_arguments()
    probe_joint = arguments.joint
    probe_delta_rad = arguments.delta
    if os.environ.get("CS625_P7_SIMULATION_EXECUTION") != "1":
        raise RuntimeError("set CS625_P7_SIMULATION_EXECUTION=1 only in the isolated P7 simulation")
    rclpy.init()
    node = rclpy.create_node("p7_joint_controller_probe")
    measured: dict[str, float] = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda message: measured.update(zip(message.name, message.position)), 10,
    )
    deadline = time.monotonic() + 4.0
    while any(name not in measured for name in JOINT_NAMES) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if any(name not in measured for name in JOINT_NAMES):
        raise RuntimeError("six-axis joint state unavailable")

    initial = {name: measured[name] for name in JOINT_NAMES}
    target = dict(initial)
    target[probe_joint] += probe_delta_rad
    client = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("trajectory action unavailable")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [target[name] for name in JOINT_NAMES]
    point.time_from_start.sec = 2
    goal.trajectory.points = [point]
    feedback_samples: list[tuple[float, float, float]] = []

    def feedback(message) -> None:
        feedback_message = message.feedback
        index = feedback_message.joint_names.index(probe_joint)
        stamp = (
            feedback_message.desired.time_from_start.sec
            + feedback_message.desired.time_from_start.nanosec / 1_000_000_000.0
        )
        sample = (
            stamp,
            feedback_message.desired.positions[index],
            feedback_message.actual.positions[index],
        )
        if not feedback_samples or stamp - feedback_samples[-1][0] >= 0.4:
            feedback_samples.append(sample)

    sent = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=6.0)
    handle = sent.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("trajectory probe rejected")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=90.0)
    result = result_future.result()
    if result is None:
        raise RuntimeError("trajectory probe result timeout")
    rclpy.spin_once(node, timeout_sec=0.2)
    for stamp, desired, actual in feedback_samples:
        print(f"FEEDBACK t={stamp:.3f} desired={desired:+.7f} actual={actual:+.7f} error={desired - actual:+.7f}")
    final = measured[probe_joint]
    print(
        f"PROBE_RESULT joint={probe_joint} action_code={result.result.error_code} "
        f"initial={initial[probe_joint]:+.7f} target={target[probe_joint]:+.7f} "
        f"final={final:+.7f} final_error={target[probe_joint] - final:+.7f}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
