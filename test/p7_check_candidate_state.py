#!/usr/bin/env python3
"""Check one versioned joint fixture with MoveIt without commanding motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class CandidateStateChecker(Node):
    def __init__(self) -> None:
        super().__init__("p7_check_candidate_state")
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def check(self, positions: dict[str, float]) -> dict[str, object]:
        if not self.client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/check_state_validity unavailable")
        request = GetStateValidity.Request()
        request.group_name = "cs625_arm"
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = [positions[name] for name in JOINT_NAMES]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("/check_state_validity timed out")
        response = future.result()
        contacts = sorted(
            {
                "<->".join(sorted((entry.contact_body_1, entry.contact_body_2)))
                for entry in response.contacts
            }
        )
        return {
            "schema": "p7_candidate_state_validity_v1",
            "motion_commanded": False,
            "group_name": request.group_name,
            "joint_positions_rad": positions,
            "valid": bool(response.valid),
            "contacts": contacts,
            "claim_boundary": "read-only MoveIt state check; not P7.4 planning or execution evidence",
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joints", required=True, type=Path)
    arguments = parser.parse_args()
    source = yaml.safe_load(arguments.joints.read_text(encoding="utf-8"))
    positions = {name: float(source[name]) for name in JOINT_NAMES}
    rclpy.init()
    node = CandidateStateChecker()
    try:
        print(json.dumps(node.check(positions), indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
