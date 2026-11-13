#!/usr/bin/env python3
"""Persist a single replayed P3 reachable-candidate message as JSON.

The helper is diagnostic-only: it does not publish commands or move the arm.
"""

from __future__ import annotations

import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cs625_ap_interfaces.msg import ViewCandidateArray


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("p7_capture_reachable_candidates")
        self.create_subscription(
            ViewCandidateArray,
            "/view_planner/reachable_candidates",
            self._on_message,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def _on_message(self, message: ViewCandidateArray) -> None:
        rows = []
        for candidate in message.candidates:
            pose = candidate.tool_pose.pose
            rows.append({
                "candidate_id": candidate.candidate_id,
                "motion_cost": candidate.motion_cost,
                "planning_time_sec": candidate.planning_time_sec,
                "tool_pose": {
                    "frame_id": candidate.tool_pose.header.frame_id,
                    "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                    "orientation": {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w},
                },
            })
        print(json.dumps({"source_candidate_count": message.source_candidate_count, "reachable": rows}, sort_keys=True), flush=True)
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = Capture()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
