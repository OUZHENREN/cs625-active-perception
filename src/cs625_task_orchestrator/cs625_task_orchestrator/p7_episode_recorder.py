"""Atomically write a P7 evidence record received from a simulation adapter."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cs625_task_orchestrator.p7_episode_contract import evaluate_episode


class P7EpisodeRecorder(Node):
    """Validate JSON evidence and emit a self-contained accepted/rejected record."""

    def __init__(self) -> None:
        super().__init__("cs625_p7_episode_recorder")
        self.declare_parameter("input_topic", "/p7/evidence")
        self.declare_parameter("output_topic", "/p7/episode_result")
        self.declare_parameter("episode_log_directory", "")
        directory = str(self.get_parameter("episode_log_directory").value).strip()
        if not directory:
            raise ValueError("episode_log_directory must be explicitly configured")
        self._directory = Path(directory)
        self._publisher = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            String, str(self.get_parameter("input_topic").value), self._on_evidence, 10
        )

    def _on_evidence(self, message: String) -> None:
        try:
            evidence = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().error("P7 evidence is not valid JSON")
            return
        if not isinstance(evidence, dict):
            self.get_logger().error("P7 evidence must be a JSON object")
            return
        scene_id = str(evidence.get("scene_id", "")).strip()
        strategy = str(evidence.get("strategy", "")).strip()
        seed = evidence.get("random_seed")
        if (
            not scene_id
            or not strategy
            or isinstance(seed, bool)
            or not isinstance(seed, int)
        ):
            self.get_logger().error("P7 evidence needs scene_id, strategy and integer random_seed")
            return
        result = {**evidence, **evaluate_episode(evidence)}
        self._directory.mkdir(parents=True, exist_ok=True)
        destination = self._directory / f"{scene_id}_{strategy}_seed{seed}_p7.json"
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(destination)
        outgoing = String()
        outgoing.data = json.dumps(result, sort_keys=True)
        self._publisher.publish(outgoing)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P7EpisodeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
