"""P4 simulation-only multi-view coordinator with explicit execution feedback."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cs625_ap_interfaces.msg import ActiveLocalizationState, ViewCandidateArray, ViewSelection
from cs625_view_evaluation.baseline_selector import SUPPORTED_STRATEGIES, select_candidate


class SimEpisodeCoordinator(Node):
    """Coordinates selection rounds; planning/execution remain in separate nodes."""

    def __init__(self) -> None:
        super().__init__("cs625_sim_episode_coordinator")
        for name, value in (
            ("reachable_candidates_topic", "/view_planner/reachable_candidates"),
            ("selected_view_topic", "/view_planner/selected_view"),
            ("execution_status_topic", "/motion/execution_status"),
            ("state_topic", "/active_localization/state"),
            ("strategy", "predefined_scan"), ("random_seed", 17),
            ("max_views", 3), ("fixed_candidate_id", ""),
            ("max_failed_attempts", 3),
            ("scene_id", ""), ("episode_log_directory", ""),
            ("selection_publish_replay_count", 5),
            ("selection_publish_replay_period_sec", 0.5),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("predefined_candidate_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self._strategy = str(self.get_parameter("strategy").value)
        if self._strategy not in SUPPORTED_STRATEGIES:
            raise ValueError(f"strategy must be one of {sorted(SUPPORTED_STRATEGIES)}")
        self._seed = int(self.get_parameter("random_seed").value)
        self._max_views = int(self.get_parameter("max_views").value)
        if not 1 <= self._max_views <= 3:
            raise ValueError("max_views must be between 1 and 3 for P4")
        self._max_failed_attempts = int(self.get_parameter("max_failed_attempts").value)
        if not 1 <= self._max_failed_attempts <= 10:
            raise ValueError("max_failed_attempts must be between 1 and 10")
        self._fixed = str(self.get_parameter("fixed_candidate_id").value)
        self._predefined = list(self.get_parameter("predefined_candidate_ids").value)
        self._scene = str(self.get_parameter("scene_id").value).strip()
        directory = str(self.get_parameter("episode_log_directory").value).strip()
        if not self._scene or not directory:
            raise ValueError("scene_id and episode_log_directory must be profile-configured")
        self._directory = Path(directory)
        self._selection_replay_count = int(
            self.get_parameter("selection_publish_replay_count").value
        )
        self._selection_replay_period = float(
            self.get_parameter("selection_publish_replay_period_sec").value
        )
        if not 1 <= self._selection_replay_count <= 10:
            raise ValueError("selection_publish_replay_count must be between 1 and 10")
        if not 0.1 <= self._selection_replay_period <= 2.0:
            raise ValueError("selection_publish_replay_period_sec must be between 0.1 and 2.0")
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._selection_pub = self.create_publisher(ViewSelection, str(self.get_parameter("selected_view_topic").value), retained)
        self._state_pub = self.create_publisher(ActiveLocalizationState, str(self.get_parameter("state_topic").value), retained)
        self.create_subscription(ViewCandidateArray, str(self.get_parameter("reachable_candidates_topic").value), self._on_candidates, retained)
        self.create_subscription(String, str(self.get_parameter("execution_status_topic").value), self._on_execution, 10)
        self._available = []
        self._header = None
        self._cycle = 0
        self._waiting = False
        self._records = []
        self._failed_attempts = 0
        self._selected_candidate_id = ""
        self._selection_replay_message = None
        self._selection_replay_remaining = 0
        self._selection_replay_timer = None

    def _on_candidates(self, message: ViewCandidateArray) -> None:
        if self._available or self._waiting or self._records:
            return
        self.get_logger().info(
            f"P4 received {len(message.candidates)}/{message.source_candidate_count} reachable candidates"
        )
        self._available = list(message.candidates)
        self._reachable_count = len(message.candidates)
        self._header = message.header
        self._source_count = message.source_candidate_count or len(message.candidates)
        self._select_next()

    def _select_next(self) -> None:
        candidate, reason = select_candidate(
            self._available, self._strategy, self._seed, self._fixed, self._predefined, self._cycle
        )
        if candidate is None:
            self._finish(reason)
            return
        selection = ViewSelection()
        selection.header = self._header
        selection.strategy = self._strategy
        selection.random_seed = self._seed
        selection.cycle_index = self._cycle
        selection.candidate = candidate
        self._publish_selection_with_replay(selection)
        self._selected_candidate_id = selection.candidate.candidate_id
        self._waiting = True
        self._publish_state(ActiveLocalizationState.PLAN_TO_VIEW, f"cycle {self._cycle}: {candidate.candidate_id}")

    def _on_execution(self, message: String) -> None:
        if not self._waiting:
            return
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            result.get("cycle_index") != self._cycle
            or result.get("candidate_id") != self._selected_candidate_id
        ):
            return
        self._stop_selection_replay()
        self._waiting = False
        self._records.append(result)
        if not result.get("success"):
            # A previously reachable view can fail to replan after an earlier
            # simulated motion. Preserve that failure in the episode, remove
            # the view, and continue with the common remaining set.
            self._available = [
                item for item in self._available
                if item.candidate_id != result.get("candidate_id")
            ]
            self._failed_attempts += 1
            if self._failed_attempts >= self._max_failed_attempts:
                self._finish("MAX_FAILED_ATTEMPTS")
                return
            if not self._available:
                self._finish(result.get("code", "NO_REACHABLE_AFTER_FAILURE"))
                return
            self._select_next()
            return
        self._available = [item for item in self._available if item.candidate_id != result.get("candidate_id")]
        self._selected_candidate_id = ""
        self._cycle += 1
        if self._cycle >= self._max_views:
            self._finish("MAX_VIEWS_REACHED")
            return
        self._select_next()

    def _finish(self, reason: str) -> None:
        self._directory.mkdir(parents=True, exist_ok=True)
        record = {
            "scene_id": self._scene,
            "strategy": self._strategy,
            "random_seed": self._seed,
            "source_candidate_count": self._source_count,
            "reachable_count": self._reachable_count,
            "view_records": self._records,
            "termination_reason": reason,
            "profile": "sim",
            "max_failed_attempts": self._max_failed_attempts,
        }
        destination = self._directory / f"{self._scene}_{self._strategy}_seed{self._seed}_p4_loop.json"
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(destination)
        self._publish_state(
            ActiveLocalizationState.SUCCESS if reason == "MAX_VIEWS_REACHED" else ActiveLocalizationState.FAILURE,
            reason,
        )
        self.get_logger().info(f"P4 episode finished: {reason}")

    def _publish_selection_with_replay(self, selection: ViewSelection) -> None:
        """Replay one identical selection while DDS endpoints settle.

        The executor accepts a selection only when idle, and this coordinator
        cancels the replay once it receives the matching execution status.
        Therefore replay cannot cause a second motion for the same cycle.
        """
        self._stop_selection_replay()
        self._selection_pub.publish(selection)
        self.get_logger().info(
            f"P4 selected {selection.candidate.candidate_id}; "
            f"executor subscribers={self._selection_pub.get_subscription_count()}"
        )
        self._selection_replay_message = selection
        self._selection_replay_remaining = self._selection_replay_count - 1
        if self._selection_replay_remaining:
            self._selection_replay_timer = self.create_timer(
                self._selection_replay_period, self._replay_selection
            )

    def _replay_selection(self) -> None:
        if self._selection_replay_message is None or self._selection_replay_remaining <= 0:
            self._stop_selection_replay()
            return
        self._selection_pub.publish(self._selection_replay_message)
        self._selection_replay_remaining -= 1

    def _stop_selection_replay(self) -> None:
        if self._selection_replay_timer is not None:
            self._selection_replay_timer.cancel()
            self.destroy_timer(self._selection_replay_timer)
            self._selection_replay_timer = None

    def _publish_state(self, state: int, detail: str) -> None:
        message = ActiveLocalizationState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = state
        message.detail = detail
        message.execute_enabled = True
        self._state_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimEpisodeCoordinator()
    try:
        rclpy.spin(node)
    # The launch system may request shutdown immediately after the episode
    # record is atomically written.  Treat that normal lifecycle event like a
    # keyboard interrupt so the process exits cleanly instead of printing a
    # misleading traceback for a successful episode.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_selection_replay()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
