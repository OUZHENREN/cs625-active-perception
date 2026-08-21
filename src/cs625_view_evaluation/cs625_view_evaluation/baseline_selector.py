"""Select a P4 baseline from the same hard-filtered candidate set."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cs625_ap_interfaces.msg import ActiveLocalizationState, ViewCandidateArray, ViewSelection
from cs625_view_evaluation.joint_score import JointScoreConfig, score_candidate
from cs625_view_evaluation.policy import SUPPORTED_STRATEGIES, select_candidate

class BaselineSelector(Node):
    """Publish one selected view and a self-contained episode JSON record."""

    def __init__(self) -> None:
        super().__init__("cs625_baseline_selector")
        self.declare_parameter("reachable_candidates_topic", "/view_planner/reachable_candidates")
        self.declare_parameter("selected_view_topic", "/view_planner/selected_view")
        self.declare_parameter("state_topic", "/active_localization/state")
        self.declare_parameter("strategy", "fixed_view")
        self.declare_parameter("random_seed", 17)
        self.declare_parameter("cycle_index", 0)
        self.declare_parameter("fixed_candidate_id", "")
        self.declare_parameter(
            "predefined_candidate_ids", rclpy.Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter("scene_id", "")
        self.declare_parameter("episode_log_directory", "")
        self.declare_parameter("git_commit", "UNRECORDED")
        self.declare_parameter("data_source", "gazebo_runtime")
        self.declare_parameter("validity_label", "scoring_proxy_only")
        self.declare_parameter("localization_gain_weight", 0.0)
        self.declare_parameter("reachability_quality_weight", 0.0)
        self.declare_parameter("motion_cost_weight", 0.0)
        self.declare_parameter("planning_time_weight", 0.0)
        self.declare_parameter("motion_cost_scale", 1.0)
        self.declare_parameter("planning_time_scale", 1.0)

        self._strategy = str(self.get_parameter("strategy").value)
        if self._strategy not in SUPPORTED_STRATEGIES:
            raise ValueError(f"strategy must be one of {sorted(SUPPORTED_STRATEGIES)}")
        self._seed = int(self.get_parameter("random_seed").value)
        self._cycle = int(self.get_parameter("cycle_index").value)
        self._fixed_id = str(self.get_parameter("fixed_candidate_id").value)
        self._predefined_ids = list(self.get_parameter("predefined_candidate_ids").value)
        self._scene_id = str(self.get_parameter("scene_id").value).strip()
        log_directory = str(self.get_parameter("episode_log_directory").value).strip()
        self._log_directory = Path(log_directory)
        if not self._scene_id or not log_directory:
            raise ValueError("scene_id and episode_log_directory must be supplied by the profile")
        self._git_commit = str(self.get_parameter("git_commit").value)
        self._data_source = str(self.get_parameter("data_source").value)
        self._validity_label = str(self.get_parameter("validity_label").value)
        self._score_config = JointScoreConfig(
            localization_gain_weight=float(self.get_parameter("localization_gain_weight").value),
            reachability_quality_weight=float(self.get_parameter("reachability_quality_weight").value),
            motion_cost_weight=float(self.get_parameter("motion_cost_weight").value),
            planning_time_weight=float(self.get_parameter("planning_time_weight").value),
            motion_cost_scale=float(self.get_parameter("motion_cost_scale").value),
            planning_time_scale=float(self.get_parameter("planning_time_scale").value),
        )
        if self._strategy == "proposed_joint_score":
            self._score_config.validate()
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._selection_pub = self.create_publisher(
            ViewSelection, str(self.get_parameter("selected_view_topic").value), retained
        )
        self._state_pub = self.create_publisher(
            ActiveLocalizationState, str(self.get_parameter("state_topic").value), retained
        )
        self._subscription = self.create_subscription(
            ViewCandidateArray,
            str(self.get_parameter("reachable_candidates_topic").value),
            self._on_candidates,
            retained,
        )
        self._handled = False

    def _on_candidates(self, message: ViewCandidateArray) -> None:
        if self._handled:
            return
        self._handled = True
        selected, termination = select_candidate(
            message.candidates,
            self._strategy,
            self._seed,
            self._fixed_id,
            self._predefined_ids,
            self._cycle,
            self._score_config if self._strategy == "proposed_joint_score" else None,
        )
        selection = ViewSelection()
        selection.header = message.header
        selection.strategy = self._strategy
        selection.random_seed = self._seed
        selection.cycle_index = self._cycle
        selection.termination_reason = termination
        if selected is not None:
            selection.candidate = selected
        self._selection_pub.publish(selection)
        self._write_episode(message, selection)
        detail = termination or f"selected {selected.candidate_id} with {self._strategy}"
        self._publish_state(
            ActiveLocalizationState.EVALUATE_VIEWS if selected else ActiveLocalizationState.FAILURE,
            detail,
        )
        self.get_logger().info(detail)

    def _write_episode(self, candidates: ViewCandidateArray, selection: ViewSelection) -> None:
        self._log_directory.mkdir(parents=True, exist_ok=True)
        reachable_ids = [candidate.candidate_id for candidate in candidates.candidates]
        record = {
            "episode_id": f"{self._scene_id}_{self._strategy}_seed{self._seed}_cycle{self._cycle}",
            "scene_id": self._scene_id,
            "strategy": self._strategy,
            "random_seed": self._seed,
            "cycle_index": self._cycle,
            "git_commit": self._git_commit,
            "data_source": self._data_source,
            "validity_label": self._validity_label,
            "candidate_count": candidates.source_candidate_count or len(reachable_ids),
            "reachable_count": len(reachable_ids),
            "reachable_candidate_ids": reachable_ids,
            "selected_view": selection.candidate.candidate_id if selection.candidate.candidate_id else None,
            "coverage_proxy": selection.candidate.coverage_proxy if selection.candidate.candidate_id else None,
            "candidate_score_terms": [
                {"candidate_id": candidate.candidate_id, **score_candidate(candidate, self._score_config)}
                for candidate in sorted(candidates.candidates, key=lambda item: item.candidate_id)
            ] if self._strategy == "proposed_joint_score" else [],
            "candidate_snapshot": [
                {
                    "candidate_id": candidate.candidate_id,
                    "coverage_proxy": candidate.coverage_proxy,
                    "joint_margin": candidate.joint_margin,
                    "motion_cost": candidate.motion_cost,
                    "planning_time_sec": candidate.planning_time_sec,
                }
                for candidate in sorted(candidates.candidates, key=lambda item: item.candidate_id)
            ],
            "termination_reason": selection.termination_reason or None,
            "execution": "planning_only",
        }
        serialized = json.dumps(record, sort_keys=True, indent=2) + "\n"
        record["config_hash"] = hashlib.sha256(serialized.encode("utf-8")).hexdigest()
        output = self._log_directory / f"{record['episode_id']}.json"
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        temporary.replace(output)

    def _publish_state(self, state: int, detail: str) -> None:
        message = ActiveLocalizationState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = state
        message.detail = detail
        message.execute_enabled = False
        self._state_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaselineSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
