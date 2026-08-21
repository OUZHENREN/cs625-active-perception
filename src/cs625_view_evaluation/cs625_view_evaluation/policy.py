"""Pure deterministic policies shared by ROS selectors and P5 batch analysis."""

from __future__ import annotations

import random

from cs625_view_evaluation.joint_score import JointScoreConfig, score_candidate


SUPPORTED_STRATEGIES = {
    "fixed_view",
    "random_reachable",
    "predefined_scan",
    "coverage_nbv",
    "reachability_only",
    "proposed_joint_score",
}


def select_candidate(
    candidates,
    strategy: str,
    seed: int,
    fixed_id: str,
    predefined_ids,
    cycle_index: int,
    score_config: JointScoreConfig | None = None,
):
    """Select only from the supplied hard-filtered set and return a reason."""
    if not candidates:
        return None, "NO_REACHABLE_CANDIDATE"
    ordered = sorted(candidates, key=lambda candidate: candidate.candidate_id)
    if strategy == "fixed_view":
        return next((item for item in ordered if item.candidate_id == fixed_id), ordered[0]), ""
    if strategy == "random_reachable":
        return random.Random(seed + cycle_index).choice(ordered), ""
    if strategy == "predefined_scan":
        configured = [identifier for identifier in predefined_ids if identifier]
        for offset in range(len(configured)):
            identifier = configured[(cycle_index + offset) % len(configured)]
            selected = next((item for item in ordered if item.candidate_id == identifier), None)
            if selected is not None:
                return selected, ""
        return None, "PREDEFINED_VIEW_UNREACHABLE"
    if strategy == "coverage_nbv":
        return max(ordered, key=lambda item: (item.coverage_proxy, item.candidate_id)), ""
    if strategy == "reachability_only":
        return max(
            ordered,
            key=lambda item: (item.joint_margin, -item.motion_cost, -item.planning_time_sec, item.candidate_id),
        ), ""
    if strategy != "proposed_joint_score":
        return None, "UNKNOWN_STRATEGY"
    if score_config is None:
        return None, "SCORE_CONFIG_REQUIRED"
    return max(
        ordered,
        key=lambda item: (score_candidate(item, score_config)["proposed_joint_score"], item.candidate_id),
    ), ""
