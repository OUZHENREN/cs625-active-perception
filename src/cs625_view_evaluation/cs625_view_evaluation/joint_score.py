"""Pure P5 joint-score terms for hard-filtered view candidates.

The localization term is a configured proxy until a validated target-localizer
publishes measured information gain.  This module deliberately makes that
limitation explicit in every exported term name.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


def _unit_interval(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass(frozen=True)
class JointScoreConfig:
    """All P5 score weights and normalizers are profile-provided values."""

    localization_gain_weight: float
    reachability_quality_weight: float
    motion_cost_weight: float
    planning_time_weight: float
    motion_cost_scale: float
    planning_time_scale: float

    def validate(self) -> None:
        values = (
            self.localization_gain_weight,
            self.reachability_quality_weight,
            self.motion_cost_weight,
            self.planning_time_weight,
        )
        if any(value < 0.0 for value in values) or not any(values):
            raise ValueError("P5 score weights must be non-negative and not all zero")
        if self.motion_cost_scale <= 0.0 or self.planning_time_scale <= 0.0:
            raise ValueError("P5 score normalizers must be positive")


def score_candidate(candidate, config: JointScoreConfig) -> Mapping[str, float]:
    """Return auditable P5 terms for one already hard-reachable candidate."""
    config.validate()
    localization_gain_proxy = _unit_interval(candidate.coverage_proxy)
    joint_margin = _unit_interval(candidate.joint_margin)
    motion_cost_penalty = _unit_interval(candidate.motion_cost / config.motion_cost_scale)
    planning_time_penalty = _unit_interval(
        candidate.planning_time_sec / config.planning_time_scale
    )
    total = (
        config.localization_gain_weight * localization_gain_proxy
        + config.reachability_quality_weight * joint_margin
        - config.motion_cost_weight * motion_cost_penalty
        - config.planning_time_weight * planning_time_penalty
    )
    return {
        "localization_gain_proxy": localization_gain_proxy,
        "joint_margin": joint_margin,
        "motion_cost_penalty": motion_cost_penalty,
        "planning_time_penalty": planning_time_penalty,
        "proposed_joint_score": total,
    }
