from types import SimpleNamespace

from cs625_view_evaluation.joint_score import JointScoreConfig, score_candidate
from cs625_view_evaluation.policy import select_candidate


def _candidate(candidate_id, coverage, margin, cost, planning_time):
    return SimpleNamespace(
        candidate_id=candidate_id,
        coverage_proxy=coverage,
        joint_margin=margin,
        motion_cost=cost,
        planning_time_sec=planning_time,
    )


def test_joint_score_prefers_reachable_high_gain_candidate():
    config = JointScoreConfig(
        localization_gain_weight=1.0,
        reachability_quality_weight=1.0,
        motion_cost_weight=0.25,
        planning_time_weight=0.25,
        motion_cost_scale=4.0,
        planning_time_scale=4.0,
    )
    candidate_a = _candidate("a", 0.85, 0.80, 1.0, 1.0)
    candidate_b = _candidate("b", 0.60, 0.15, 3.0, 3.0)

    terms = score_candidate(candidate_a, config)
    selected, reason = select_candidate(
        [candidate_b, candidate_a],
        "proposed_joint_score",
        seed=17,
        fixed_id="",
        predefined_ids=[],
        cycle_index=0,
        score_config=config,
    )

    assert terms["proposed_joint_score"] > 0.0
    assert reason == ""
    assert selected.candidate_id == "a"
