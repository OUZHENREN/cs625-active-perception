import json
from pathlib import Path

from cs625_experiment_tools.p4_matrix_summary import run


def test_p4_matrix_summary_requires_and_exports_complete_matrix(tmp_path):
    root = Path(__file__).resolve().parents[3]
    manifest = json.loads(
        (root / "src" / "cs625_bringup" / "config" / "minimum_showcase_matrix.json").read_text(
            encoding="utf-8"
        )
    )
    episodes = tmp_path / "episodes"
    episodes.mkdir()
    for scene_id in manifest["scene_ids"]:
        for seed in manifest["seeds"]:
            for strategy in manifest["strategies"]:
                payload = {
                    "scene_id": scene_id,
                    "strategy": strategy,
                    "random_seed": seed,
                    "source_candidate_count": 24,
                    "reachable_count": 16,
                    "episode_wall_time_sec": 8.5,
                    "replan_after_execution_failure_count": 1,
                    "candidate_collision_rejection_count": 2,
                    "candidate_collision_rejection_rate": 2.0 / 24.0,
                    "candidate_collision_metric_scope": "p3_moveit_state_validity",
                    "termination_reason": "MAX_VIEWS_REACHED",
                    "view_records": [
                        {
                            "success": True,
                            "code": "SENSOR_SETTLED",
                            "motion_cost": 0.25,
                            "planning_time_sec": 0.10,
                            "execution_ik_time_sec": 0.02,
                            "execution_motion_planning_time_sec": 0.03,
                            "trajectory_execution_time_sec": 1.5,
                            "sensor_settle_wait_time_sec": 0.10,
                        },
                        {
                            "success": False,
                            "code": "PLANNING_FAILED",
                            "motion_cost": 0.0,
                            "planning_time_sec": 0.20,
                            "execution_ik_time_sec": 0.02,
                            "execution_motion_planning_time_sec": 0.03,
                            "trajectory_execution_time_sec": 0.0,
                            "sensor_settle_wait_time_sec": 0.0,
                        },
                    ],
                }
                filename = f"{scene_id}_{strategy}_seed{seed}.json"
                (episodes / filename).write_text(json.dumps(payload), encoding="utf-8")

    output = tmp_path / "summary"
    result = run(
        episodes,
        root / "src" / "cs625_bringup" / "config" / "minimum_showcase_matrix.json",
        output,
    )

    assert result["episode_count"] == 45
    assert "mean_successful_observation_rate" in (output / "strategy_summary.csv").read_text(
        encoding="utf-8"
    )
    assert "mean_episode_wall_time_sec" in (output / "strategy_summary.csv").read_text(
        encoding="utf-8"
    )
    assert "candidate_collision_rejection_rate" in (output / "episode_metrics.csv").read_text(
        encoding="utf-8"
    )
    assert "PLANNING_FAILED" in (output / "failure_codes.csv").read_text(encoding="utf-8")
    assert "Matrix status: COMPLETE" in (output / "matrix_validation.md").read_text(
        encoding="utf-8"
    )
