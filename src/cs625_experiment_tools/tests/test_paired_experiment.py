from pathlib import Path

from cs625_experiment_tools.paired_experiment import run


def test_paired_experiment_writes_reproducible_artifacts(tmp_path):
    root = Path(__file__).resolve().parents[3]
    summary = run(root / "test" / "data" / "p5_paired_experiment.json", tmp_path)

    assert summary["episode_count"] == 18
    for filename in (
        "episodes.jsonl",
        "candidate_scores.csv",
        "paired_summary.csv",
        "statistics.md",
        "config_snapshot.json",
        "summary_plot.png",
    ):
        assert (tmp_path / filename).exists()
