"""Validate and summarize a recorded 3-scene × seed × baseline P4 matrix."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable


def _episode_metrics(record: dict) -> dict:
    views = list(record.get("view_records", []))
    successes = [view for view in views if view.get("success")]
    failures = [view for view in views if not view.get("success")]
    attempts = len(views)
    return {
        "scene_id": record["scene_id"],
        "random_seed": int(record["random_seed"]),
        "strategy": record["strategy"],
        "source_candidate_count": int(record["source_candidate_count"]),
        "reachable_count": int(record["reachable_count"]),
        "reachability_rate": int(record["reachable_count"]) / max(1, int(record["source_candidate_count"])),
        "attempt_count": attempts,
        "successful_observation_count": len(successes),
        "successful_observation_rate": len(successes) / max(1, attempts),
        "planning_time_sec_total": sum(float(view.get("planning_time_sec", 0.0)) for view in views),
        "motion_cost_total": sum(float(view.get("motion_cost", 0.0)) for view in successes),
        "failure_codes": "|".join(sorted(str(view.get("code", "UNKNOWN")) for view in failures)),
        "termination_reason": record.get("termination_reason", "UNKNOWN"),
    }


def _mean(rows: list[dict], key: str) -> float:
    return sum(float(row[key]) for row in rows) / len(rows)


def run(input_directory: Path, manifest_path: Path, output_directory: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    required_scenes = set(manifest["scene_ids"])
    required_seeds = {int(seed) for seed in manifest["seeds"]}
    required_strategies = set(manifest["strategies"])
    episodes = []
    for path in sorted(input_directory.glob("*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        if {"scene_id", "strategy", "random_seed", "view_records"} <= set(payload):
            episodes.append(_episode_metrics(payload))
    observed = {(row["scene_id"], row["random_seed"], row["strategy"]) for row in episodes}
    expected = {(scene, seed, strategy) for scene in required_scenes for seed in required_seeds for strategy in required_strategies}
    missing = sorted(expected - observed)
    unexpected = sorted(observed - expected)
    duplicates = sorted(key for key, count in Counter(
        (row["scene_id"], row["random_seed"], row["strategy"]) for row in episodes
    ).items() if count > 1)
    if missing or unexpected or duplicates:
        raise ValueError(
            f"matrix invalid: missing={missing}, unexpected={unexpected}, duplicates={duplicates}"
        )
    output_directory.mkdir(parents=True, exist_ok=True)
    episode_fields = list(episodes[0])
    with (output_directory / "episode_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=episode_fields)
        writer.writeheader()
        writer.writerows(sorted(episodes, key=lambda row: (row["scene_id"], row["random_seed"], row["strategy"])))
    by_strategy: dict[str, list[dict]] = defaultdict(list)
    failures = Counter()
    for row in episodes:
        by_strategy[row["strategy"]].append(row)
        for code in filter(None, row["failure_codes"].split("|")):
            failures[(row["strategy"], code)] += 1
    summaries = [
        {
            "strategy": strategy,
            "episode_count": len(rows),
            "mean_reachability_rate": _mean(rows, "reachability_rate"),
            "mean_planning_time_sec_total": _mean(rows, "planning_time_sec_total"),
            "mean_motion_cost_total": _mean(rows, "motion_cost_total"),
            "mean_successful_observation_rate": _mean(rows, "successful_observation_rate"),
        }
        for strategy, rows in sorted(by_strategy.items())
    ]
    with (output_directory / "strategy_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    with (output_directory / "failure_codes.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["strategy", "failure_code", "count"])
        writer.writeheader()
        writer.writerows(
            {"strategy": strategy, "failure_code": code, "count": count}
            for (strategy, code), count in sorted(failures.items())
        )
    (output_directory / "matrix_validation.md").write_text(
        "# Minimum showcase matrix validation\n\n"
        f"- Expected episodes: {len(expected)}\n"
        f"- Observed episodes: {len(episodes)}\n"
        "- Matrix status: COMPLETE\n",
        encoding="utf-8",
    )
    return {"episode_count": len(episodes), "output_directory": str(output_directory)}


def main(args: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-directory", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parsed = parser.parse_args(args)
    print(json.dumps(run(parsed.input_directory, parsed.manifest, parsed.output_directory), sort_keys=True))


if __name__ == "__main__":
    main()
