"""Run deterministic paired P5 policy comparisons from recorded candidate sets.

The input files are candidate snapshots, not a simulator.  Their data source
and validity label are carried unchanged into every output to prevent an
interface fixture from being mistaken for a research result.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from random import Random
from types import SimpleNamespace
from typing import Iterable
import struct
import zlib

from cs625_view_evaluation.joint_score import JointScoreConfig, score_candidate
from cs625_view_evaluation.policy import SUPPORTED_STRATEGIES, select_candidate


def _candidate(record: dict) -> SimpleNamespace:
    required = (
        "candidate_id", "coverage_proxy", "joint_margin", "motion_cost", "planning_time_sec"
    )
    missing = [name for name in required if name not in record]
    if missing:
        raise ValueError(f"candidate snapshot is missing {missing}")
    return SimpleNamespace(**{name: record[name] for name in required})


def _load_snapshot(path: Path) -> tuple[dict, list[SimpleNamespace]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    metadata = payload.get("metadata", {})
    if not metadata:
        metadata = {
            "data_source": payload.get("data_source", "unknown"),
            "validity_label": payload.get("validity_label", "interface_only"),
        }
    candidates = [_candidate(item) for item in payload.get("candidates", payload.get("candidate_snapshot", []))]
    if not candidates:
        raise ValueError(f"candidate snapshot {path} is empty")
    return metadata, candidates


def _bootstrap_ci(values: list[float], seed: int, samples: int) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    rng = Random(seed)
    means = sorted(
        sum(rng.choice(values) for _ in values) / len(values) for _ in range(samples)
    )
    return means[int(0.025 * (samples - 1))], means[int(0.975 * (samples - 1))]


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def _write_score_plot(path: Path, rows: list[dict]) -> None:
    """Write a small dependency-free PNG; bars are descriptive, not inferential."""
    width, height = 480, 240
    image = bytearray([255, 255, 255] * width * height)
    totals = defaultdict(list)
    for row in rows:
        totals[row["strategy"]].append(float(row["selected_score"]))
    names = sorted(totals)
    means = [sum(totals[name]) / len(totals[name]) for name in names]
    minimum, maximum = min(0.0, *means), max(0.0, *means)
    span = maximum - minimum or 1.0
    baseline = int((height - 35) - ((0.0 - minimum) / span) * (height - 70))
    for x in range(width):
        for y in range(max(0, baseline - 1), min(height, baseline + 1)):
            index = (y * width + x) * 3
            image[index:index + 3] = b"\x80\x80\x80"
    palette = ((42, 157, 143), (38, 70, 83), (233, 196, 106), (231, 111, 81), (69, 123, 157), (138, 201, 38))
    bar_width = max(12, (width - 40) // max(1, len(names) * 2))
    for index, mean in enumerate(means):
        center = 30 + (index * 2 + 1) * bar_width
        value_y = int((height - 35) - ((mean - minimum) / span) * (height - 70))
        low, high = sorted((baseline, value_y))
        color = palette[index % len(palette)]
        for x in range(center - bar_width // 2, center + bar_width // 2):
            for y in range(max(0, low), min(height - 35, high + 1)):
                pixel = (y * width + x) * 3
                image[pixel:pixel + 3] = bytes(color)
    scanlines = b"".join(b"\x00" + bytes(image[y * width * 3:(y + 1) * width * 3]) for y in range(height))
    png = b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) + _png_chunk(b"IDAT", zlib.compress(scanlines)) + _png_chunk(b"IEND", b"")
    path.write_bytes(png)


def run(config_path: Path, output_dir: Path) -> dict:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    experiment = config["experiment"]
    strategies = list(experiment["strategies"])
    if not {"fixed_view", "random_reachable", "proposed_joint_score"}.issubset(strategies):
        raise ValueError("P5 paired comparison must include fixed_view, random_reachable and proposed_joint_score")
    if any(strategy not in SUPPORTED_STRATEGIES for strategy in strategies):
        raise ValueError("unknown policy in paired experiment configuration")
    score_config = JointScoreConfig(**experiment["score_config"])
    score_config.validate()
    output_dir.mkdir(parents=True, exist_ok=True)
    snapshots = []
    for item in experiment["candidate_sets"]:
        path = (config_path.parent / item["file"]).resolve()
        metadata, candidates = _load_snapshot(path)
        snapshots.append((item["scene_id"], metadata, candidates, path.name))

    episodes, candidate_rows = [], []
    for scene_id, metadata, candidates, source_file in snapshots:
        for seed in experiment["seeds"]:
            for strategy in strategies:
                selected, reason = select_candidate(
                    candidates,
                    strategy,
                    int(seed),
                    experiment["fixed_candidate_id"],
                    experiment["predefined_candidate_ids"],
                    0,
                    score_config if strategy == "proposed_joint_score" else None,
                )
                if selected is None:
                    raise RuntimeError(f"{scene_id}/{strategy}: {reason}")
                terms = score_candidate(selected, score_config)
                episodes.append({
                    "scene_id": scene_id,
                    "random_seed": int(seed),
                    "strategy": strategy,
                    "selected_candidate_id": selected.candidate_id,
                    "selected_score": terms["proposed_joint_score"],
                    "data_source": metadata.get("data_source", experiment["data_source"]),
                    "validity_label": metadata.get("validity_label", experiment["validity_label"]),
                    "candidate_snapshot": source_file,
                    "termination_reason": "PLANNING_ONLY_SELECTION",
                })
                for candidate in candidates:
                    candidate_rows.append({
                        "scene_id": scene_id,
                        "random_seed": int(seed),
                        "strategy": strategy,
                        "candidate_id": candidate.candidate_id,
                        "selected": candidate.candidate_id == selected.candidate_id,
                        **score_candidate(candidate, score_config),
                    })

    with (output_dir / "episodes.jsonl").open("w", encoding="utf-8") as handle:
        for row in episodes:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    with (output_dir / "candidate_scores.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(candidate_rows[0]))
        writer.writeheader()
        writer.writerows(candidate_rows)
    by_key = {(row["scene_id"], row["random_seed"], row["strategy"]): row for row in episodes}
    paired_rows = []
    for strategy in strategies:
        values = [
            by_key[(scene_id, seed, "proposed_joint_score")]["selected_score"] - by_key[(scene_id, seed, strategy)]["selected_score"]
            for scene_id, _, _, _ in snapshots for seed in experiment["seeds"]
        ]
        low, high = _bootstrap_ci(values, int(experiment["bootstrap_seed"]), int(experiment["bootstrap_samples"]))
        paired_rows.append({
            "reference_strategy": strategy,
            "paired_count": len(values),
            "mean_proposed_score_difference": sum(values) / len(values),
            "bootstrap_ci_95_low": low,
            "bootstrap_ci_95_high": high,
        })
    with (output_dir / "paired_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(paired_rows[0]))
        writer.writeheader()
        writer.writerows(paired_rows)
    validity = experiment["validity_label"]
    statistics = [
        "# P5 paired-policy summary",
        "",
        f"- Data source: `{experiment['data_source']}`",
        f"- Validity label: `{validity}`",
        "- Research use allowed: `false`" if validity != "research_candidate" else "- Research use allowed: `true`",
        "- Effect: proposed joint-score selection minus each paired reference in the same scene/seed.",
        "- Interval: deterministic non-parametric bootstrap of the mean; this is not a p-value.",
        "",
        "| reference | n | mean difference | 95% bootstrap CI |",
        "|---|---:|---:|---|",
    ]
    statistics.extend(
        f"| {row['reference_strategy']} | {row['paired_count']} | {row['mean_proposed_score_difference']:.6f} | [{row['bootstrap_ci_95_low']:.6f}, {row['bootstrap_ci_95_high']:.6f}] |"
        for row in paired_rows
    )
    (output_dir / "statistics.md").write_text("\n".join(statistics) + "\n", encoding="utf-8")
    (output_dir / "config_snapshot.json").write_text(json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_score_plot(output_dir / "summary_plot.png", episodes)
    return {"episode_count": len(episodes), "output_dir": str(output_dir), "validity_label": validity}


def main(args: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parsed = parser.parse_args(args)
    print(json.dumps(run(parsed.config, parsed.output_dir), sort_keys=True))


if __name__ == "__main__":
    main()
