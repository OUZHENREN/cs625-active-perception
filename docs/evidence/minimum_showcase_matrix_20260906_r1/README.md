# Minimum showcase matrix 2026-09-06 r1

This directory freezes the completed minimum-showcase simulation matrix:

- Scenes: `occlusion_light`, `occlusion_medium`, `occlusion_severe`
- Seeds: 17, 18, 19, 20, 21
- Baselines: `fixed_view`, `random_reachable`, `coverage_nbv`
- Episodes: 45/45 complete

The accepted summary files are in `summary/`:

- `matrix_validation.md`: confirms the matrix is complete.
- `episode_metrics.csv`: per-episode reachability, planning time, execution timing, motion cost, observation rate and failure codes.
- `strategy_summary.csv`: per-baseline means over 15 episodes each.
- `failure_codes.csv`: failure-code counts per baseline.

Important interpretation: this run is a complete matrix, but all 45 episodes terminate with `MAX_FAILED_ATTEMPTS`; the mean successful-observation rate is 0.0 for all three baselines. It is therefore useful as auditable reachability / planning / execution-failure evidence, not as a successful-observation comparison.

Core strategy-level results:

| Strategy | Episodes | Mean reachability rate | Mean planning time total (s) | Mean wall time (s) | Mean successful observation rate |
| --- | ---: | ---: | ---: | ---: | ---: |
| `coverage_nbv` | 15 | 0.752778 | 0.195406 | 102.413204 | 0.0 |
| `fixed_view` | 15 | 0.758333 | 0.187946 | 101.152591 | 0.0 |
| `random_reachable` | 15 | 0.744444 | 0.175477 | 90.748936 | 0.0 |

