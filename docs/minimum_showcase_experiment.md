# Minimum showcase simulation experiment

## Registered matrix

This is a simulation-only P4 loop evaluation.  Every cell starts a fresh
Gazebo + MoveIt process, so the preceding cell's robot state cannot affect the
next one.

| Factor | Registered values |
| --- | --- |
| Occlusion scene | `occlusion_light`, `occlusion_medium`, `occlusion_severe` |
| Random seed | 17, 18, 19, 20, 21 |
| Baseline | `fixed_view`, `random_reachable`, `coverage_nbv` |
| Total | 45 recorded episodes |

`occlusion_medium` preserves the pre-existing `minimal_occlusion` geometry.
The light and severe worlds respectively reduce and increase the central
occluder's extent while retaining the same target pose.

## Recorded metrics

The collector only accepts complete, unique scene × seed × strategy cells and
exports:

- `episode_metrics.csv`: source/reachable candidate counts and reachability
  rate, total planning time, successful-observation count/rate, successful
  motion cost, episode termination, and per-episode failure codes.
- `strategy_summary.csv`: the mean reachability rate, planning time, motion
  cost and successful-observation rate over the 15 cells per baseline.
- `failure_codes.csv`: failure-code counts per baseline.
- `matrix_validation.md`: confirms all 45 cells were present exactly once.

Planning time is the sum of recorded planning-service durations.  Motion cost
is summed only for successful observed views; it is a candidate trajectory
cost, not physical energy.  `SENSOR_SETTLED` is the P4 success observation
code; all other returned codes are retained in `failure_codes.csv`.

Each episode ends after three successful observations (`MAX_VIEWS_REACHED`) or
three failed execution attempts (`MAX_FAILED_ATTEMPTS`).  This bounded retry
rule is identical for every registered cell and is recorded in each episode
JSON.  Before publishing the synthetic target, the runner checks that the
simulated trajectory controller is `active`; it also isolates and reaps each
cell's Gazebo/ROS process group so no action server can leak into the next
cell.

## Run

After building into an isolated install, choose a **new** empty output path:

```bash
scripts/run_minimum_showcase_matrix.sh <install>/setup.bash <new-output-directory>
```

For a non-evidence smoke test of one registered cell, append
`--scene occlusion_medium --seed 17 --strategy fixed_view`.  Such a filtered
run deliberately does not produce the complete-matrix summary.

The script uses only `sim_view_planning.launch.py` and
`sim_p4_loop.launch.py` with a simulation profile.  It does not invoke any
real-hardware launch or controller activation.  If a cell times out or its
P4 process exits, it stops and preserves that cell's logs; do not treat a
partial directory as research evidence.
