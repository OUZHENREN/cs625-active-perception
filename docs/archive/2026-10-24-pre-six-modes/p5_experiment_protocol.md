# P5 paired experiment protocol

## Objective and hypothesis

Evaluate whether `proposed_joint_score` can select a view with higher joint
score than fixed, random, predefined, coverage-only and reachability-only
baselines while using the same hard-filtered candidate set.

```text
score = w_gain * localization_gain_proxy
      + w_reach * joint_margin
      - w_motion * normalized_motion_cost
      - w_time * normalized_planning_time
```

All weights and normalizers are profile parameters in
`config/p5_joint_score_sim.yaml`; no score weight is a source-code constant.
`localization_gain_proxy` is currently a candidate geometry proxy, not a
measured localization gain.

## Registered comparison

Paired factors are scene, candidate snapshot and random seed. Every strategy
uses the same hard-reachable candidate IDs: `fixed_view`, `random_reachable`,
`predefined_scan`, `coverage_nbv`, `reachability_only` and
`proposed_joint_score`.

Outputs are episode JSONL, complete candidate-score CSV, configuration
snapshot, paired effect sizes, deterministic 95% bootstrap intervals and PNG.
The repository fixture is deliberately marked `synthetic_fixture` /
`interface_only`: it validates tooling only and is not a research result.

```bash
ros2 run cs625_experiment_tools run_paired_experiment \
  --config <candidate-snapshot experiment config> \
  --output-dir <empty output directory>
```

Research-eligible outputs require recorded Gazebo candidate snapshots with a
pinned commit/config, paired multi-scene truth and predeclared stopping rules.
