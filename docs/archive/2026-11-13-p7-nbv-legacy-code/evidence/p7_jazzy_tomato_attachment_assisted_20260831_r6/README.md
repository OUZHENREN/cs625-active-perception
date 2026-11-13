# P7 r6 reproducibility-failure evidence

This directory preserves the independent 2026-08-31 r6 run.  It used a new
ROS domain (`81`), Gazebo partition (`p7_acceptance_20260831_r6`), run ID and
output directory.  The abort-on-failure runner exited at E1 and was not
retried.

- Preflight: passed.
- E0 detached state: passed.
- Full fixture planning scene: passed.
- Read-only pregrasp path gate: `PATH_OK`, 150 points, 14.804 s planned duration.
- Authoritative E1 adapter result: `PLANNING_FAILED`.
- MoveIt explanation: the independently generated E1 path was rejected by
  `ValidateSolution` for `wrist_2_link` / `forearm_link` self-collision.
- Episode end and `episode.json`: intentionally absent because the runner
  stopped at the first failed phase.
- External retry: none.

`manifest.json` hashes the P7 adapters, capture tools, runner, model, scene and
configuration used by this run.  `launch.txt` and `e1_pregrasp.json` are the
primary failure records; `checksums.sha256` covers all JSON/TXT files present.

Compared with r5, the primary attachment-assisted engineering outcome did not
reproduce.  Do not merge r6 with successful episodes or omit it from a planning
failure denominator.
