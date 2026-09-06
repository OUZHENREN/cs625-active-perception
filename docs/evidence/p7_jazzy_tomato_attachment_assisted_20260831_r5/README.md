# P7 r5 evidence index

This directory is the 2026-08-31 fresh, abort-on-failure P7 Jazzy simulation
episode for YCB `005_tomato_soup_can`.

- Engineering result: attachment-assisted kinematic chain passed.
- Formal P7 result: not accepted (`PERCEPTION_STALE`).
- Perception: not executed; fixture pose only.
- Contact monitoring: not instrumented.
- Real robot: not connected and no real motion occurred.
- External retry after a failed phase: none.

Primary records:

- `episode.json`: assembled result and contract evaluation;
- `manifest.json`: runtime, Git and asset identity;
- `preflight.json`: controller, initial joint and clock gate;
- `e1_pregrasp.json`, `e2_approach.json`, `e4_lift.json`: MoveIt and controller receipts;
- `e3_geometry.json`, `e3_gripper.json`, `e3_attachment.json`: grasp-centre, finger and attachment receipts;
- `target_before_lift.json`, `target_after_lift.json`, `e4_lift_measurement.json`: measured target lift;
- `hold_trace.json`: 2.226 s effective pose trace;
- `launch.txt`: raw MoveIt/Gazebo launch log, including unresolved warnings;
- `checksums.sha256`: SHA-256 identity for all JSON/TXT evidence files.

The human-readable decision is
[`../../p7_simulation_temporary_acceptance_2026-08-31.md`](../../p7_simulation_temporary_acceptance_2026-08-31.md).
