# P7 r9 failed-run evidence index

This directory preserves the single pre-registered r9 attempt on ROS domain
`84` and Gazebo partition `p7_acceptance_20260831_r9` after the E1 and unified
launcher changes.

- Unified launcher readiness: passed. Gazebo, all three controllers, MoveIt,
  and the arm/gripper/attachment P7 adapters were present.
- Preflight: passed (initial joints within tolerance; sampled simulation clock
  showed no regression).
- E0 detach: passed with an observed `ATTACHMENT_DETACHED` receipt.
- Full fixture planning scene: passed.
- E1: failed. The authoritative request used an explicit current start state
  and a pose goal (`1 mm` position tolerance, `0.005 rad` orientation
  tolerance), but OMPL/RRTConnect returned `PLANNING_FAILED` after `4.012036 s`.
- MoveIt logged `Unable to solve the planning problem`; it did not log a
  collision pair or another more specific cause for this request.
- E2--E4 and `episode.json`: intentionally absent because the runner is
  abort-on-failure.
- External retry: none. The pre-registered r10 replication was not started
  after r9 failed.
- Real robot: not connected; no real motion occurred.

Primary records:

- `manifest.json`: source, launcher, runtime, Git, and asset identity;
- `preflight.json`: controller, initial-state, and clock gate;
- `e0_detach.json`, `scene_full.json`: successful gates before E1;
- `e1_pregrasp.json`: authoritative pose-goal request receipt and failure;
- `failure_runtime_snapshot.txt`: live adapter/node state and the relevant
  MoveIt failure excerpt;
- `launch.txt`: raw Gazebo, ros2_control, MoveIt, and adapter launch/teardown
  log;
- `checksums.sha256`: SHA-256 identity for all preserved payload/index files.

This directory is failure evidence. It must not be cited as a successful P7
episode or combined with r5 to calculate a grasp success rate.
