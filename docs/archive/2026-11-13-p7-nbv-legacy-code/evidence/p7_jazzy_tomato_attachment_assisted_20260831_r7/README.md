# P7 r7 failed-run evidence index

This directory preserves the single pre-registered r7 attempt on ROS domain
`82` and Gazebo partition `p7_acceptance_20260831_r7`.

- Preflight: passed (controllers active, initial joints within tolerance, no
  sampled clock regression).
- First commanded gate: failed before E0 completed.
- Failure: `P7 attachment adapter subscription unavailable`.
- Root observation: neither `/p7/attachment_command` nor
  `/p7/attachment/state` existed, and the live node/process snapshot contained
  no P7 arm, gripper, or attachment adapter.
- E1 pose-goal planning: not reached and therefore not evaluated by r7.
- E2--E4 and `episode.json`: intentionally absent because the runner is
  abort-on-failure.
- External retry: none. The planned r8 replication was not started after this
  infrastructure failure.
- Real robot: not connected; no real motion occurred.

Primary records:

- `manifest.json`: source/runtime identity captured before the failed gate;
- `preflight.json`: successful controller, initial-state, and clock gate;
- `episode_start.json`: episode boundary marker;
- `failure_runtime_snapshot.txt`: live ROS nodes, missing attachment topics,
  and matching process list after failure;
- `launch.txt`: raw Gazebo, ros2_control, and MoveIt launch/teardown log;
- `checksums.sha256`: SHA-256 identity for the preserved evidence files.

This directory is failure evidence. It must not be cited as a P7 motion-chain
success or as a test of the new E1 pose-goal planner.
