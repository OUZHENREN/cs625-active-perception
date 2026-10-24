# P6 on-site R1--R4 runbook

This runbook reuses the senior CS625 worktree as the underlay/interface source
but never launches its fixed-endpoint convenience scripts. Record every result
in the work log with the used driver revision and operator name.

## R1: driver and kinematic calibration

1. In the approved Ubuntu 22.04/Humble environment, pin and build the official
   Elite driver/SDK and the senior CS625 MoveIt underlay revision.
2. With E-stop accessible and the arm controller inactive, obtain the
   robot-kinematic correction using the senior `eli_cs_robot_calibration`
   tooling. Its controller connection and output write are operator actions,
   not actions performed by this repository.
3. Start this repository's `real_base.launch.py` only with a reviewed endpoint,
   `launch_driver:=true`, `execute:=false` and
   `activate_joint_controller:=false`. Verify all six configured joint names
   are fresh on the configured joint-state topic.

## R2: camera, hand-eye and grasp-interface manifest

Calibrate the eye-in-hand transform and store an operator-reviewed JSON file
outside this repository. It must contain these fields:

```json
{
  "base_frame": "base_link",
  "camera_frame": "camera_depth_optical_frame",
  "method": "documented_hand_eye_method",
  "operator": "responsible_person",
  "timestamp_utc": "RFC3339 timestamp",
  "transform": {"translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
}
```

Verify the normalized sensor status is connected and fresh, and TF resolves
`base_link -> camera_depth_optical_frame`. The senior `/target_pose` and
`/cs625/grasp_state_event` conventions are only a later optional handoff;
keep `grasp_handoff_enabled=false` until this R2 evidence is accepted.

## R3: planning-only evidence

Launch `real_readiness.launch.py` after the driver, camera adapter and MoveIt
are already running. It only observes joint states, sensor status, TF and the
three MoveIt service names. Then run the existing reachability filter on a
recorded target; retain its collision/IK/planning results. Neither entry point
contains a trajectory action client.

## R4: separately authorized low-speed motion

Do not start R4 from this repository by default. The operator must review
workspace, payload/tool, E-stop, controller lifecycle, timeout and stop test.
Only then may an explicitly approved session set all of `execute=true`,
`require_confirmation=false`, `r4_authorized=true`, a non-empty operator ID
and a non-empty approval ID in an external reviewed execution configuration,
then invoke `real_execution.launch.py start_executor:=true`. Save the exact
configuration, operator confirmation and trajectory outcome; on any failure,
return to R3.
