# Interfaces and parameter contract

This document freezes the application-facing contract. P3 implements
planning-only candidate generation and hard reachability filtering; P4 adds
baseline selection, and P5 adds a reproducible joint-score policy. P6 must
preserve this application contract while changing only profile and underlay
composition.

## 1. Phase 1 messages

### `cs625_ap_interfaces/msg/ActiveLocalizationState.msg`

Carries the state-machine state, timestamp, human-readable detail and whether execution is enabled. It does not contain a strategy-specific field.

### `cs625_ap_interfaces/msg/SensorStatus.msg`

Carries status timestamp, source frame, connection state, freshness state, received count and a diagnostic detail. A `fresh=true` value is only valid when every required stream in the configured adapter has a recent valid header.

## 2. Sensor adapter parameters

All topic names are required profile parameters; no camera-brand default is allowed in code.

```yaml
input_color_topic: ""
input_depth_topic: ""
input_camera_info_topic: ""
input_points_topic: ""
output_color_topic: ""
output_depth_topic: ""
output_camera_info_topic: ""
output_points_topic: ""
status_topic: ""
freshness_timeout_sec: 1.0
drop_invalid_messages: true
```

The four normalized output topics are supplied by the profile and must match:

```text
/sensors/camera/color/image
/sensors/camera/depth/image
/sensors/camera/depth/camera_info
/sensors/camera/points
```

## 3. Execution parameters

The following parameters are reserved for bringup and later motion layers:

```text
execute:=false
require_confirmation:=true
max_velocity_scale
max_acceleration_scale
workspace_bounds
```

No Phase 1 node may issue a robot trajectory. `execute` is a safety gate, not a command to bypass MoveIt.

## 4. Later interfaces

### `cs625_ap_interfaces/msg/ViewCandidate.msg`

One candidate is always a `base_link -> camera_depth_optical_frame` pose.  The
motion adapter appends its calculated `tool0` pose, hard reachability result,
failure code, planning duration and motion-cost proxy.  A false `reachable`
value is never forwarded to the reachable-candidate topic.

### `cs625_ap_interfaces/msg/ViewCandidateArray.msg`

Carries one reproducible candidate set.  `/view_planner/raw_candidates` is
published only by `cs625_view_generation`; `/view_planner/reachable_candidates`
is published only by `cs625_motion_adapter` after TF, IK, validity and MoveIt
planning checks.  P3 is planning-only: it has no trajectory action client.

### `cs625_ap_interfaces/msg/ViewSelection.msg`

P4 publishes the selected reachable candidate, strategy, random seed and cycle
index on `/view_planner/selected_view`.  The strategy selector is a retained
subscriber of the hard-filtered set and records a planning-only episode JSON.
`fixed_view`, `random_reachable`, `predefined_scan` and `coverage_nbv` use the
same retained candidate IDs.  The current coverage score is explicitly a
geometry proxy, not a measured target-surface coverage result.

## 5. P5 score and experiment interfaces

`proposed_joint_score` evaluates only candidates that have passed the P3 hard
filter. Its recorded terms are `localization_gain_proxy`, `joint_margin`,
`motion_cost_penalty`, `planning_time_penalty` and `proposed_joint_score`.
The localization term is explicitly a geometry proxy until a validated
localizer provides an observed information-gain estimate. Score weights and
normalization scales are profile parameters, captured in each experiment
snapshot; they are never hard-coded in the policy.

`cs625_experiment_tools` accepts an immutable candidate snapshot and emits
JSONL episode records, candidate-score CSV, paired-summary CSV, a statistics
note and a PNG summary plot. The record validity label travels with the input;
synthetic or proxy-only data may not be reported as a physical experiment.

## 6. P6 real-profile boundary

`real_preflight` is status-only: it has no trajectory action client and always
reports `real_motion_occurred=false`. It requires an empty-by-default robot
endpoint, `execute=false`, `require_confirmation=true`, bounded workspace and
low velocity/acceleration scales. The real bringup may activate a controller
only after the external R4 review supplies `execute=true` and explicitly
disables the confirmation gate; this repository does not perform that review.

## 7. Deferred interfaces

The following remain planned for later phases:

- plan/execute action;
- active-localization task action.

They will be added only after the corresponding simulated or physical gate is
accepted.
