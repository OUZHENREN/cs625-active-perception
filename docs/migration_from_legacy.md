# Legacy asset migration and reference boundaries

This file records what is reused from the three reference sources and what is
deliberately not copied into this repository.

## Reference sources

| Source | Role | Local evidence | Policy |
|---|---|---|---|
| AIRLab-POLIMI `active-vision` | Package boundaries and active-vision pipeline shape | `av_bringup`, `av_interfaces`, `av_pointcloud`, `av_octomap`, `av_planning` | Architecture reference only; do not import robot-specific packages or its MoveIt fork |
| Senior CS625 project | CS625 model, driver, ros2_control, simulation and deployment experience | `.real_nbv_experiment_20260729/src/eli_cs_robot_description`, `eli_cs_robot_driver`, `eli_cs_robot_simulation_gz`, `elite_cs625_moveit_config` | Reuse through an underlay or a pinned reference snapshot; do not edit vendor code in this repository |
| `OUZHENREN/Robot` | Existing kinematics, NBV, monitoring and experiment assets | `.real_nbv_experiment_20260729/src/cs625_kinematics`, `cs625_nbv`, `cs625_state_monitor`, `cs625_trajectory_tools` | Migrate by responsibility after interface review; do not copy the legacy workspace wholesale |

The project specification names the senior repository as
`https://github.com/addission/elite_robot_project_20260129`. The local legacy
worktree currently records an `upstream` remote ending in
`elite_robot_project_20260306`. These are kept as two explicit provenance
records until the exact Humble-compatible revision is checked in the VM.
The clean audit snapshot is branch `experiment/real-nbv` at commit
`ae7327e836419a617e9d446ee35c37b6680c1dc0`.

## Package-level reuse matrix

| Legacy/reference asset | Destination in the target architecture | Action now | Reason |
|---|---|---|---|
| `active-vision/av_interfaces` | `cs625_ap_interfaces` | Borrow separation and stable application-facing interfaces | Brand- and robot-neutral contract is required |
| `active-vision/av_bringup` | `cs625_bringup` | Borrow launch composition and profile grouping | Launch composes packages; it does not own algorithms |
| `active-vision/av_pointcloud` | Later `cs625_scene_mapping` | Borrow stage boundary only | Do not import agriculture-specific processing yet |
| `active-vision/av_octomap` | Optional later `cs625_scene_mapping` | Keep optional | The paper can start with target cloud plus simple collision geometry |
| `active-vision/av_planning` | Later `cs625_motion_adapter` / `cs625_view_evaluation` boundary | Borrow planning/evaluation split | Candidate filtering must precede strategy scoring |
| Senior `eli_cs_robot_description` | `cs625_ap_description` dependency | Add xacro wrapper that calls the official macro and adds Eye-in-Hand links | Avoid copying the robot model |
| Senior `eli_cs_robot_driver` | Humble underlay / real profile | Reference only in Phase 1 | The application must not fork or edit the hardware plugin in place |
| Senior `eli_cs_robot_simulation_gz` | Humble underlay / sim profile | Expose an explicit launch hook | Controller names and launch arguments must be verified before defaulting to it |
| Senior `elite_cs625_moveit_config` | Humble underlay | Reuse after description link names are verified | Existing SRDF uses `my_end_effector_link`; the wrapper preserves it and adds `tool0` |
| `cs625_kinematics` | Later `cs625_motion_adapter` | Keep source for review, then adapt behind a new reachability interface | Existing solver has hard-coded limits and a legacy end-effector name |
| `cs625_trajectory_tools` | Later `cs625_motion_adapter` | Retain planning-client ideas and services selectively | Do not bring the large legacy service surface into Phase 1 |
| `cs625_state_monitor` | Later `cs625_experiment_tools` or a small monitor package | Keep logging/state parsing assets | Avoid making it a dependency of the sensor adapter |
| `cs625_nbv/viewpoint_sampler` | Later `cs625_view_generation` | Migrate after candidate frame contract is frozen | Generator must not call MoveIt or execute motion |
| `cs625_nbv/information_gain` | Later `cs625_view_evaluation` | Migrate as one strategy/scoring component | NBV is a pluggable strategy, not the task state machine |
| `cs625_nbv/baseline_strategies` | Later `cs625_view_evaluation` | Preserve fixed/random/predefined baselines | All strategies must receive the same reachable candidate set |
| `cs625_nbv/covariance_estimator` | Later perception/evaluation boundary | Review ownership before moving | It currently mixes registration uncertainty with episode logic |
| `cs625_nbv/experiment_logger` | Later `cs625_experiment_tools` | Preserve schema and provenance fields | Experiments must record seed, commit, strategy and failure code |
| `cs625_nbv/synthetic_camera_publisher_node` | `cs625_simulation` | Move only after the sim topic contract is fixed | Simulated sensors must not live in the evaluation package |
| `cs625_nbv/nbv_orchestrator` | Later `cs625_task_orchestrator` | Redesign state/action interfaces; do not copy | The orchestrator must own the closed loop, not NBV mathematics |
| `cs625_full_system` | `cs625_bringup` | Do not migrate as a whole | It is the legacy mega-launch explicitly rejected by the specification |

## Audited reusable APIs

| Existing symbol/file | Reuse decision | Required adaptation |
|---|---|---|
| `ViewpointSampler::generate_candidates` | Reuse sampling mathematics in later `cs625_view_generation` | Replace embedded camera defaults with profile parameters; keep output as camera poses in a declared frame |
| `InformationGain::compute_ig`, `expected_covariance`, `compute_utility` | Reuse scoring implementation in later `cs625_view_evaluation` | Remove direct ownership of legacy `cs625_nbv/msg`; consume the frozen candidate/evaluation contract |
| `BaselineStrategies::select_next` | Reuse fixed/random/coverage/uncertainty/pose-gain/path-cost baselines | Preserve the existing invariant that input candidates have already passed reachability filtering |
| `IkSolver::solveIKAll` and FK/conversion helpers | Adapt inside later `cs625_motion_adapter` | Replace hard-coded joint limits and legacy frame assumptions with URDF/MoveIt state; use as diagnostic/fallback, not a collision checker |
| `NbvOrchestrator` callbacks/state machine | Do not copy as architecture | Split plan/move/capture/fuse ownership into the future task orchestrator and stable actions; map states to `ActiveLocalizationState` |
| `ExperimentLogger` and `EpisodeMetadata` | Reuse schema concepts in later experiment tools | Remove the default home-directory path; require run directory, commit, seed, profile and validity label as explicit inputs |
| `synthetic_camera_publisher_node` | Adapt only inside `cs625_simulation` | Keep `synthetic_smoke_test/interface_only` provenance and never label it as Gazebo or real perception evidence |
| `nbv_pipeline.launch.py` | Do not copy | It mixes synthetic sensor, orchestrator and visualization and repeats algorithm parameters inside launch code |

The legacy `ViewpointCandidate` message already carries pose, information gain,
path cost and reachability fields, but it mixes generation, hard filtering and
evaluation ownership. Its fields are evidence for the future interface review,
not permission to move the message unchanged during Phase 1.

## Audited underlay compatibility

- `eli_cs_robot_driver/elite_control.launch.py` is the reusable real driver
  entry. It requires `robot_ip` and supports selecting the description xacro,
  controller config and inactive/active initial controller state.
- That launch couples `description_package` to the vendor parameter YAML paths.
  The application therefore passes an absolute installed wrapper as
  `description_file` while retaining `eli_cs_robot_description` as the config
  owner; no YAML is copied into this repository.
- The senior MoveIt `move_group.launch.py` currently constructs its own
  `robot_description`. Model equality with the application wrapper is an S1
  runtime gate and remains unproven.
- The local `vision_bridge` package is a TCP target-pose receiver, not a
  PS800E1/Percipio RGB-D driver. It must not be mislabeled as camera reuse.

## PS800E1 / Percipio camera boundary

The current local source scan did not find a checked-in `PS800E1`, Percipio or
图漾 RGB-D driver package or a verified topic map. `vision_bridge` was also
audited and only converts TCP pose strings to `PoseStamped`. Therefore the target repository
does not invent vendor topic names. The real profile will later provide the
driver-specific input topics to `cs625_sensor_adapter`; the application-facing
outputs remain the normalized RGB-D contract in `frames_and_topics.md`.

The Eye-in-Hand geometry is handled separately by the description wrapper.
Camera intrinsics, calibration and vendor transport are profile data, not URDF
algorithm code.

## Phase boundary

During Phase 0–1 only the five baseline packages remain in `src/`. The matrix
is a migration plan and provenance record, not permission to add all future
packages immediately. Algorithm migration begins only after the official
CS625/Humble simulation baseline has a reproducible build and launch result.
