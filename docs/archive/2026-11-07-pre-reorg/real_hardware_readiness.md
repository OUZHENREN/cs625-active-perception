# P6 real-hardware readiness and safety gates

The P6 application core remains shared with simulation. Only source topics,
clock, driver composition and safety configuration differ. No real trajectory
execution is implemented in this repository.

| Gate | Objective | Required evidence | Current code behavior |
|---|---|---|---|
| R0 | Profile safety | preflight, empty IP, safe defaults, bounded workspace and scales | Implemented without hardware |
| R1 | Driver connection | Official Elite driver + Elite CS SDK, robot connection and state feedback | Physical robot/operator required |
| R2 | Camera and TF | Verified source topics, hand-eye transform and fresh cloud | Physical camera/calibration required |
| R3 | Planning-only | Current state and collision-checked MoveIt plan without controller activation | Verified R1/R2 session required |
| R4 | Low-speed confirmed motion | On-site review, limits, stop test and an explicitly confirmed trajectory | Default-off executor implemented; physical approval/evidence required |

`real_preflight` has no `ActionClient` or trajectory-command path. It emits an
auditable `/motion/status` JSON record and returns `REAL_EXECUTION_REFUSED`
when `execute=true`. `real_base.launch.py` blocks controller activation unless
both `execute=true` and `require_confirmation=false` follow external R4 review.

`real_execution.launch.py` does not start an executor unless
`start_executor=true` is explicitly supplied. Even then, its real-profile
executor refuses every selection unless `execute=true`, confirmation is
released after review, `r4_authorized=true`, operator and approval IDs are
non-empty, and both MoveIt scaling factors are positive and at most `0.10`.
It replans with MoveIt immediately before sending the controller action.

Before R1--R4, confirm network, E-stop and workspace with the responsible
operator; calibrate the camera and hand-eye transform; and record controller
lifecycle, joint-state, camera freshness, TF and planning-only evidence. Never
save a robot IP in this repository.

The driver and SDK are provisioned in the shared Jazzy underlay (see
[`dependencies.md`](dependencies.md)); `scripts/source_dev_env.sh` adds the SDK
prefix to the compiler and loader search paths when it is present.

## Profile base entries

Simulation and hardware share one application core and one MoveIt
configuration. Only the base composition differs, and the active-perception
pipeline is layered on top instead of being folded into either base.

| Entry | Starts | Does not start |
|---|---|---|
| `sim_base.launch.py` | Gazebo + `gz_ros2_control` + application description + MoveIt | active-perception nodes |
| `real_base.launch.py` | Elite driver + `ros2_control` + application description + MoveIt | active-perception nodes |

`real_base.launch.py` keeps `launch_driver:=false`, `robot_ip:=` empty,
`activate_joint_controller:=false` and `execute:=false` by default, so it is a
composition shell until R1 is authorised. MoveIt is skipped when the driver is
not running, because no robot description is published in that case.

Both bases reuse `elite_cs625_moveit_config` for SRDF, kinematics, joint limits
and planning pipelines, and both build the planning model from the application
Xacro, so the planning frame, octomap frame and self-collision exemptions are
identical. The only intended difference is the clock: the simulation profile
runs MoveIt on `/clock`, the real profile on wall time.

## Reuse map: senior CS625 worktree

The local senior worktree at `Ubuntu_Share/elite_ros_ws` is the preferred P6
reuse source for components it has already demonstrated. It remains a separate,
dirty reference worktree: do not launch it as the P6 entry point and do not
copy its vendor packages into this application repository.

| Existing component | Proven capability | P6 reuse decision |
|---|---|---|
| `elite_cs625_moveit_config` and `eli_cs_robot_*` | CS625 MoveIt planning, driver and ros2_control composition | Reuse as the pinned underlay after R1 verification; keep controller activation behind `real_base.launch.py` |
| `tcp_bridge/tcp_server.py` | Visual target-pose and grasp/insert TCP-pose conversion | Reuse topic semantics only; move transform/calibration values into the real profile after R2 hand-eye validation |
| `cs625_task_manager` | Grasp/release operator-confirmed state machine and IO checks | Reuse its task-state/confirmation pattern after the perception-to-grasp handoff is specified |
| `cs625_compliant_placement` | Contact-aware insertion/placement flow | Keep out of NBV scope until R4, then evaluate as a separately confirmed end-effector workflow |

The senior launchers contain fixed endpoints and default controller activation.
Those defaults are intentionally not inherited by this repository; all
endpoints, transforms, controller names and motion permissions must come from
reviewed P6 configuration.
