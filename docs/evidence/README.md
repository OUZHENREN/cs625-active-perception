# Runtime evidence

This directory contains machine-written or source-derived evidence for
simulation acceptance.  A receipt records only the interface response that
was observed during the named run; higher-level acceptance is evaluated
separately and must not infer missing perception, contact, or real-robot
evidence.

## P7 run index

| Directory | Role | May be used as final single-attempt evidence? |
| --- | --- | --- |
| `p7_jazzy_tomato_attachment_assisted_20260831_r3` | diagnostic; E0 failed because startup was already detached | no |
| `p7_jazzy_tomato_attachment_assisted_20260831_r4` | diagnostic; used to repair carried-object scene and hold timing; E1 required an explicit retry | no |
| `p7_jazzy_tomato_attachment_assisted_20260831_r5` | fresh, abort-on-failure single episode | yes, but only for the attachment-assisted engineering chain |
| `p7_jazzy_tomato_attachment_assisted_20260831_r6` | independent equivalent run; read-only path gate passed but actual E1 planning failed | no; retained as reproducibility-failure evidence |
| `p7_jazzy_tomato_attachment_assisted_20260831_r7` | preflight passed; failed before E0 because the launch entry omitted all three P7 adapters; E1 not reached | no; retained as infrastructure-failure evidence |
| `p7_jazzy_tomato_attachment_assisted_20260831_r9` | unified launcher, preflight, E0, and scene passed; authoritative single-request pose-goal E1 failed after 4.012 s | no; retained as post-fix planning-failure evidence |
| `p7_1_sensor_input_20260901_r1` | P7.1 diagnostic; capture infrastructure failed after the first window and the target was outside the old static view | no; immutable failure evidence |
| `p7_1_sensor_input_20260901_r2` | P7.1 diagnostic; initial-joint preflight failed and the old runner incorrectly continued | no; immutable failure evidence, checksums verified |
| `p7_1_sensor_input_20260901_r3` | first complete five-window P7.1 attempt; initial state, streams, synchronization and exact-time TF passed, but target support was 0/12 in all windows | no; immutable gate-failure evidence, checksums verified |
| `p7_1_sensor_input_20260901_r4` | isolated sensor-pose hypothesis test; five windows complete but target support remained 0/12 | no; rejected-hypothesis evidence, checksums verified |
| `p7_1_sensor_input_20260901_r5` | OBJ plus point-cloud axis-normalization attempt; five windows complete but target support remained 0/12 | no; immutable gate-failure evidence, checksums verified |
| `p7_1_axis_marker_diagnostic_20260901_d1` | diagnostic marker invisible; isolated camera-axis/self-occlusion fault | no; diagnostic only |
| `p7_1_axis_marker_diagnostic_20260901_d2` | preflight authorization variable missing | no; infrastructure-failure evidence |
| `p7_1_axis_marker_diagnostic_20260901_d3` | strict requested-joint preflight failed before capture | no; diagnostic failure evidence |
| `p7_1_axis_marker_diagnostic_20260901_d4` | failed requested-state preflight; post-failure marker capture proved camera mount fix | no; diagnostic only, marker 274 px |
| `p7_1_axis_marker_diagnostic_20260901_d5` | settled-state preflight and marker visibility both passed | no; diagnostic only, never acceptance evidence |
| `p7_1_sensor_input_20260901_r6` | formal YCB attempt stopped at joint preflight while simulation-time settling was incomplete | no; immutable gate-failure evidence |
| `p7_1_sensor_input_20260901_r7` | formal YCB five-window sensor-input gate | **yes; P7.1 PASS, checksums verified** |
| `p7_2_pose_baseline_20260901_b1` | frozen-r7 RGB-D cylinder baseline plus post-hoc GT scorer | no; translation 5/5, full SE(3) 0/5 because texture yaw is unobservable |
| `p7_5_perception_grasp_20260906_r12` | actual post-NBV perception plus physical-contact top-grasp execution on severe_v5 tomato | yes, as a single-scene P7.5 simulation episode; not a formal matrix |
| `p7_5_gate_summary_20260906_r12` | frozen P7.5 physical-contact gate summary for r12 | **yes; P7.5 single-scene PASS** |
| `minimum_showcase_matrix_20260906_r1` | completed 3-scene x 5-seed x 3-baseline P4 simulation matrix | **yes; matrix COMPLETE**, but all episodes ended with `MAX_FAILED_ATTEMPTS` |

The selected full-6D reuse path and its isolated-environment blocker are
recorded in [`../diagnostics/p7/p7_2_estimator_reuse_audit_20260901.md`](../diagnostics/p7/p7_2_estimator_reuse_audit_20260901.md).

The 2026-08-31 r5 contract intentionally remains `task_success=false` with
`PERCEPTION_STALE`: target perception was not executed and physical contact
monitoring was not instrumented.  Because r6 did not reproduce the r5
engineering outcome, r7 did not reach E1, and r9 reached but failed the new E1,
that historical attachment-assisted P7-A chain remains open.  See
[`../p7_simulation_temporary_acceptance_2026-08-31.md`](../p7_simulation_temporary_acceptance_2026-08-31.md)
before citing that historical field.

P7.1 is frozen by r7: all five windows passed message layout, synchronization,
timestamp-exact TF and real YCB visibility, with 110--112 target-support points
per window against a threshold of 12.  Ground truth was used only for this
visibility audit.  P7.2--P7.4 are frozen by the severe-v5 reobservation and
MoveIt evidence indexed above.  P7.5 is accepted only for the single r12
simulation episode; the formal multi-scene, multi-seed grasp matrix remains
unrun.
