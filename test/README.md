# Repository checks and P7 tools

`contract_checks.py` is host-side and does not require ROS. It verifies the Phase 0–1 package set, required governance files, package XML names, safe launch defaults and Python syntax.

Run from the repository root:

```bash
python3 test/contract_checks.py
```

Ubuntu build, xacro, launch, TF, controller and sensor checks remain separate gates.

## Layout

This directory is intentionally kept at the repository root because the P7
evidence manifest, shell runners and package tests still reference stable
`test/...` paths.  Do not move these scripts without updating the manifest,
docs and regression tests in one change.

| Group | Files | Purpose |
| --- | --- | --- |
| Static contract | `contract_checks.py` | Host-side repository governance check |
| Unit/regression tests | `test_*.py` | Pytest tests for P7 geometry, pose, NBV and planning helpers |
| P7.1 sensor gate | `run_p7_1_*`, `p7_capture_sensor_gate.py`, `p7_capture_axis_marker_diagnostic.py`, `p7_static_camera_projection.py`, `p7_solve_static_observation_pose.py` | Sensor input capture, TF timing and target visibility audit |
| P7.2 pose gate | `p7_2_*` | Frozen observation inspection, textured/cylindrical pose estimation and handoff validation |
| P7.3 NBV gate | `p7_3_*` | Geometry NBV scoring and static candidate screening |
| P7.4 MoveIt gate | `p7_4_*`, `p7_execute_view_trace.py`, `p7_check_candidate_state.py` | Candidate planning, trajectory execution receipts and terminal-state checks |
| P7.5 grasp gate | `p7_run_grasp_from_estimate.py`, `p7_generate_grasp_candidates.py`, `p7_pregrasp_path_search.py`, `p7_apply_fixture_scene.py`, `p7_execute_*`, `p7_capture_hold_trace.py`, `p7_evaluate_*` | Grasp generation, scene application, gripper/contact/lift/hold receipts |
| Launch helpers | `run_p7_*.sh`, `build_p7_contact_monitor.sh`, `p7_send_*.sh` | Reproducible shell entry points for Gazebo/MoveIt sessions |
| Fixtures | `data/` | Small JSON/YAML fixtures used by tests and scripted runs |
