# Repository checks and task tooling

`contract_checks.py` is host-side and does not require ROS. It verifies the Phase 0–1 package set, required governance files, package XML names, safe launch defaults and Python syntax.

Run from the repository root:

```bash
python3 test/contract_checks.py
```

Ubuntu build, xacro, launch, TF, controller and sensor checks remain separate gates.

## Layout

| Group | Files | Purpose |
| --- | --- | --- |
| Static contract | `contract_checks.py` | Host-side repository governance check |
| Unit/regression tests | `test_*.py` | Pytest tests for geometry, insertion contract, grasping, sensors and planning helpers |
| Insertion episode | `assemble_insertion_episode.py`, `run_insertion_sequence.sh`, `test_insertion_episode_contract.py` | Insertion sequence validation against `insertion_episode_contract` |
| Surviving P7 receipt tools | `p7_execute_pose_capture.py`, `p7_capture_gazebo_contacts.py`, `p7_attachment_smoke_publisher.py` | Execution receipts still reused by `run_insertion_sequence.sh` (their P7 neighbours were archived 2026-11-13) |
| Fixtures | `data/` | Small JSON/YAML fixtures used by tests and scripted runs |

## The 2026-11-13 P7 archive

The old P7 five-gate simulation pipeline (sensor gate, pose gates, geometry NBV,
MoveIt NBV, attachment grasp, the `p7_ycb_tomato_*` worlds and their configs) was
unusable and has been archived to
[`docs/archive/2026-11-13-p7-nbv-legacy-code/`](../docs/archive/2026-11-13-p7-nbv-legacy-code/README.md),
including the P7 test files that used to live here.  Do not import or run anything
from the archive; if a piece needs revival, copy it out and adapt it first.
