# Project file layout

This repository is organized around ROS 2 package boundaries.  Keep the
package architecture stable first; use documentation and local archives for
cleanup before moving runtime scripts that are already part of evidence
manifests.

## Root

| Path | Role | Move policy |
| --- | --- | --- |
| `src/` | ROS 2 application packages | Do not flatten or merge packages |
| `scripts/` | environment entry point plus top-level build/test/matrix commands | Keep commands that are user-facing and not tied to one P7 gate |
| `test/` | repository contract check plus P7 gate tools and fixtures | Keep stable until P7 manifests/docs are migrated together |
| `docs/` | architecture, protocols, diagnostics and accepted evidence | Safe to add indexes; avoid rewriting evidence records |
| `.repos/` | underlay dependency manifests | Keep as dependency entry points |
| `config/` | reserved cross-profile config placeholders | Keep until common/real/sim config ownership is finalized |

`scripts/source_dev_env.sh` is the single supported environment entry point; see
the `Environment entry (mandatory)` section of `AGENTS.md` for the fixed
ROS Jazzy → CS625 underlay → repository overlay chain.

Generated directories such as `build/`, `install/`, `log/`, `.pytest_cache/`
and dated `build_*`, `install_*`, `log_*` directories are not source.  They are
ignored by Git and may be moved to the local archive when the root gets noisy.

## ROS 2 packages

This table is the single detailed source for the package structure.  It must
match `colcon list` exactly: **11 packages**, no more and no fewer.  Other
documents (including `README.md` and `AGENTS.md`) reference this file instead of
maintaining their own lists.

| Package | Build type | Responsibility |
| --- | --- | --- |
| `cs625_ap_interfaces` | `ament_cmake` | shared ROS messages, services and actions |
| `cs625_ap_description` | `ament_cmake` | CS625 application URDF/Xacro extensions, camera and gripper frames |
| `cs625_bringup` | `ament_cmake` | launch composition and profile-specific runtime configuration |
| `cs625_experiment_tools` | `ament_python` | matrix summaries, paired experiments and metrics export |
| `cs625_motion_adapter` | `ament_python` | TF conversion, IK, MoveIt planning and execution adapters |
| `cs625_sensor_adapter` | `ament_python` | simulation/real sensor topic normalization |
| `cs625_simulation` | `ament_cmake` | Gazebo worlds, YCB assets and simulation-only resources |
| `cs625_target_perception` | `ament_python` | target pose and localization quality interface layer |
| `cs625_task_orchestrator` | `ament_python` | P7 grasp evidence, task receipts and atomic episode helpers |
| `cs625_view_evaluation` | `ament_python` | view scoring, strategy selection and P4 episode coordination |
| `cs625_view_generation` | `ament_python` | candidate view generation |

Check for drift against the build system with:

```bash
colcon list
```

## Evidence and diagnostics

| Path | Contents |
| --- | --- |
| `docs/evidence/README.md` | accepted run index and claim boundaries |
| `docs/evidence/minimum_showcase_matrix_20260906_r1/` | 45-cell matrix JSON/CSV/log summaries |
| `docs/evidence/p7_5_gate_summary_20260906_r12/` | frozen single-scene P7.5 summary |
| `docs/evidence/p7_5_perception_grasp_20260906_r12/` | r12 non-raw receipts, logs and pose diagnostic figures |
| `docs/diagnostics/p7/` | small diagnostic outputs for camera/pose/NBV debugging |
| `docs/worklogs/` | repository work logs; filenames start with `YYYY-MM-DD` |

Large raw captures stay local: `.cdr`, `.npy` and `.ppm` under `docs/evidence`
are ignored by `.gitignore`.  Use Git LFS or a separate data-release archive
before publishing raw sensor frames.

## Cleanup rules

1. Do not move a file if `rg` shows hardcoded `test/...`, `docs/evidence/...`
   or package-share references unless all call sites and tests move with it.
2. Do not move files installed by `setup.py`, `CMakeLists.txt`, `package.xml`,
   launch files or xacro includes without rebuilding the owning package.
3. Do not edit historical evidence logs just to satisfy formatting checks.
4. Prefer adding a local README/index before moving many runtime scripts.
5. Keep generated outputs outside the repository root after each long run.
