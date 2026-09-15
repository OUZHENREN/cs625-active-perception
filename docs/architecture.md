# Architecture

## 1. System boundary

The application has one common active-localization core and two deployment profiles:

```text
sim profile:  Gazebo + simulated RGB-D adapter
real profile: official Elite driver + real RGB-D adapter
                         ↓
              common application interfaces
```

The system pipeline is:

```text
observe
  → estimate target
  → check termination
  → generate camera candidates
  → transform camera pose to tool pose
  → filter hard reachability constraints
  → evaluate reachable views
  → MoveIt plan
  → optional execution
  → wait for a new settled sensor frame
  → observe
```

The research contribution is reachability-aware view planning for occluded target localization. NBV is one strategy plugin, alongside fixed, random, predefined and reachability-only baselines.

## 1.1 Single-repository profile invariant

There is one Git repository, one common application layer and two profile
front-ends:

```text
one Git repository
├── one common application/source layer
├── one sim configuration and launch entry
└── one real configuration and launch entry
```

The sim and real profiles may select different underlay drivers, source topics,
clock settings and launch arguments, but they must expose the same normalized
interfaces and TF/topic contract. No second copy of the active-perception core
may be created for real hardware. Vendor and senior-reference packages stay in
the underlay and are composed by `cs625_bringup`.

## 2. Target package responsibilities (future target architecture / roadmap)

This section is the **future target architecture / roadmap**, not the current
repository layout.  Some rows below are planned and do not exist yet, so this
table must not be read as an inventory of available packages, and a
`planned / not currently implemented` row must never be referenced as an
existing dependency.

The single source of truth for the ROS 2 packages that actually exist today is
[`project_layout.md`](project_layout.md); that list must match `colcon list`
exactly (currently **11 packages**).  The table below is a superset of it.

The `Status` column refers to **package existence only**: `implemented` means the
package exists in this repository today.  It does not claim that the target
responsibility, or any runtime capability, is complete — see the capability
gates in [`p7_five_gate_protocol.md`](p7_five_gate_protocol.md) and `README.md`.

| Package | Responsibility | Phase | Status |
|---|---|---|---|
| `cs625_ap_interfaces` | Stable messages, services and actions only | P1 | implemented |
| `cs625_ap_description` | Include official CS625 description and add camera/tool links | P1 | implemented |
| `cs625_sensor_adapter` | Normalize simulated/real RGB-D input and freshness checks | P1/P2 | implemented |
| `cs625_simulation` | Gazebo worlds, sensors, target/occluder models and truth | P1/P2 | implemented |
| `cs625_bringup` | Launch composition and profile configuration | P1 | implemented |
| `cs625_target_perception` | Target cloud, pose and localization quality | P2 | implemented |
| `cs625_scene_mapping` | ROI, fused cloud and optional OctoMap | P2/P3 | planned / not currently implemented |
| `cs625_view_generation` | Candidate camera poses only | P3 | implemented |
| `cs625_view_evaluation` | Strategy plugins and score terms | P4/P5 | implemented |
| `cs625_motion_adapter` | Camera/tool transform, IK, collision, MoveIt and execution gate | P3/P6 | implemented |
| `cs625_task_orchestrator` | P7 task-level evidence, grasp state orchestration and atomic episode logging | P7 | implemented |
| `cs625_experiment_tools` | Episode logs, rosbag and metrics | P4/P5 | implemented |

## 3. Phase 0–1 restriction

Only the first five packages in the table may be created before the Phase 1 review. No NBV implementation, target perception algorithm, candidate sampler, motion adapter or mega-node belongs in this phase.

## 4. Dependency direction

```text
official underlay / vendor drivers
              ↓
       profile adapters
              ↓
     common application interfaces
              ↓
   perception / view / motion / task layers
              ↓
       experiment evidence
```

Vendor code is not edited in this repository. Launch files compose packages and parameters; they do not calculate research algorithms.
