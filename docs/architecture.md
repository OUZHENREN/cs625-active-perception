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

## 2. Target package responsibilities

| Package | Responsibility | Phase |
|---|---|---|
| `cs625_ap_interfaces` | Stable messages, services and actions only | P1 |
| `cs625_ap_description` | Include official CS625 description and add camera/tool links | P1 |
| `cs625_sensor_adapter` | Normalize simulated/real RGB-D input and freshness checks | P1/P2 |
| `cs625_simulation` | Gazebo worlds, sensors, target/occluder models and truth | P1/P2 |
| `cs625_bringup` | Launch composition and profile configuration | P1 |
| `cs625_target_perception` | Target cloud, pose and localization quality | P2 |
| `cs625_scene_mapping` | ROI, fused cloud and optional OctoMap | P2/P3 |
| `cs625_view_generation` | Candidate camera poses only | P3 |
| `cs625_view_evaluation` | Strategy plugins and score terms | P4/P5 |
| `cs625_motion_adapter` | Camera/tool transform, IK, collision, MoveIt and execution gate | P3/P6 |
| `cs625_task_orchestrator` | Active-localization state machine | P4 |
| `cs625_experiment_tools` | Episode logs, rosbag and metrics | P4/P5 |

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
