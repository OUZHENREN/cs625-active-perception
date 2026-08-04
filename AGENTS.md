# CS625 Active Perception Development Rules

## Project goal

This repository implements the ROS 2 framework for the thesis:

**面向遮挡目标定位的机械臂可达性规划方法研究**

The system has two deployment profiles with one shared application core:

- simulation: Elite CS625 + Gazebo + simulated RGB-D camera;
- real: official Elite CS625 ROS 2 driver + real RGB-D camera.

The research core is reachability-aware observation/view planning for occluded target localization. NBV is a pluggable strategy, not the system architecture.

## Supported baseline

- Ubuntu 22.04 LTS in the approved VM;
- ROS 2 Humble;
- MoveIt 2 Humble;
- Gazebo Fortress through ros_gz;
- C++17, Python 3, colcon and rosdep.

Do not mix Humble and Jazzy workspaces. Existing Jazzy code is legacy/experimental.

## Phase 0–1 package boundary

Until the Phase 1 review passes, only these application packages may be created:

- `cs625_ap_interfaces`: ROS messages, services and actions only;
- `cs625_ap_description`: camera/tool extensions to the official CS625 description;
- `cs625_sensor_adapter`: normalized simulated and real camera inputs;
- `cs625_simulation`: Gazebo-only worlds, sensors and ground truth;
- `cs625_bringup`: launch composition and profiles only.

Later packages and their responsibilities are defined in `docs/architecture.md`. Do not migrate NBV or create a mega-node in Phase 0–1.

## Third-party code

Elite official ROS 2 Driver, Elite CS SDK, MoveIt 2, ros2_control, ros_gz, gz_ros2_control and camera drivers are upstream dependencies.

- Do not edit vendor packages in place.
- Prefer `.repos` files and an underlay workspace.
- If an upstream modification is unavoidable, use a dedicated fork and pin the commit.
- Record URL, branch/tag/commit, purpose, vendor status and modification policy in `docs/dependencies.md`.
- Never commit `build/`, `install/`, `log/`, rosbag or generated database files.

## Sim/real contract

Required frames:

```text
base_link
tool0
camera_link
camera_depth_optical_frame
target_frame (when available)
```

Required normalized topics:

```text
/sensors/camera/points
/perception/target_pose
/perception/localization_quality
/scene/fused_cloud
/view_planner/raw_candidates
/view_planner/reachable_candidates
/view_planner/selected_view
/motion/status
/active_localization/state
```

Camera-brand topic names must not escape `cs625_sensor_adapter`. Every pose and cloud needs a valid timestamp and `frame_id`. Simulation and real profiles must expose the same application-facing contract.

## Safety and configuration

Real execution defaults are always:

```text
execute:=false
require_confirmation:=true
```

Use conservative velocity/acceleration scaling, bounded workspace, explicit timeout and explicit failure codes. Never bypass MoveIt collision checking or use undocumented raw TCP motion when the official interface is available.

Do not hardcode IP addresses, absolute user paths, topic names, frame names, workspace bounds, camera intrinsics, hand-eye transforms, score weights, controller names or velocity limits. Put shared values in `config/common` and profile values in `config/sim` or `config/real`.

## View-planning order

The eventual pipeline is:

```text
observe → estimate target → check termination → generate candidates
→ transform camera poses → filter hard reachability constraints
→ score reachable candidates → plan → optionally execute
→ wait for a new settled sensor frame → repeat
```

Hard constraints include IK, joint limits, self collision, environment collision and required TF. Fixed, random, predefined and NBV strategies must use one common interface and candidate set.

## Task workflow and verification

Before editing, read this file and the relevant `docs/` files, inspect the current package and tests, and state the package/interface impact. Make the smallest coherent change, add/update tests, and update documentation when behavior or interfaces change.

Minimum checks for a changed package:

```bash
colcon build --symlink-install --packages-up-to <changed_package>
colcon test --packages-select <changed_package>
colcon test-result --verbose
```

For launch and TF changes, also use the commands documented in `docs/simulation.md` and `docs/frames_and_topics.md`.

Every work log must state changed files, exact commands, pass/fail results, remaining limitations, whether fake hardware/Gazebo/real hardware was used, and whether real motion occurred.

## Prohibited patterns

- Do not use `pkill ros2` for lifecycle management.
- Do not create backup or duplicate source scripts.
- Do not commit generated artifacts.
- Do not add a mega-node owning perception, planning, execution and logging.
- Do not publish the same TF from multiple nodes.
- Do not consume stale point clouds after robot motion.
- Do not label synthetic smoke tests as real perception validation.
- Do not modify several architecture layers in one task without explicit scope.

