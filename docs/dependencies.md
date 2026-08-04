# Dependencies

Status vocabulary:

- `UNVERIFIED`: source is identified but the VM/commit has not been checked;
- `REFERENCE_ONLY`: source is used for architecture or migration evidence only;
- `PINNED`: exact commit/tag has been recorded and reproduced in the Humble VM;
- `FORKED`: an upstream fork is required and its commit is recorded.

Every dependency must eventually include URL, branch/tag/commit, ROS distribution, purpose, vendor status and modification policy.

| Component | Official source | ROS baseline | Pin status | Purpose | Modification policy |
|---|---|---|---|---|---|
| Elite ROS 2 Driver | https://github.com/Elite-Robots/Elite_Robots_CS_ROS2_Driver | Humble | UNVERIFIED | Official CS625 driver and simulation/ros2_control integration | Do not edit in place |
| Elite CS SDK | https://github.com/Elite-Robots/Elite_Robots_CS_SDK | Humble integration | UNVERIFIED | Official SDK dependency where required by driver | Do not vendor-edit |
| Senior CS625 reference snapshot | https://github.com/addission/elite_robot_project_20260129 | Humble reference | REFERENCE_ONLY | CS625 URDF/Xacro, driver launch, controller and simulation lessons | Do not copy-and-edit as vendor code; verify exact revision before underlay use |
| Local senior remote observed in legacy worktree | https://github.com/addission/elite_robot_project_20260306 | Humble reference | UNVERIFIED | Provenance candidate for the local `eli_*` packages | Keep separate from the specification URL until verified |
| AIRLab-POLIMI active vision | https://github.com/AIRLab-POLIMI/active-vision | Humble/Fortress reference | REFERENCE_ONLY | Modular bringup/interfaces/pointcloud/octomap/planning boundaries | Architecture reference only; do not import robot-specific code |
| Existing CS625/NBV project | https://github.com/OUZHENREN/Robot | Humble/Jazzy legacy mix | REFERENCE_ONLY | Existing NBV, IK, trajectory, monitor and experiment assets | Migrate by responsibility; do not copy the legacy workspace wholesale |
| MoveIt 2 | https://github.com/moveit/moveit2 | Humble | UNVERIFIED | Planning, IK integration and collision checking | Use upstream/underlay |
| ros2_control | https://github.com/ros-controls/ros2_control | Humble | UNVERIFIED | Controller interfaces and joint state/trajectory chain | Use upstream/underlay |
| ros_gz | https://github.com/gazebosim/ros_gz | Humble/Fortress | UNVERIFIED | ROS 2 ↔ Gazebo transport and simulation integration | Use upstream/underlay |
| gz_ros2_control | https://github.com/ros-controls/gz_ros2_control | Humble/Fortress | UNVERIFIED | Gazebo control plugin | Use upstream/underlay |
| MoveIt occupancy map monitor | https://github.com/moveit/moveit2/tree/main/moveit_ros/occupancy_map_monitor | Humble | UNVERIFIED | Occupancy-map monitor base used by planning-scene updates | Use upstream binary; do not vendor |
| MoveIt perception | https://github.com/moveit/moveit2/tree/main/moveit_ros/perception | Humble | UNVERIFIED | Provides the upstream `occupancy_map_monitor/PointCloudOctomapUpdater` plugin | Install/use `moveit_ros_perception`; do not reimplement or vendor |
| RGB-D camera driver | To be selected | Humble | UNVERIFIED | Real profile sensor input | Keep below `cs625_sensor_adapter` |

## Provenance rule

The specification and the local legacy worktree currently disagree on the
senior repository suffix (`20260129` versus `20260306`). This is intentionally
visible here. No `.repos` entry is promoted to `PINNED` until the exact URL,
revision and Humble VM build have been checked.

## Observed local revisions

- Clean reference worktree `.real_nbv_experiment_20260729`: branch
  `experiment/real-nbv`, commit `ae7327e836419a617e9d446ee35c37b6680c1dc0`,
  `origin=https://github.com/OUZHENREN/Robot.git`,
  `upstream=https://github.com/addission/elite_robot_project_20260306.git`.
- Shared legacy workspace `Ubuntu_Share/elite_ros_ws`: commit
  `7f0c39bc3c159c22d9e50f39b9b364657df7be4f`, branch `master`, with local
  uncommitted NBV/simulation changes. It is evidence to inspect, not a pin.

Neither observation is a Humble underlay reproducibility result. The first
commit is the current clean audit source; the second must never be imported as
an opaque dirty workspace.

## P0 rule

No dependency is imported from `.repos` until its exact revision has been checked in the Ubuntu 22.04 VM and added to this table. The current `.repos` files are intentionally empty placeholders, not a claim that the dependency chain has been validated.

## Underlay/application split

```text
cs625_underlay_ws          # vendor, official driver, MoveIt and simulator
cs625_active_perception_ws # this repository's application packages
```

The application workspace must never copy `build/`, `install/` or `log/` from the underlay or another machine.

## Elite SDK build gate

The audited `eli_cs_robot_driver` source requires the official Elite CS SDK and
currently checks for SDK 1.2.0 under:

```text
/opt/elite-sdk-custom/include/Elite/EliteDriver.hpp
/opt/elite-sdk-custom/lib/libelite-cs-series-sdk.*
/opt/elite-sdk-custom/lib/cmake/elite-cs-series-sdk/
```

This is an underlay prerequisite, not application code. The official SDK
repository documents Ubuntu 22.04 support, an Ubuntu PPA installation route,
and a source-build route. Because the audited driver CMake uses the explicit
`/opt/elite-sdk-custom` prefix, the Humble validation must either provide the
matching vendor installation there or use a verified driver revision whose
SDK prefix is configurable. Do not create substitute headers, stub libraries,
or modify the vendor driver inside this repository.

Until the SDK is present and the driver builds in Humble, description,
simulation and MoveIt configuration can be validated independently, but the
real hardware driver must remain an unverified underlay dependency.

## VMware shared-folder build rule

When the source repository is mounted through VMware HGFS at `/mnt/hgfs`, keep
the Git source root there but place colcon artifacts on the VM's local ext4
filesystem. ROSIDL C/C++ generation creates deep paths that can exceed HGFS
path limits. `scripts/build.sh` and `scripts/test.sh` therefore default to:

```text
${HOME}/cs625_colcon/{build,install,log}
```

Override this location with `CS625_COLCON_ROOT` when needed. After a successful
build, source `${HOME}/cs625_colcon/install/setup.bash` before running the
workspace packages.
