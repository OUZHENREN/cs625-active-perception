# Dependencies

Status vocabulary:

- `UNVERIFIED`: source is identified but the VM/commit has not been checked;
- `REFERENCE_ONLY`: source is used for architecture or migration evidence only;
- `PINNED`: exact commit/tag has been recorded and reproduced in the Humble VM;
- `FORKED`: an upstream fork is required and its commit is recorded.

Every dependency must eventually include URL, branch/tag/commit, ROS distribution, purpose, vendor status and modification policy.

| Component | Official source | ROS baseline | Pin status | Purpose | Modification policy |
|---|---|---|---|---|---|
| Elite ROS 2 Driver | https://github.com/Elite-Robots/Elite_Robots_CS_ROS2_Driver | Jazzy | REFERENCE_ONLY | Upstream reference for the CS625 driver interface | Do not edit in place; the audited revision used here comes from the senior snapshot below |
| Elite CS SDK | https://github.com/Elite-Robots/Elite_Robots_CS_SDK | Jazzy | `v1.2.0` = `092f159575ab6c7a8977005785d18e89512a6bba` | `elite-cs-series-sdk` 1.2.0 required by `eli_cs_robot_driver` (`find_package(... 1.2.0 REQUIRED)`) | MIT. Built with plain CMake and installed to `$HOME/elite-sdk-1.2.0`; never vendor-edit |
| Senior CS625 snapshot (driver source) | https://github.com/addission/elite_robot_project_20260305 | Jazzy | `main` = `5c003831aedc744ccce39c1aa1028133d1d89680` | Source of `eli_cs_robot_driver` and `eli_cs_controllers` copied into the shared underlay | Copy only the missing packages; do not edit in place and do not overwrite the locally modified `eli_cs_robot_description` / `eli_cs_robot_simulation_gz` |
| `elite_io_rviz_plugin` (same snapshot) | https://github.com/addission/elite_robot_project_20260305 | Jazzy | `main` = `5c003831aedc744ccce39c1aa1028133d1d89680` | Elite tool-IO RViz panel referenced by the vendor `moveit.rviz` | **Not usable as-is**: its `package.xml` has no `<build_type>ament_cmake</build_type>`, so colcon installs it as a plain CMake package, it never lands on `AMENT_PREFIX_PATH`, and RViz cannot discover it. Do not patch vendor in place; the application RViz layout drops all `elite_*` panels instead |
| `elite_dashboard_rviz_plugin` | Not published in the audited snapshot | Jazzy | ABSENT | Panels referenced by the vendor `moveit.rviz` | No source in any audited revision; removed from the application RViz layout |
| Senior CS625 reference (earlier snapshot) | https://github.com/addission/elite_robot_project_20260130 | Jazzy | `main` = `1b1e6c524b8cc2666e4b1eef5b83346f44eaa9c7` | Earlier audited revision; superseded by the 20260305 snapshot | Historical reference only |
| AIRLab-POLIMI active vision | https://github.com/AIRLab-POLIMI/active-vision | Humble/Fortress reference | REFERENCE_ONLY | Modular bringup/interfaces/pointcloud/octomap/planning boundaries | Architecture reference only; do not import robot-specific code |
| Existing CS625/NBV project | https://github.com/OUZHENREN/Robot | Humble/Jazzy legacy mix | REFERENCE_ONLY | Existing NBV, IK, trajectory, monitor and experiment assets | Migrate by responsibility; do not copy the legacy workspace wholesale |
| MoveIt 2 | https://github.com/moveit/moveit2 | Humble | UNVERIFIED | Planning, IK integration and collision checking | Use upstream/underlay |
| ros2_control | https://github.com/ros-controls/ros2_control | Humble | UNVERIFIED | Controller interfaces and joint state/trajectory chain | Use upstream/underlay |
| ros_gz | https://github.com/gazebosim/ros_gz | Humble/Fortress | UNVERIFIED | ROS 2 ↔ Gazebo transport and simulation integration | Use upstream/underlay |
| gz_ros2_control | https://github.com/ros-controls/gz_ros2_control | Humble/Fortress | UNVERIFIED | Gazebo control plugin | Use upstream/underlay |
| MoveIt occupancy map monitor | https://github.com/moveit/moveit2/tree/main/moveit_ros/occupancy_map_monitor | Humble | UNVERIFIED | Occupancy-map monitor base used by planning-scene updates | Use upstream binary; do not vendor |
| MoveIt perception | https://github.com/moveit/moveit2/tree/main/moveit_ros/perception | Jazzy runtime verified; system install pending | PARTIALLY_PINNED | Provides the upstream `occupancy_map_monitor/PointCloudOctomapUpdater` plugin | Install `ros-jazzy-moveit-ros-perception` system-wide; a temporary extracted deb overlay was used only for the P2 runtime gate; do not vendor |
| RGB-D camera driver | To be selected | Humble | UNVERIFIED | Real profile sensor input | Keep below `cs625_sensor_adapter` |

## Provenance rule

The senior repository URL and revision are now resolved: the audited driver
source is `elite_robot_project_20260305` at
`5c003831aedc744ccce39c1aa1028133d1d89680`. Earlier suffixes seen in the legacy
notes (`20260129`, `20260306`) are historical and are not used.

## Real-profile underlay provisioning

The real driver needs two things the simulation profile does not: the Elite CS
SDK and the senior driver packages. Provisioning is deliberately manual and
minimal because the shared underlay already carries locally modified copies of
`eli_cs_robot_description` and `eli_cs_robot_simulation_gz` that must not be
overwritten.

1. Build and install the SDK (plain CMake, no colcon):

   ```bash
   git clone --branch v1.2.0 --depth 1 \
     https://github.com/Elite-Robots/Elite_Robots_CS_SDK "$HOME/elite_sdk_1_2_0_src"
   cmake -S "$HOME/elite_sdk_1_2_0_src" -B "$HOME/elite_sdk_1_2_0_src/build" \
     -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/elite-sdk-1.2.0"
   cmake --build "$HOME/elite_sdk_1_2_0_src/build" -j"$(nproc)"
   cmake --install "$HOME/elite_sdk_1_2_0_src/build"
   ```

2. Copy **only** the two missing packages from the pinned senior checkout into
   the shared underlay, then build them there:

   ```bash
   U="$HOME/cs625_underlay_jazzy/src"
   cp -a <pinned-checkout>/src/eli_cs_robot_driver   "$U/"
   cp -a <pinned-checkout>/src/eli_cs_controllers    "$U/"
   source scripts/source_dev_env.sh --no-overlay
   ( cd "$HOME/cs625_underlay_jazzy" && \
     colcon build --symlink-install \
       --packages-up-to eli_cs_robot_driver eli_cs_controllers \
       --cmake-args -DCMAKE_BUILD_TYPE=Release )
   ```

   `--packages-up-to` is required because the underlay sources contain
   `eli_common_interface` and `eli_dashboard_interface`, which the driver
   depends on and which the original underlay build never installed.

3. Verify:

   ```bash
   source scripts/source_dev_env.sh --verify
   ros2 pkg prefix eli_cs_robot_driver
   ldd "$(ros2 pkg prefix eli_cs_robot_driver)/lib/libeli_cs_hardware_interface_plugin.so" | grep elite
   ```

   The `Elite CS SDK` include/library paths come from
   `scripts/source_dev_env.sh`; the driver links the SDK by plain library name
   rather than through its imported CMake target, so `CPLUS_INCLUDE_PATH`,
   `LIBRARY_PATH` and `LD_LIBRARY_PATH` must all carry the prefix.

## Observed local revisions

- Clean reference worktree `.real_nbv_experiment_20260729`: branch
  `experiment/real-nbv`, commit `ae7327e836419a617e9d446ee35c37b6680c1dc0`,
  `origin=https://github.com/OUZHENREN/Robot.git`,
  `upstream=https://github.com/addission/elite_robot_project_20260306.git`.
- Shared legacy workspace `Ubuntu_Share/elite_ros_ws`: commit
  `7f0c39bc3c159c22d9e50f39b9b364657df7be4f`, branch `master`, with local
  uncommitted NBV/simulation changes. It is evidence to inspect, not a pin.

Neither observation is an underlay reproducibility result. The first commit is
the current clean audit source; the second must never be imported as an opaque
dirty workspace.

## P0 rule

No dependency is imported from `.repos` until its exact revision has been
checked in the supported Jazzy environment and added to this table. The
`.repos` files remain placeholders for the shared underlay because the underlay
packages here are plain copies, not per-package git repositories.

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
