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
| RGB-D camera driver | https://github.com/percipioxyz/camport_ros2 | Jazzy | `FORKED` (local clone `~/cs625_camport_ws`, bundled camport4 SDK 4.2.33; edits listed below) | Real profile sensor input for the PS800-E1 (RGB + depth + registered cloud) | Dedicated local fork; every edit is listed in "Real-profile RGB-D camera driver"; originals kept as backups |

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

## Real-profile RGB-D camera driver (camport_ros2 fork)

The real cell's RGB-D camera is a Percipio **PS800-E1** (GigE). RobotVisionSuite
also drives this camera, and it cannot share the device with ROS, so the ROS path
takes the camera only while RVS is stopped.

- Source: https://github.com/percipioxyz/camport_ros2 (bundles camport4 SDK 4.2.33)
- Local clone: `~/cs625_camport_ws/src/camport_ros2`, built into
  `~/cs625_camport_ws/install`; it is an overlay loaded *after*
  `scripts/source_dev_env.sh --full`
- Purpose: real-profile sensor input (`/camera/color/image_raw`,
  `/camera/depth/image_raw`, `camera_info`, registered cloud). Vendor topic names
  must stay below the sensor adapter, not leak into the application contract.

Local modifications (vendor code is never edited in place; each item was made on
this clone with the original file kept as a backup):

| File | Change | Why |
|---|---|---|
| `src/percipio_device.cpp` | when the requested device id is an IP, open with `TYOpenDeviceWithIP` and do not return early (the `alive = true` flag and the event-callback registration live at the end of the function) | broadcast discovery is unreliable inside WSL; the camera must be opened by IP |
| `src/percipio_camera_node_driver.cpp` | if broadcast enumeration finds nothing, walk the ethernet interfaces and open by IP | same reason; the fallback path |
| `src/percipio_camera_node.cpp` / `include/percipio_camera_node.h` | `publishDepthFrame` converts raw depth counts to millimetres with the device's own `DepthScaleUnit` (0.25 mm per count on the PS800-E1) before publishing `16UC1` | the driver read the scale into `depth_scale_` and never used it, so every depth value was four times too large; masked clouds were stretched from 0.44 m to metres |

Rebuild after a change:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/cs625_camport_ws && colcon build --symlink-install --packages-select percipio_camera
ros2 launch percipio_camera percipio_camera.launch.py device_ip:=<camera IP> \
  color_resolution:=640x480 depth_resolution:=640x480 \
  depth_enable:=true depth_registration_enable:=true point_cloud_enable:=false
```

Measured behaviour (640x480): ~23.5 fps colour-only, ~0.74 fps with depth (the
depth hardware limit), so depth is used as a static "stop and shoot" frame, not a
stream. `1280x960` gives no usable depth on this metal part (specular surface),
which is why every pose computation uses 640x480.

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

## Legacy automation underlay (frozen 2026-10-20)

The senior's ROS automation stack is reused from a frozen archive as a separate
WSL underlay, not copied into this repository. Decision record: `ADR-004`.

| Item | Value |
|---|---|
| Source archive | `/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/newfolder/elite_ros_ws20260414.zip` |
| SHA-256 | `d19cef1917bc2662fba615302dd792b866a321d9ba1fab5f4779b741ca8b5b18` |
| WSL underlay | `~/cs625_legacy_underlay_jazzy/` |
| Snapshot | `~/cs625_legacy_snapshot_20261020/` (`legacy_src_14pkgs_20261020_r2.tar.gz`, `SHA256SUMS`, `SHA256SUMS.as_extracted`, `PACKAGE_MANIFEST.tsv`, `SOURCE_PROVENANCE.md`, `BUILD_STATUS.md`, `P2_SMOKE_20261020.txt`) |
| ROS baseline | Jazzy / Ubuntu 24.04 WSL (the earlier Humble VM passages above are the historical Phase-0 baseline) |
| Build result | **14/14 packages, zero source changes, 1 min 06 s** (2026-10-20) |
| Runtime smoke | `cs625_compliant_placement`: **8/8 PASS** (2026-10-20), stub `/cs625/raw_state`, no robot |

Packages (all `ament_cmake` except where noted): `cs625_compliant_placement`,
`cs625_state_monitor`, `cs625_task_manager`, `cs625_trajectory_tools`,
`cs625_kinematics`, `cs625_tcp_force_probe`, `eli_cs_robot_calibration`,
`environment_collision_generator`, `environment_point_cloud_publisher`,
`workspace_pointcloud_sampler`, `pointcloud_workflow_interfaces`, `model_visualizer`
(`ament_python`), `tcp_bridge` (`ament_python`), `vision_bridge`.

Why the archive and not the on-disk worktree: the 2026-04-14 archive is the last
coherent built workspace and **contains** `cs625_compliant_placement` and
`cs625_tcp_force_probe`; the later on-disk worktree at
`D:\...\Model\newfolder\elite_ros_ws` (HEAD `ad730f1`, 364 uncommitted changes,
remote `elite_robot_project_20260306` which does not exist on GitHub) lacks both,
has the compliant package flattened into the RVS model directory, and carries an
older `CS625State.msg` without the TCP wrench fields.

Excluded on purpose: the eight packages already present in
`~/cs625_underlay_jazzy/src`; `cs625_full_system` (mega-launch, hard-coded
endpoints); `elite_dashboard_rviz_plugin` (missing `<build_type>`, not on
`AMENT_PREFIX_PATH`); all `build/`, `install/`, `log/`, `__pycache__` and `*.pyc`.

Boundaries:

- RVS is a Windows program and stays on Windows; it exchanges target poses over
  TCP only.
- `tcp_bridge/robot_commander` (robot port 60000, `external_control.script`) is a
  direct-control path that bypasses MoveIt and collision checking: **not** used
  for automatic motion; manual diagnosis only.
- The packages are the senior's code. Cite the senior thesis, and do not publish
  or redistribute them without permission.
- Any runtime fix is made inside the legacy underlay, not by editing this
  repository's source, and is recorded in the snapshot provenance.

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

### P2 amendment: compliant-placement version and its real capability boundary

The archive's `cs625_compliant_placement` is the **old REPORT_ONLY** revision: it has
only `get_compliant_placement_state`, no Start/Stop, and no `eli_common_interface`
dependency. The newer revision with Start/Stop/metrics exists only as a flattened copy
in the RVS model directory (`Model/newfolder/*.cpp|hpp`). That copy was reassembled into
the layout its own `CMakeLists.txt` expects — **file positions only, no logic change** —
and its per-file hashes are recorded in
`~/cs625_legacy_snapshot_20261020/cs625_compliant_placement_RECONSTRUCTED.tsv`.
The reassembled package rebuilds and advertises `/start_compliant_placement`,
`/stop_compliant_placement` and `/get_compliant_placement_state`.

Runtime smoke (2026-10-20, stub `/cs625/raw_state`, no robot) passes 8/8: initial state,
Start accepted, target reached, soft approach, contact hold, safety abort, Stop(abort),
GetState. Smoke harness: `test/legacy_compliant_smoke.py` (copy in the underlay
`tools/`).

**Capability boundary of this revision — do not overstate it:**

- **Jam detection is not implemented.** `PlacementController::update()` never sets
  `jam_detected`; the node also never declares the `progress_window_sec`,
  `progress_epsilon_mm` or `*_confirm_cycles` parameters that appear in the YAML.
  The thesis must not claim an existing jam detector; a progress-window criterion has
  to be added on top.
- The node is a **supervisor**, not an executor: it publishes state and requests
  `/io_and_status_controller/set_speed_slider`; it does not plan or execute motion.
  Motion still comes from the RVS plugin / MoveIt client.
- The contact metric is `sum(|joint_torques|)` residual between a fast EMA
  (`contact_alpha=0.2`) and a slow baseline EMA (`baseline_alpha=0.01`); thresholds
  are 0.8 (contact) and 2.0 (safety) in that residual unit. A torque step is a
  transient window, so the thresholds need recalibration on real robot data.
- Standalone (no `ros2_control` IO controller) the speed-slider call only warns; state
  logic is unaffected.
