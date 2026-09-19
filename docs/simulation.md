# Simulation baseline

## 1. Scope

The accepted simulation baseline is WSL2 Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic 8.11 / `ros_gz`. It reuses the pre-existing Jazzy underlay and application overlay supplied by the local workspace; it must not be reconstructed as a Humble environment. P4 validates the official CS625 model, `gz_ros2_control`, MoveIt, the Eye-in-Hand description extension, a minimal RGB-D sensor and normalized topics. It does not, by itself, validate a learned perception or grasp method.

## 2. Validation levels

| Level | Content | Evidence boundary |
|---|---|---|
| S0 | fake hardware or static scripted input; TF/topic/state checks | interface smoke evidence only |
| S1 | Gazebo CS625 motion, camera motion and MoveIt planning | geometry/control evidence; not real perception |
| S2 | target and occluders; target localization loop | synthetic truth evaluation, explicitly labeled |
| S3 | multi-scene, multi-seed batch experiments | research evidence after logging and protocol freeze |

## 3. Required Phase 1 entry points

```bash
ros2 launch cs625_bringup sim_base.launch.py launch_rviz:=false
ros2 launch cs625_bringup sim_active_localization.launch.py strategy:=disabled launch_rviz:=false
```

`execute` must be safe by default. `sim_active_localization` in Phase 1 starts the framework only and does not start NBV.

The corresponding real-profile entry point is:

```bash
ros2 launch cs625_bringup real_base.launch.py
```

It is intentionally a safe composition shell. It does not start a robot driver
or a camera adapter until the underlay and vendor source topics are passed
explicitly:

```bash
ros2 launch cs625_bringup real_base.launch.py \
  launch_driver:=true robot_ip:=<robot-ip> \
  launch_sensor_adapter:=true \
  input_color_topic:=<verified-color-topic> \
  input_depth_topic:=<verified-depth-topic> \
  input_camera_info_topic:=<verified-camera-info-topic> \
  input_points_topic:=<verified-point-cloud-topic>
```

The real profile reuses `eli_cs_robot_driver/elite_control.launch.py`. It passes
the installed application xacro as an absolute `description_file`, while the
underlay `eli_cs_robot_description` package remains responsible for the CS625
joint, kinematic, physical and visual YAML files. The arm controller remains
inactive unless `activate_joint_controller:=true` is supplied explicitly.

The placeholders are deliberate. Until the PS800E1/Percipio driver and topic
contract are verified in the Jazzy environment, no vendor topic is hardcoded in this
repository.

Both profiles include exactly the same common sensor launch when
`launch_sensor_adapter:=true`; only the source topics and clock profile differ.

For the simulation profile, `cs625_ap_description` reuses the senior CS625
robot macro and adds the application eye-in-hand camera frames. The top-level
Xacro wrapper also reuses the senior `gz_ros2_control` plugin pattern. When
`launch_sensor_adapter:=true`, `sim_base.launch.py` starts
`ros_gz_bridge/parameter_bridge` with `sim_gz_bridge.yaml`, mapping the senior
Gazebo `/camera/*` source topics to `/sim/camera/*` before the common adapter.

## 3.1 MoveIt description composition gate

The audited senior `move_group.launch.py` rebuilds `robot_description` from
its setup-assistant metadata and therefore omits the application Eye-in-Hand
extension. The application does not copy that vendor launch. Instead,
`cs625_bringup/sim_moveit.launch.py` reuses the senior SRDF, kinematics,
joint-limit, planning and controller files through `MoveItConfigsBuilder`,
while explicitly loading the application Xacro and the normalized point-cloud
sensor configuration. This keeps Gazebo/robot_state_publisher and MoveIt on
the Jazzy overlay smoke test must still show that the
control graph, planning scene and camera TF agree before S1 is declared pass.

The simulation profile defaults to `cs625_bringup/config/sim_controllers.yaml`.
It reuses the senior CS625 Xacro macro, meshes, physical/kinematic parameters
and MoveIt configuration. The local `sim_control.launch.py` owns only process
ordering: it uses the installed Jazzy `robot_description` topic spawn pattern
and standard `joint_state_broadcaster` / `joint_trajectory_controller`
plugins. No vendor model or driver implementation is copied into this
repository.

The required Phase 1 framework entry is:

```bash
ros2 launch cs625_bringup sim_active_localization.launch.py strategy:=disabled
```

It is a thin wrapper around `sim_base.launch.py`. The `strategy` argument is
reserved for later common-core strategies; `disabled` is the only implemented
value in Phase 1, so this entry starts no target-perception or NBV algorithm.
Its defaults use the same application `sim_controllers.yaml` profile and the
same deterministic simulation control composition as the base entry.

## 3.2 Gazebo entity startup gate

Runtime isolation showed that the senior `create -string` process can time out
while Gazebo is starting, after which the former application retry could race
the original request and create overlapping entities. `sim_control.launch.py`
therefore follows the installed Jazzy `gz_ros2_control_demos` pattern: publish
one robot description, create entity `cs` once with the create tool's default
renaming-disabled behavior, then start the joint-state broadcaster and trajectory controller in order via
process-exit event handlers. There is no timer-based second spawn.

When `headless:=true`, the control launch passes both server-only mode and
Harmonic `--headless-rendering`. The latter selects EGL for server-side camera
rendering and requires the worlds' Sensors system to use Ogre2; `-s` alone is
not a headless rendering configuration.

## 3.3 MoveIt point-cloud plugin gate

The normalized `/sensors/camera/points` stream is consumed by MoveIt's
upstream `occupancy_map_monitor/PointCloudOctomapUpdater`. The plugin is
provided by the Jazzy `moveit_ros_perception` package, while
`moveit_ros_occupancy_map_monitor` provides the base monitor API. Both are
runtime dependencies of `cs625_bringup`; the application does not implement
or vendor an alternative octomap updater.

Verify the plugin package in the sourced Jazzy environment before launch:

```bash
ros2 pkg prefix moveit_ros_perception
```

The application keeps the senior CS625 `gz_ros2_control` model integration,
and explicitly points the Gazebo plugin at the `robot_description` parameter
on `robot_state_publisher`. The controller manager name uses the Jazzy
`controller_manager_name` element. On Jazzy, MoveIt's occupancy-map monitor
uses the planning frame as its effective Octomap frame. The CS625 planning
frame is `world`, so the application declares `octomap_frame=world` and keeps
the 0.02 m resolution; only the point-cloud source is replaced by the
normalized common topic. `world -> base_link` is a fixed TF, and timestamped
point-cloud transforms to both frames have been verified at runtime.

Repeated `Message Filter dropping message` output is downstream evidence that
the arm joint transforms are unavailable. It is not accepted as a sensor gate
pass merely because point-cloud messages are flowing: controller activation,
`/joint_states`, and the full camera TF chain must all pass.

For Jazzy/Harmonic, `sim_base.launch.py` resolves the installed
`gz_ros2_control` prefix through the ament index, verifies that
`libgz_ros2_control-system.so` exists, and prepends its library directory to
both `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` before
starting the application simulation composition. This turns an otherwise silent
Gazebo model-plugin lookup failure into an immediate launch error and supports
the installed Harmonic plugin environment.

## 3.4 Visualization and manual run

By default the Gazebo entity is spawned from a visual-mesh-free URDF.
`sim_control.launch.py` runs `prepare_gazebo_model.py`, which removes every
mesh-based `<visual>` before `ros_gz_sim create`.  On the supported WSL2
software-renderer path this keeps the RGB-D render scene from stalling, while
links, joints, inertials, collisions, `ros2_control` and the eye-in-hand sensor
remain intact.  The consequence is that the arm is **not drawn in the Gazebo
GUI**; only world geometry (ground plane, occluder, target) is visible there.

`gazebo_visuals:=true` keeps the mesh visuals so the arm is drawn in the Gazebo
GUI:

```bash
source scripts/source_dev_env.sh
ros2 launch cs625_bringup sim_base.launch.py \
  headless:=false gazebo_visuals:=true launch_rviz:=true
```

The mesh path was exercised on the supported WSL2 machine on 2026-09-16 with
`headless:=true gazebo_visuals:=true`: the controller manager received the
robot description and all three controllers activated, and the eye-in-hand
sensor advertised `/camera/image`, `/camera/depth_image`, `/camera/camera_info`
and `/camera/points`.  So the meshes did **not** stall the render scene on that
path.  The point-cloud rate was only about 2 Hz on the software renderer, well
below the configured 10 Hz, so a GUI demo should still be treated as a
visualisation path rather than a timing-representative one; the mesh-free model
remains the configuration for recorded evidence and matrix runs.

Re-check throughput whenever the renderer or driver changes:

```bash
source scripts/source_dev_env.sh
ros2 topic hz /sensors/camera/points
ros2 topic echo /sensors/camera/status --once
```

See the arm without touching the Gazebo renderer through RViz, which is given
the complete application Xacro by MoveIt.  The switch must be requested
explicitly because it defaults to `false`:

```bash
source scripts/source_dev_env.sh
ros2 launch cs625_bringup sim_base.launch.py headless:=true launch_rviz:=true
```

Gazebo stays server-only (`headless:=true`), the accepted sensor/physics
configuration, while RViz renders the arm and the MoveIt planning scene.  Both
MoveIt profiles load the application layout `cs625_bringup/config/cs625_moveit.rviz`
(switchable through `moveit_rviz_config`) instead of the vendor `moveit.rviz`.
The application layout is a copy of the vendor layout with every `elite_*` panel
removed: `elite_dashboard_rviz_plugin` has no source in the audited underlay at
all, and `elite_io_rviz_plugin` omits `<build_type>ament_cmake</build_type>` in
its `package.xml`, so colcon installs it as a plain CMake package and RViz
cannot discover it.  Removing those panels is the application-layer fix; the
vendor file is never edited.

One vendor display is retained as-is: a standalone `RobotModel` display whose
description topic `/preview_robot_description` has no publisher.  It reports a
display status only; the MoveIt MotionPlanning display loads the model itself
(RViz log line `Loading robot model 'cs625'`), and the arm pose follows
`/joint_states` once the controllers are active.

The Gazebo GUI path keeps `LIBGL_ALWAYS_SOFTWARE=1`, which `sim_base.launch.py`
sets for the whole launch.

## 4. Required checks

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /sensors/camera/points --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

The task-specified smoke command is:

```bash
timeout 60s ros2 launch cs625_bringup sim_base.launch.py launch_rviz:=false
```

The exact command output, exit code and known limitations must be recorded in a work log. A launch that cannot be reproduced from the pinned Jazzy overlay is a P0/P1 blocker; do not modify the official driver first.

## 5. Reference checked for the fixture

The fixture uses the `rgbd_camera` sensor shape and is exercised in the accepted Jazzy/Harmonic overlay. The official Harmonic documentation and the installed `gz_ros2_control_demos` examples remain the reference for future regression checks.

## 6. Task scene: shielding module and wedge slot fixture

`worlds/cs625_insertion_scene.sdf` is the first world that models the actual task
rather than an occlusion study. It places two models from
`assets/cs625_task/`, both derived from the SolidWorks export:

| Model | Static | Triangles | Mass | Notes |
|---|---|---:|---:|---|
| `slot_fixture` | yes | 9 006 | 12.965 kg | wedge slot, mesh collision |
| `shielding_module` | no | 12 644 | 19.0 kg | measured inertia, box collision |

Run it with:

```bash
ros2 launch cs625_bringup sim_task_scene.launch.py
```

That composes `sim_base.launch.py` with `launch_fixture_world:=false` (the
generic RGB-D fixture world is a second Gazebo session and serves no purpose
here) and, 20 s later, mirrors the static fixture into MoveIt's planning scene so
the planner cannot route the arm through it.

### 6.1 Geometry is owned by one file

`config/cs625_task_scene.yaml` is the authority. The world file, the model SDFs
and the planning-scene mirror all consume it, and `test/contract_checks.py`
asserts that the world `<include>` poses still equal the config. Change a
dimension in one place only and the contract fails.

### 6.2 Why the fixture collides as a mesh

The repository's other static objects use primitive collisions. This fixture
cannot: its convex hull is 57% of its bounding box, and the slot is a wedge whose
walls are the contact surface for the insertion, so a primitive decomposition
would be both loose and wrong. Being static it costs nothing at runtime.

**Verify on first run** that dartsim has not degenerated the concave mesh into
its convex hull; if it has, the slot becomes a solid block and the fixture has to
be rebuilt from explicit wedge slabs.

### 6.3 Why the module collides as a box

Its convex hull is 85% of its bounding box, so the envelope is a close
approximation of the outer shape and is far more stable for a wedge insertion
than a concave mesh. The part is a thin-walled housing (10.9% fill), so this is a
conservative envelope, not the real wall thickness.

### 6.4 Placement and the insertion axis

See §6 of the task scene config. The fixture sits at world
`(0.620, 0.000, 0.2851)` so that the slot opens toward the robot base and the
module, which protrudes 19.3 mm below the fixture when seated, clears the ground.
The module starts upright on the work surface at `(0.700, -0.550, 0.240)`.

The seated pose is an exact rigid transform in the assembly frame, not a
hand-placed guess: it was solved by matching triangle signatures between the
assembly export and the part export, and every vertex then lands within 0.03 um.
It includes a real 6 degree roll, because the slot is a wedge.

### 6.5 Known limitations

- Nothing in this section has been run in Gazebo from the agent side; the sandbox
  cannot write `/dev/shm` or `~/.ros`, so `gz_ros2_control` never activates.
- The module's mass is user-overridden in SolidWorks, so its derived density is
  not a material property.
- The grasp is a preloaded interference fit, and the gripper geometry is still
  the placeholder parallel gripper. See `docs/real_hardware_readiness.md`.
