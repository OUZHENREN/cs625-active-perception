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

## 7. Eye-in-hand camera extrinsics

The simulated camera pose is no longer a tuned guess.  It is the real hand-eye
calibration produced by RobotVisionSuite for the physical camera, expressed in the
flange frame, and the Arm FK reproduces it to the last digit.

```text
src/cs625_bringup/config/camera_extrinsics_sim.yaml   the authority
scripts/camera_extrinsics.py                          regenerates it from RVS output
```

### 7.1 Why the values live in a config

`AGENTS.md` forbids hardcoding hand-eye transforms in the description.
`sim_control.launch.py` reads the YAML and passes all five arguments to xacro, and
it refuses to start when the file is missing rather than silently falling back to
an uncalibrated pose.  The xacro carries the same numbers as defaults so a bare
`xacro` invocation is still correct, and `test/contract_checks.py` compares the two
numerically and fails on drift.

To regenerate against a local RVS install (the path is machine-local and is never
stored in the repository):

```bash
python3 scripts/camera_extrinsics.py --rvs-dir "<RVS>/runtime"
python3 scripts/camera_extrinsics.py --rvs-dir "<RVS>/runtime" --check
```

`CS625_RVS_RUNTIME_DIR` can be used instead of `--rvs-dir`.  Both encodings RVS
writes are parsed — `HandEyeTool.ini` in mm and degrees, `ColorToRobotTCP.txt` in
metres and radians — and cross-checked, so a silent unit change aborts.

### 7.2 Two frame decisions that make the numbers meaningful

**Reference frame is the flange.** The tool owner measured the camera at about
98 mm from the flange, which matches the raw values, so the controller's
`(-46, 0, 353)` mm TCP offset must **not** be composed on top.  That offset is real
— it was verified independently against the recorded `tool_data` CSV to 0.0 mm —
but applying it here would put the camera 450 mm out, in mid-air past the tool's
207 mm tip.

**The recorded pose is the optical frame.** Its `+Z` comes out along the flange
`+Z`, which is what an eye-in-hand camera looking down the tool must do.
`camera_link` is therefore the optical pose with the standard body→optical rotation
removed, and the composition is asserted to reproduce the eye to `1e-12`.

### 7.3 The real camera is a stereo pair

Two eyes, 24.411 mm apart and 0.788 degrees from parallel, calibrated separately.
`camera_link` anchors on the **depth** eye, because Gazebo's `rgbd_camera` is
attached to `camera_link` and looks along that link's `+X`, while the point-cloud
contract names `camera_depth_optical_frame`.  Anchoring there makes the simulated
sensor and the declared optical frame coincide exactly — verified as 0.0000 mm and
0.0000 degrees — and the colour eye carries the offset instead.  An RGB-D registers
its colour image to the depth frame anyway, so this is also the honest way round.

```text
depth optical frame in the flange   (96.754, 17.803, 97.328) mm
colour optical frame in the flange  (96.950, 42.212, 97.540) mm
stereo baseline                     24.411 mm
```

### 7.4 P7.1 was re-run against the calibrated camera, and passed

The previous P7.1 evidence belonged to the placeholder camera, so the gate had to
be re-run.  Doing that also exposed that the observation pose itself was stale: it
was solved for the placeholder, and with the real camera it pointed 1.21 m behind
the target and produced an empty point cloud.  Re-solving it offline (see section
7.5) and re-recording the measured settled state produced a passing run on
2026-09-21:

```text
gate                      P7.1_PERCEPTION_INPUT
gate_pass                 true
windows                   5 / 5
failure_codes             []
evidence (local archive)  ~/p7_1_sensor_gate/20260921_152724
target_point_count        151   (minimum 12)
finite_point_count        57418
target pixel              (162.6178, 94.4440) in 320x240
```

The raw point cloud was re-checked independently of the gate's own summary: 150
points land inside the target's collision proxy, spanning the proxy's cylinder, and
re-projecting the target centre from the recorded TF reproduces the reported pixel
to 2.8e-14 px.  The offline solve had predicted (0.0179, -0.1747, 1.8950) m and
pixel (162.6, 94.4); Gazebo measured (0.017896054, -0.174709352, 1.894976353) m and
(162.6178, 94.4440), and the predicted camera position matched the simulated one to
1e-10 m.  That agreement validates the hand-eye calibration, the frame conventions,
the depth-eye anchoring and the offline solver at once.

One earlier worry is now settled with evidence rather than argument: an eye-in-hand
camera looking straight down the tool axis does NOT inherently have the gripper in
the way.  The real gripper occupies 2440 pixels of the frame but none within 20 px
of the target.

To re-run the gate, against an isolated partition and a fresh evidence directory:

```bash
# 终端 1：起仿真。test/*.sh 在仓库里没有可执行位（这是仓库约定，不是缺陷），
# 所以用 bash 调用，不要直接 ./ 。
bash test/run_p7_1_sensor_sim.sh
```

启动脚本会在四路归一化传感器话题和两个控制器都就绪后打印：

```text
P7_1_SIM_READY partition=<partition> ros_domain_id=<...>
  this terminal now holds the simulator open on purpose; leave it running.
```

**这行 READY 本身就是一次接口检查**：它只在
`/sensors/camera/color/image`、`/sensors/camera/depth/image`、
`/sensors/camera/depth/camera_info`、`/sensors/camera/points` 四路话题类型正确、
且 `joint_state_broadcaster` 与 `joint_trajectory_controller` 都 active 时才打印。

> **这一行之后脚本会一直停着，这不是卡住。** 末尾的 `wait` 是故意的前台持有，
> 用来让仿真保持存活；仿真跑在第一个终端里，不要去关它，也不要 Ctrl-C。

等仿真就绪后，**另开一个终端**抓证据：

```bash
# 终端 2（需要先 source 过环境）
export P7_1_EVIDENCE_DIR="$HOME/p7_1_sensor_gate/$(date +%Y%m%d_%H%M%S)"
CS625_P7_1_SENSOR_GATE=1 CS625_P7_SIMULATION_EXECUTION=1 \
  bash test/run_p7_1_sensor_gate_capture.sh "$P7_1_EVIDENCE_DIR"
```

证据目录必须是**全新的**——脚本会拒绝复用已有目录，所以上面的路径带时间戳。
两个环境变量是前置门，缺一个脚本会以 exit 2 拒绝。

> Gazebo 用 partition 隔离会话，而抓取脚本要跑 `gz model` 读目标真值位姿，
> 那是 gz-transport、**依赖 partition**。所以启动脚本会把 partition 写进
> `/tmp/cs625_p7_1_partition`，抓取脚本未显式给出 `IGN_PARTITION` 时自动读取，
> 避免"忘了导出 -> 真值位姿为空"这种静默失败。

Until that passes, treat the RGB-D input chain as `NOT ACCEPTED` at this pose and
do not build active-perception results on top of it.  See
[`p7_five_gate_protocol.md`](p7_five_gate_protocol.md).

### 7.5 Known limitations

- The calibration is relative to the FLANGE as measured on 2026-04-01.  If the
  camera bracket is moved, or the controller's TCP is redefined and the camera
  calibration is redone against that TCP instead, these numbers become wrong and
  `scripts/camera_extrinsics.py` must be re-run.
- Gazebo simulates one RGB-D camera, not a stereo pair.  The colour eye exists as a
  TF frame and as the calibrated offset, but the simulated image comes from the
  depth eye's position.
- `camera_mount_xyz` is a flange-frame pose; the insertion scene and the camera
  calibration are independent, so moving the fixture does not affect it.
