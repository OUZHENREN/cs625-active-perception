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
`controller_manager_name` element. The application MoveIt composition also
preserves the senior profile's `octomap_frame=base_link` and 0.02 m resolution;
only the point-cloud source is replaced by the normalized common topic.

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
