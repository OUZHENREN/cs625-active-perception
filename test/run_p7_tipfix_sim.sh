#!/usr/bin/env bash
# Launch the existing Jazzy/Harmonic P7 fixture for a tool-tip alignment audit.
# This starts no real hardware and retains the caller-provided Gazebo partition.
# ROS 2 Jazzy setup scripts intentionally probe optional unset environment
# variables, so nounset must not be enabled before sourcing them.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Keep all Gazebo / ROS bridge processes in the same isolated fixture.
export IGN_PARTITION="${IGN_PARTITION:-p7_tipfix_$(date +%Y%m%d_%H%M%S)_$$}"
export GZ_PARTITION="${GZ_PARTITION:-$IGN_PARTITION}"
export ROS2CLI_NO_DAEMON=1
# sim_base.launch.py owns the final partition selection.  Propagate the
# fixture identifier through its explicit override as well, otherwise an
# isolated P7 run can silently join a time-derived partition.
export CS625_GZ_PARTITION="$IGN_PARTITION"
launch_log="${P7_LAUNCH_LOG:-/tmp/p7_tipfix_launch.log}"
p7_initial_positions="${P7_OBSERVATION_INITIAL_POSITIONS:-$cs625_app_install/share/cs625_bringup/config/p7_safe_initial_positions.yaml}"
camera_image_width="${P7_CAMERA_IMAGE_WIDTH:-640}"
camera_image_height="${P7_CAMERA_IMAGE_HEIGHT:-480}"
if [[ ! -f "$p7_initial_positions" ]]; then
  echo "P7 initial-position profile does not exist: $p7_initial_positions" >&2
  exit 2
fi
# ros_gz_sim expands ``gz_args`` through a shell-like command string.  A
# workspace mounted below a Windows directory with spaces therefore reaches
# `gz sim` as multiple argv entries and the server exits before any robot is
# spawned.  Retain the tracked source world but expose it through a no-space
# per-partition symlink for this simulation-only fixture.
world_source="${P7_WORLD_SOURCE:-$repo_root/src/cs625_simulation/worlds/p7_ycb_tomato_light.sdf}"
if [[ ! -f "$world_source" ]]; then
  echo "P7 world does not exist: $world_source" >&2
  exit 2
fi
world_link="/tmp/${IGN_PARTITION}_$(basename "$world_source")"
ln -sfn "$world_source" "$world_link"
p7_initial_detach="${P7_INITIAL_DETACH:-true}"
if [[ "$p7_initial_detach" != true && "$p7_initial_detach" != false ]]; then
  echo "P7_INITIAL_DETACH must be true or false" >&2
  exit 2
fi
p7_require_sensor_streams="${P7_REQUIRE_SENSOR_STREAMS:-true}"
if [[ "$p7_require_sensor_streams" != true && "$p7_require_sensor_streams" != false ]]; then
  echo "P7_REQUIRE_SENSOR_STREAMS must be true or false" >&2
  exit 2
fi
p7_launch_sensor_adapter="${P7_LAUNCH_SENSOR_ADAPTER:-true}"
if [[ "$p7_launch_sensor_adapter" != true && "$p7_launch_sensor_adapter" != false ]]; then
  echo "P7_LAUNCH_SENSOR_ADAPTER must be true or false" >&2
  exit 2
fi
if [[ "$p7_require_sensor_streams" == true && "$p7_launch_sensor_adapter" != true ]]; then
  echo "P7_REQUIRE_SENSOR_STREAMS=true requires P7_LAUNCH_SENSOR_ADAPTER=true" >&2
  exit 2
fi
# P7 needs one deterministic robot description owner.  Start control without
# MoveIt, wait until all three controllers are explicitly active, and only
# then start MoveIt.  This removes the old absolute TimerAction race in which
# MoveIt could publish a mock-hardware robot_description before Gazebo had
# initialized its GazeboSimSystem resource manager.
: >"$launch_log"
control_pid=""
moveit_pid=""
arm_adapter_pid=""
gripper_adapter_pid=""
attachment_adapter_pid=""
cleanup() {
  set +e
  if [[ -n "$attachment_adapter_pid" ]]; then
    kill -INT -- "-$attachment_adapter_pid" 2>/dev/null
  fi
  if [[ -n "$gripper_adapter_pid" ]]; then
    kill -INT -- "-$gripper_adapter_pid" 2>/dev/null
  fi
  if [[ -n "$arm_adapter_pid" ]]; then
    kill -INT -- "-$arm_adapter_pid" 2>/dev/null
  fi
  if [[ -n "$moveit_pid" ]]; then
    kill -INT -- "-$moveit_pid" 2>/dev/null
  fi
  if [[ -n "$control_pid" ]]; then
    kill -INT -- "-$control_pid" 2>/dev/null
  fi
  [[ -z "$attachment_adapter_pid" ]] || wait "$attachment_adapter_pid" 2>/dev/null
  [[ -z "$gripper_adapter_pid" ]] || wait "$gripper_adapter_pid" 2>/dev/null
  [[ -z "$arm_adapter_pid" ]] || wait "$arm_adapter_pid" 2>/dev/null
  [[ -z "$moveit_pid" ]] || wait "$moveit_pid" 2>/dev/null
  [[ -z "$control_pid" ]] || wait "$control_pid" 2>/dev/null
}
trap cleanup EXIT INT TERM

setsid ros2 launch cs625_bringup sim_active_localization.launch.py \
  strategy:=disabled launch_official_sim:=true launch_moveit:=false \
  launch_fixture_world:=false launch_sensor_adapter:="$p7_launch_sensor_adapter" \
  camera_enabled:=true camera_image_width:="$camera_image_width" \
  camera_image_height:="$camera_image_height" headless:=true \
  initial_positions_file:="$p7_initial_positions" \
  initial_detach:="$p7_initial_detach" initial_detach_topic:=/p7/attachment/detach \
  world:="$world_link" >>"$launch_log" 2>&1 &
control_pid=$!

controllers_ready=false
for _ in $(seq 1 60); do
  if ! kill -0 "$control_pid" 2>/dev/null; then
    echo "P7 control launch exited before readiness" >&2
    exit 3
  fi
  controller_state="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  if grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<<"$controller_state" \
    && grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' <<<"$controller_state" \
    && grep -Eq '^gripper_controller[[:space:]].*[[:space:]]active$' <<<"$controller_state"; then
    controllers_ready=true
    break
  fi
  sleep 1
done
if [[ "$controllers_ready" != true ]]; then
  echo "P7 controllers did not become active within 60 s" >&2
  exit 4
fi

setsid ros2 launch cs625_bringup sim_moveit.launch.py \
  cs_type:=cs625 use_fake_hardware:=true launch_rviz:=false \
  description_package:=cs625_ap_description \
  description_file:=cs625_active_perception.urdf.xacro \
  initial_positions_file:="$p7_initial_positions" \
  semantic_package:=cs625_bringup \
  semantic_file:=config/cs625_active_perception.srdf \
  >>"$launch_log" 2>&1 &
moveit_pid=$!

moveit_ready=false
for _ in $(seq 1 40); do
  if ! kill -0 "$moveit_pid" 2>/dev/null; then
    echo "P7 MoveIt launch exited before readiness" >&2
    exit 5
  fi
  if [[ "$(ros2 service type /compute_ik 2>/dev/null || true)" == "moveit_msgs/srv/GetPositionIK" ]] \
    && [[ "$(ros2 service type /compute_cartesian_path 2>/dev/null || true)" == "moveit_msgs/srv/GetCartesianPath" ]]; then
    moveit_ready=true
    break
  fi
  sleep 1
done
if [[ "$moveit_ready" != true ]]; then
  echo "P7 MoveIt services did not become ready within 40 s" >&2
  exit 6
fi

setsid bash test/run_p7_adapter.sh arm >>"$launch_log" 2>&1 &
arm_adapter_pid=$!
setsid bash test/run_p7_adapter.sh gripper >>"$launch_log" 2>&1 &
gripper_adapter_pid=$!
setsid bash test/run_p7_adapter.sh attachment >>"$launch_log" 2>&1 &
attachment_adapter_pid=$!

adapters_ready=false
for _ in $(seq 1 30); do
  for adapter_pid in "$arm_adapter_pid" "$gripper_adapter_pid" "$attachment_adapter_pid"; do
    if ! kill -0 "$adapter_pid" 2>/dev/null; then
      echo "A P7 adapter exited before readiness" >&2
      exit 7
    fi
  done
  adapter_nodes="$(timeout 2 ros2 node list 2>/dev/null || true)"
  if grep -Fxq '/cs625_p7_arm_motion_adapter' <<<"$adapter_nodes" \
    && grep -Fxq '/cs625_p7_gripper_adapter' <<<"$adapter_nodes" \
    && grep -Fxq '/cs625_p7_attachment_adapter' <<<"$adapter_nodes"; then
    adapters_ready=true
    break
  fi
  sleep 1
done
if [[ "$adapters_ready" != true ]]; then
  echo "P7 adapters did not become ready within 30 s" >&2
  exit 8
fi

if [[ "$p7_require_sensor_streams" == true ]]; then
  sensor_ready=false
  for _ in $(seq 1 60); do
    sensor_nodes="$(timeout 2 ros2 node list 2>/dev/null || true)"
    if grep -Fxq '/cs625_sensor_adapter' <<<"$sensor_nodes" \
      && [[ "$(ros2 topic type /sensors/camera/color/image 2>/dev/null || true)" == "sensor_msgs/msg/Image" ]] \
      && [[ "$(ros2 topic type /sensors/camera/depth/image 2>/dev/null || true)" == "sensor_msgs/msg/Image" ]] \
      && [[ "$(ros2 topic type /sensors/camera/depth/camera_info 2>/dev/null || true)" == "sensor_msgs/msg/CameraInfo" ]] \
      && [[ "$(ros2 topic type /sensors/camera/points 2>/dev/null || true)" == "sensor_msgs/msg/PointCloud2" ]]; then
      sensor_ready=true
      break
    fi
    sleep 1
  done
  if [[ "$sensor_ready" != true ]]; then
    echo "P7 normalized RGB-D streams did not become ready within 60 s" >&2
    exit 9
  fi
fi

echo "P7_SIM_READY partition=$IGN_PARTITION ros_domain_id=${ROS_DOMAIN_ID:-default}"
wait -n "$control_pid" "$moveit_pid" "$arm_adapter_pid" "$gripper_adapter_pid" "$attachment_adapter_pid"
