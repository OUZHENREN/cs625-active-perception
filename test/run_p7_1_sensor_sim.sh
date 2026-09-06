#!/usr/bin/env bash
# Launch only the P7.1 eye-in-hand RGB-D gate in the existing Jazzy/Harmonic stack.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IGN_PARTITION="${IGN_PARTITION:-p7_1_sensor_$(date +%Y%m%d_%H%M%S)_$$}"
export GZ_PARTITION="${GZ_PARTITION:-$IGN_PARTITION}"
export CS625_GZ_PARTITION="$IGN_PARTITION"
export ROS2CLI_NO_DAEMON=1
export CS625_P7_ATTACHMENT_ENABLED="${CS625_P7_ATTACHMENT_ENABLED:-false}"
launch_log="${P7_1_LAUNCH_LOG:-/tmp/p7_1_sensor_launch.log}"
initial_positions="${P7_OBSERVATION_INITIAL_POSITIONS:-$cs625_app_install/share/cs625_bringup/config/p7_1_observation_initial_positions.yaml}"
camera_image_width="${P7_CAMERA_IMAGE_WIDTH:-320}"
camera_image_height="${P7_CAMERA_IMAGE_HEIGHT:-240}"
if [[ ! -f "$initial_positions" ]]; then
  echo "P7.1 initial-position profile is not installed: $initial_positions" >&2
  exit 2
fi

default_world_source="$repo_root/src/cs625_simulation/worlds/p7_ycb_tomato_light.sdf"
world_source="${P7_1_WORLD_SOURCE:-${P7_1_DIAGNOSTIC_WORLD_SOURCE:-$default_world_source}}"
world_basename="$(basename "$world_source")"
case "$world_basename" in
  p7_ycb_tomato_light.sdf|p7_ycb_tomato_occlusion_light_v1.sdf|p7_ycb_tomato_occlusion_medium_v1.sdf|p7_ycb_tomato_occlusion_severe_v1.sdf|p7_ycb_tomato_occlusion_severe_v2.sdf|p7_ycb_tomato_occlusion_severe_v3.sdf|p7_ycb_tomato_occlusion_severe_v4.sdf|p7_ycb_tomato_occlusion_severe_v5.sdf)
    ;;
  *)
    if [[ "${CS625_P7_1_DIAGNOSTIC:-}" != "1" ]]; then
      echo "A non-acceptance world requires CS625_P7_1_DIAGNOSTIC=1" >&2
      exit 2
    fi
    ;;
esac
if [[ ! -f "$world_source" ]]; then
  echo "P7.1 world source does not exist: $world_source" >&2
  exit 2
fi
world_link="/tmp/${IGN_PARTITION}_$(basename "$world_source")"
ln -sfn "$world_source" "$world_link"
: >"$launch_log"
launch_pid=""
cleanup() {
  set +e
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# No MoveIt, motion adapter, trajectory command, gripper command or grasp
# process is started.  initial_detach is fixture preparation only: Gazebo's
# DetachableJoint starts attached even though P7.1 must observe a free target.
setsid ros2 launch cs625_bringup sim_active_localization.launch.py \
  strategy:=disabled launch_official_sim:=true launch_moveit:=false \
  launch_fixture_world:=false launch_sensor_adapter:=true \
  camera_enabled:=true headless:=true \
  camera_image_width:="$camera_image_width" \
  camera_image_height:="$camera_image_height" \
  initial_positions_file:="$initial_positions" \
  initial_detach:=true initial_detach_topic:=/p7/attachment/detach \
  world:="$world_link" >>"$launch_log" 2>&1 &
launch_pid=$!

ready=false
for _ in $(seq 1 120); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "P7.1 simulation launch exited before readiness" >&2
    exit 3
  fi
  controller_state="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  nodes="$(timeout 2 ros2 node list 2>/dev/null || true)"
  if grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<<"$controller_state" \
    && grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' <<<"$controller_state" \
    && grep -Fxq '/cs625_sensor_adapter' <<<"$nodes" \
    && [[ "$(ros2 topic type /sensors/camera/color/image 2>/dev/null || true)" == "sensor_msgs/msg/Image" ]] \
    && [[ "$(ros2 topic type /sensors/camera/depth/image 2>/dev/null || true)" == "sensor_msgs/msg/Image" ]] \
    && [[ "$(ros2 topic type /sensors/camera/depth/camera_info 2>/dev/null || true)" == "sensor_msgs/msg/CameraInfo" ]] \
    && [[ "$(ros2 topic type /sensors/camera/points 2>/dev/null || true)" == "sensor_msgs/msg/PointCloud2" ]]; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "$ready" != true ]]; then
  echo "P7.1 controllers and four normalized sensor streams were not ready within 120 s" >&2
  exit 4
fi

echo "P7_1_SIM_READY partition=$IGN_PARTITION ros_domain_id=${ROS_DOMAIN_ID:-default}"
wait "$launch_pid"
