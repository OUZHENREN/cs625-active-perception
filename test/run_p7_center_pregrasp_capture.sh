#!/usr/bin/env bash
# Execute exactly one P7 simulation pregrasp after a read-only path-search
# candidate has been selected.  No gripper, attachment, approach or lift
# command is issued here.  A deliberate environment guard prevents accidental
# use outside the explicitly launched Gazebo fixture.
# ROS 2 Jazzy setup probes optional unset variables, so delay nounset until
# after the setup files have completed.
set -eo pipefail

if [[ "${CS625_P7_SIMULATION_EXECUTION:-}" != "1" ]]; then
  echo "Refusing motion: set CS625_P7_SIMULATION_EXECUTION=1 for the isolated P7 simulation." >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
source "${CS625_APP_INSTALL:-$HOME/cs625_p56_install}/setup.bash"
set -u
export IGN_PARTITION="${IGN_PARTITION:-p7_tipfix_20260829}"
export GZ_PARTITION="${GZ_PARTITION:-$IGN_PARTITION}"

capture_path="${1:-/tmp/p7_e1_center_pregrasp_status.yaml}"
rm -f "$capture_path"
# The receipt is one long JSON string.  Jazzy's default echo truncates long
# strings, which destroys the terminal-joint evidence; request its complete
# first message and exit immediately after the phase finishes.
timeout 245 ros2 topic echo /p7/arm_motion_status std_msgs/msg/String \
  --full-length --once --timeout 240 >"$capture_path" &
monitor_pid=$!
sleep 1
ros2 topic pub --once /p7/arm_motion_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e1-center-pregrasp\",\"phase\":\"pregrasp\",\"pose\":{\"frame_id\":\"base_link\",\"position\":{\"x\":0.7800,\"y\":-0.2600,\"z\":0.3800},\"orientation\":{\"x\":0.6199534,\"y\":0.7749167,\"z\":-0.0769224,\"w\":0.0961499}}}"}'
wait "$monitor_pid" || true
cat "$capture_path"
