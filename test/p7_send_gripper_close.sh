#!/usr/bin/env bash
# Send one deterministic can-width close command in the isolated P7 fixture.
set -eo pipefail
if [[ "${CS625_P7_SIMULATION_EXECUTION:-}" != "1" ]]; then
  echo "Refusing gripper command outside the isolated P7 simulation." >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
source "${CS625_APP_INSTALL:-$HOME/cs625_p56_install}/setup.bash"
ros2 topic pub --once /p7/gripper_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e3-close-can\",\"target_position_m\":0.023}"}'
