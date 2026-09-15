#!/usr/bin/env bash
# Send one deterministic can-width close command in the isolated P7 fixture.
set -eo pipefail
if [[ "${CS625_P7_SIMULATION_EXECUTION:-}" != "1" ]]; then
  echo "Refusing gripper command outside the isolated P7 simulation." >&2
  exit 2
fi
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
ros2 topic pub --once /p7/gripper_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e3-close-can\",\"target_position_m\":0.023}"}'
