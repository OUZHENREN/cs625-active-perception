#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
ros2 topic pub --once /p7/attachment_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e0-detach\",\"action\":\"detach\"}"}'
