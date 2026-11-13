#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
# Same verified ray/orientation as E1, at a 0.10 m target-centre offset.
ros2 topic pub --once /p7/arm_motion_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e2-approach\",\"phase\":\"approach\",\"pose\":{\"frame_id\":\"base_link\",\"position\":{\"x\":0.7331,\"y\":-0.0582,\"z\":0.1402},\"orientation\":{\"x\":-0.0961499,\"y\":0.0769224,\"z\":0.7749167,\"w\":0.6199534}}}"}'
