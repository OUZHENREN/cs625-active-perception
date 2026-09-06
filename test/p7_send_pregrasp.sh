#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
# Derived from the real-target reachable candidate fibonacci_23. The position
# is 0.20 m from the tomato-can centre on its verified view ray.
ros2 topic pub --once /p7/arm_motion_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e1-pregrasp\",\"phase\":\"pregrasp\",\"pose\":{\"frame_id\":\"base_link\",\"position\":{\"x\":0.7462,\"y\":-0.1164,\"z\":0.2204},\"orientation\":{\"x\":-0.0961499,\"y\":0.0769224,\"z\":0.7749167,\"w\":0.6199534}}}"}'
