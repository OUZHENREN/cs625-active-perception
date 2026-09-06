#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
# Same verified ray/orientation as E1, at a 0.10 m target-centre offset.
ros2 topic pub --once /p7/arm_motion_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e2-approach\",\"phase\":\"approach\",\"pose\":{\"frame_id\":\"base_link\",\"position\":{\"x\":0.7331,\"y\":-0.0582,\"z\":0.1402},\"orientation\":{\"x\":-0.0961499,\"y\":0.0769224,\"z\":0.7749167,\"w\":0.6199534}}}"}'
