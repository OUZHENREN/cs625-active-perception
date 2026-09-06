#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
ros2 topic pub --once /p7/attachment_command std_msgs/msg/String \
  '{data: "{\"command_id\":\"p7-e0-detach\",\"action\":\"detach\"}"}'
