#!/usr/bin/env bash
# Deterministic entry point for a non-motion P7 planning probe.
# ROS setup scripts may read optional unset variables, so enable nounset only
# after the underlays have been sourced.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
set -u

case "${1:?expected generator or filter}" in
  generator)
    exec ros2 run cs625_view_generation candidate_generator --ros-args \
      --params-file "$cs625_app_install/share/cs625_bringup/config/view_planning_sim.yaml"
    ;;
  filter)
    exec ros2 run cs625_motion_adapter reachability_filter --ros-args \
      --params-file "$cs625_app_install/share/cs625_bringup/config/view_planning_sim.yaml"
    ;;
  *)
    echo "unsupported probe role: $1" >&2
    exit 64
    ;;
esac
