#!/usr/bin/env bash
# Deterministic entry point for a non-motion P7 planning probe.
# ROS setup scripts may read optional unset variables, so enable nounset only
# after the underlays have been sourced.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
cs625_app_install="$CS625_APP_INSTALL"
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
