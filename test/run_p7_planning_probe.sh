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
# Resolve the package share directory through the installed environment rather
# than concatenating a workspace layout by hand.  These scripts used to assume
# "$CS625_APP_INSTALL/share/cs625_bringup", which only holds for a --merge-install
# workspace; this repository builds with the default isolated layout, where the
# path is "$CS625_APP_INSTALL/cs625_bringup/share/cs625_bringup".  `ros2 pkg
# prefix` answers correctly for both.
if ! cs625_bringup_prefix="$(ros2 pkg prefix cs625_bringup 2>/dev/null)"; then
  echo "cs625_bringup is not on the environment; source scripts/source_dev_env.sh first" >&2
  exit 2
fi
cs625_bringup_share="$cs625_bringup_prefix/share/cs625_bringup"
if [[ ! -d "$cs625_bringup_share/config" ]]; then
  echo "cs625_bringup config directory is missing: $cs625_bringup_share/config" >&2
  exit 2
fi
set -u

case "${1:?expected generator or filter}" in
  generator)
    exec ros2 run cs625_view_generation candidate_generator --ros-args \
      --params-file "$cs625_bringup_share/config/view_planning_sim.yaml"
    ;;
  filter)
    exec ros2 run cs625_motion_adapter reachability_filter --ros-args \
      --params-file "$cs625_bringup_share/config/view_planning_sim.yaml"
    ;;
  *)
    echo "unsupported probe role: $1" >&2
    exit 64
    ;;
esac
