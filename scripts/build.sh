#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The Humble setup and an optional verified underlay may read variables that
# are unset. Keep nounset enabled for this script, but disable it while
# sourcing the environments.
underlay_setup="${CS625_UNDERLAY_SETUP:-}"
set +u
source "/opt/ros/humble/setup.bash"
if [[ -n "${underlay_setup}" ]]; then
  if [[ ! -f "${underlay_setup}" ]]; then
    echo "ERROR: CS625_UNDERLAY_SETUP does not exist: ${underlay_setup}" >&2
    exit 2
  fi
  source "${underlay_setup}"
fi
set -u

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: ROS 2 Humble is required after sourcing the underlay" >&2
  exit 2
fi

cd "${repo_root}"
colcon_root="${CS625_COLCON_ROOT:-${HOME}/cs625_colcon}"
mkdir -p "${colcon_root}"
echo "colcon_artifacts   ${colcon_root}"
colcon \
  --log-base "${colcon_root}/log" \
  build \
  --symlink-install \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  "$@"
