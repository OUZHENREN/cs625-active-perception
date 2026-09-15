#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Load the single supported environment entry point.  A first build uses
# --no-overlay because this repository's own install/ does not exist yet.
# The entry point fails fast when the ROS Jazzy or vendor underlay setup file
# is missing, so no silent fallback to another distribution is possible.
# shellcheck source=scripts/source_dev_env.sh
source "${repo_root}/scripts/source_dev_env.sh" --no-overlay

cd "${repo_root}"
# Build artifacts stay inside this repository: build/, install/ and log/.
colcon_root="${CS625_COLCON_ROOT:-${repo_root}}"
echo "colcon_artifacts   ${colcon_root}"
colcon \
  --log-base "${colcon_root}/log" \
  build \
  --symlink-install \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  "$@"
