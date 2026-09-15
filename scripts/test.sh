#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Load the single supported environment entry point.  Testing runs the
# installed overlay, so the full chain (ROS Jazzy + vendor underlay + this
# repository's install/) is required and a missing setup file fails fast.
# shellcheck source=scripts/source_dev_env.sh
source "${repo_root}/scripts/source_dev_env.sh" --full

cd "${repo_root}"
# Test artifacts stay inside this repository: build/, install/ and log/.
colcon_root="${CS625_COLCON_ROOT:-${repo_root}}"
echo "colcon_artifacts   ${colcon_root}"
colcon \
  --log-base "${colcon_root}/log" \
  test \
  --build-base "${colcon_root}/build" \
  --install-base "${colcon_root}/install" \
  "$@"
colcon test-result \
  --test-result-base "${colcon_root}/build" \
  --verbose
