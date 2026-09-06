#!/usr/bin/env bash
# Non-motion helper for persisting the transient-local reachable-candidate set.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
exec python3 "$(dirname "$0")/p7_capture_reachable_candidates.py"
