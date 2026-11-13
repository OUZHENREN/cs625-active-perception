#!/usr/bin/env bash
# Non-motion helper for persisting the transient-local reachable-candidate set.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
exec python3 "$(dirname "$0")/p7_capture_reachable_candidates.py"
