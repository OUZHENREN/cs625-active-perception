#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: source ROS 2 Humble before bootstrapping; ROS_DISTRO=${ROS_DISTRO:-unset}" >&2
  exit 2
fi

if ! command -v lsb_release >/dev/null 2>&1 || [[ "$(lsb_release -rs)" != "22.04" ]]; then
  echo "ERROR: this project baseline requires Ubuntu 22.04 in the approved VM" >&2
  exit 2
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "ERROR: vcs is required before importing .repos files" >&2
  exit 2
fi

if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep is required before resolving dependencies" >&2
  exit 2
fi

echo "P0 bootstrap guard passed for: ${repo_root}"
echo "The .repos files are intentionally empty until upstream revisions are verified."
echo "When entries are pinned, run vcs import and rosdep install from the VM."

