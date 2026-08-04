#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
failures=0

check_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf 'OK   %-18s %s\n' "${command_name}" "$(command -v "${command_name}")"
  else
    printf 'MISS %-18s\n' "${command_name}"
    failures=$((failures + 1))
  fi
}

if command -v lsb_release >/dev/null 2>&1; then
  ubuntu_version="$(lsb_release -rs)"
  if [[ "${ubuntu_version}" == "22.04" ]]; then
    printf 'OK   %-18s %s\n' "ubuntu" "${ubuntu_version}"
  else
    printf 'FAIL %-18s expected 22.04, got %s\n' "ubuntu" "${ubuntu_version}"
    failures=$((failures + 1))
  fi
else
  printf 'MISS %-18s\n' "lsb_release"
  failures=$((failures + 1))
fi

if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  printf 'OK   %-18s %s\n' "ROS_DISTRO" "${ROS_DISTRO}"
else
  printf 'FAIL %-18s expected humble, got %s\n' "ROS_DISTRO" "${ROS_DISTRO:-unset}"
  failures=$((failures + 1))
fi

for command_name in ros2 colcon rosdep vcs xacro; do
  check_command "${command_name}"
done

printf 'repo_root          %s\n' "${repo_root}"
printf 'failures           %s\n' "${failures}"
exit "${failures}"
