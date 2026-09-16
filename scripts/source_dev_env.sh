#!/usr/bin/env bash
# scripts/source_dev_env.sh -- the single supported environment entry point.
#
# This file must be SOURCED, never executed.  It loads, in this fixed order:
#
#   /opt/ros/jazzy/setup.bash
#     -> vendor underlay   ${CS625_UNDERLAY_SETUP:-$HOME/cs625_underlay_jazzy/install/setup.bash}
#     -> project overlay   ${CS625_OVERLAY_SETUP:-<repo>/install/setup.bash}
#
# Usage:
#
#   source scripts/source_dev_env.sh              # full chain: ROS Jazzy + underlay + this repo
#   source scripts/source_dev_env.sh --full       # same as above, explicit (required in scripts)
#   source scripts/source_dev_env.sh --no-overlay # ROS Jazzy + underlay only (first build)
#   source scripts/source_dev_env.sh --verify     # full chain + package resolution assertions
#
# Call sites inside a script MUST pass an explicit mode flag.  Bash makes
# `source <file>` (with no arguments) inherit the calling script's positional
# parameters, so a bare `source` inside a script that takes arguments would have
# this entry point parse those arguments as its own flags.  Always write:
#
#   source "$repo_root/scripts/source_dev_env.sh" --full
#
# Overrides are environment-only; this script never reads or writes user shell
# configuration files and never edits any file:
#
#   CS625_ROS_SETUP       ROS 2 setup file       (default /opt/ros/jazzy/setup.bash)
#   CS625_UNDERLAY_SETUP  vendor underlay setup  (default $HOME/cs625_underlay_jazzy/install/setup.bash)
#   CS625_OVERLAY_SETUP   project overlay setup  (default <repo>/install/setup.bash)
#   ELITE_CS_SDK_PREFIX   Elite CS SDK prefix    (default $HOME/elite-sdk-1.2.0; skipped if absent)
#
# Guarantees:
#   - refuses direct execution (the environment must land in the caller's shell);
#   - fails immediately when ROS_DISTRO is already set to anything but jazzy;
#   - never silently skips a missing setup.bash;
#   - never uses ROS Humble;
#   - never references the legacy pre-Jazzy colcon or temporary install workspaces.
#
# Exit status when sourced: 0 = loaded, 2 = environment contract violation,
# 3 = --verify package resolution failure.

# Refuse direct execution: `bash scripts/source_dev_env.sh` must not pretend to work.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: scripts/source_dev_env.sh must be sourced, not executed." >&2
  echo "       Use: source scripts/source_dev_env.sh [--full|--no-overlay|--verify]" >&2
  exit 2
fi

_cs625_dev_env_main() {
  local want_overlay=1
  local want_verify=0
  local arg
  for arg in "$@"; do
    case "${arg}" in
      --full) want_overlay=1 ;;
      --no-overlay) want_overlay=0 ;;
      --verify) want_verify=1 ;;
      -h|--help)
        echo "Usage: source scripts/source_dev_env.sh [--full|--no-overlay|--verify]"
        echo "  --full        load the complete chain (ROS Jazzy + underlay + this repository)"
        echo "  --no-overlay  load ROS Jazzy + vendor underlay only (first build)"
        echo "  --verify      assert ROS_DISTRO=jazzy and resolve all three packages"
        return 0
        ;;
      *)
        echo "ERROR: unknown argument: ${arg}" >&2
        echo "       (a bare 'source' inside a script inherits that script's arguments; pass an explicit mode)" >&2
        echo "Usage: source scripts/source_dev_env.sh [--full|--no-overlay|--verify]" >&2
        return 2
        ;;
    esac
  done

  if [[ "${want_verify}" == "1" && "${want_overlay}" == "0" ]]; then
    echo "ERROR: --verify asserts the full chain including this repository's overlay," >&2
    echo "       so it cannot be combined with --no-overlay." >&2
    return 2
  fi

  local repo_root
  repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

  local ros_setup="${CS625_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  local underlay_setup="${CS625_UNDERLAY_SETUP:-${HOME}/cs625_underlay_jazzy/install/setup.bash}"
  local overlay_setup="${CS625_OVERLAY_SETUP:-${repo_root}/install/setup.bash}"

  # Distribution guard: never mix ROS distributions inside one sourced shell.
  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
    echo "ERROR: this shell already has ROS_DISTRO=${ROS_DISTRO} sourced." >&2
    echo "       This project only supports ROS 2 Jazzy; start a fresh shell." >&2
    return 2
  fi
  case "${ros_setup}" in
    *humble*)
      echo "ERROR: refusing a Humble ROS setup file: ${ros_setup}" >&2
      return 2
      ;;
  esac

  # No silent skipping: every setup file in the requested chain must exist.
  local setup_file
  for setup_file in "${ros_setup}" "${underlay_setup}"; do
    if [[ ! -f "${setup_file}" ]]; then
      echo "ERROR: required setup file not found: ${setup_file}" >&2
      return 2
    fi
  done
  if [[ "${want_overlay}" == "1" && ! -f "${overlay_setup}" ]]; then
    echo "ERROR: this repository is not built yet; missing ${overlay_setup}" >&2
    echo "       Build first: source scripts/source_dev_env.sh --no-overlay && colcon build --symlink-install" >&2
    return 2
  fi

  local chain_id="${ros_setup}|${underlay_setup}"
  if [[ "${want_overlay}" == "1" ]]; then
    chain_id="${chain_id}|${overlay_setup}"
  fi

  if [[ "${CS625_DEV_ENV_CHAIN:-}" == "${chain_id}" ]]; then
    echo "CS625 dev env already loaded (chain unchanged); not re-sourcing."
  else
    # ROS setup scripts read possibly-unset variables; relax nounset only around them
    # and restore the caller's option afterwards (without tripping the caller's set -e).
    local had_nounset=0
    if [[ $- == *u* ]]; then
      had_nounset=1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${ros_setup}"
    # shellcheck disable=SC1090
    source "${underlay_setup}"
    if [[ "${want_overlay}" == "1" ]]; then
      # shellcheck disable=SC1090
      source "${overlay_setup}"
    fi
    if [[ "${had_nounset}" == "1" ]]; then
      set -u
    fi

    if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
      echo "ERROR: expected ROS_DISTRO=jazzy after sourcing ${ros_setup}," >&2
      echo "       got '${ROS_DISTRO:-unset}'." >&2
      return 2
    fi

    export CS625_ROS_SETUP="${ros_setup}"
    export CS625_UNDERLAY_SETUP="${underlay_setup}"
    export CS625_OVERLAY_SETUP="${overlay_setup}"
    export CS625_APP_INSTALL="${repo_root}/install"
    export CS625_DEV_ENV_CHAIN="${chain_id}"
    export CS625_DEV_ENV_SOURCED=1

    # Real-profile Elite CS SDK.  The underlay driver writes `#include
    # <Elite/...>` and links `elite-cs-series-sdk` by plain library name rather
    # than through the SDK's imported target, so the compiler and the loader
    # both need this prefix on their search paths.  A missing prefix means a
    # simulation-only shell: skip it instead of failing the entry point.
    local elite_sdk_prefix="${ELITE_CS_SDK_PREFIX:-${HOME}/elite-sdk-1.2.0}"
    if [[ -d "${elite_sdk_prefix}" ]]; then
      export CPLUS_INCLUDE_PATH="${elite_sdk_prefix}/include${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}"
      export LIBRARY_PATH="${elite_sdk_prefix}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"
      export LD_LIBRARY_PATH="${elite_sdk_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
      export ELITE_CS_SDK_PREFIX="${elite_sdk_prefix}"
    fi

    echo "CS625 dev env:"
    echo "  ROS_DISTRO ${ROS_DISTRO}"
    echo "  ros        ${ros_setup}"
    echo "  underlay   ${underlay_setup}"
    if [[ "${want_overlay}" == "1" ]]; then
      echo "  overlay    ${overlay_setup}"
    else
      echo "  overlay    (not loaded: --no-overlay)"
    fi
    if [[ -n "${ELITE_CS_SDK_PREFIX:-}" ]]; then
      echo "  elite sdk  ${ELITE_CS_SDK_PREFIX}"
    fi
  fi

  if [[ "${want_verify}" == "1" ]]; then
    if ! command -v ros2 >/dev/null 2>&1; then
      echo "ERROR: ros2 CLI not found after loading the environment" >&2
      return 3
    fi
    local verify_rc=0
    local package
    local prefix
    for package in elite_cs625_moveit_config eli_cs_robot_description cs625_bringup; do
      prefix=""
      prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)" || prefix=""
      if [[ -n "${prefix}" ]]; then
        echo "  verify OK   ${package} -> ${prefix}"
      else
        echo "  verify FAIL ${package} is not resolvable in this environment" >&2
        verify_rc=3
      fi
    done
    if [[ "${verify_rc}" == "0" ]]; then
      echo "  verify OK   full chain is intact (ROS Jazzy + underlay + overlay)"
    fi
    return "${verify_rc}"
  fi

  return 0
}

_cs625_dev_env_main "$@"
_cs625_dev_env_status=$?
unset -f _cs625_dev_env_main
if [[ "${_cs625_dev_env_status}" == "0" ]]; then
  unset _cs625_dev_env_status
  return 0
fi
if [[ "${_cs625_dev_env_status}" == "3" ]]; then
  unset _cs625_dev_env_status
  return 3
fi
unset _cs625_dev_env_status
return 2
