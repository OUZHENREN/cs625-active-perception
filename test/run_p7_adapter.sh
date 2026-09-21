#!/usr/bin/env bash
# Start exactly one P7 simulation adapter in the existing Jazzy overlay.
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
p7_params_file="${P7_PARAMS_FILE:-$cs625_bringup_share/config/p7_static_grasp_sim.yaml}"

case "${1:?expected arm, gripper, or attachment}" in
  arm)
    arm_args=(--ros-args --params-file "$p7_params_file")
    if [[ -n "${P7_ARM_PHYSICAL_TOOL_FRAME:-}" ]]; then
      arm_args+=(-p "physical_tool_frame:=$P7_ARM_PHYSICAL_TOOL_FRAME")
    fi
    exec ros2 run cs625_motion_adapter p7_arm_motion_adapter "${arm_args[@]}"
    ;;
  gripper) exec ros2 run cs625_task_orchestrator p7_gripper_adapter --ros-args --params-file "$p7_params_file" ;;
  attachment) exec ros2 run cs625_task_orchestrator p7_attachment_adapter --ros-args --params-file "$p7_params_file" ;;
  *) echo "unsupported adapter: $1" >&2; exit 64 ;;
esac
