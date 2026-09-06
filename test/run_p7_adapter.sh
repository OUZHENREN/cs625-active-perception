#!/usr/bin/env bash
# Start exactly one P7 simulation adapter in the existing Jazzy overlay.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"
p7_params_file="${P7_PARAMS_FILE:-$cs625_app_install/share/cs625_bringup/config/p7_static_grasp_sim.yaml}"

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
