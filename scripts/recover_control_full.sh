#!/bin/bash
# FULL controller recovery -- DANGEROUS WITH A PAYLOAD.
# Deactivates and re-activates arm_controller.  This releases the servo briefly;
# on 2026-11-03 the arm sagged 16 mm in Z while carrying a 6.8 kg part.
# Run only when scripts/recover_control.sh is not enough AND the arm carries no
# payload (or the owner explicitly approves it).
# Usage: source_env; bash scripts/recover_control_full.sh
set -u
cd "$(dirname "$0")/.."
bash -c 'source /opt/ros/jazzy/setup.bash; source "$HOME/cs625_legacy_underlay_jazzy/install/setup.bash"'
timeout 20 ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger "{}" >/dev/null 2>&1
sleep 1
timeout 20 ros2 control switch_controllers --controller-manager /eli_ros2_control_node --deactivate arm_controller --strict >/dev/null 2>&1
sleep 1
timeout 20 ros2 control switch_controllers --controller-manager /eli_ros2_control_node --activate arm_controller --strict >/dev/null 2>&1
sleep 1
timeout 15 ros2 service call /io_and_status_controller/set_speed_slider eli_common_interface/srv/SetSpeedSliderFraction "{speed_slider_fraction: 1.0}" >/dev/null 2>&1
echo "control recovered (FULL: controller deactivated and re-activated)"
