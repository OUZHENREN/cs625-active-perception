#!/bin/bash
# Light controller recovery (2026-11-03 split).
# ONLY re-sends the external_control script and restores the speed slider.
# It NEVER deactivates the controller: a deactivate/activate cycle releases the
# servo for a moment, which let the arm drop 16 mm under a 19 kg payload on
# 2026-11-03.  When this light recovery is not enough, the operator must decide
# whether to run scripts/recover_control_full.sh.
# Usage: source_env; bash scripts/recover_control.sh
set -u
cd "$(dirname "$0")/.."
bash -c 'source /opt/ros/jazzy/setup.bash; source "$HOME/cs625_legacy_underlay_jazzy/install/setup.bash"'
timeout 20 ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger "{}" >/dev/null 2>&1
sleep 1
timeout 15 ros2 service call /io_and_status_controller/set_speed_slider eli_common_interface/srv/SetSpeedSliderFraction "{speed_slider_fraction: 1.0}" >/dev/null 2>&1
echo "control recovered (light: resend + speed slider only)"
