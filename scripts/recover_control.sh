#!/bin/bash
# Light controller recovery (2026-11-03 split).
# ONLY re-sends the external_control script and restores the speed slider.
# It NEVER deactivates the controller: a deactivate/activate cycle releases the
# servo for a moment, which let the arm drop 16 mm under a 6.8 kg payload on
# 2026-11-03.  When this light recovery is not enough, the operator must decide
# whether to run scripts/recover_control_full.sh.
# Usage: source_env; bash scripts/recover_control.sh
set -u
cd "$(dirname "$0")/.."
bash -c 'source /opt/ros/jazzy/setup.bash; source "$HOME/cs625_legacy_underlay_jazzy/install/setup.bash"'
timeout 20 ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger "{}" >/dev/null 2>&1
sleep 1
# 2026-11-20：以前这里设为 1.0（100%）。当"下发目标"与"实际位置"出现跳变时，
# 机器人自己的伺服会按滑块速度全速追过去——我们治理的 2.86°/s 对它无效。
# 现在默认 30%：仍然远高于正常轨迹速度（2.9°/s ≈ 1.6% 额定），但把失控跳变
# 的速度上限压到原来的 3 成。需要更快时用 CS625_SPEED_SLIDER 环境变量覆盖。
SLIDER="${CS625_SPEED_SLIDER:-0.3}"
timeout 15 ros2 service call /io_and_status_controller/set_speed_slider eli_common_interface/srv/SetSpeedSliderFraction "{speed_slider_fraction: $SLIDER}" >/dev/null 2>&1
echo "speed slider = $SLIDER"
echo "control recovered (light: resend + speed slider only)"
