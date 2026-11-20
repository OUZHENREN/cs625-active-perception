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
# 2026-11-20：滑块按我们的关节速度上限换算（scripts/set_speed_slider.py），
# 不再写死 100%。失控跳变时这是唯一还生效的物理限速。
python3 scripts/set_speed_slider.py || echo "警告：速度滑块未设置成功"
echo "control recovered (FULL: controller deactivated and re-activated)"
