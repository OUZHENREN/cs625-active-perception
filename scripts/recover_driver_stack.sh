#!/usr/bin/env bash
# Restart the real-machine driver stack and wait until motion is possible again.
#
# Why (owner, 2026-11-07): every protective stop leaves the robot in a state where it
# accepts goals without executing them ("NO_MOTION" with safety=1, robot=7, three ESTAB
# channels and an active controller).  Nothing on this side revives it except a full driver
# restart -- measured six times on 2026-11-05, each time costing a manual restart.
#
# What it does:
#   1. SIGINT the launch process group (verified PID, never a wildcard kill);
#   2. wait for the driver node to disappear;
#   3. relaunch the same command as a detached process, appending to /tmp/cs625_driver.log;
#   4. poll until: driver process alive + arm_controller active + 3 reverse-control channels;
#   5. exit 0 on success, 1 on timeout (with the reason printed).
#
# Usage:
#   scripts/recover_driver_stack.sh [--timeout 120] [--quiet]
#
# Environment overrides:
#   CS625_ROBOT_IP   default 192.168.1.200
#   CS625_HOST_IP    default 192.168.1.100   (the address advertised to the robot)
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TIMEOUT=120
QUIET=0
while [ $# -gt 0 ]; do
  case "$1" in
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --quiet)   QUIET=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROBOT_IP="${CS625_ROBOT_IP:-192.168.1.200}"
HOST_IP="${CS625_HOST_IP:-192.168.1.100}"
LOG=/tmp/cs625_driver.log
say() { [ "$QUIET" -eq 1 ] || echo "$@"; }

reverse_channels() {
  ss -tn 2>/dev/null | grep -cE ':5000[134] ' || true
}

driver_pid() {
  ps -eo pid,cmd | awk '/eli_ros2_control_no[d]e/ {print $1; exit}'
}

launch_pid() {
  ps -eo pid,cmd | awk '/ros2 laun[c]h cs625_bringup/ {print $1; exit}'
}

say "[recover] stopping the driver stack"
pid="$(launch_pid)"
if [ -n "${pid}" ]; then
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    sleep 1
    [ -z "$(driver_pid)" ] && break
  done
fi
if [ -n "$(driver_pid)" ]; then
  say "[recover] driver still alive after SIGINT; sending a second SIGINT"
  kill -INT "$(driver_pid)" 2>/dev/null || true
  sleep 5
fi
if [ -n "$(driver_pid)" ]; then
  say "[recover] FAILED: the driver node will not stop"
  exit 1
fi

say "[recover] relaunching (log: ${LOG})"
# shellcheck disable=SC1091
source "${REPO}/scripts/source_dev_env.sh" --full >/dev/null 2>&1
nohup env CS625_HOST_IP="${HOST_IP}" ros2 launch cs625_bringup real_base.launch.py \
  launch_driver:=true \
  robot_ip:="${ROBOT_IP}" \
  local_ip:="${HOST_IP}" \
  description_package:=eli_cs_robot_description \
  description_file:="${REPO}/install/cs625_ap_description/share/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro" \
  headless_mode:=true launch_moveit:=true launch_rviz:=false \
  launch_sensor_adapter:=false activate_joint_controller:=false \
  execute:=false require_confirmation:=true \
  >>"${LOG}" 2>&1 &
say "[recover] waiting for readiness (up to ${TIMEOUT}s)"

deadline=$(( $(date +%s) + TIMEOUT ))
while [ "$(date +%s)" -lt "${deadline}" ]; do
  sleep 5
  alive="$(driver_pid)"
  channels="$(reverse_channels)"
  controller="$(ros2 control list_controllers --controller-manager /eli_ros2_control_node 2>/dev/null \
                | awk '/arm_controller/ {print $NF; exit}')"
  say "   driver=${alive:-none} channels=${channels}/3 controller=${controller:-?}"
  if [ -n "${alive}" ] && [ "${channels}" -ge 3 ] && [ "${controller}" = "active" ]; then
    say "[recover] READY"
    exit 0
  fi
done

say "[recover] TIMEOUT after ${TIMEOUT}s"
say "   driver=$(driver_pid)"
say "   channels=$(reverse_channels)/3"
say "   If the channels are 0 while the driver is alive, the robot is not dialling back:"
say "   check the pendant (program running) and the Windows firewall for 50001/50003/50004."
exit 1
