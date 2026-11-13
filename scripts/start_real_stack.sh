#!/usr/bin/env bash
# 开工第一步：把真机驱动栈拉起来（并识别"陈旧进程"这种情况）。
#
# 为什么需要它
# ------------
# 原来的开工任务只判断"有没有 ros2 launch 进程"，于是昨天关机后遗留的
# 陈旧驱动栈会让它以为"已在运行"而跳过启动；但那个进程到机器人的控制连接
# （反向控制端口 50001/50003/50004）早就断了，健康检查必然失败。
# 这里改成：进程在但控制端口没有 ESTAB → 判定陈旧，停掉并重启。
#
# 用法：
#   bash scripts/start_real_stack.sh            # 正常开工
#   bash scripts/start_real_stack.sh --check-only   # 只检查，不启动
set -u

cd "$(dirname "$0")/.." || exit 1
source scripts/source_dev_env.sh --full || exit 1
if [ -f "$HOME/.cs625_local.env" ]; then . "$HOME/.cs625_local.env"; fi

ROBOT_IP="${CS625_ROBOT_IP:-}"
LOCAL_IP="${CS625_HOST_IP:-}"
if [ -z "$ROBOT_IP" ]; then
  echo "[开工] CS625_ROBOT_IP 未设置，请写进 ~/.cs625_local.env"
  exit 1
fi
if [ -z "$LOCAL_IP" ]; then
  echo "[开工] CS625_HOST_IP 未设置，请写进 ~/.cs625_local.env"
  exit 1
fi
CHECK_ONLY=0
[ "${1:-}" = "--check-only" ] && CHECK_ONLY=1

sockets_live() {
  # 直接复用 health 判据，避免这里再写一套端口逻辑（两处口径迟早会不一致）
  python3 scripts/control_health.py 2>/dev/null | grep -q "'sockets': True"
}

start_driver() {
  if ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "[开工] 机器人 $ROBOT_IP 可达"
  else
    echo "[开工] 机器人 $ROBOT_IP 不可达——先检查网线/电源/控制器是否上电"
    return 1
  fi
  mkdir -p runtime/logs
  local log="runtime/logs/driver_$(date +%H%M%S).log"
  # setsid：让整棵驱动进程树属于同一个进程组，后面可以整组停掉
  setsid nohup ros2 launch cs625_bringup real_base.launch.py \
    launch_driver:=true robot_ip:="$ROBOT_IP" local_ip:="$LOCAL_IP" \
    description_package:=eli_cs_robot_description \
    description_file:="$PWD/install/cs625_ap_description/share/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro" \
    headless_mode:=true launch_moveit:=true launch_rviz:=false \
    launch_sensor_adapter:=false activate_joint_controller:=false \
    execute:=false require_confirmation:=true >> "$log" 2>&1 &
  echo $! > runtime/driver.pgid
  echo "[开工] 已在后台启动驱动栈（进程组 $(cat runtime/driver.pgid)），日志 $log"
}

kill_tree() {
  # 递归停掉一棵进程树（不用 pkill/通配，避免误伤自己的 shell）
  local parent="$1"
  local child
  for child in $(pgrep -P "$parent" 2>/dev/null); do
    kill_tree "$child"
  done
  kill -TERM "$parent" 2>/dev/null
}

PID=$(pgrep -f "[r]os2 launch cs625_bringup" | head -1)
if [ -n "$PID" ]; then
  if sockets_live; then
    echo "[开工] 驱动栈已在运行，控制连接正常（PID $PID）"
  elif [ "$CHECK_ONLY" = "1" ]; then
    echo "[开工] 驱动栈进程在（PID $PID），但控制端口未建立——陈旧进程，需要重启"
    exit 3
  else
    echo "[开工] 驱动栈进程在（PID $PID），但控制端口未建立 → 判定陈旧，重启"
    # 昨天的教训：只杀 launch 进程会留下孤儿驱动子进程，它们占着端口，
    # 新驱动起来会报 "bind: Address already in use"。所以要整组停 + 兜底清理。
    if [ -f runtime/driver.pgid ]; then
      PGID=$(cat runtime/driver.pgid)
      kill -INT -- "-$PGID" 2>/dev/null
      sleep 8
      kill -TERM -- "-$PGID" 2>/dev/null
      rm -f runtime/driver.pgid
    fi
    kill_tree "$PID"
    sleep 4
    start_driver || exit 1
  fi
else
  [ "$CHECK_ONLY" = "1" ] && { echo "[开工] 驱动栈未运行"; exit 3; }
  start_driver || exit 1
fi

[ "$CHECK_ONLY" = "1" ] && exit 0

echo "[开工] 等待驱动栈起来（约 45 s）…"
sleep 45
python3 scripts/control_health.py --activate
python3 scripts/preflight_motion.py

# 机器人就绪检查：safety 必须 NORMAL；若停在 IDLE/未使能则自动重挂（约 5 s，不再干等）
python3 scripts/ensure_robot_ready.py
READY_CODE=$?
if [ "$READY_CODE" != "0" ]; then
  echo "[开工] 机器人未就绪（code=$READY_CODE），已停止后续动作"
  exit "$READY_CODE"
fi

# 夹爪不是 ROS 驱动（Windows COM9 + 可选常驻桥）；这里只确保"桥在或明确报告不在"，
# 失败不中断开工——测量/观测不需要夹爪，示教/插取件才需要。
bash scripts/gripper_link.sh --ensure || true
