#!/usr/bin/env bash
# 收工：关闭这台机器上所有由本项目启动的驱动与调试进程。
#
# 关什么
# ------
#   1. 机械臂驱动栈（ros2 launch cs625_bringup ...，含 move_group / ros2_control / 驱动组件）
#   2. 相机驱动（ros2 launch percipio_camera ...）
#   3. 相机查看器（scripts/camera_view.py）
#
# 怎么关（为什么不是 pkill）
# ------------------------
# 只用两种"可核验"的方式：
#   · 启动时用 setsid 记下的进程组（runtime/driver.pgid），先 SIGINT 再 SIGTERM；
#   · 递归 kill_tree() 停掉指定 PID 的整棵子树。
# 绝不用 pkill/通配，避免误伤别的东西（这个坑昨天刚踩过）。
#
# 用法
# ----
#   bash scripts/stop_all_drivers.sh              # 正常收工
#   bash scripts/stop_all_drivers.sh --dry-run    # 只报告，不动
set -u

cd "$(dirname "$0")/.." || exit 1
DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

kill_tree() {
  local parent="$1" child
  for child in $(pgrep -P "$parent" 2>/dev/null); do
    kill_tree "$child"
  done
  [ "$DRY" = "1" ] || kill -TERM "$parent" 2>/dev/null
}

sweep_orphans() {
  # 兜底：对 SIGTERM 不敏感的组件容器/驱动组件，被 init 收养后仍会占端口/设备。
  # 只处理"父进程是 1（孤儿）+ 名字属于 ROS 组件"的进程，不动别的东西。
  local pid ppid
  for pid in $(pgrep -f "rclcpp_components/component_container|eli_cs_robot_driver/lib" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    ppid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    parent_args=$(ps -o args= -p "${ppid:-0}" 2>/dev/null)
    # WSL 里孤儿会被 /init（PID 647，comm 显示成 "Relay(648)"）收养，
    # 所以统一按父进程的 args 判断，不能只看 ppid==1 或 comm=="init"
    if [ "${ppid:-0}" = "1" ] || [[ "${parent_args:-}" == /init* ]]; then
      echo "[收工] 清理孤儿进程 $pid（$(ps -o comm= -p "$pid" 2>/dev/null)）"
      kill -TERM "$pid" 2>/dev/null
      sleep 2
      kill -KILL "$pid" 2>/dev/null
    fi
  done
}

stopped_any=0

# 1) 机械臂驱动栈：先按进程组，再按 launch 进程兜底
if [ -f runtime/driver.pgid ]; then
  PGID=$(cat runtime/driver.pgid)
  if ps -p "$PGID" >/dev/null 2>&1; then
    echo "[收工] 停止机械臂驱动栈（进程组 $PGID）"
    stopped_any=1
    if [ "$DRY" = "0" ]; then
      kill -INT -- "-$PGID" 2>/dev/null
      sleep 6
      kill -TERM -- "-$PGID" 2>/dev/null
      sleep 2
      rm -f runtime/driver.pgid
    fi
  else
    rm -f runtime/driver.pgid
  fi
fi
ARM_PID=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch cs625_bringup" | head -1)
if [ -n "${ARM_PID:-}" ]; then
  echo "[收工] 停止机械臂驱动栈（launch PID $ARM_PID 及其子进程）"
  stopped_any=1
  [ "$DRY" = "1" ] || { kill_tree "$ARM_PID"; sleep 4; kill -TERM "$ARM_PID" 2>/dev/null; }
fi

# 2) 相机驱动
CAM_PID=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch percipio_camera" | head -1)
if [ -n "${CAM_PID:-}" ]; then
  echo "[收工] 停止相机驱动（launch PID $CAM_PID）"
  stopped_any=1
  # 先子后父：先 kill_tree 再停 launch。反过来会先杀掉 launch，
  # 子进程被 init 收养变成孤儿继续占着相机（2026-11-11 实测踩过）。
  [ "$DRY" = "1" ] || { kill_tree "$CAM_PID"; sleep 4; kill -TERM "$CAM_PID" 2>/dev/null; }
fi

# 3) 夹爪常驻桥（Windows 侧，COM9）——不是 ROS 驱动
if [ -x scripts/gripper_link.sh ]; then
  echo "[收工] 处理夹爪常驻桥"
  [ "$DRY" = "1" ] || bash scripts/gripper_link.sh --stop || true
fi

# 4) 相机查看器
for VIEWER in $(pgrep -f "^python3 scripts/camera_view.py" 2>/dev/null); do
  echo "[收工] 停止相机查看器（PID $VIEWER）"
  stopped_any=1
  [ "$DRY" = "1" ] || kill_tree "$VIEWER"
done

if [ "$DRY" = "1" ]; then
  echo "[收工] --dry-run：以上为将要停止的进程，未执行"
  exit 0
fi

sweep_orphans
sleep 2
echo "[收工] 剩余检查："
LEFT_ARM=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch cs625_bringup" | head -1)
LEFT_CAM=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch percipio_camera" | head -1)
LEFT_VIEW=$(pgrep -f "^python3 scripts/camera_view.py" | head -1)
LEFT_LINK=$(ss -tn 2>/dev/null | grep -cE "192\.168\.1\.(200|120):(30001|30003)")
echo "  机械臂驱动栈: ${LEFT_ARM:-已停止}"
echo "  相机驱动:     ${LEFT_CAM:-已停止}"
echo "  相机查看器:   ${LEFT_VIEW:-已停止}"
echo "  到机器人/相机的连接数: ${LEFT_LINK:-0}"

if [ -z "${LEFT_ARM:-}" ] && [ -z "${LEFT_CAM:-}" ] && [ -z "${LEFT_VIEW:-}" ]; then
  echo "[收工] 全部已关闭"
  exit 0
fi
echo "[收工] 仍有残留，请检查上面的 PID" >&2
exit 1
