#!/usr/bin/env bash
# 收工：关闭这台机器上所有由本项目启动的驱动与调试进程。
#
# 关什么
# ------
#   1. 机械臂驱动栈（ros2 launch cs625_bringup ...，含 move_group / ros2_control / 驱动组件）
#   2. 相机驱动（ros2 launch percipio_camera ...；优先按 runtime/camera.pid 停）
#   3. 相机查看器（scripts/camera_view.py）
#
# 怎么关（为什么不是 pkill）
# ------------------------
# 只用两种"可核验"的方式：
#   · 启动时用 setsid 记下的进程组（runtime/driver.pgid），先 SIGINT 再 SIGTERM；
#   · 递归 kill_tree() 停掉指定 PID 的整棵子树。
# 绝不用 pkill/通配，避免误伤别的东西（这个坑昨天刚踩过）。
#
# 相机有独立的 runtime/camera.pid（由 scripts/camera_mode.py 写）。收工优先用它，
# 并在停完后删除这个 pid 文件；没有 pid 文件时才退回 pgrep，同样只停校验过的
# percipio_camera launch。
#
# 用法
# ----
#   bash scripts/stop_all_drivers.sh              # 正常收工
#   bash scripts/stop_all_drivers.sh --dry-run    # 只报告，不动
set -u

cd "$(dirname "$0")/.." || exit 1
if [ -f "$HOME/.cs625_local.env" ]; then . "$HOME/.cs625_local.env"; fi
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
  local pid ppid parent_args
  # 2026-11-20：以前漏了 move_group。旧栈被停掉后它的 move_group 会被 /init 收养并
  # 继续占着规划服务，新栈再起一个 → 同一时刻两个 move_group，规划/状态互相干扰。
  for pid in $(pgrep -f "rclcpp_components/component_container|eli_cs_robot_driver/lib|moveit_ros_move_group" 2>/dev/null); do
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

is_camera_launch_pid() {
  # A matching command line alone is not enough: a shell/grep that merely
  # contains the string "ros2 launch percipio_camera" would match pgrep -f and
  # must never be killed.  Require the process itself to be the ros2/python
  # launcher, like camera_mode.is_camera_launch() does.
  local pid="$1" comm args
  [ -n "$pid" ] || return 1
  ps -p "$pid" >/dev/null 2>&1 || return 1
  comm=$(ps -o comm= -p "$pid" 2>/dev/null | tr -d '[:space:]')
  case "$comm" in
    ros2|python*) ;;
    *) return 1 ;;
  esac
  args=$(ps -o args= -p "$pid" 2>/dev/null)
  case "$args" in
    *"ros2 launch percipio_camera"*) return 0 ;;
    *) return 1 ;;
  esac
}

camera_launch_pids() {
  # pid 文件优先；只有确实是 percipio_camera launch 的进程才采用。
  # 再合并 pgrep 结果（去重），覆盖"pid 文件没记录但相机确实在跑"的情况。
  local pids="" cand pid
  if [ -f runtime/camera.pid ]; then
    cand=$(cat runtime/camera.pid 2>/dev/null | tr -d '[:space:]')
    if is_camera_launch_pid "$cand"; then
      pids="$cand"
    fi
  fi
  for pid in $(pgrep -f "ros2 launch percipio_camera" 2>/dev/null); do
    is_camera_launch_pid "$pid" || continue
    case " $pids " in
      *" $pid "*) ;;
      *) pids="${pids:+$pids }$pid" ;;
    esac
  done
  printf '%s\n' "$pids"
}

# 1) 机械臂驱动栈：先按进程组，再按 launch 进程兜底
if [ -f runtime/driver.pgid ]; then
  PGID=$(cat runtime/driver.pgid)
  if ps -p "$PGID" >/dev/null 2>&1; then
    echo "[收工] 停止机械臂驱动栈（进程组 $PGID）"
    if [ "$DRY" = "0" ]; then
      kill -INT -- "-$PGID" 2>/dev/null
      sleep 6
      kill -TERM -- "-$PGID" 2>/dev/null
      sleep 2
      rm -f runtime/driver.pgid
    fi
  else
    [ "$DRY" = "1" ] || rm -f runtime/driver.pgid
  fi
fi
ARM_PID=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch cs625_bringup" | head -1)
if [ -n "${ARM_PID:-}" ]; then
  echo "[收工] 停止机械臂驱动栈（launch PID $ARM_PID 及其子进程）"
  [ "$DRY" = "1" ] || { kill_tree "$ARM_PID"; sleep 4; kill -TERM "$ARM_PID" 2>/dev/null; }
fi

# 2) 相机驱动：pid 文件优先，其次 pgrep；只停 cmdline 校验过的 launch
CAMERA_PIDS=$(camera_launch_pids)
if [ -n "$CAMERA_PIDS" ]; then
  # 先子后父：先 kill_tree 再停 launch。反过来会先杀掉 launch，
  # 子进程被 init 收养变成孤儿继续占着相机（2026-11-11 实测踩过）。
  for CAM_PID in $CAMERA_PIDS; do
    echo "[收工] 停止相机驱动（PID $CAM_PID）"
    [ "$DRY" = "1" ] || { kill_tree "$CAM_PID"; sleep 4; kill -TERM "$CAM_PID" 2>/dev/null; }
  done
fi
[ "$DRY" = "1" ] || rm -f runtime/camera.pid

# 3) 夹爪常驻桥（Windows 侧，COM9）——不是 ROS 驱动
if [ -x scripts/gripper_link.sh ]; then
  echo "[收工] 处理夹爪常驻桥"
  [ "$DRY" = "1" ] || bash scripts/gripper_link.sh --stop || true
fi

# 4) 相机查看器
for VIEWER in $(pgrep -f "^python3 scripts/camera_view.py" 2>/dev/null); do
  echo "[收工] 停止相机查看器（PID $VIEWER）"
  [ "$DRY" = "1" ] || kill_tree "$VIEWER"
done

if [ "$DRY" = "1" ]; then
  echo "[收工] --dry-run：以上为将要停止的进程，未执行"
  if [ -f runtime/camera.pid ]; then
    echo "  runtime/camera.pid: 存在（dry-run 未清理，内容 $(cat runtime/camera.pid 2>/dev/null)）"
  else
    echo "  runtime/camera.pid: 不存在"
  fi
  exit 0
fi

sweep_orphans
sleep 2
echo "[收工] 剩余检查："
LEFT_ARM=$(pgrep -f "/opt/ros/jazzy/bin/ros2 launch cs625_bringup" | head -1)
LEFT_CAM=""
for CAM_PID_CAND in $(pgrep -f "ros2 launch percipio_camera" 2>/dev/null); do
  if is_camera_launch_pid "$CAM_PID_CAND"; then
    LEFT_CAM="$CAM_PID_CAND"
    break
  fi
done
LEFT_VIEW=$(pgrep -f "^python3 scripts/camera_view.py" | head -1)
echo "  机械臂驱动栈: ${LEFT_ARM:-已停止}"
echo "  相机驱动:     ${LEFT_CAM:-已停止}"
echo "  相机查看器:   ${LEFT_VIEW:-已停止}"
if [ -f runtime/camera.pid ]; then
  echo "  runtime/camera.pid: 仍在（$(cat runtime/camera.pid 2>/dev/null)）"
else
  echo "  runtime/camera.pid: 已清理"
fi

# 到机器人/相机的连接数：地址来自 ~/.cs625_local.env，禁止硬编码 IP。
ROBOT_IP="${CS625_ROBOT_IP:-}"
CAMERA_IP="${CS625_CAMERA_IP:-}"
if [ -z "$ROBOT_IP" ] && [ -z "$CAMERA_IP" ]; then
  echo "  到机器人/相机的连接数: 未检查（CS625_ROBOT_IP/CS625_CAMERA_IP 未配置）"
else
  HOSTS=""
  for LINK_IP in "$ROBOT_IP" "$CAMERA_IP"; do
    [ -z "$LINK_IP" ] && continue
    HOSTS="${HOSTS:+$HOSTS|}${LINK_IP//./\\.}"
  done
  LEFT_LINK=$(ss -tn 2>/dev/null | grep -cE "(${HOSTS}):(30001|30003)" || true)
  echo "  到机器人/相机的连接数: ${LEFT_LINK:-0}"
fi

if [ -z "${LEFT_ARM:-}" ] && [ -z "${LEFT_CAM:-}" ] && [ -z "${LEFT_VIEW:-}" ] \
   && [ ! -f runtime/camera.pid ]; then
  echo "[收工] 全部已关闭"
  exit 0
fi
echo "[收工] 仍有残留，请检查上面的 PID" >&2
exit 1
