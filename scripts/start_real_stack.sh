#!/usr/bin/env bash
# 开工第一步：把真机驱动栈和相机拉起来（并识别"陈旧进程"这种情况）。
#
# 为什么需要它
# ------------
# 原来的开工任务只判断"有没有 ros2 launch 进程"，于是昨天关机后遗留的
# 陈旧驱动栈会让它以为"已在运行"而跳过启动；但那个进程到机器人的控制连接
# （反向控制端口 50001/50003/50004）早就断了，健康检查必然失败。
# 这里改成：进程在但控制链不健康 → 判定陈旧，停掉并重启。
#
# 相机是独立的两段式启动：驱动在 ~/cs625_camport_ws，由 scripts/camera_mode.py
# 管理（写 runtime/camera.pid）。本脚本默认在机械臂四项门通过后把相机切到
# modeling（1280x960 彩色+深度+点云），因为观测/效率流程需要深度和内参；
# 若 Windows 侧 RVS_AI_Center 正在运行（RVS 与 ROS 相机互斥），则跳过相机并
# 明确打印原因，走示教/RVS 流程时不受影响。
#
# 用法：
#   bash scripts/start_real_stack.sh                 # 正常开工（驱动 + 四项门 + 相机 auto）
#   bash scripts/start_real_stack.sh --check-only    # 只检查，不启动任何东西
#   bash scripts/start_real_stack.sh --camera auto   # 默认：检测到 RVS 就跳过相机
#   bash scripts/start_real_stack.sh --camera on     # 强制尝试启动相机（RVS 占用时也会起）
#   bash scripts/start_real_stack.sh --camera off    # 本次不碰相机（等价 --no-camera）
#   bash scripts/start_real_stack.sh --force-ready   # 上一次运动被中止后，先 --force 重挂机器人
set -u

cd "$(dirname "$0")/.." || exit 1
source scripts/source_dev_env.sh --full || exit 1
if [ -f "$HOME/.cs625_local.env" ]; then . "$HOME/.cs625_local.env"; fi

CHECK_ONLY=0
WITH_CAMERA=1
CAMERA_POLICY=auto
FORCE_READY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --check-only) CHECK_ONLY=1 ;;
    --camera)
      if [ $# -lt 2 ]; then
        echo "[开工] --camera 需要一个值（auto|on|off）"
        exit 2
      fi
      CAMERA_POLICY="$2"
      shift
      ;;
    --camera=*) CAMERA_POLICY="${1#--camera=}" ;;
    --no-camera) WITH_CAMERA=0 ;;
    --force-ready) FORCE_READY=1 ;;
    -h|--help)
      cat <<'USAGE'
用法：
  bash scripts/start_real_stack.sh                 # 正常开工（驱动 + 四项门 + 相机 auto）
  bash scripts/start_real_stack.sh --check-only    # 只检查，不启动任何东西
  bash scripts/start_real_stack.sh --camera auto   # 默认：检测到 RVS 就跳过相机
  bash scripts/start_real_stack.sh --camera on     # 强制尝试启动相机
  bash scripts/start_real_stack.sh --camera off    # 本次不碰相机（等价 --no-camera）
  bash scripts/start_real_stack.sh --force-ready   # 中止后先 --force 重挂机器人
USAGE
      exit 0
      ;;
    *)
      echo "[开工] 未知参数：$1"
      echo "        可用：--check-only --camera auto|on|off --no-camera --force-ready"
      exit 2
      ;;
  esac
  shift
done
case "$CAMERA_POLICY" in
  auto|on|off) ;;
  *)
    echo "[开工] 无效的 --camera 值：$CAMERA_POLICY（可用 auto|on|off）"
    exit 2
    ;;
esac
[ "$CAMERA_POLICY" = "off" ] && WITH_CAMERA=0

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

# 相机地址只用于启动前的 ping 预检；camera_mode.py 自己也会从同一份
# ~/.cs625_local.env 读取，这里读不到时仍交给它去报告。
CAMERA_IP="${CS625_CAMERA_IP:-}"
if [ -z "$CAMERA_IP" ] && [ -f "$HOME/.cs625_local.env" ]; then
  CAMERA_IP=$(sed -n 's/^CS625_CAMERA_IP=//p' "$HOME/.cs625_local.env" | tail -1 | tr -d "\"'")
fi

driver_healthy() {
  # 复用 control_health 的完整判据（sockets + controller + joint_states），
  # 不再只 grep sockets：驱动"半死"时端口仍在，只看 sockets 会误判健康。
  python3 scripts/control_health.py >/dev/null 2>&1
}

camera_ready() {
  # 只读体检：pid/launch 校验 + color/depth/camera_info 发布者 + 1280x960 分辨率。
  python3 scripts/camera_mode.py health --expect modeling >/dev/null 2>&1
}

rvs_busy() {
  # RVS 与 ROS 相机互斥；Windows 侧进程从 WSL 用 tasklist.exe 查。
  command -v tasklist.exe >/dev/null 2>&1 && \
    tasklist.exe 2>/dev/null | grep -qi "RVS_AI_Center"
}

report_camera_check_only() {
  echo "[开工] 相机状态（只检查，不启动）："
  if [ "$WITH_CAMERA" = "0" ]; then
    echo "  未检查（--no-camera）"
    return 0
  fi
  local out rc
  out=$(python3 scripts/camera_mode.py health --expect modeling 2>&1)
  rc=$?
  printf '%s\n' "$out" | sed 's/^/  /'
  case "$rc" in
    0)
      echo "  结论：OK（modeling 1280x960 已就绪）"
      ;;
    1)
      if [ "$CAMERA_POLICY" = "auto" ] && rvs_busy; then
        echo "  结论：跳过（RVS_AI_Center 占用相机；RVS 与 ROS 相机互斥）"
      else
        echo "  结论：未运行（原因：runtime/camera.pid 缺失或进程不在；开工时会自动执行 camera_mode.py modeling）"
      fi
      ;;
    2)
      echo "  结论：未就绪（原因见上面的 health 输出；开工时会重启到 modeling）"
      ;;
    *)
      echo "  结论：未检查（camera_mode health 无法执行，rc=$rc）"
      ;;
  esac
}

start_driver() {
  if ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "[开工] 机器人 $ROBOT_IP 可达"
  else
    echo "[开工] 机器人 $ROBOT_IP 不可达——先检查网线/电源/控制器是否上电"
    return 1
  fi
  mkdir -p runtime/logs
  # 2026-11-20 自愈：起栈前清掉上次运行留下的僵尸 FastDDS 共享内存段。
  # 僵尸段会占住端口，导致新进程建参与者失败 → "节点进程活着、图里却没有发布者"。
  # 清理器只删"没有任何活进程占用"的段（含 open() 打开的 _el 文件，见脚本注释）。
  if [ -f scripts/clean_dds_shm.py ]; then
    python3 scripts/clean_dds_shm.py --clean --threshold 8 || true
  fi
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
  # 2026-11-20：滑块必须按我们的关节速度上限换算，避免失控跳变时全速冲出安全区间。
  sleep 1
  python3 scripts/set_speed_slider.py || echo "[开工] 警告：速度滑块未设置成功"
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
  if driver_healthy; then
    echo "[开工] 驱动栈已在运行，控制链健康（PID $PID）"
    if [ "$CHECK_ONLY" = "1" ]; then
      report_camera_check_only
      exit 0
    fi
  elif [ "$CHECK_ONLY" = "1" ]; then
    echo "[开工] 驱动栈进程在（PID $PID），但控制链不健康："
    python3 scripts/control_health.py 2>&1 | sed 's/^/         /'
    report_camera_check_only
    echo "[开工] 判定为陈旧驱动栈，需要重启（去掉 --check-only 再运行）"
    exit 3
  else
    echo "[开工] 驱动栈进程在（PID $PID），但控制链不健康 → 判定陈旧，重启"
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
  if [ "$CHECK_ONLY" = "1" ]; then
    echo "[开工] 驱动栈未运行"
    report_camera_check_only
    exit 3
  fi
  start_driver || exit 1
fi

echo "[开工] 等待驱动栈起来（约 45 s）…"
sleep 45

# 1) 激活控制器（任何一步失败都必须停在这里，不能"打印失败但仍显示完成"）
python3 scripts/control_health.py --activate || {
  echo "[开工] 控制器激活失败；先处理：python3 scripts/control_health.py --activate"
  exit 3
}

# 2) 让机器人回到 RUNNING（safety 必须 NORMAL；--force 用于上一次运动被中止之后）
echo "[开工] 重挂机器人（safety=NORMAL / robot_mode=RUNNING）…"
if [ "$FORCE_READY" = "1" ]; then
  python3 scripts/ensure_robot_ready.py --force
else
  python3 scripts/ensure_robot_ready.py
fi
READY_CODE=$?
if [ "$READY_CODE" != "0" ]; then
  echo "[开工] 机器人未就绪（code=$READY_CODE），已停止后续动作"
  exit "$READY_CODE"
fi

# 3) 四项门（反控端口 / arm_controller active / safety+robot / 力矩活）
python3 scripts/preflight_motion.py || {
  echo "[开工] 四项门未通过（看上面的 FAIL 行）；不要开始运动"
  echo "        常见修复：python3 scripts/ensure_robot_ready.py --force"
  exit 4
}

# 4) 相机：默认 auto。只有在 modeling 已就绪时复用，否则启动一次。
CAMERA_STATE="未检查"
if [ "$WITH_CAMERA" = "1" ]; then
  camera_ready
  CAMERA_RC=$?
  if [ "$CAMERA_RC" = "0" ]; then
    echo "[开工] 相机已就绪：modeling 1280x960（pid $(cat runtime/camera.pid 2>/dev/null || echo '?')）"
    CAMERA_STATE="OK(modeling)"
  elif [ "$CAMERA_RC" = "1" ] && [ "$CAMERA_POLICY" = "auto" ] && rvs_busy; then
    echo "[开工] 相机未启动：Windows 侧 RVS_AI_Center 正在占用相机（RVS 与 ROS 相机互斥）"
    echo "        今天走示教/RVS 流程：保持现状即可；走观测/效率流程："
    echo "        先在 Windows 关闭 RVS_AI_Center，再运行 python3 scripts/camera_mode.py modeling"
    CAMERA_STATE="跳过(RVS 占用)"
  else
    if [ -z "$CAMERA_IP" ]; then
      echo "[开工] CS625_CAMERA_IP 未设置（写进 ~/.cs625_local.env）；无法启动相机"
      exit 5
    fi
    if ! ping -c1 -W2 "$CAMERA_IP" >/dev/null 2>&1; then
      echo "[开工] 相机不可达：$CAMERA_IP"
      echo "        按顺序检查：相机电源 → 网线/交换机口 → ping -c3 $CAMERA_IP"
      exit 5
    fi
    echo "[开工] 启动相机 modeling（典型 20–30 s，最多等 150 s）…"
    if ! timeout 150 python3 scripts/camera_mode.py modeling; then
      echo "[开工] 相机启动失败。按顺序执行："
      echo "  1) tasklist.exe | grep -i RVS_AI_Center     # 有输出 = 先关 RVS"
      echo "  2) tail -40 runtime/camera.log"
      echo "  3) python3 scripts/camera_mode.py status"
      echo "  4) ls ~/cs625_camport_ws/install/setup.bash  # 缺失就在该工作区 colcon build"
      exit 5
    fi
    if ! camera_ready; then
      python3 scripts/camera_mode.py health --expect modeling 2>&1 | sed 's/^/         /'
      echo "[开工] 相机进程在，但数据链不完整（见上）。"
      echo "        若当前是 fast 档：python3 scripts/camera_mode.py modeling"
      exit 5
    fi
    echo "[开工] 相机已启动：modeling 1280x960（pid $(cat runtime/camera.pid 2>/dev/null || echo '?')）"
    CAMERA_STATE="OK(modeling)"
  fi
else
  CAMERA_STATE="跳过(--no-camera)"
  echo "[开工] 相机：跳过（--no-camera）"
fi

# 夹爪不是 ROS 驱动（Windows COM9 + 可选常驻桥）；这里只确保"桥在或明确报告不在"，
# 失败不中断开工——测量/观测不需要夹爪，示教/插取件才需要。
bash scripts/gripper_link.sh --ensure || true

echo "[开工] 汇总：驱动=OK 四项门=OK 相机=${CAMERA_STATE}"
