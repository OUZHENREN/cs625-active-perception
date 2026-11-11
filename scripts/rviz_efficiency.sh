#!/usr/bin/env bash
# 效率模式的两个 RViz（与示教模式的 RViz 分开）：
#   --planning  看碰撞场景与规划（机器人模型、planning scene、轨迹）
#   --cloud     看点云（相机点云 / 环境点云，不含规划面板）
#   --both      两个都开
#
# 为什么分开：把"碰撞与规划"和"点云观察"塞进同一个 RViz 会互相拖慢，
# 而这两件事在调试时通常不是同时需要的。
set -u
cd "$(dirname "$0")/.." || exit 1
source scripts/source_dev_env.sh --full >/dev/null 2>&1 || exit 1

PLANNING_RVIZ="src/cs625_bringup/config/cs625_moveit.rviz"
CLOUD_RVIZ="src/cs625_bringup/config/environment_point_cloud.rviz"

open_rviz() {
  local config="$1" title="$2"
  [ -f "$config" ] || { echo "[rviz] 缺少配置 $config"; return 1; }
  setsid nohup rviz2 -d "$config" --title "$title" >/dev/null 2>&1 &
  echo "[rviz] 已打开：$title（$config）"
}

case "${1:---both}" in
  --planning) open_rviz "$PLANNING_RVIZ" "CS625 效率模式 · 碰撞与规划" ;;
  --cloud)    open_rviz "$CLOUD_RVIZ"    "CS625 效率模式 · 点云" ;;
  --both)
    open_rviz "$PLANNING_RVIZ" "CS625 效率模式 · 碰撞与规划"
    sleep 2
    open_rviz "$CLOUD_RVIZ" "CS625 效率模式 · 点云"
    ;;
  *) echo "用法：bash scripts/rviz_efficiency.sh [--planning|--cloud|--both]" >&2; exit 2 ;;
esac
