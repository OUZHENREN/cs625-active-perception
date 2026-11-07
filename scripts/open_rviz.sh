#!/usr/bin/env bash
# 打开与示教链路配套的 RViz（直接复用 MoveIt 的显示设置，不自造 3D）。
#
# 显示内容（与 src/cs625_bringup/config/cs625_moveit.rviz 一致，仅新增点云）：
#   MotionPlanning  规划场景、碰撞体、规划轨迹、交互手柄
#   RobotModel      /preview_robot_description
#   TF              全量坐标变换
#   PointCloud2     /sensors/camera/points
#   Marker/MarkerArray  夹爪与调试标记
#
# 用法：scripts/open_rviz.sh [配置文件]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${1:-${REPO}/config/rviz/cs625_teaching.rviz}"
# shellcheck disable=SC1091
source "${REPO}/scripts/source_dev_env.sh" --full >/dev/null 2>&1

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "rviz2 不在环境里：先 source scripts/source_dev_env.sh --full" >&2
  exit 1
fi
if [ ! -f "${CONFIG}" ]; then
  echo "找不到 RViz 配置：${CONFIG}" >&2
  exit 2
fi
echo "[rviz] config=${CONFIG}"
exec rviz2 -d "${CONFIG}"
