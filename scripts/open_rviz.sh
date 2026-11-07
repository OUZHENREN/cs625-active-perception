#!/usr/bin/env bash
# 打开示教链路配套的 RViz（复用 MoveIt 的参数与显示设置，不自造 3D）。
#
# 关键点：RViz 必须带着 MoveIt 参数启动，否则 RobotModel 找不到 robot_description，
# 场景里只剩一个空坐标系（这正是 `rviz2 -d <config>` 单独启动时的报错）。
# 所以这里走 cs625_bringup 的 teaching_rviz.launch.py：它复用 real_moveit.launch.py，
# 只跳过 move_group，其余参数与正常 MoveIt RViz 完全一致。
#
# 用法：
#   scripts/open_rviz.sh                       # 默认配置（MoveIt 显示 + 点云）
#   scripts/open_rviz.sh --config <file.rviz>  # 指定显示配置
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${REPO}/config/rviz/cs625_teaching.rviz"
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done
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

echo "[rviz] 配置 ${CONFIG}"
echo "[rviz] 通过 teaching_rviz.launch.py 启动（带 MoveIt 参数，跳过 move_group）"
exec ros2 launch cs625_bringup teaching_rviz.launch.py \
  moveit_rviz_config:="${CONFIG}"
