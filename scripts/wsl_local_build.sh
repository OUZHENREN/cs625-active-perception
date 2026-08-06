#!/usr/bin/env bash
# Copy elite_ros source tree to WSL ext4, build, test, then launch.
#
# The /mnt/b symlink-install forces Gazebo/Ogre to load SDF, DAE/STL meshes
# and textures across the Windows→WSL filesystem boundary.  Fixture worlds
# (simple geometry) tolerate this; robot worlds (complex meshes) often don't.
# This script copies the source tree to a local ext4 directory and builds
# there, eliminating cross-filesystem IO jitter during rendering init.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_COPY="${CS625_LOCAL_WS:-$HOME/cs625_local_ws}"
UNDERLAY_SETUP="${CS625_UNDERLAY_SETUP:-$HOME/cs625_underlay_humble/install/setup.bash}"
COLCON_ROOT="${CS625_COLCON_ROOT:-$HOME/cs625_colcon}"

# ROS 2 and underlay setup scripts assume variables like AMENT_TRACE_SETUP_FILES
# may be unset at source time.  Temporarily relax nounset around them so the
# scripts don't abort before colcon can run.
source_ros_setup() {
    local setup_file="$1"
    if [[ ! -f "${setup_file}" ]]; then
        echo "ERROR: Setup file not found: ${setup_file}" >&2
        return 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
}

echo "=== 清理旧 Gazebo 进程 ==="
pkill -TERM -f "ign gazebo|gz sim" 2>/dev/null || true
sleep 2
pkill -KILL -f "ign gazebo|gz sim" 2>/dev/null || true

echo "=== 同步源码到 WSL ext4 ==="
mkdir -p "$LOCAL_COPY"
rsync -a --delete --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' --exclude '.pytest_cache' "$REPO_ROOT/" "$LOCAL_COPY/"

echo "=== 构建 ==="
cd "$LOCAL_COPY"
source_ros_setup /opt/ros/humble/setup.bash
if [[ -n "${UNDERLAY_SETUP}" ]] && [[ -f "${UNDERLAY_SETUP}" ]]; then
    source_ros_setup "${UNDERLAY_SETUP}"
fi
bash scripts/build.sh

echo "=== 测试 ==="
source_ros_setup "${COLCON_ROOT}/install/setup.bash"
bash scripts/test.sh --event-handlers console_direct+ || true

echo "=== 静态契约 ==="
python3 test/contract_checks.py

echo "=== GPU 状态 ==="
glxinfo -B 2>/dev/null | grep -iE "Device|renderer|NVIDIA|llvmpipe" || echo "glxinfo not available"

echo "=== 就绪: source ${COLCON_ROOT}/install/setup.bash ==="
