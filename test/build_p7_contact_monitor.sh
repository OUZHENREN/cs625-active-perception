#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PKG_CONFIG_PATH="/opt/ros/jazzy/opt/gz_cmake_vendor/lib/pkgconfig:/opt/ros/jazzy/opt/gz_transport_vendor/lib/pkgconfig:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/pkgconfig:/opt/ros/jazzy/opt/gz_utils_vendor/lib/pkgconfig:/opt/ros/jazzy/opt/gz_math_vendor/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
g++ -O2 -std=c++17 "$repo_root/test/p7_gazebo_contact_monitor.cpp" \
  -o /tmp/p7_gazebo_contact_monitor $(pkg-config --cflags --libs gz-transport13 gz-msgs10) -pthread
