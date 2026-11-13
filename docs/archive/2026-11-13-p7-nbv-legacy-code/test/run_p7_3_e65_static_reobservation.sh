#!/usr/bin/env bash
# Launch only the FK-screened P7.3 static re-observation fixture.
# This wrapper delegates to the P7.1 sensor-only launcher: it never starts
# MoveIt, the motion adapter, a trajectory client, or a gripper client.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IGN_PARTITION="${IGN_PARTITION:-p7_3_e65_static_$(date +%Y%m%d_%H%M%S)_$$}"
export GZ_PARTITION="${GZ_PARTITION:-$IGN_PARTITION}"
export P7_CAMERA_IMAGE_WIDTH="${P7_CAMERA_IMAGE_WIDTH:-640}"
export P7_CAMERA_IMAGE_HEIGHT="${P7_CAMERA_IMAGE_HEIGHT:-480}"
export P7_OBSERVATION_INITIAL_POSITIONS="${P7_OBSERVATION_INITIAL_POSITIONS:-$repo_root/src/cs625_bringup/config/p7_3_severe_v5_nbv_e65_vminus110_a10_initial_positions.yaml}"
export P7_1_WORLD_SOURCE="${P7_1_WORLD_SOURCE:-$repo_root/src/cs625_simulation/worlds/p7_ycb_tomato_occlusion_severe_v5.sdf}"
export P7_1_LAUNCH_LOG="${P7_1_LAUNCH_LOG:-/tmp/p7_3_e65_static_launch.log}"
exec bash "$repo_root/test/run_p7_1_sensor_sim.sh"
