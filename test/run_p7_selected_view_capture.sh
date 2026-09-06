#!/usr/bin/env bash
# Reproduce r13's selected NBV through MoveIt, real Gazebo motion and RGB-D.
# Requires run_p7_tipfix_sim.sh in the same ROS domain / Gazebo partition.
# Optional second argument --capture-only starts a NEW capture without motion.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
source "${CS625_APP_INSTALL:-$HOME/cs625_p56_install}/setup.bash"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
out="${1:?provide an evidence directory}"
mkdir -p "$out"
if [[ -e "$out/view_trace.json" || -e "$out/post_move_sensor/raw" ]]; then
  echo "Execution evidence already exists: $out/view_trace.json" >&2
  exit 2
fi
export CS625_P7_SIMULATION_EXECUTION=1 CS625_P7_1_SENSOR_GATE=1
if [[ "${2:-}" != --capture-only ]]; then
python3 test/p7_apply_fixture_scene.py --profile severe_v5 --mode full \
  --output "$out/scene_full.json" >"$out/scene_capture.log"
python3 test/p7_execute_view_trace.py \
  --command-id "$(basename "$out")_e75_a22" \
  --position 0.8521671070014576 -0.012368950733672252 0.7270755784023478 \
  --orientation 0.8655549724749436 0.49972839635682736 -0.016478163569807715 -0.02854101651833752 \
  --joint-goal-file src/cs625_bringup/config/p7_3_severe_v5_nbv_r13_e75_vminus110_a22_positions.yaml \
  --contact-monitor-bin /tmp/p7_gazebo_contact_monitor \
  --output "$out/view_trace.json"
fi
python3 test/p7_capture_gazebo_model_pose.py --model target_object \
  --output "$out/post_move_target_ground_truth.json" >"$out/target_capture.log"
python3 test/p7_capture_sensor_gate.py \
  --target-pose-file "$out/post_move_target_ground_truth.json" \
  --output-dir "$out/post_move_sensor/raw" --expected-windows 3 \
  --timeout-sec 45 --sync-slop-sec 0.15 \
  --startup-warmup-sec 2 \
  --minimum-window-separation-sec 0.5 --minimum-target-points 12 \
  >"$out/sensor_capture.log"
echo "P7_POST_MOVE_SENSOR_CAPTURED"
model=src/cs625_simulation/assets/ycb/005_tomato_soup_can/google_16k/textured.obj
texture=src/cs625_simulation/assets/ycb/005_tomato_soup_can/google_16k/texture_map.png
mkdir -p "$out/post_move_pose"
for index in 01 02 03; do
  python3 test/p7_2_estimate_textured_pose.py \
    --window "$out/post_move_sensor/raw/window_$index" \
    --model-obj "$model" --texture "$texture" \
    --output "$out/post_move_pose/window_${index}_estimate.json" \
    --diagnostic-dir "$out/post_move_pose/diagnostics_window_$index" \
    >"$out/post_move_pose/window_$index.log"
  echo "P7_POST_MOVE_POSE_ESTIMATED window=$index"
done
python3 test/p7_2_evaluate_pose_estimates.py \
  --estimate-dir "$out/post_move_pose" \
  --ground-truth "$out/post_move_target_ground_truth.json" \
  --model-obj "$model" --occlusion-stratum severe_v5_post_move \
  --output "$out/post_move_pose/evaluation.json"
