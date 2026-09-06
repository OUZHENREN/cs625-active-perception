#!/usr/bin/env bash
# Run one abort-on-failure P7 fixture episode and preserve every gate receipt.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cs625_underlay_setup="${CS625_UNDERLAY_SETUP:-$HOME/cs625_colcon_jazzy/install/setup.bash}"
cs625_app_install="${CS625_APP_INSTALL:-$HOME/cs625_p56_install}"
source "$cs625_underlay_setup"
source "$cs625_app_install/setup.bash"

if [[ "${CS625_P7_SIMULATION_EXECUTION:-}" != "1" ]]; then
  echo "Set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation." >&2
  exit 2
fi
output_dir="${1:?usage: run_p7_static_episode_capture.sh NEW_OUTPUT_DIRECTORY}"
if [[ -e "$output_dir" ]]; then
  echo "Refusing to reuse an evidence directory: $output_dir" >&2
  exit 3
fi
mkdir -p "$output_dir"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
p7_run_id="${P7_RUN_ID:-p7-$(date +%Y%m%d-%H%M%S)}"

initial_positions="$cs625_app_install/share/cs625_bringup/config/p7_safe_initial_positions.yaml"
target_local_x=-0.0091685
target_local_y=0.0840170
target_local_z=0.0510065
pregrasp_x=0.7000
pregrasp_y=-0.3400
pregrasp_z=0.3000
approach_x=0.7108315
approach_y=0.0840170
approach_z=0.0509275
lift_z=0.1709275
qx=0.6199534
qy=0.7749167
qz=-0.0769224
qw=0.0961499

python3 test/p7_capture_manifest.py \
  --repo-root "$repo_root" --output "$output_dir/manifest.json"
python3 test/p7_capture_preflight.py \
  --initial-positions-file "$initial_positions" \
  --output "$output_dir/preflight.json"
python3 test/p7_capture_episode_marker.py \
  --stage start --output "$output_dir/episode_start.json"
python3 test/p7_execute_attachment_capture.py \
  --command-id "${p7_run_id}-e0-detach" --action detach \
  --output "$output_dir/e0_detach.json"
python3 test/p7_apply_fixture_scene.py \
  --mode full --output "$output_dir/scene_full.json"

# E1 is one authoritative pose-goal plan/execute transaction.  Do not run an
# independent IK/path probe here: it can select a different branch and cannot
# certify the trajectory that the adapter will actually execute.
python3 test/p7_execute_pose_capture.py \
  --command-id "${p7_run_id}-e1-pregrasp" --phase pregrasp \
  --position "$pregrasp_x" "$pregrasp_y" "$pregrasp_z" \
  --orientation "$qx" "$qy" "$qz" "$qw" \
  --output "$output_dir/e1_pregrasp.json" --timeout-sec 245

python3 test/p7_apply_fixture_scene.py \
  --mode approach --output "$output_dir/scene_approach.json"
python3 test/p7_execute_pose_capture.py \
  --command-id "${p7_run_id}-e2-approach" --phase approach \
  --position "$approach_x" "$approach_y" "$approach_z" \
  --orientation "$qx" "$qy" "$qz" "$qw" \
  --output "$output_dir/e2_approach.json" --timeout-sec 245

python3 test/p7_capture_gazebo_model_pose.py \
  --model target_object --output "$output_dir/target_at_approach.json"
python3 test/p7_capture_tf_pose.py \
  --parent-frame base_link --child-frame p7_grasp_center_link \
  --output "$output_dir/grasp_at_approach.json"
python3 test/p7_evaluate_geometry_capture.py \
  --model-pose "$output_dir/target_at_approach.json" \
  --grasp-pose "$output_dir/grasp_at_approach.json" \
  --target-local-center-m "$target_local_x" "$target_local_y" "$target_local_z" \
  --threshold-m 0.015 --output "$output_dir/e3_geometry.json"
python3 test/p7_execute_gripper_capture.py \
  --command-id "${p7_run_id}-e3-close" --target-position-m 0.023 \
  --output "$output_dir/e3_gripper.json"
python3 test/p7_execute_attachment_capture.py \
  --command-id "${p7_run_id}-e3-attach" --action attach \
  --output "$output_dir/e3_attachment.json"
python3 test/p7_capture_gazebo_model_pose.py \
  --model target_object --output "$output_dir/target_before_lift.json"
python3 test/p7_capture_tf_pose.py \
  --parent-frame base_link --child-frame p7_grasp_center_link \
  --output "$output_dir/grasp_before_lift.json"
python3 test/p7_apply_fixture_scene.py \
  --mode attached \
  --model-pose-file "$output_dir/target_before_lift.json" \
  --grasp-pose-file "$output_dir/grasp_before_lift.json" \
  --target-local-center-m "$target_local_x" "$target_local_y" "$target_local_z" \
  --output "$output_dir/scene_attached.json"

python3 test/p7_execute_pose_capture.py \
  --command-id "${p7_run_id}-e4-lift" --phase lift \
  --position "$approach_x" "$approach_y" "$lift_z" \
  --orientation "$qx" "$qy" "$qz" "$qw" \
  --output "$output_dir/e4_lift.json" --timeout-sec 245
python3 test/p7_capture_gazebo_model_pose.py \
  --model target_object --output "$output_dir/target_after_lift.json"
python3 test/p7_evaluate_lift_capture.py \
  --before-model-pose "$output_dir/target_before_lift.json" \
  --after-model-pose "$output_dir/target_after_lift.json" \
  --target-local-center-m "$target_local_x" "$target_local_y" "$target_local_z" \
  --minimum-height-m 0.10 --output "$output_dir/e4_lift_measurement.json"
python3 test/p7_capture_hold_trace.py \
  --model target_object --duration-sec 2.2 --sample-period-sec 0.2 \
  --target-local-center-m "$target_local_x" "$target_local_y" "$target_local_z" \
  --max-height-drift-m 0.01 --output "$output_dir/hold_trace.json"
python3 test/p7_capture_episode_marker.py \
  --stage end --output "$output_dir/episode_end.json"

python3 test/p7_assemble_episode.py \
  --single-attempt-runner \
  --preflight "$output_dir/preflight.json" \
  --episode-start "$output_dir/episode_start.json" \
  --detach "$output_dir/e0_detach.json" \
  --scene-full "$output_dir/scene_full.json" \
  --pregrasp "$output_dir/e1_pregrasp.json" \
  --scene-approach "$output_dir/scene_approach.json" \
  --approach "$output_dir/e2_approach.json" \
  --gripper "$output_dir/e3_gripper.json" \
  --geometry "$output_dir/e3_geometry.json" \
  --attachment "$output_dir/e3_attachment.json" \
  --scene-attached "$output_dir/scene_attached.json" \
  --lift "$output_dir/e4_lift.json" \
  --lift-measurement "$output_dir/e4_lift_measurement.json" \
  --hold "$output_dir/hold_trace.json" \
  --episode-end "$output_dir/episode_end.json" \
  --output "$output_dir/episode.json"

for evidence_file in "$output_dir"/*.json "$output_dir"/*.txt; do
  (cd "$output_dir" && sha256sum "$(basename "$evidence_file")")
done >"$output_dir/checksums.sha256"
echo "P7_EPISODE_CAPTURED output=$output_dir"
