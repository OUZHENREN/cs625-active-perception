#!/usr/bin/env bash
# Capture one immutable five-window P7.1 gate record from an already-ready sim.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Single supported environment entry point: ROS Jazzy -> vendor underlay ->
# this repository's installed overlay.
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
cs625_app_install="$CS625_APP_INSTALL"
# Resolve the package share directory through the installed environment rather
# than concatenating a workspace layout by hand.  These scripts used to assume
# "$CS625_APP_INSTALL/share/cs625_bringup", which only holds for a --merge-install
# workspace; this repository builds with the default isolated layout, where the
# path is "$CS625_APP_INSTALL/cs625_bringup/share/cs625_bringup".  `ros2 pkg
# prefix` answers correctly for both.
if ! cs625_bringup_prefix="$(ros2 pkg prefix cs625_bringup 2>/dev/null)"; then
  echo "cs625_bringup is not on the environment; source scripts/source_dev_env.sh first" >&2
  exit 2
fi
cs625_bringup_share="$cs625_bringup_prefix/share/cs625_bringup"
if [[ ! -d "$cs625_bringup_share/config" ]]; then
  echo "cs625_bringup config directory is missing: $cs625_bringup_share/config" >&2
  exit 2
fi
# The installed overlays can clear GZ_PARTITION while leaving IGN_PARTITION.
# Harmonic discovery uses GZ_PARTITION, so restore the isolated session's
# matching value after all setup scripts are sourced.  This is essential for
# the read-only target-pose audit and does not create a new simulator session.
if [[ -z "${GZ_PARTITION:-}" && -n "${IGN_PARTITION:-}" ]]; then
  export GZ_PARTITION="$IGN_PARTITION"
fi

if [[ "${CS625_P7_1_SENSOR_GATE:-}" != "1" ]]; then
  echo "Set CS625_P7_1_SENSOR_GATE=1 only for the isolated P7.1 simulation." >&2
  exit 2
fi
if [[ "${CS625_P7_SIMULATION_EXECUTION:-}" != "1" ]]; then
  echo "Set CS625_P7_SIMULATION_EXECUTION=1 only for the isolated P7 simulation." >&2
  exit 2
fi
output_dir="${1:?usage: run_p7_1_sensor_gate_capture.sh NEW_OUTPUT_DIRECTORY}"
if [[ -e "$output_dir" ]]; then
  echo "Refusing to reuse an evidence directory: $output_dir" >&2
  exit 3
fi
mkdir -p "$output_dir"
cd "$repo_root"
initial_positions="${P7_OBSERVATION_INITIAL_POSITIONS:-$cs625_bringup_share/config/p7_1_observation_initial_positions.yaml}"
expected_positions="${P7_OBSERVATION_EXPECTED_POSITIONS:-$cs625_bringup_share/config/p7_1_observation_settled_positions.yaml}"
discard_initial_windows="${P7_DISCARD_INITIAL_WINDOWS:-0}"

finalized=false
finalize_evidence() {
  if [[ "$finalized" == "true" ]]; then
    return
  fi
  if [[ -n "${P7_1_LAUNCH_LOG:-}" && -f "$P7_1_LAUNCH_LOG" ]]; then
    cp "$P7_1_LAUNCH_LOG" "$output_dir/launch_snapshot.txt"
  fi
  (
    cd "$output_dir"
    find . -type f ! -name checksums.sha256 -print0 \
      | sort -z \
      | xargs -0 sha256sum
  ) >"$output_dir/checksums.sha256"
  finalized=true
}
trap finalize_evidence EXIT

manifest_arguments=(
  --repo-root "$repo_root"
  --output "$output_dir/manifest.json"
)
if [[ -n "${P7_1_WORLD_SOURCE:-}" ]]; then
  manifest_arguments+=(--artifact "${P7_1_WORLD_SOURCE#$repo_root/}")
fi
python3 test/p7_capture_manifest.py "${manifest_arguments[@]}"
python3 test/p7_capture_preflight.py \
  --initial-positions-file "$initial_positions" \
  --expected-positions-file "$expected_positions" \
  --output "$output_dir/preflight.json"
python3 test/p7_capture_gazebo_model_pose.py \
  --model target_object --output "$output_dir/target_ground_truth.json"

set +e
python3 test/p7_capture_sensor_gate.py \
  --target-pose-file "$output_dir/target_ground_truth.json" \
  --output-dir "$output_dir/raw" \
  --expected-windows 5 --timeout-sec 90 \
  --discard-initial-windows "$discard_initial_windows" \
  --sync-slop-sec 0.15 --minimum-window-separation-sec 0.50 \
  --minimum-target-points 12
capture_status=$?
set -e

finalize_evidence

if [[ "$capture_status" -ne 0 ]]; then
  echo "P7_1_SENSOR_GATE_FAILED output=$output_dir status=$capture_status" >&2
  exit "$capture_status"
fi
echo "P7_1_SENSOR_GATE_CAPTURED output=$output_dir"
