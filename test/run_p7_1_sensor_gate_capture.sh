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
# The capture runs from a second terminal, while run_p7_1_sensor_sim.sh holds the
# simulator open in the first.  Gazebo isolates sessions by partition and the
# ground-truth audit below calls `gz model`, which is partition-sensitive, so the
# value has to cross the terminal boundary.  run_p7_1_sensor_sim.sh writes it here;
# taking it automatically is what stops a forgotten export from turning into an
# empty target pose.
if [[ -z "${IGN_PARTITION:-}" ]]; then
  partition_file="${P7_1_PARTITION_FILE:-/tmp/cs625_p7_1_partition}"
  if [[ -f "$partition_file" ]]; then
    IGN_PARTITION="$(<"$partition_file")"
    export IGN_PARTITION
    echo "using the running P7.1 session partition: $IGN_PARTITION"
  fi
fi
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
# gz model talks over gz-transport, which is partition-scoped, and it returns 0
# even when the service call times out -- the only signal is the text.  Left
# unchecked that surfaced as "Gazebo output does not identify model
# 'target_object'", a parser error that says nothing about the real cause, which is
# that this shell is looking at a different partition than the running simulator.
gz_probe="$(timeout 8 gz model --list 2>&1 || true)"
if grep -Eq "timed out|Command failed|No such file" <<<"$gz_probe"; then
  echo "Cannot reach a running simulation through gz-transport." >&2
  echo "  IGN_PARTITION=${IGN_PARTITION:-<unset>}  GZ_PARTITION=${GZ_PARTITION:-<unset>}" >&2
  echo "  gz model said: $(tr -s ' \n' ' ' <<<"$gz_probe")" >&2
  echo "  A simulator started before this check existed does not record its" >&2
  echo "  partition, so restart it (run_p7_1_sensor_sim.sh now writes" >&2
  echo "  ${P7_1_PARTITION_FILE:-/tmp/cs625_p7_1_partition}), or export the" >&2
  echo "  partition printed in its P7_1_SIM_READY line." >&2
  exit 4
fi
if ! grep -qw "target_object" <<<"$gz_probe"; then
  echo "The running simulation has no model named target_object; gz model listed:" >&2
  tr -s ' \n' ' ' <<<"$gz_probe" >&2
  echo >&2
  exit 4
fi

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
