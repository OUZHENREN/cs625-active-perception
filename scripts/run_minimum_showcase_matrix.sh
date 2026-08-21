#!/usr/bin/env bash
# Run the registered P4 simulation matrix.  This script never starts a real
# robot profile and writes only newly created output directories.
set -eo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <ros-install-setup.bash> <new-output-directory> [--scene ID] [--seed N] [--strategy ID]" >&2
  exit 2
fi

setup_file="$1"
output_directory="$2"
shift 2
scene_filter=""
seed_filter=""
strategy_filter=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) scene_filter="${2:?--scene requires an ID}"; shift 2 ;;
    --seed) seed_filter="${2:?--seed requires an integer}"; shift 2 ;;
    --strategy) strategy_filter="${2:?--strategy requires an ID}"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_directory}/.." && pwd)"
manifest="${project_root}/src/cs625_bringup/config/minimum_showcase_matrix.json"

if [[ ! -f "${setup_file}" ]]; then
  echo "ROS install setup file not found: ${setup_file}" >&2
  exit 2
fi
if [[ -e "${output_directory}" ]]; then
  echo "Output directory already exists; choose a new directory: ${output_directory}" >&2
  exit 2
fi

ros_distro="${ROS_DISTRO:-jazzy}"
source "/opt/ros/${ros_distro}/setup.bash"
source "${setup_file}"
set -u
simulation_share="$(ros2 pkg prefix cs625_simulation)/share/cs625_simulation"
mkdir -p "${output_directory}/episodes" "${output_directory}/logs" "${output_directory}/models"
cp "${manifest}" "${output_directory}/matrix_manifest.json"

cleanup_pids=()
stop_process() {
  local pid="$1"
  # Every per-cell launch is started through setsid below, so its PID is also
  # its process-group ID.  Signal the group instead of just ros2 launch:
  # Gazebo otherwise survives as an orphan and can become a second action
  # server in the next matrix cell using the same ROS domain.
  kill -INT -- "-${pid}" 2>/dev/null || true
  sleep 1
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  for pid in "${cleanup_pids[@]:-}"; do
    stop_process "${pid}"
  done
}
trap cleanup EXIT INT TERM

cell=0
mapfile -t scenes < <(python3 -c 'import json,sys; print("\n".join(json.load(open(sys.argv[1]))["scene_ids"]))' "${manifest}")
mapfile -t seeds < <(python3 -c 'import json,sys; print("\n".join(map(str,json.load(open(sys.argv[1]))["seeds"])))' "${manifest}")
mapfile -t strategies < <(python3 -c 'import json,sys; print("\n".join(json.load(open(sys.argv[1]))["strategies"]))' "${manifest}")
planned_cells=0
for scene in "${scenes[@]}"; do
  [[ -z "${scene_filter}" || "${scene}" == "${scene_filter}" ]] || continue
  for seed in "${seeds[@]}"; do
    [[ -z "${seed_filter}" || "${seed}" == "${seed_filter}" ]] || continue
    for strategy in "${strategies[@]}"; do
      [[ -z "${strategy_filter}" || "${strategy}" == "${strategy_filter}" ]] || continue
      planned_cells=$((planned_cells + 1))
    done
  done
done
if (( planned_cells == 0 )); then
  echo "The requested filters select no registered matrix cell." >&2
  exit 2
fi

for scene in "${scenes[@]}"; do
  [[ -z "${scene_filter}" || "${scene}" == "${scene_filter}" ]] || continue
  # Use the installed package path rather than the Windows-mounted source
  # path.  The latter contains spaces here and GZ Sim otherwise treats it as
  # a Fuel-world argument instead of one SDF filename.
  world="${simulation_share}/worlds/${scene}.sdf"
  [[ -f "${world}" ]] || { echo "Missing scene world: ${world}" >&2; exit 2; }
  for seed in "${seeds[@]}"; do
    [[ -z "${seed_filter}" || "${seed}" == "${seed_filter}" ]] || continue
    for strategy in "${strategies[@]}"; do
      [[ -z "${strategy_filter}" || "${strategy}" == "${strategy_filter}" ]] || continue
      domain=$((70 + cell))
      export ROS_DOMAIN_ID="${domain}"
      label="${scene}_${strategy}_seed${seed}"
      planning_log="${output_directory}/logs/${label}_planning.log"
      loop_log="${output_directory}/logs/${label}_loop.log"
      episode="${output_directory}/episodes/${scene}_${strategy}_seed${seed}_p4_loop.json"
      gazebo_model="${output_directory}/models/${label}.urdf"
      echo "[$((cell + 1))/${planned_cells}] ${label} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"

      # Start P4 before P3.  This avoids depending on transient-local replay,
      # which is unreliable for this WSL DDS setup even though publisher and
      # subscriber QoS profiles are compatible.
      setsid ros2 launch cs625_bringup sim_p4_loop.launch.py \
        execute_sim_motion:=true require_confirmation:=false \
        strategy:="${strategy}" random_seed:="${seed}" scene_id:="${scene}" \
        episode_log_directory:="${output_directory}/episodes" >"${loop_log}" 2>&1 &
      loop_pid=$!
      cleanup_pids=("${loop_pid}")
      sleep 5

      setsid ros2 launch cs625_bringup sim_view_planning.launch.py \
        launch_sim:=true execute:=false require_confirmation:=true headless:=true \
        world:="${world}" gazebo_model_file:="${gazebo_model}" >"${planning_log}" 2>&1 &
      planning_pid=$!
      cleanup_pids=("${planning_pid}" "${loop_pid}")

      # Do not publish the target against an empty startup JointState.  The
      # spawner can transiently return nonzero in this Gazebo/WSL setup even
      # after controller_manager has activated the controller, so query the
      # controller manager itself instead of relying on the spawner wording.
      controller_deadline=$((SECONDS + 180))
      controller_ready=false
      until [[ "${controller_ready}" == true ]]; do
        if ! kill -0 "${planning_pid}" 2>/dev/null; then
          echo "Simulation process exited before controller activation; see ${planning_log}" >&2
          exit 1
        fi
        if grep -q "Successfully switched controllers" "${planning_log}" && \
          ros2 control list_controllers -c /controller_manager 2>/dev/null | \
            grep -Eq '^joint_trajectory_controller[[:space:]].*active'; then
          controller_ready=true
          break
        fi
        if (( SECONDS >= controller_deadline )); then
          echo "Timed out waiting for simulated controller activation in ${label}; see ${planning_log}" >&2
          exit 1
        fi
        sleep 1
      done

      # The synthetic target is part of the P3/P4 perception contract; the
      # actual RGB-D observation after motion still comes from Gazebo.
      ros2 topic pub --once /perception/target_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: base_link}, pose: {position: {x: 0.95, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}" \
        >"${output_directory}/logs/${label}_target.log" 2>&1
      # ROS CLI topic discovery can report a false negative under WSL/DDS even
      # after the transient-local candidate message was published.  The P3
      # node's own completion record is the readiness condition; P4 then
      # receives that retained candidate set on subscription.
      readiness_deadline=$((SECONDS + 120))
      until grep -q "P3 reachability complete" "${planning_log}"; do
        if ! kill -0 "${planning_pid}" 2>/dev/null; then
          echo "P3 planning process exited before ${label} became ready; see ${planning_log}" >&2
          exit 1
        fi
        if (( SECONDS >= readiness_deadline )); then
          echo "Timed out waiting for P3 reachability in ${label}; see ${planning_log}" >&2
          exit 1
        fi
        sleep 1
      done

      deadline=$((SECONDS + 360))
      until [[ -f "${episode}" ]]; do
        if ! kill -0 "${loop_pid}" 2>/dev/null; then
          echo "P4 loop exited before recording ${label}; see ${loop_log}" >&2
          exit 1
        fi
        if (( SECONDS >= deadline )); then
          echo "Timed out waiting for ${label}; preserve logs for inspection." >&2
          exit 1
        fi
        sleep 1
      done
      stop_process "${loop_pid}"
      stop_process "${planning_pid}"
      cleanup_pids=()
      cell=$((cell + 1))
    done
  done
done

if (( planned_cells == 45 )); then
  ros2 run cs625_experiment_tools summarize_p4_matrix \
    --input-directory "${output_directory}/episodes" \
    --manifest "${output_directory}/matrix_manifest.json" \
    --output-directory "${output_directory}/summary"
  echo "Simulation matrix complete: ${output_directory}/summary"
else
  echo "Pilot complete: ${output_directory}/episodes (not eligible for complete-matrix summary)"
fi
