#!/usr/bin/env bash
# Run one insertion episode in the task scene and record what happened.
#
# The task owner recorded the module-to-fixture fit as a real constraint rather than
# working around it, so this script is expected to end with the descent failing in
# contact.  That is the evidence the constraint is real; the assembler classifies it
# and refuses to call a contact-free abort a jam.
#
# Every leg is driven by the existing P7 execution CLI, the arms by the P7 gripper
# adapter, and contacts by the P7 contact capture.  Nothing here re-implements motion.
#
# Runtime validation: NOT DONE.  This orchestration has never been executed against a
# live simulator; it reuses validated pieces but the sequencing itself is unproven.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/source_dev_env.sh
source "$repo_root/scripts/source_dev_env.sh" --full
cs625_bringup_prefix="$(ros2 pkg prefix cs625_bringup)"
cs625_bringup_share="$cs625_bringup_prefix/share/cs625_bringup"

# Resolve the sequence from the source checkout first.  package share directories are
# real copies, not symlinks, so a config added since the last build is absent from the
# install tree -- and this script is run from the checkout, where the committed file is
# the authority anyway.  The installed copy is the fallback rather than the default.
sequence_config="$repo_root/src/cs625_bringup/config/cs625_insertion_sequence.yaml"
if [[ ! -f "$sequence_config" ]]; then
  sequence_config="$cs625_bringup_share/config/cs625_insertion_sequence.yaml"
fi
if [[ ! -f "$sequence_config" ]]; then
  echo "ERROR: cs625_insertion_sequence.yaml not found in the source tree or the" >&2
  echo "       installed cs625_bringup share; regenerate it with" >&2
  echo "       python3 scripts/insertion_sequence.py --write" >&2
  exit 2
fi
episode_dir="${INSERTION_EPISODE_DIR:-$HOME/cs625_insertion_episodes/$(date +%Y%m%d_%H%M%S)}"
if [[ -e "$episode_dir" ]]; then
  echo "Refusing to reuse an episode directory: $episode_dir" >&2
  exit 3
fi
mkdir -p "$episode_dir/legs"

# The phase names are the P7 adapter's vocabulary.  The mapping is semantic, not
# incidental: "approach" is the one phase that plans a collision-checked Cartesian
# straight line at 5 mm resolution and rejects a truncated path, which is exactly what
# an insertion leg needs and exactly how a jam will surface.
leg_phase() {
  case "$1" in
    pregrasp_module) echo pregrasp ;;
    grasp_module)    echo approach ;;
    lift_module)     echo lift ;;
    insert_entry)    echo pregrasp ;;
    insert_seated)   echo approach ;;
    release_retreat) echo lift ;;
    *) echo "unmapped leg: $1" >&2; return 1 ;;
  esac
}

echo "deriving the sequence"
python3 - "$sequence_config" "$episode_dir/sequence.json" <<'PY'
import json
import pathlib
import sys

import yaml
from scipy.spatial.transform import Rotation

config, output = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
parameters = yaml.safe_load(config.read_text(encoding="utf-8"))[
    "cs625_insertion_sequence"
]["ros__parameters"]
payload = {"waypoints": {}, "arm_command_m": parameters["arm_command_m"]}
for name, pose in parameters["waypoints_world"].items():
    quaternion = Rotation.from_euler("xyz", pose[3:6]).as_quat()
    payload["waypoints"][name] = {
        "position": [float(v) for v in pose[:3]],
        "orientation": [float(v) for v in quaternion],
    }
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(f"  {len(payload['waypoints'])} waypoints -> {output}")
PY

read_waypoint() {
  python3 - "$episode_dir/sequence.json" "$1" "$2" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
value = payload["waypoints"][sys.argv[2]][sys.argv[3]]
print(" ".join(f"{v:.9f}" for v in value))
PY
}

run_leg() {
  local leg="$1"
  local phase
  phase="$(leg_phase "$leg")"
  local position orientation
  position="$(read_waypoint "$leg" position)"
  orientation="$(read_waypoint "$leg" orientation)"
  echo "leg ${leg} (phase ${phase})"
  # A leg is allowed to fail: for this task that is the expected outcome, and the
  # receipt is the evidence.
  set +e
  python3 "$repo_root/test/p7_execute_pose_capture.py" \
    --command-id "insertion_${leg}_$(date +%s%N)" \
    --phase "$phase" \
    --position $position \
    --orientation $orientation \
    --output "$episode_dir/legs/${leg}.json"
  local status=$?
  set -e
  echo "  exit ${status}"
}

command_arms() {
  local target="$1" label="$2"
  echo "arms ${label} (${target} m)"
  ros2 topic pub --once /p7/gripper_command std_msgs/msg/String \
    "{data: '{\"command_id\": \"insertion_${label}_$(date +%s%N)\", \"target_position_m\": ${target}}}'}" \
    >/dev/null
  sleep 1
}

contacts="$episode_dir/contacts.json"
echo "listening for module contacts"
python3 "$repo_root/test/p7_capture_gazebo_contacts.py" \
  --topic /cs625_task/contacts/module \
  --duration-sec 90 \
  --output "$contacts" >"$episode_dir/contacts.log" 2>&1 &
contacts_pid=$!

run_leg pregrasp_module
run_leg grasp_module
command_arms 0.020 close_arms
run_leg lift_module
run_leg insert_entry
run_leg insert_seated
command_arms 0.0 open_arms
run_leg release_retreat

kill "$contacts_pid" 2>/dev/null || true
wait "$contacts_pid" 2>/dev/null || true

echo "assembling the episode"
set +e
python3 "$repo_root/test/assemble_insertion_episode.py" \
  --leg-dir "$episode_dir/legs" \
  --contacts "$contacts" \
  --sequence "$sequence_config" \
  --output "$episode_dir/episode.json"
status=$?
set -e
echo
echo "episode directory: $episode_dir"
exit "$status"
