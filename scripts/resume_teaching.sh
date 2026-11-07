#!/usr/bin/env bash
# One command for "the run stopped, get me going again where it left off".
#
# Why (owner, 2026-11-07): after a protective stop or an over-torque event two things had to
# happen by hand -- restart the driver stack, then work out which --start-from value to use.
# Both are now automatic:
#   * recover_driver_stack.sh restarts the stack and waits until motion is possible;
#   * teaching_sequence --resume reopens the chain just before the last completed phase
#     (runtime/teaching_run_state.json), so a failure at the grasp comes back at the
#     pre-grasp instead of replaying the observation.
#
# Usage:
#   scripts/resume_teaching.sh              # restart the stack, then resume
#   scripts/resume_teaching.sh --no-restart # only resume (stack already healthy)
#   scripts/resume_teaching.sh -- --stop-after-pre-insert   # extra chain flags after --
#
# Exit code: the chain's exit code, or 1 when the stack could not be brought back.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESTART=1
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-restart) RESTART=0; shift ;;
    --) shift; EXTRA=("$@"); break ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done

STATE="${REPO}/runtime/teaching_run_state.json"
if [ ! -f "${STATE}" ]; then
  echo "[resume] no completed phase recorded (${STATE}); nothing to resume from"
  echo "         start the chain normally with --start-from 1"
  exit 1
fi
echo "[resume] last completed phase:"
sed 's/^/         /' "${STATE}"

if [ "${RESTART}" -eq 1 ]; then
  echo "[resume] restarting the driver stack"
  if ! "${REPO}/scripts/recover_driver_stack.sh" --timeout 120; then
    echo "[resume] FAILED: the driver stack did not come back; not starting the chain"
    exit 1
  fi
else
  echo "[resume] skipping the restart by request"
fi

cd "${REPO}" || exit 1
# shellcheck disable=SC1091
source "${REPO}/scripts/source_dev_env.sh" --full >/dev/null 2>&1

# The controller comes back inactive after a driver restart.
python3 scripts/control_health.py --activate >/dev/null 2>&1 || true

echo "[resume] continuing the teaching chain with --resume"
python3 -u scripts/teaching/teaching_sequence.py --confirm --resume "${EXTRA[@]}"
status=$?
echo "[resume] chain finished with exit code ${status}"
exit "${status}"
