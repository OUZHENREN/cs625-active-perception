#!/usr/bin/env bash
# Mirror a tracked work log into the personal Obsidian vault.
#
# Usage:
#   scripts/sync_worklog.sh docs/worklogs/2026-09-16_WORKLOG_<TOPIC>.md
#
# Destination resolution order:
#   1. $CS625_OBSIDIAN_WORKLOG_DIR from the environment;
#   2. CS625_OBSIDIAN_WORKLOG_DIR from ${HOME}/.cs625_local.env;
#   3. otherwise fail with instructions.
#
# The vault location is machine-local configuration and is deliberately not
# stored in this repository, for the same reason as the robot address.  When the
# vault drive is not mounted, this script fails loudly instead of silently
# skipping the mirror step.
set -euo pipefail

readonly env_file="${HOME}/.cs625_local.env"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <work-log-file>" >&2
  echo "  e.g. $0 docs/worklogs/2026-09-16_WORKLOG_GAZEBO_VISUALS_AND_SIM_REAL_BASES.md" >&2
  exit 2
fi

source_file="$1"
[[ -f "${source_file}" ]] || die "work log not found: ${source_file}"

base_name="$(basename "${source_file}")"
if [[ ! "${base_name}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}_ ]]; then
  die "work-log filename must start with YYYY-MM-DD_: ${base_name}"
fi

# The local env file only holds machine-local values; loading it is best effort.
if [[ -z "${CS625_OBSIDIAN_WORKLOG_DIR:-}" && -f "${env_file}" ]]; then
  # shellcheck disable=SC1090
  . "${env_file}"
fi

destination_directory="${CS625_OBSIDIAN_WORKLOG_DIR:-}"
if [[ -z "${destination_directory}" ]]; then
  {
    echo "ERROR: CS625_OBSIDIAN_WORKLOG_DIR is not set."
    echo "       Add this line to ${env_file}:"
    echo "         CS625_OBSIDIAN_WORKLOG_DIR=\"<vault work-log directory>\""
  } >&2
  exit 3
fi

if [[ ! -d "${destination_directory}" ]]; then
  {
    echo "ERROR: Obsidian work-log directory does not exist:"
    echo "       ${destination_directory}"
    echo "       Mount the vault drive first, then re-run this script."
  } >&2
  exit 4
fi

cp "${source_file}" "${destination_directory}/${base_name}"

if command -v sha256sum >/dev/null 2>&1; then
  source_hash="$(sha256sum < "${source_file}" | cut -d' ' -f1)"
  target_hash="$(sha256sum < "${destination_directory}/${base_name}" | cut -d' ' -f1)"
  if [[ "${source_hash}" != "${target_hash}" ]]; then
    die "checksum mismatch after copying ${base_name}"
  fi
fi

echo "synced work log:"
echo "  from ${source_file}"
echo "  to   ${destination_directory}/${base_name}"
