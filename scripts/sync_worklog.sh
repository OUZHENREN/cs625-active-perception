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
  # The vault usually lives on a WSL-unmounted Windows drive (G:), so the native
  # path above does not exist inside WSL even though the vault is perfectly
  # reachable.  Rather than failing, hand the copy to Windows PowerShell, which
  # reads the file straight out of the distribution (\\wsl.localhost\...).  The
  # Windows path is derived from the same configured value, so there is still one
  # place to edit when the vault moves.
  if command -v powershell.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1 \
     && [[ "${destination_directory}" =~ ^/mnt/([a-zA-Z])/(.*)$ ]]; then
    drive="${BASH_REMATCH[1]^^}"
    rest="${BASH_REMATCH[2]}"
    windows_directory="${drive}:\\${rest//\//\\}"
    windows_source="$(wslpath -w "${source_file}")"
    windows_destination="${windows_directory}\\${base_name}"

    ps1="$(mktemp --suffix=.ps1)"
    trap 'rm -f "${ps1}"' EXIT
    {
      printf '\xef\xbb\xbf'          # BOM: Windows PowerShell 5.1 needs it for UTF-8
      printf '$ErrorActionPreference = "Stop"\n'
      printf '$src = %s\n' "'${windows_source//\'/\'\'}'"
      printf '$dst = %s\n' "'${windows_destination//\'/\'\'}'"
      printf 'if (-not (Test-Path -LiteralPath (Split-Path -Parent $dst))) {\n'
      printf '  Write-Output ("ERROR: Obsidian work-log directory does not exist: " + (Split-Path -Parent $dst))\n'
      printf '  exit 4\n'
      printf '}\n'
      printf 'Copy-Item -LiteralPath $src -Destination $dst -Force\n'
      printf '$a = (Get-FileHash -LiteralPath $src -Algorithm SHA256).Hash\n'
      printf '$b = (Get-FileHash -LiteralPath $dst -Algorithm SHA256).Hash\n'
      printf 'if ($a -ne $b) { Write-Output "ERROR: checksum mismatch after copying"; exit 5 }\n'
      printf 'Write-Output ("synced: " + $dst)\n'
      printf 'Write-Output ("sha256: " + $b)\n'
    } >"${ps1}"

    echo "vault drive is not mounted in WSL; copying through Windows PowerShell"
    if ! powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w "${ps1}")" \
         | tr -d '\r'; then
      die "PowerShell copy of ${base_name} failed"
    fi
    echo "  from ${source_file}"
    echo "  to   ${windows_destination}"
    exit 0
  fi

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
