#!/usr/bin/env bash
set -euo pipefail
# Workspace entry is authoritative. WLA_NAV_WORKSPACE supports relocated workspaces.
workspace="${WLA_NAV_WORKSPACE:-$HOME/ros2_ws}"
entry="$workspace/r680_mapping.sh"
[[ -f "$entry" ]] || { echo "Workspace entry missing: $entry" >&2; exit 1; }
[[ "$(readlink -f -- "$entry")" != "$(readlink -f -- "${BASH_SOURCE[0]}")" ]] || {
  echo 'Recursive workspace entry refused' >&2; exit 1;
}
exec bash "$entry" "$@"
