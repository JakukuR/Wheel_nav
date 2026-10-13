#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
entry="$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/r680_nav.sh"
[[ -f "$entry" ]] || { echo '请先在 ros2_ws 构建 wla_r680_navigation' >&2; exit 1; }
exec bash "$entry" "$@"
