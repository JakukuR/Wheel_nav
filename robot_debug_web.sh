#!/usr/bin/env bash
# Kalo debug page for the new robot: read-only foxglove_bridge (whitelisted topics) + static
# page/WS relay. Top view in base_Link (the body frame used on this robot). No publishing.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-run}";shift || true
[[ "$operation" == run ]] || { echo 'robot_debug_web.sh run [--port 18081]' >&2;exit 2; }
port=18081
[[ "${1:-}" != --port ]] || { port="$2";shift 2; }
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
fi
share="$workspace/install/wla_r680_navigation/share/wla_r680_navigation"
nav="$workspace/install/wla_r680_navigation/lib/wla_r680_navigation"
ros2 run foxglove_bridge foxglove_bridge --ros-args -r __node:=kalo_bridge --params-file "$share/config/kalo_bridge.yaml" &
bridge=$!
trap 'kill $bridge 2>/dev/null || true' EXIT INT TERM
/usr/bin/python3 "$nav/kalo_debug_web" --port "$port" --base-frame base_Link "$@"
