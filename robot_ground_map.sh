#!/usr/bin/env bash
# Read-only current-frame ground/obstacle map, with dedicated preview frame/topics.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/stereo-ground-map.lock"
flock -n 9 || { echo 'Ground map preview already running' >&2; exit 1; }
exec "$workspace/install/wla_uvc_depth/lib/wla_uvc_depth/stereo_ground_map" "$@"
