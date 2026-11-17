#!/usr/bin/env bash
# Read-only input adapter using the existing follower native cloud frontend.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/stereo-follow-cloud-preview.lock"
flock -n 9 || { echo 'Follow cloud preview already running' >&2; exit 1; }
exec /usr/bin/python3 -m wla_person_follow.frontend_preview "$@"
