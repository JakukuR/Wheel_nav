#!/usr/bin/env bash
# Read-only full-frame JPEG previews; raw camera topics remain unchanged.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
fi
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/calibration-preview.lock"
flock -n 9 || { echo 'Calibration previews are already running' >&2;exit 1; }
pids=()
cleanup() { trap - EXIT INT TERM; for pid in "${pids[@]}";do kill "$pid" 2>/dev/null || true;done;wait || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for eye in left right rgb;do
  if [[ "$eye" == rgb ]];then input=/camera/dicota/color/image_raw;else input="/camera/sunplus/$eye/image_raw";fi
  "$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/kalo_jpeg" --ros-args \
    -r "__node:=calibration_jpeg_$eye" -p "input_topic:=$input" \
    -p "output_topic:=/r680/debug/calibration/$eye/compressed" -p max_rate:=10.0 -p quality:=80 &
  pids+=("$!")
done
wait -n "${pids[@]}"
