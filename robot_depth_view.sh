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
exec 9>"$workspace/.runtime/depth-preview.lock"
flock -n 9 || { echo 'Depth previews are already running' >&2;exit 1; }
pids=()
cleanup() { trap - EXIT INT TERM; for pid in "${pids[@]}";do kill "$pid" 2>/dev/null || true;done;wait || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for eye in color disparity aligned;do
  case "$eye" in
    color) input=/camera/processed/color/image_raw ;;
    disparity) input=/camera/processed/disparity/preview ;;
    aligned) input=/camera/processed/aligned_depth_to_color/preview ;;
  esac
  "$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/kalo_jpeg" --ros-args \
    -r "__node:=depth_jpeg_$eye" -p "input_topic:=$input" \
    -p "output_topic:=/r680/debug/depth/$eye/compressed" -p max_rate:=10.0 -p quality:=80 &
  pids+=("$!")
done
wait -n "${pids[@]}"
