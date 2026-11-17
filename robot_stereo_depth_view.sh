#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/stereo-preview.lock"
flock -n 9 || { echo 'Stereo previews already running' >&2;exit 1; }
pids=()
cleanup(){ trap - INT TERM EXIT;for pid in "${pids[@]}";do kill -INT "$pid" 2>/dev/null || true;done;wait || true; }
trap cleanup INT TERM EXIT
jpeg="$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/kalo_jpeg"
for pair in 'depth /camera/stereo/depth/preview /r680/debug/stereo/depth/compressed' 'aligned /camera/stereo/aligned_depth_to_color/preview /r680/debug/stereo/aligned/compressed' 'color /camera/processed/color/image_raw /r680/debug/depth/color/compressed';do
 read -r name input output <<< "$pair"
 "$jpeg" --ros-args -r "__node:=stereo_jpeg_${name}" -p "input_topic:=$input" -p "output_topic:=$output" -p max_rate:=10.0 -p quality:=80 &
 pids+=("$!")
done
wait -n "${pids[@]}"
