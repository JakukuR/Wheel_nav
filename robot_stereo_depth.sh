#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/uvc-stereo-network.lock"
flock -n 9 || { echo 'A stereo network already owns the output topics' >&2;exit 1; }
exec "$workspace/install/wla_uvc_depth/lib/wla_uvc_depth/uvc_stereo_trt" \
 --engine "$workspace/models/depth_comparison/c_fast_stereo_192x320_fp32.engine" \
 --calibration "${WLA_UVC_CALIBRATION_DIR:-$workspace/config/uvc_rig/calibrated_20261116}/rig.yaml" --max-rate 2 "$@"
