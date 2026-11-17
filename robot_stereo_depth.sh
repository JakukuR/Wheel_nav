#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
model="${WLA_STEREO_MODEL:-light_ess}"
args=()
while (($#));do
 if [[ "$1" == --model ]];then
  [[ $# -ge 2 ]] || { echo '--model requires a value' >&2;exit 2; };model="$2";shift 2
 else args+=("$1");shift;fi
done
plugin=()
ess="$workspace/models/dnn_stereo_disparity_v4.1.0_onnx_trt10.16"
case "$model" in
 light_ess) engine="$ess/light_ess_fp16.engine";plugin=(--plugin "$ess/plugins/aarch64/ess_plugins.so");;
 ess) engine="$ess/ess_fp16.engine";plugin=(--plugin "$ess/plugins/aarch64/ess_plugins.so");;
 las2_s|las2_m|lightstereo_s) engine="$workspace/models/stereo_candidates/${model}_fp16.engine";;
 cfast) engine="$workspace/models/depth_comparison/c_fast_stereo_192x320_fp32.engine";;
 *) echo 'Models: light_ess (default), ess, las2_s, las2_m, lightstereo_s, cfast' >&2;exit 2;;
esac
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/uvc-stereo-network.lock"
flock -n 9 || { echo 'A stereo network already owns the output topics' >&2;exit 1; }
for arg in "${args[@]}";do
 if [[ "$arg" == --direct-camera ]];then
  exec 8>"$workspace/.runtime/uvc-sunplus.lock"
  flock -n 8 || { echo 'Sunplus camera already has an owner' >&2;exit 1; }
 fi
done
exec "$workspace/install/wla_uvc_depth/lib/wla_uvc_depth/uvc_stereo_trt" \
 --model "$model" --engine "$engine" "${plugin[@]}" \
 --calibration "${WLA_UVC_CALIBRATION_DIR:-$workspace/config/uvc_rig/calibrated_20261116}/rig.yaml" --max-rate 5 "${args[@]}"
