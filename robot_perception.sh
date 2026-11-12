#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-check}";shift || true
[[ "$operation" == check || "$operation" == run ]] || { echo 'robot_perception.sh check|run [detector arguments]' >&2;exit 2; }
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export WLA_ROS_WS="$workspace" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export WLA_YOLOE_ENGINE="${WLA_YOLOE_ENGINE-$workspace/models/perception/yoloe/new-orin/yoloe-26s-seg-fp16.engine}"
export WLA_REID_ENGINE="${WLA_REID_ENGINE-$workspace/models/perception/reid/new-orin}"
agent="${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}"
export PYTHONPATH="$agent/src${PYTHONPATH:+:$PYTHONPATH}"
nav="$workspace/install/wla_r680_navigation"
args=(--model "$agent/models/perception/yoloe/yoloe-26s-seg.pt"
 --classes-file "$agent/deploy/yoloe_classes.json"
 --text-encoder "$agent/models/perception/yoloe/mobileclip2_b.ts"
 --tracker "$nav/share/wla_r680_navigation/config/perception.botsort.yaml"
 --device 0 --chroma-smoothing
 --color-topic /camera/processed/color/image_raw
 --depth-topic /camera/processed/aligned_depth_to_color/image_raw
 --camera-info-topic /camera/processed/color/camera_info
 --optical-frame dicota_color_optical_frame --base-frame base_Link)
[[ "$operation" != check ]] || args+=(--check-dependencies)
# Detector alone: no Agent task or motion backend.
exec "$workspace/.venv-boxmot/bin/python" "$nav/lib/wla_r680_navigation/nav_yolo_detector" "${args[@]}" "$@"
