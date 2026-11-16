#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-probe}";shift || true
sunplus=/dev/v4l/by-id/usb-SunplusIT_Inc_USB_Global_Camera-video-index0
dicota=/dev/v4l/by-id/usb-DHZJ-260305-J_DICOTA_4K_01.00.00-video-index0
if [[ "$operation" == probe ]]; then
  v4l2-ctl --list-devices
  for device in "$sunplus" "$dicota";do [[ ! -e "$device" ]] || v4l2-ctl --device="$device" --list-formats-ext;done
  exit 0
fi
if [[ "$operation" == pipeline ]]; then
  source /opt/ros/jazzy/setup.bash
  source "$workspace/install/setup.bash"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
  fi
  mkdir -p "$workspace/.runtime"
  exec 9>"$workspace/.runtime/uvc-pipeline.lock"
  flock -n 9 || { echo 'A processed camera pipeline already owns the output topics' >&2;exit 1; }
  args=()
  [[ -z "${WLA_UVC_CALIBRATION_DIR:-}" ]] || args+=(--calibration "$WLA_UVC_CALIBRATION_DIR/rig.yaml")
  exec ros2 run wla_uvc_camera uvc_depth "${args[@]}" "$@"
fi
if [[ "$operation" == run && "${1:-}" == both ]]; then
  shift
  source /opt/ros/jazzy/setup.bash
  source "$workspace/install/setup.bash"
  exec "$workspace/install/wla_uvc_camera/lib/wla_uvc_camera/uvc_cameras" --workspace "$workspace" "$@"
fi
[[ "$operation" == run && $# -ge 1 ]] || { echo 'robot_camera.sh probe | run sunplus|dicota|both [camera arguments] | pipeline [depth arguments]' >&2;exit 2; }
which="$1";shift
case "$which" in
  sunplus) args=(--name sunplus --prefix /camera/sunplus --device "$sunplus" --layout side_by_side --swap-eyes --left-rotation 180 --right-rotation 180 --width 2560 --height 720 --capture-fps 60 --publish-fps 30) ;;
  dicota) args=(--name dicota --prefix /camera/dicota --device "$dicota" --layout mono --width 1920 --height 1080 --capture-fps 30 --publish-fps 30) ;;
  *) echo 'Choose sunplus or dicota' >&2;exit 2 ;;
esac
if [[ -n "${WLA_UVC_CALIBRATION_DIR:-}" ]]; then
  if [[ "$which" == sunplus ]]; then
    args+=(--left-calibration "$WLA_UVC_CALIBRATION_DIR/sunplus_left.yaml" --right-calibration "$WLA_UVC_CALIBRATION_DIR/sunplus_right.yaml")
  else
    args+=(--left-calibration "$WLA_UVC_CALIBRATION_DIR/dicota.yaml")
  fi
fi
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
fi
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/uvc-${which}.lock"
flock -n 9 || { echo 'This UVC device already has a managed owner' >&2;exit 1; }
exec "$workspace/install/wla_uvc_camera/lib/wla_uvc_camera/uvc_camera" "${args[@]}" "$@"
