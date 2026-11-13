#!/usr/bin/env bash
# Four-ToF receiver (ESP32 TF4 over the FT232R USB-UART). Read-only: no serial commands,
# DTR/RTS untouched (forcing them low resets the ESP32), no motion topics.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-run}";shift || true
[[ "$operation" == run || "$operation" == check ]] || { echo 'robot_tof.sh run|check [ros args]' >&2;exit 2; }
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace/config/uvc_rig/fastdds_uvc.xml}"
fi
params="$workspace/config/tof/new_orin.yaml"
device="$(sed -n 's/^ *device: *//p' "$params")"
if [[ "$operation" == check ]]; then
  ls -l "$device" && fuser -v "$device" 2>&1 || true
  exit 0
fi
mkdir -p "$workspace/.runtime"
exec 9>"$workspace/.runtime/tof-pipeline.lock"
flock -n 9 || { echo 'ToF pipeline already running' >&2; exit 1; }
state_pid=
if ! pgrep -f "^$workspace/install/wla_robot_state/lib/wla_robot_state/robot_state_adapter( |$)" >/dev/null; then
  "$workspace/install/wla_robot_state/lib/wla_robot_state/robot_state_adapter" --ros-args --params-file "$workspace/config/robot_state/new_orin.yaml" &
  state_pid=$!
fi
"$workspace/install/wla_tof/lib/wla_tof/tof_obstacles" --ros-args --params-file "$params" &
filter_pid=$!
receiver_pid=
trap 'kill "$filter_pid" ${state_pid:+"$state_pid"} ${receiver_pid:+"$receiver_pid"} 2>/dev/null || true' EXIT INT TERM
"$workspace/install/wla_tof/lib/wla_tof/tof_serial" --ros-args --params-file "$params" "$@" &
receiver_pid=$!
wait "$receiver_pid"
