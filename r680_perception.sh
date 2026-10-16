#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export WLA_ROS_WS="$workspace"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
task_python="${R680_PERCEPTION_PYTHON:-$workspace/.venv-perception/bin/python}"
task_nav="$workspace/install/wla_r680_navigation"
task_args=(--model "${R680_YOLO_MODEL:-${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}/models/perception/yolo/yolo11n.pt}"
           --tracker "$task_nav/share/wla_r680_navigation/config/perception.botsort.yaml"
           --device "${R680_PERCEPTION_DEVICE:-0}")
operation="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$operation" in
  check)
    exec "$task_python" "$task_nav/lib/wla_r680_navigation/nav_yolo_detector" "${task_args[@]}" --check-dependencies "$@"
    ;;
  run)
    exec /usr/bin/python3 "$task_nav/lib/wla_r680_navigation/nav_perception_bringup" --python "$task_python" "${task_args[@]}" "$@"
    ;;
  debug)
    exec /usr/bin/python3 "$task_nav/lib/wla_r680_navigation/nav_perception_debug" "$@"
    ;;
  *) echo 'Usage: r680_perception.sh check | run [--base-frame r680_mapping_floor] [--no-agent] [--debug] | debug [--port 18081]' >&2; exit 2 ;;
esac
