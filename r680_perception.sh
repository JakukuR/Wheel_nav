#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export WLA_ROS_WS="$workspace"
export OPENBLAS_NUM_THREADS="${WLA_PERCEPTION_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${WLA_PERCEPTION_NUM_THREADS:-1}"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
task_python="${R680_PERCEPTION_PYTHON:-$workspace/.venv-boxmot/bin/python}"
task_nav="$workspace/install/wla_r680_navigation"
export PYTHONPATH="${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}/src${PYTHONPATH:+:$PYTHONPATH}"
task_args=(--model "${R680_YOLO_MODEL:-${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}/models/perception/yoloe/yoloe-26s-seg.pt}"
           --classes-file "${WLA_YOLOE_CLASSES:-${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}/deploy/yoloe_classes.json}"
           --text-encoder "${WLA_YOLOE_TEXT_ENCODER:-${WLA_AGENT_ROOT:-$HOME/Wheel_Legged_Agent}/models/perception/yoloe/mobileclip2_b.ts}"
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
  window)
    exec /usr/bin/python3 "$task_nav/lib/wla_r680_navigation/agent_observation_window" --config "$task_nav/share/wla_r680_navigation/config/agent_observation_window.json" "$@"
    ;;
  debug)
    exec /usr/bin/python3 "$task_nav/lib/wla_r680_navigation/nav_perception_debug" "$@"
    ;;
  debug-camera)
    exec /usr/bin/python3 "$task_nav/lib/wla_r680_navigation/perception_debug_standalone" --python "$task_python" "${task_args[@]}" "$@"
    ;;
  *) echo 'Usage: r680_perception.sh check | run [--no-agent] [--no-window] [--cache-only] [--debug] | window [--check] | debug [--mode local|odom] | debug-camera [--port 18081]' >&2; exit 2 ;;
esac
