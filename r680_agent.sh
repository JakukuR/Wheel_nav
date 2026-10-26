#!/usr/bin/env bash
# Unified entry in the existing ros2_ws. No trial workspace is sourced.
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
map_directory="${WLA_AGENT_MAP_DIRECTORY:-$workspace/maps/map-2026-10-08-1}"
configuration="$workspace/config/agent_semantic"
policy="${WLA_AGENT_POLICY:-$configuration/actual-map-preview-policy.json}"
command="${1:-preview}"
if [[ $# -gt 0 ]]; then shift; fi
case "$command" in
  preview|execute)
    enabled=false; [[ "$command" != execute ]] || enabled=true
    exec ros2 launch wla_behavior_bridge agent_semantic_api.launch.py \
      map_directory:="$map_directory" policy_file:="$policy" execution_enabled:="$enabled" "$@"
    ;;
  person-api)
    exec ros2 launch wla_behavior_bridge agent_person_api.launch.py "$@"
    ;;
  person)
    exec /usr/bin/python3 "${WLA_AGENT_SOURCE_DIR:?}/scripts/agent_person_cli.py" "$@"
    ;;
  observations)
    exec /usr/bin/python3 "$workspace/install/wla_behavior_bridge/lib/wla_behavior_bridge/nav_perception_adapter" "$@"
    ;;
  query)
    case "${1:-map}" in
      map) request="$configuration/list-map.json" ;;
      visible) request="$configuration/list-visible.json" ;;
      navigate) request="$configuration/navigate-table-preview.json" ;;
      *) request="$1" ;;
    esac
    if [[ $# -gt 0 ]]; then shift; fi
    exec python3 "${WLA_AGENT_SOURCE_DIR:?}/scripts/agent_semantic_cli.py" --request "$request" "$@"
    ;;
  bridge)
    [[ $# -ge 1 ]] || { echo 'bridge requires this navigation run selected_map.json' >&2; exit 2; }
    selected="$1"; shift
    exec ros2 launch wla_behavior_bridge behavior_bridge.launch.py \
      map_directory:="$map_directory" policy_file:="$policy" selected_map_record:="$selected" "$@"
    ;;
  navigation)
    exec bash "$workspace/r680_nav.sh" --agent-control "$@"
    ;;
  -h|--help|help)
    echo 'r680_agent.sh person-api | person visible | person start --mode 0|1|2 --target-id ID [--request-id RID] [--seconds N] | person heartbeat|stop|status [--request-id RID] | observations [--whitelist FILE] | preview | execute | query map|visible|navigate|REQUEST.json | bridge SELECTED_MAP.json [launch arguments] | navigation [r680_nav.sh arguments]'
    ;;
  *) echo "Unknown operation: $command" >&2; exit 2 ;;
esac
