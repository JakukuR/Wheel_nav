#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$operation" in
  -h|--help|help)
    echo 'r680_person_follow.sh check | run [enable_motion:=false|true] [ROS launch arguments]'
    echo '相对人物执行：Agent通过PersonCommand选择follow或approach_interact并注入ID。'
    echo '默认预览；无YOLO、map、odom或Nav2依赖。启用运动仍需底盘PERSON源和AUTO授权。'
    exit 0 ;;
  check|run) ;;
  indoor|outdoor)
    echo '旧场景入口统一使用相对人物执行，不再启用地图规划。' >&2 ;;
  *) echo "Unknown operation: $operation" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
export WLA_FOLLOW_PYTHON="${WLA_FOLLOW_PYTHON:-/usr/bin/python3}"
"$WLA_FOLLOW_PYTHON" - <<'PY'
import importlib, json, sys
from ament_index_python.packages import get_package_prefix
report = {'owner':'ros2_ws/wla_person_follow', 'mode':'relative_only',
          'input':'/r680/perception/relative_objects',
          'command_service':'/r680/person_follow/command', 'errors':[]}
for module in ('rclpy', 'numpy', 'cv2', 'tf2_ros', 'wla_person_follow.relative_control'):
    try: importlib.import_module(module)
    except Exception as exc: report['errors'].append(module+':'+str(exc))
try:
    from wla_behavior_interfaces.srv import PersonCommand
    report['package_prefix'] = get_package_prefix('wla_person_follow')
except Exception as exc: report['errors'].append(str(exc))
print(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(bool(report['errors']))
PY
[[ "$operation" != check ]] || exit 0
mkdir -p "$workspace/.runtime"
exec 8>"$workspace/.runtime/person-follow-${ROS_DOMAIN_ID}.lock"
flock -n 8 || { echo '当前ROS域已有人物执行入口运行' >&2; exit 1; }
exec ros2 launch wla_person_follow follow.launch.py python:="$WLA_FOLLOW_PYTHON" "$@"
