#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$operation" in
  -h|--help|help)
    echo 'r680_object_follow.sh check | run [enable_motion:=false|true] [config:=...]'
    echo 'YOLO 物体靠近/跟随；默认预览，运动还需 OBJECT 源及 AUTO 授权。'
    exit 0 ;;
  check|run) ;;
  *) echo "Unknown operation: $operation" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
/usr/bin/python3 - <<'PY'
import json
from ament_index_python.packages import get_package_prefix
from wla_behavior_interfaces.srv import ObjectCommand
from wla_object_follow.node import ObjectFollow
from wla_object_follow.control import ObjectFollower
from _wla_follow_local import close_object
print(json.dumps(dict(owner='ros2_ws/wla_object_follow',package_prefix=get_package_prefix('wla_object_follow'),
    mode='relative_surface_clearance',command='/wla_agent/object_command',default_motion=False),indent=2))
PY
[[ "$operation" != check ]] || exit 0
mkdir -p "$workspace/.runtime"
exec 8>"$workspace/.runtime/object-follow-${ROS_DOMAIN_ID}.lock"
flock -n 8 || { echo '当前 ROS 域已有物体执行入口运行' >&2; exit 1; }
exec ros2 launch wla_object_follow follow.launch.py "$@"
