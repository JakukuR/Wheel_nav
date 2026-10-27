#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$operation" in
  -h|--help|help)
    echo 'r680_person_follow.sh check | run [transport:=cpp|python] [enable_motion:=false|true] [ROS launch arguments]'
    echo '当前统一链路：mode 0 人物跟随，mode 1 人物靠近，mode 2 物体靠近。'
    echo '默认预览；follow 命令由操作者选择目标 ID 后启动。'
    exit 0 ;;
  check|run) ;;
  indoor|outdoor)
    echo '场景参数已移除；请使用统一跟随链路。' >&2; exit 2 ;;
  *) echo "Unknown operation: $operation" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
export OPENBLAS_NUM_THREADS="${WLA_FOLLOW_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${WLA_FOLLOW_NUM_THREADS:-1}"
export WLA_FOLLOW_PYTHON="${WLA_FOLLOW_PYTHON:-/usr/bin/python3}"
"$WLA_FOLLOW_PYTHON" - <<'PY'
import importlib, json, sys
from ament_index_python.packages import get_package_prefix
report = {'owner':'ros2_ws/wla_person_follow', 'modes':['0_person_follow','1_person_approach','2_object_approach'],
          'input':'/r680/perception/relative_objects',
          'command_service':'/r680/person_follow/command', 'errors':[]}
for module in ('rclpy', 'numpy', 'cv2', 'tf2_ros', 'wla_person_follow.task_session', 'wla_person_follow.sim_node', 'wla_person_follow.hybrid_worker', '_wla_sim_geometry'):
    try: importlib.import_module(module)
    except Exception as exc: report['errors'].append(module+':'+str(exc))
try:
    from wla_behavior_interfaces.srv import PersonCommand
    report['package_prefix'] = get_package_prefix('wla_person_follow')
    from pathlib import Path
    executable=Path(get_package_prefix('wla_follow_transport'))/'lib/wla_follow_transport/follow_transport'
    if not executable.is_file():raise RuntimeError('C++ follow transport is not built')
    report['transport']='cpp_ros_python_compute'
    report['transport_executable']=str(executable)
except Exception as exc: report['errors'].append(str(exc))
print(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(bool(report['errors']))
PY
[[ "$operation" != check ]] || exit 0
mkdir -p "$workspace/.runtime"
exec 8>"$workspace/.runtime/person-follow-${ROS_DOMAIN_ID}.lock"
flock -n 8 || { echo '当前ROS域已有人物执行入口运行' >&2; exit 1; }
exec ros2 launch wla_person_follow follow.launch.py python:="$WLA_FOLLOW_PYTHON" enable_reid:=true "$@"
