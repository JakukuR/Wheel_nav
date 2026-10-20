#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
operation="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$operation" in
  -h|--help|help)
    echo 'r680_person_follow.sh check | run [enable_motion:=false|true] [ROS launch arguments]'
    echo '默认 close-wide-flex 仿真后端，follow 指定人物；backend:=legacy 可回退原实现。'
    echo '默认预览；需要共享D455/检测/轮速。默认走原人物跟随 mux 接口，保留手柄接管。'
    exit 0 ;;
  check|run) ;;
  indoor|outdoor)
    echo '旧场景入口统一使用相对人物执行，不再启用地图规划。' >&2 ;;
  *) echo "Unknown operation: $operation" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
# Keep the verified mux overlay, then load the optimized follower/native map.
source "$workspace/releases/sim-follow-20261020-v1/install/local_setup.bash"
source "$workspace/releases/sim-follow-20261020-latency/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
export OPENBLAS_NUM_THREADS="${WLA_FOLLOW_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${WLA_FOLLOW_NUM_THREADS:-1}"
export WLA_REID_PYTHON="${WLA_REID_PYTHON:-$workspace/.venv-perception/bin/python}"
export WLA_REID_MODEL="${WLA_REID_MODEL:-/home/orin/Wheel_Legged_Agent/models/perception/reid/osnet_x0_25_msmt17.pth}"
export WLA_FOLLOW_PYTHON="${WLA_FOLLOW_PYTHON:-/usr/bin/python3}"
"$WLA_FOLLOW_PYTHON" - <<'PY'
import importlib, json, sys
from ament_index_python.packages import get_package_prefix
report = {'owner':'ros2_ws/wla_person_follow', 'mode':'simulation_close_wide_flex',
          'input':'/r680/perception/relative_objects',
          'command_service':'/r680/person_follow/command', 'errors':[]}
for module in ('rclpy', 'numpy', 'cv2', 'tf2_ros', 'wla_person_follow.relative_control', 'wla_person_follow.sim_node', '_wla_sim_geometry'):
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
exec ros2 launch wla_person_follow follow.launch.py python:="$WLA_FOLLOW_PYTHON" enable_reid:=true "$@"
