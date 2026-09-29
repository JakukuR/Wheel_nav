#!/usr/bin/env bash
set -Ee -o pipefail

usage() {
  cat <<'EOF'
用法: r680_nav.sh [选项]

  --map NAME             选择 maps 下的地图目录；默认读取 storage.yaml 的 active_map
  --storage-config PATH  指定存储配置文件
  --initial-pose "X Y Z R P Y"  给 RTAB-Map 提供出生点初值（米、弧度）
  --enable-motion        显式开放真实底盘导航输出（默认仅定位、规划和预览）
  --mpc                  使用单独的 nav2_mpc.yaml（默认继续使用 nav2.yaml/MPPI）
  --no-rviz              不启动 RViz
  -h, --help             显示帮助

示例:
  ./r680_nav.sh
  ./r680_nav.sh --map map-2026-09-20-1
  ./r680_nav.sh --map map-2026-09-20-1 --initial-pose "0 0 0 0 0 0"
  ./r680_nav.sh --map map-2026-09-20-1 --enable-motion
EOF
}

MAP_NAME=""
STORAGE_CONFIG=""
INITIAL_POSE=""
ENABLE_MOTION=false
USE_MPC=false
START_RVIZ=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      [[ $# -ge 2 ]] || { echo '--map 缺少地图名称' >&2; exit 2; }
      MAP_NAME="$2"; shift 2 ;;
    --storage-config)
      [[ $# -ge 2 ]] || { echo '--storage-config 缺少路径' >&2; exit 2; }
      STORAGE_CONFIG="$2"; shift 2 ;;
    --initial-pose)
      [[ $# -ge 2 ]] || { echo '--initial-pose 缺少六维位姿' >&2; exit 2; }
      INITIAL_POSE="$2"; shift 2 ;;
    --enable-motion) ENABLE_MOTION=true; shift ;;
    --mpc) USE_MPC=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml}"

CONFIG_DIR=$(ros2 pkg prefix wla_r680_navigation)/share/wla_r680_navigation/config
NAV2_PARAMS_FILE="$CONFIG_DIR/nav2.yaml"
if [[ "$USE_MPC" == true ]]; then
  NAV2_PARAMS_FILE="$CONFIG_DIR/nav2_mpc.yaml"
  [[ -f "$NAV2_PARAMS_FILE" ]] || { echo "MPC 参数文件不存在: $NAV2_PARAMS_FILE" >&2; exit 1; }
  ros2 pkg prefix wla_diff_mpc >/dev/null || { echo '未构建 wla_diff_mpc 插件' >&2; exit 1; }
  export LD_LIBRARY_PATH="$HOME/.local/wla_mpc_deps/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ -z "$STORAGE_CONFIG" ]]; then
  STORAGE_CONFIG="$CONFIG_DIR/storage.yaml"
fi
[[ -f "$STORAGE_CONFIG" ]] || { echo "存储配置不存在: $STORAGE_CONFIG" >&2; exit 1; }

RESOLVE_ARGS=(--storage-config "$STORAGE_CONFIG")
if [[ -n "$MAP_NAME" ]]; then
  RESOLVE_ARGS+=(--map "$MAP_NAME")
fi
MAP_JSON=$(ros2 run wla_r680_navigation resolve_navigation_map "${RESOLVE_ARGS[@]}")
MAP_DIR=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["directory"])' <<<"$MAP_JSON")
MAP_YAML=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["map_yaml"])' <<<"$MAP_JSON")
DB_PATH=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["database_path"])' <<<"$MAP_JSON")

if [[ -n "$INITIAL_POSE" ]]; then
  python3 - "$INITIAL_POSE" <<'PY'
import math
import sys
parts = sys.argv[1].split()
if len(parts) != 6:
    raise SystemExit('--initial-pose 必须包含 6 个数: x y z roll pitch yaw')
if not all(math.isfinite(float(v)) for v in parts):
    raise SystemExit('--initial-pose 只能包含有限数值')
PY
fi

RUN_ROOT=$(python3 - "$STORAGE_CONFIG" <<'PY'
import sys
from pathlib import Path
from wla_r680_navigation_py.storage import load_storage_config
storage = load_storage_config(Path(sys.argv[1]))
print(Path(storage['run_root']).expanduser())
PY
)
NAV_RUN_DIR="$RUN_ROOT/nav-$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$NAV_RUN_DIR/logs"
SOURCE_DB_PATH="$DB_PATH"
DB_PATH="$NAV_RUN_DIR/rtabmap.db"
cp --reflink=auto "$SOURCE_DB_PATH" "$DB_PATH"
printf '%s\n' "$MAP_JSON" > "$NAV_RUN_DIR/selected_map.json"
cat > "$NAV_RUN_DIR/navigation.json" <<EOF
{
  "selected_map": "$MAP_DIR",
  "map_yaml": "$MAP_YAML",
  "source_database": "$SOURCE_DB_PATH",
  "working_database": "$DB_PATH",
  "initial_pose": "$INITIAL_POSE",
  "hardware_output_enabled": $ENABLE_MOTION,
  "started_at": "$(date --iso-8601=seconds)"
}
EOF

PAUSE_UNITS=(
  r680-d455-perception-v1.service
  r680-m260c-perception-p1.service
  r680-interest-shadow-p1.service
  r680-persistent-semantic-map.service
  r680-state-layer.service
  r680-behavior-shadow.service
  r680-unified-web.service
  r680-diagnostic-history.service
  r680-d455-dynamic-map-p0.service
)
PAUSED_UNITS=()
BRINGUP_PID=""; PERMISSION_PID=""; RVIZ_PID=""; STARTED=false
READONLY_INPUTS_ACTIVE=false

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  # The setsid leader may exit before its ROS children. Check the process group,
  # otherwise an orphaned lifecycle manager can break the next bringup.
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  set +e
  echo
  echo '[R680 NAV] 正在撤销运动授权并停止导航节点……'
  stop_group "$PERMISSION_PID"
  timeout 2 ros2 topic pub --once /r680_nav/mission_motion_allowed \
    std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
  sleep 1
  stop_group "$RVIZ_PID"
  stop_group "$BRINGUP_PID"
  systemctl --user stop r680-d455-localization-stack.service >/dev/null 2>&1 || true
  if [[ "$READONLY_INPUTS_ACTIVE" == true ]]; then
    systemctl --user restart r680-readonly-inputs.service >/dev/null 2>&1 || true
    for unit in "${PAUSED_UNITS[@]}"; do
      systemctl --user start "$unit" >/dev/null 2>&1 || true
    done
  fi
  echo "[R680 NAV] 已关闭本次导航并恢复原定位服务。日志：$NAV_RUN_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

[[ -e /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00 ]] || {
  echo '[R680 NAV] 未找到候选底盘串口 0002' >&2; exit 1;
}

if systemctl --user is-active --quiet r680-readonly-inputs.service; then
  READONLY_INPUTS_ACTIVE=true
fi
systemctl --user stop r680-d455-localization-stack.service
if ! systemctl --user is-active --quiet r680-d455-observation.service; then
  systemctl --user restart r680-readonly-inputs.service
fi
for unit in "${PAUSE_UNITS[@]}"; do
  if systemctl --user is-active --quiet "$unit"; then
    PAUSED_UNITS+=("$unit")
    systemctl --user stop "$unit"
  fi
done

echo "[R680 NAV] 地图：$MAP_DIR"
echo "[R680 NAV] 二维栅格：$MAP_YAML"
echo "[R680 NAV] RTAB-Map 只读源库：$SOURCE_DB_PATH"
echo "[R680 NAV] RTAB-Map 本次工作副本：$DB_PATH"
echo "[R680 NAV] 本次日志：$NAV_RUN_DIR"
echo "[R680 NAV] Nav2 参数：$NAV2_PARAMS_FILE"
cp "$NAV2_PARAMS_FILE" "$NAV_RUN_DIR/nav2.yaml"
echo '[R680 NAV] 前 5 秒保持车辆静止，正在估计车身 IMU 零偏……'

LAUNCH_ARGS=(
  mode:=localization database_path:="$DB_PATH" web_map_yaml:="$MAP_YAML"
  nav_params_file:="$NAV2_PARAMS_FILE"
  start_d455:=false start_chassis:=true start_nav2:=true start_navigation_servers:=true
  start_state_estimation:=true use_d455_imu:=false use_chassis_imu:=true
  publish_mount_tf:=true enable_hardware_output:="$ENABLE_MOTION"
  start_semantics:=false semantic_output:="$MAP_DIR/semantic.geojson"
  semantic_map_id:="$(basename "$MAP_DIR")" semantic_mark_home:=false
)
if [[ -n "$INITIAL_POSE" ]]; then
  LAUNCH_ARGS+=(initial_pose:="$INITIAL_POSE")
fi
setsid ros2 launch wla_r680_navigation bringup.launch.py "${LAUNCH_ARGS[@]}" \
  >"$NAV_RUN_DIR/logs/bringup.log" 2>&1 </dev/null &
BRINGUP_PID=$!
STARTED=true
WEB_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "[R680 NAV] Web： http://${WEB_IP:-127.0.0.1}:8080"

RVIZ_CONFIG="$CONFIG_DIR/r680_navigation.rviz"
if [[ "$START_RVIZ" == true ]]; then
  if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
    setsid ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" \
      >"$NAV_RUN_DIR/logs/rviz.log" 2>&1 </dev/null &
    RVIZ_PID=$!
    echo '[R680 NAV] RViz 已启动，可用 2D Pose Estimate 校正出生位姿，用 Nav2 Goal 下发目标。'
  else
    echo '[R680 NAV] 当前无图形会话，跳过 RViz；可在车载桌面终端重新运行。'
  fi
fi

WLA_BRINGUP_LOG="$NAV_RUN_DIR/logs/bringup.log" python3 - <<'PY'
import math
import os
import time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('r680_navigation_ready_waiter')
ready_count = 0
map_ok = False

def ready_cb(msg):
    global ready_count
    ready_count = ready_count + 1 if msg.data else 0

def map_cb(msg):
    global map_ok
    map_ok = msg.info.width > 0 and msg.info.height > 0 and len(msg.data) > 0

node.create_subscription(Bool, '/r680_nav/localization_ready', ready_cb, 10)
map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(OccupancyGrid, '/r680/d455/map', map_cb, map_qos)
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
names = ['controller_server', 'planner_server', 'bt_navigator',
         'velocity_smoother', 'collision_monitor']
clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in names}
deadline = time.monotonic() + 120.0
last_report = 0.0
bringup_log = Path(os.environ['WLA_BRINGUP_LOG'])
log_offset = 0
log_tail = ''
manager_active = False
while time.monotonic() < deadline:
    with bringup_log.open(errors='replace') as stream:
        stream.seek(log_offset)
        new_log = stream.read()
        log_offset = stream.tell()
    if 'Failed to bring up all requested nodes' in log_tail + new_log:
        raise SystemExit('Nav2 生命周期启动失败；查看本次 logs/bringup.log')
    if 'Managed nodes are active' in log_tail + new_log:
        manager_active = True
    log_tail = (log_tail + new_log)[-80:]
    rclpy.spin_once(node, timeout_sec=0.2)
    states = {name: 0 for name in names}
    # The lifecycle manager owns these services during bringup. Poll only after activation.
    if manager_active:
        for name, client in clients.items():
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
            states[name] = future.result().current_state.id if future.done() and future.result() else 0
    tf_ok = tf_buffer.can_transform('map', 'r680_mapping_floor', rclpy.time.Time(),
                                    timeout=Duration(seconds=0.05))
    if ready_count >= 5 and map_ok and tf_ok and all(v == 3 for v in states.values()):
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    if time.monotonic() - last_report > 10.0:
        print('[R680 NAV] 等待就绪:', f'接口={ready_count >= 5}', f'地图={map_ok}',
              f'TF={tf_ok}', '生命周期=' + ','.join(f'{k}:{v}' for k, v in states.items()),
              flush=True)
        last_report = time.monotonic()
node.destroy_node()
rclpy.shutdown()
raise SystemExit('120 秒内导航未就绪；检查本次 logs/bringup.log')
PY

if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
  echo '[R680 NAV] 导航主进程在就绪检查后已退出。' >&2
  exit 1
fi

if [[ "$ENABLE_MOTION" == true ]]; then
  setsid ros2 topic pub -r 10 /r680_nav/mission_motion_allowed \
    std_msgs/msg/Bool '{data: true}' >"$NAV_RUN_DIR/logs/motion_permission.log" 2>&1 </dev/null &
  PERMISSION_PID=$!
  echo '[R680 NAV] 实车底盘输出已开放。确认 RViz 中机器人位置正确后再下发 Nav2 Goal。'
else
  echo '[R680 NAV] 当前为安全预览：定位、规划和碰撞链已启动，真实底盘输出保持关闭。'
  echo '[R680 NAV] 需要实车运动时停止本脚本，再显式加 --enable-motion 启动。'
fi
echo '[R680 NAV] 就绪检查只证明传感器、地图、TF 与 Nav2 正常，不等同于视觉重定位已匹配正确。'
echo '[R680 NAV] 按 Ctrl+C 将撤销运动授权、发送零速、关闭本次节点并恢复原定位服务。'

while kill -0 "$BRINGUP_PID" 2>/dev/null; do sleep 1; done
echo '[R680 NAV] 导航主进程意外退出。' >&2
exit 1
