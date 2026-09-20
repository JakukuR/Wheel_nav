#!/usr/bin/env bash
set -Ee -o pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml}"

DRY_RUN=false
STORAGE_CONFIG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    --storage-config)
      [[ $# -ge 2 ]] || { echo '--storage-config 缺少路径' >&2; exit 2; }
      STORAGE_CONFIG="$2"; shift 2 ;;
    *) echo "用法: $0 [--dry-run] [--storage-config PATH]" >&2; exit 2 ;;
  esac
done
ENABLE_HARDWARE=true
PREPARE_ARGS=(--imu-mode chassis-full)
if [[ -n "$STORAGE_CONFIG" ]]; then
  PREPARE_ARGS+=(--storage-config "$STORAGE_CONFIG")
fi
if [[ "$DRY_RUN" == true ]]; then
  ENABLE_HARDWARE=false
else
  PREPARE_ARGS+=(--hardware-output)
fi

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
BRINGUP_PID=""; GAMEPAD_PID=""; PERMISSION_PID=""; RVIZ_PID=""
RUN_DIR=""; SAVE_OK=false; STARTED=false

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
  kill -TERM -- "-$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  set +e
  echo
  echo "[R680] 正在停止手柄并撤销运动授权……"
  stop_group "$PERMISSION_PID"
  stop_group "$GAMEPAD_PID"
  timeout 2 ros2 topic pub --once /r680_nav/mission_motion_allowed std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
  sleep 2

  if [[ "$STARTED" == true ]] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
    echo "[R680] 建图节点保持运行，正在备份 RTAB-Map 并生成二维地图归档……"
    if ros2 run wla_r680_navigation save_manual_map --run-dir "$RUN_DIR" --timeout 45; then
      SAVE_OK=true
      echo "[R680] 地图保存成功：$RUN_DIR/map_archive"
      NAV_MAP=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["directory"])' \
        "$RUN_DIR/navigation_map_result.json")
      echo "[R680] 精简导航地图已发布：$NAV_MAP"
    else
      echo "[R680] 地图自动保存失败，请先不要删除运行目录：$RUN_DIR" >&2
      status=1
    fi
  fi

  stop_group "$RVIZ_PID"
  stop_group "$BRINGUP_PID"
  systemctl --user stop r680-d455-localization-stack.service >/dev/null 2>&1 || true
  if ((${#PAUSED_UNITS[@]})); then
    systemctl --user restart r680-readonly-inputs.service >/dev/null 2>&1 || true
    for unit in "${PAUSED_UNITS[@]}"; do
      systemctl --user start "$unit" >/dev/null 2>&1 || true
    done
  fi
  if [[ "$SAVE_OK" == true ]]; then
    echo "[R680] 已关闭本次节点并恢复原定位服务。"
    find "$RUN_DIR/map_archive" -maxdepth 1 -type f -printf '  %f\n' | sort
  else
    echo "[R680] 已关闭本次节点并尝试恢复原定位服务。" >&2
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

if [[ ! -e /dev/input/js0 && "$DRY_RUN" != true ]]; then
  echo '[R680] 未找到手柄 /dev/input/js0' >&2
  exit 1
fi
[[ -e /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00 ]] || {
  echo '[R680] 未找到候选底盘串口 0002' >&2; exit 1;
}

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

RUN_DIR=$(ros2 run wla_r680_navigation prepare_mapping_run "${PREPARE_ARGS[@]}")
echo "$RUN_DIR" > /tmp/wla_manual_mapping_run
DB_PATH=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["database"])' "$RUN_DIR/run.json")
CONFIG_DIR=$(ros2 pkg prefix wla_r680_navigation)/share/wla_r680_navigation/config
RVIZ_CONFIG="$CONFIG_DIR/r680_mapping.rviz"

echo "[R680] 本次运行目录：$RUN_DIR"
echo "[R680] 前 5 秒保持车辆和手柄完全静止，正在估计 IMU 零偏……"
setsid ros2 launch wla_r680_navigation bringup.launch.py \
  mode:=mapping database_path:="$DB_PATH" \
  start_d455:=false start_chassis:=true start_nav2:=true start_navigation_servers:=false \
  start_state_estimation:=true use_d455_imu:=false use_chassis_imu:=true \
  publish_mount_tf:=true enable_hardware_output:="$ENABLE_HARDWARE" \
  >"$RUN_DIR/logs/bringup.log" 2>&1 </dev/null &
BRINGUP_PID=$!
STARTED=true

python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Bool
rclpy.init(); n=Node('r680_mapping_ready_waiter'); ready=[False,0]
def cb(m):
    ready[0]=m.data
    ready[1]=ready[1]+1 if m.data else 0
n.create_subscription(Bool,'/r680_nav/localization_ready',cb,10)
end=time.monotonic()+90
while time.monotonic()<end and ready[1]<5: rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node(); rclpy.shutdown()
if ready[1]<5: raise SystemExit('定位、深度点云或地图接口在 90 秒内未就绪')
PY

echo "[R680] 定位与深度安全链已就绪。"
if [[ -e /dev/input/js0 ]]; then
  setsid ros2 run gamepad_control gamepad_teleop --ros-args \
    --params-file "$HOME/ros2_ws/install/gamepad_control/share/gamepad_control/config/gamepad_params.yaml" \
    -p device:=/dev/input/js0 -p cmd_vel_topic:=/cmd_vel_nav \
    -p deadman_enabled:=false -p max_linear_speed:=0.30 \
    -p max_angular_speed:=0.50 -p spin_angular_speed:=0.40 \
    -p filter_alpha:=0.55 -p max_linear_accel:=0.80 -p max_linear_decel:=1.20 \
    -p max_angular_accel:=1.80 -p max_angular_decel:=2.50 \
    >"$RUN_DIR/logs/gamepad.log" 2>&1 </dev/null &
else
  echo '[R680] dry-run：手柄不在线，使用零速度模拟源。'
  setsid ros2 topic pub -r 50 /cmd_vel_nav geometry_msgs/msg/Twist '{}' \
    >"$RUN_DIR/logs/gamepad.log" 2>&1 </dev/null &
fi
GAMEPAD_PID=$!

# With deadman disabled, refuse to enable chassis motion unless the controller is neutral.
python3 - <<'PY'
import rclpy, time
from geometry_msgs.msg import Twist
from rclpy.node import Node
rclpy.init(); n=Node('r680_gamepad_neutral_check'); values=[]
n.create_subscription(Twist,'/cmd_vel_nav',lambda m: values.append((m.linear.x,m.angular.z)),50)
end=time.monotonic()+3
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
n.destroy_node(); rclpy.shutdown()
if len(values)<20: raise SystemExit('手柄没有稳定发布 /cmd_vel_nav')
peak=max(max(abs(x),abs(z)) for x,z in values[-40:])
if peak>0.03: raise SystemExit(f'手柄不在零位，拒绝开放底盘，峰值={peak:.3f}')
PY

if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  setsid ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RUN_DIR/logs/rviz.log" 2>&1 </dev/null &
  RVIZ_PID=$!
  echo '[R680] RViz 已启动：二维地图、D455 点云、融合里程计和 TF。'
else
  echo '[R680] 当前没有图形显示会话，跳过 RViz；在车载桌面终端或 ssh -X 下运行即可自动打开。'
fi

setsid ros2 topic pub -r 10 /r680_nav/mission_motion_allowed std_msgs/msg/Bool '{data: true}' \
  >"$RUN_DIR/logs/motion_permission.log" 2>&1 </dev/null &
PERMISSION_PID=$!

echo
printf '%s\n' \
  '[R680] 手柄死人开关已关闭，底盘运动授权已开放。' \
  '[R680] 左摇杆控制前后，右摇杆控制转向；速度上限 0.30 m/s、0.50 rad/s。' \
  '[R680] 建图完成后在本终端按 Ctrl+C：自动停车 → 保存二维地图 → 关闭节点 → 恢复原定位服务。'

while kill -0 "$BRINGUP_PID" 2>/dev/null; do sleep 1; done
echo '[R680] 建图主进程意外退出。' >&2
exit 1
