#!/usr/bin/env bash
set -Ee -o pipefail

DRY_RUN=false
STORAGE_CONFIG=""
ODOM_SOURCE=rgbd
AUTO_VIO_INIT=false
START_RVIZ=true
START_SEMANTICS=true
usage() {
  cat <<'EOF'
用法: r680_mapping.sh [--cuvslam|--rgbd] [--auto-vio-init] [--dry-run]
                       [--storage-config PATH] [--no-rviz] [--no-semantics]
  --cuvslam        D455 双目 + 车身 IMU，RTAB-Map RGB-D 建图
  --rgbd           原 RGB-D VO + EKF（默认、回退入口）
  --auto-vio-init   启动低速运动初始化，成功后交接给手柄；仅与 --cuvslam 配合
  --dry-run        关闭真实底盘输出；静止时 VIO 可能无法初始化
  --no-rviz        跳过 RViz；Web 保留
  --no-semantics   跳过家具识别；默认保留家具与出生点标注
  Ctrl+C           撤销运动 → 停稳 → 备份数据库与地图 → 关闭节点
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cuvslam) ODOM_SOURCE=cuvslam; shift ;;
    --rgbd) ODOM_SOURCE=rgbd; shift ;;
    --auto-vio-init) AUTO_VIO_INIT=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    --no-semantics) START_SEMANTICS=false; shift ;;
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --storage-config)
      [[ $# -ge 2 ]] || { echo '--storage-config 缺少路径' >&2; exit 2; }
      STORAGE_CONFIG="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ "$AUTO_VIO_INIT" == true && "$ODOM_SOURCE" != cuvslam ]]; then
  echo '--auto-vio-init 需要同时指定 --cuvslam' >&2; exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  CUVSLAM_SETUP="$HOME/ros2_ws/.runtime/cuvslam_navigation/install/setup.bash"
  [[ -f "$CUVSLAM_SETUP" ]] || { echo 'cuVSLAM 未构建，使用 --rgbd 回退' >&2; exit 1; }
  source "$CUVSLAM_SETUP"
  ros2 pkg prefix wla_cuvslam_navigation >/dev/null
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml}"
python3 - <<'PYCHECK'
from pathlib import Path
import os
for p in Path('/proc').iterdir():
    if not p.name.isdigit(): continue
    try: target = os.readlink(p/'exe')
    except OSError: continue
    if target.endswith('/wheeltec_robot_node'):
        raise SystemExit('底盘驱动已运行；先停止独立底盘/手柄会话，避免串口冲突。')
PYCHECK
ENABLE_HARDWARE=true
PREPARE_ARGS=(--imu-mode "$(if [[ "$ODOM_SOURCE" == cuvslam ]]; then echo chassis-vio; else echo chassis-full; fi)" --odom-source "$ODOM_SOURCE")
if [[ "$AUTO_VIO_INIT" == true ]]; then PREPARE_ARGS+=(--auto-vio-init); fi
COMMAND_TOPIC=/cmd_vel_nav
if [[ "$AUTO_VIO_INIT" == true ]]; then COMMAND_TOPIC=/r680_nav/nav_command_input; fi
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
RUN_DIR=""; SAVE_OK=false; STARTED=false; MAPPING_READY=false
READONLY_INPUTS_ACTIVE=false

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do kill -0 -- "-$pid" 2>/dev/null || return 0; sleep 0.1; done
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do kill -0 -- "-$pid" 2>/dev/null || return 0; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  set +e
  echo
  echo "[R680] 正在停止手柄并撤销运动授权……"
  if [[ "$AUTO_VIO_INIT" == true && "$STARTED" == true ]]; then
    timeout 2 ros2 topic pub --once /r680_nav/vio_init_cancel std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
  fi
  stop_group "$PERMISSION_PID"
  stop_group "$GAMEPAD_PID"
  timeout 2 ros2 topic pub --once /r680_nav/mission_motion_allowed std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
  sleep 2

  if [[ "$STARTED" == true && "$MAPPING_READY" == true ]] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
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
  if [[ "$STARTED" == true && "$MAPPING_READY" == false ]]; then
    echo "[R680] 建图尚未就绪，保留运行日志和工作库，不发布导航地图：$RUN_DIR"
  fi

  stop_group "$RVIZ_PID"
  stop_group "$BRINGUP_PID"
  systemctl --user stop r680-d455-localization-stack.service >/dev/null 2>&1 || true
  if ((${#PAUSED_UNITS[@]})); then
    if [[ "$READONLY_INPUTS_ACTIVE" == true ]]; then
      systemctl --user restart r680-readonly-inputs.service >/dev/null 2>&1 || true
    fi
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

if systemctl --user is-active --quiet r680-readonly-inputs.service; then READONLY_INPUTS_ACTIVE=true; fi
systemctl --user stop r680-d455-localization-stack.service
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  for unit in r680-d455-observation.service r680-readonly-inputs.service; do
    if systemctl --user is-active --quiet "$unit"; then
      PAUSED_UNITS+=("$unit"); systemctl --user stop "$unit"
    fi
  done
else
  if ! systemctl --user is-active --quiet r680-d455-observation.service; then
    systemctl --user restart r680-readonly-inputs.service
  fi
fi
for unit in "${PAUSE_UNITS[@]}"; do
  if systemctl --user is-active --quiet "$unit"; then
    PAUSED_UNITS+=("$unit")
    systemctl --user stop "$unit"
  fi
done
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  python3 - <<'PYCAMERA'
from pathlib import Path
import os
for p in Path('/proc').iterdir():
    if not p.name.isdigit(): continue
    try: target = os.readlink(p/'exe')
    except OSError: continue
    if target.endswith('/realsense2_camera_node'):
        raise SystemExit('仍有独立 RealSense 驱动，请先关闭该相机会话。')
PYCAMERA
fi

RUN_DIR=$(ros2 run wla_r680_navigation prepare_mapping_run "${PREPARE_ARGS[@]}")
echo "$RUN_DIR" > /tmp/wla_manual_mapping_run
DB_PATH=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["database"])' "$RUN_DIR/run.json")
CONFIG_DIR=$(ros2 pkg prefix wla_r680_navigation)/share/wla_r680_navigation/config
RVIZ_CONFIG="$CONFIG_DIR/r680_mapping.rviz"

echo "[R680] 本次运行目录：$RUN_DIR"
echo "[R680] 里程计前端：$ODOM_SOURCE；真实底盘输出：$ENABLE_HARDWARE"
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  echo '[R680] VIO 稳定前不写入地图、不开放手柄；无需旧地图重定位。'
  if [[ "$AUTO_VIO_INIT" == true ]]; then
    echo '[R680] 自动初始化：最高 0.06m/s、0.20rad/s；仅限已确认空旷的起步区。'
  else
    echo '[R680] 未启用自动激励；可在关闭真实输出的预览下手推完成初始化。'
  fi
else
  echo '[R680] 前 5 秒保持车辆和手柄完全静止，正在估计 IMU 零偏……'
fi
setsid ros2 launch wla_r680_navigation bringup.launch.py \
  mode:=mapping database_path:="$DB_PATH" \
  start_d455:="$(if [[ "$ODOM_SOURCE" == cuvslam ]]; then echo true; else echo false; fi)" \
  start_chassis:=true start_nav2:=true start_navigation_servers:=false start_dynamic_obstacles:=false \
  start_state_estimation:=true use_d455_imu:=false use_chassis_imu:=true \
  odom_source:="$ODOM_SOURCE" auto_vio_init:="$AUTO_VIO_INIT" \
  cuvslam_statistics_path:="$RUN_DIR/cuvslam_statistics.json" vio_init_result_path:="$RUN_DIR/vio_init.json" \
  publish_mount_tf:=true enable_hardware_output:="$ENABLE_HARDWARE" \
  start_semantics:="$START_SEMANTICS" semantic_output:="$RUN_DIR/semantic.geojson" \
  semantic_map_id:="$(basename "$RUN_DIR")" semantic_mark_home:=true \
  >"$RUN_DIR/logs/bringup.log" 2>&1 </dev/null &
BRINGUP_PID=$!
STARTED=true
WEB_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "[R680] Web： http://${WEB_IP:-127.0.0.1}:8080"

READY_ARGS=(--frontend "$ODOM_SOURCE" --run-dir "$RUN_DIR" --bringup-pid "$BRINGUP_PID")
if [[ "$AUTO_VIO_INIT" == true ]]; then READY_ARGS+=(--auto-init); fi
ros2 run wla_r680_navigation wait_mapping_ready "${READY_ARGS[@]}"
MAPPING_READY=true

echo "[R680] 定位与深度安全链已就绪。"
if [[ -e /dev/input/js0 ]]; then
  setsid ros2 run gamepad_control gamepad_teleop --ros-args \
    --params-file "$HOME/ros2_ws/install/gamepad_control/share/gamepad_control/config/gamepad_params.yaml" \
    -p device:=/dev/input/js0 -p cmd_vel_topic:="$COMMAND_TOPIC" \
    -p deadman_enabled:=false -p max_linear_speed:=1.00 \
    -p max_reverse_speed:=0.70 \
    -p max_angular_speed:=0.50 -p spin_angular_speed:=0.40 \
    -p filter_alpha:=0.55 -p max_linear_accel:=0.80 -p max_linear_decel:=1.20 \
    -p max_angular_accel:=1.80 -p max_angular_decel:=2.50 \
    >"$RUN_DIR/logs/gamepad.log" 2>&1 </dev/null &
else
  echo '[R680] dry-run：手柄不在线，使用零速度模拟源。'
  setsid ros2 topic pub -r 50 "$COMMAND_TOPIC" geometry_msgs/msg/Twist '{}' \
    >"$RUN_DIR/logs/gamepad.log" 2>&1 </dev/null &
fi
GAMEPAD_PID=$!

# With deadman disabled, refuse to enable chassis motion unless the controller is neutral.
python3 - "$COMMAND_TOPIC" <<'PY'
import rclpy, time, sys
from geometry_msgs.msg import Twist
from rclpy.node import Node
rclpy.init(); n=Node('r680_gamepad_neutral_check'); values=[]
n.create_subscription(Twist,sys.argv[1],lambda m: values.append((m.linear.x,m.angular.z)),50)
end=time.monotonic()+3
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
n.destroy_node(); rclpy.shutdown()
if len(values)<20: raise SystemExit('手柄没有稳定发布输入速度话题')
peak=max(max(abs(x),abs(z)) for x,z in values[-40:])
if peak>0.03: raise SystemExit(f'手柄不在零位，拒绝开放底盘，峰值={peak:.3f}')
PY

if [[ "$START_RVIZ" == true && ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]]; then
  setsid ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" >"$RUN_DIR/logs/rviz.log" 2>&1 </dev/null &
  RVIZ_PID=$!
  echo '[R680] RViz 已启动：二维地图、融合里程计和 TF；近场障碍点云仅在导航界面默认显示。'
else
  echo '[R680] 本次未启动 RViz（--no-rviz 或无图形会话）；可使用 Web 查看地图。'
fi

setsid ros2 topic pub -r 10 /r680_nav/mission_motion_allowed std_msgs/msg/Bool '{data: true}' \
  >"$RUN_DIR/logs/motion_permission.log" 2>&1 </dev/null &
PERMISSION_PID=$!

echo
printf '%s\n' \
  '[R680] 手柄死人开关已关闭。' \
  '[R680] 左摇杆控制前后，右摇杆控制转向；速度上限前进 1.00 m/s、倒车 0.70 m/s、转向 0.50 rad/s。' \
  '[R680] 建图完成后在本终端按 Ctrl+C：自动停车 → 保存二维地图 → 关闭节点 → 恢复原定位服务。'
echo "[R680] 真实底盘输出：$ENABLE_HARDWARE"

while kill -0 "$BRINGUP_PID" 2>/dev/null; do sleep 1; done
echo '[R680] 建图主进程意外退出。' >&2
exit 1
