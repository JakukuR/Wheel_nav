#!/usr/bin/env bash
# Standalone R680 chassis + gamepad teleoperation (no navigation or mapping).
set -Ee -o pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml}"

PORT=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
GAMEPAD=/dev/input/js0
CHASSIS_PID=""
GAMEPAD_PID=""
RUN_DIR=""

usage() {
  echo "用法: $0 [--dry-run]"
  echo "单独启动 R680 底盘驱动和手柄；Ctrl-C 时先停手柄、发零速，再停底盘。"
}

DRY_RUN=false
while (($#)); do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..30}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
}

send_zero() {
  timeout 5 python3 - <<'PY' || true
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

rclpy.init()
node = Node('r680_teleop_exit_zero')
pub = node.create_publisher(Twist, '/cmd_vel', 1)
deadline = time.monotonic() + 1.5
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
for _ in range(30):
    pub.publish(Twist())
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  set +e
  stop_group "$GAMEPAD_PID"
  if [[ -n "$CHASSIS_PID" ]] && kill -0 "$CHASSIS_PID" 2>/dev/null; then
    echo '[R680 TELEOP] 发送零速……'
    send_zero
    stop_group "$CHASSIS_PID"
  fi
  [[ -z "$RUN_DIR" ]] || echo "[R680 TELEOP] 日志：$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

[[ -e "$PORT" ]] || { echo "[R680 TELEOP] 找不到底盘串口：$PORT" >&2; exit 1; }
[[ -e "$GAMEPAD" ]] || { echo "[R680 TELEOP] 找不到手柄：$GAMEPAD" >&2; exit 1; }
[[ -r "${CYCLONEDDS_URI#file://}" ]] || { echo '[R680 TELEOP] DDS 配置不可读' >&2; exit 1; }
command -v fuser >/dev/null || { echo '[R680 TELEOP] 缺少 fuser' >&2; exit 1; }

echo "[R680 TELEOP] ROS_DOMAIN_ID=$ROS_DOMAIN_ID 串口=$PORT 手柄=$GAMEPAD"
if [[ "$DRY_RUN" == true ]]; then
  echo '[R680 TELEOP] dry-run：配置检查通过，未启动节点。'
  exit 0
fi

if fuser -s "$PORT" 2>/dev/null; then
  echo '[R680 TELEOP] 底盘串口已被占用；请先关闭导航、建图或其他底盘驱动。' >&2
  exit 1
fi
if pgrep -f '/gamepad_control/.*/gamepad_teleop' >/dev/null; then
  echo '[R680 TELEOP] 手柄节点已运行；请先关闭原遥控链路。' >&2
  exit 1
fi
topic_info=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null || true)
if grep -Eq '^Publisher count: [1-9]' <<<"$topic_info"; then
  echo '[R680 TELEOP] /cmd_vel 已有发布者；为避免双重控制，拒绝启动。' >&2
  exit 1
fi

RUN_DIR="$HOME/teleop_run/teleop-$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$RUN_DIR"
setsid ros2 launch turn_on_wheeltec_robot r680_chassis.launch.py >"$RUN_DIR/chassis.log" 2>&1 &
CHASSIS_PID=$!

ready=false
for _ in {1..8}; do
  kill -0 "$CHASSIS_PID" 2>/dev/null || break
  if timeout 3 ros2 topic echo /odom --once --field twist.twist >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.25
done
if [[ "$ready" != true ]]; then
  echo "[R680 TELEOP] 未收到 /odom，查看 $RUN_DIR/chassis.log" >&2
  exit 1
fi

echo '[R680 TELEOP] 底盘反馈正常，启动手柄。请先确认摇杆处于中位。'
setsid ros2 launch gamepad_control gamepad_teleop.launch.py >"$RUN_DIR/gamepad.log" 2>&1 &
GAMEPAD_PID=$!
echo '[R680 TELEOP] 已启动。左摇杆前后、右摇杆转向；Ctrl-C 结束。'
echo "[R680 TELEOP] 日志：$RUN_DIR"
wait -n "$CHASSIS_PID" "$GAMEPAD_PID" || true
echo '[R680 TELEOP] 底盘或手柄节点意外退出，正在安全停止。' >&2
exit 1
