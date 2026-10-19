#!/usr/bin/env bash
# Records existing ROS topics only. Does not start sensors or command motion.
set -eo pipefail
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: bash $0 [seconds: 1..600; default 180]"
  echo "Start this recorder before r680_nav.sh --openvins. No motion is authorized by this script."
  exit 0
fi
seconds="${1:-180}"
[[ "$seconds" =~ ^[1-9][0-9]{0,2}$ ]] && (( seconds <= 600 )) || { echo 'seconds must be 1..600' >&2; exit 2; }
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"
set -u
mkdir -p "$HOME/nav_run"
output="$(mktemp -d "$HOME/nav_run/d455-debug-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
config="$workspace/install/wla_openvins_navigation/share/wla_openvins_navigation/config/r680_d455_20261013"
cp -aL "$config" "$output/config_at_record_start"
printf 'duration_seconds=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\n' "$seconds" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" > "$output/recording.txt"
printf '录制目录：%s\n限时 %s 秒；请在另一个终端启动 ./r680_nav.sh --openvins\n' "$output" "$seconds"
# Keep recorder in the terminal process group so Ctrl+C reaches it.
# Preserve the wrapper long enough to inspect the finalized metadata.
interrupted=false
trap 'interrupted=true' INT
echo "提前结束请按一次 Ctrl+C，等待收尾；Ctrl+Z 只会挂起进程。"
set +e
timeout --foreground --signal=INT --kill-after=15s "${seconds}s" ros2 bag record \
  --storage mcap --output "$output/bag" --disable-keyboard-controls \
  --topics /r680/d455/infra1/image_rect_raw /r680/d455/infra2/image_rect_raw \
  /r680/d455/infra1/camera_info /r680/d455/infra2/camera_info \
  /r680/d455/imu /r680/d455/accel/sample /r680/d455/gyro/sample \
  /wheel/odom /openvins/odomimu /d455_slam/odom /tf /tf_static \
  /r680_base/control_state /r680_nav/localization_ready /parameter_events \
  > >(tee -i "$output/recorder.log") 2>&1
status=$?
trap - INT
set -e
if [[ "$status" != 0 && "$status" != 124 && "$status" != 130 ]] && ! [[ "$interrupted" == true && "$status" == 2 ]]; then
  echo "Recorder failed: status=$status; see $output/recorder.log" >&2
  exit "$status"
fi
if [[ ! -f "$output/bag/metadata.yaml" ]]; then
  echo "No finalized bag metadata; recording is not confirmed: $output" >&2
  exit 1
fi
if [[ "$interrupted" == true && "$status" == 2 ]]; then
  echo "收到 Ctrl+C；ROS 退出码为 2，正在检查已完成的录包元数据。"
fi
ros2 bag info "$output/bag" | tee "$output/bag_info.txt"
echo "录制结束。需检查 bag_info.txt 中双目、IMU、轮速是否均有消息：$output"
