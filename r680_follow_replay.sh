#!/usr/bin/env bash
# Replays a recorded follow run to the Kalo debug page, on its own ROS domain and with
# display topics only: velocity commands stay in the bag, so a replay cannot move the robot.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bags="$workspace/.runtime/follow-bags"
usage() {
  cat <<'TEXT'
r680_follow_replay.sh <包名或目录> [--rate 倍速] [--port 端口] [--start 秒]
  包名是 .runtime/follow-bags 下的目录名，不带参数时列出已有的包。
  默认 1 倍速、循环播放、网页端口 18082。打开 http://<车 IP>:<端口>/，Ctrl+C 结束。
TEXT
}
bag=''; rate=1.0; port=18082; start=0
while (( $# )); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --rate) rate="$2"; shift 2 ;;
    --port) port="$2"; shift 2 ;;
    --start) start="$2"; shift 2 ;;
    -*) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    *) bag="$1"; shift ;;
  esac
done
if [[ -z "$bag" ]]; then usage; echo; ls -1 "$bags" 2>/dev/null | grep -v '\.json$' || true; exit 0; fi
[[ -d "$bag" ]] || bag="$bags/$bag"
[[ -f "$bag/metadata.yaml" ]] || { echo "找不到录制包，或它没有正常结束：$bag" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID=199   # never the robot's domain
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml"
bridge_port=$((port-250))
topics=(
  /r680/debug/color/compressed /r680/d455/color/camera_info /tf /tf_static
  /r680/perception/detections /r680/perception/relative_objects /r680/perception/target_mask
  /r680_nav/perception/status /r680_nav/perception/timing
  /r680/person_follow/status /r680/person_follow/depth_obstacles
  /r680/person_follow/local_obstacles /r680/person_follow/local_path
)
share="$workspace/install/wla_r680_navigation/share/wla_r680_navigation"
pids=()
# Background jobs of a script ignore SIGINT, so stop them with SIGTERM.
stop() { for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; }
trap stop INT TERM EXIT
nice -n 10 ros2 run foxglove_bridge foxglove_bridge --ros-args -r __node:=kalo_bridge \
  --params-file "$share/config/kalo_bridge.yaml" -p port:="$bridge_port" >/dev/null 2>&1 &
pids+=($!)
nice -n 10 /usr/bin/python3 "$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/kalo_debug_web" \
  --port "$port" --bridge-port "$bridge_port" >/dev/null 2>&1 &
pids+=($!)
echo "回放 $(basename "$bag")（${rate} 倍速，循环），页面 http://$(hostname -I | awk '{print $1}'):$port/，Ctrl+C 结束"
nice -n 10 ros2 bag play "$bag" --loop --rate "$rate" --start-offset "$start" --topics "${topics[@]}" &
player=$!
pids+=($player)
while kill -0 "$player" 2>/dev/null; do wait "$player" || true; done
