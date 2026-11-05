#!/usr/bin/env bash
# Records a follow run for offline replay. Only subscribes: it publishes nothing and starts nothing.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
  cat <<'TEXT'
r680_follow_bag.sh <mpc|rla|其他标签> [--camera] [--depth] [--color]
  默认只录规划、感知结果和底盘话题，不录原始图像。
  --camera 加录 JPEG 画面（10 Hz，硬件编码，Kalo 网页回放能看到画面）
  --depth  加录对齐深度图（可离线重跑地图，约 0.6 MB/帧）
  --color  加录彩色图（可看画面，约 0.9 MB/帧）
  标签是 mpc 或 rla 时，会核对正在运行的后端用的规划器，不一致就拒绝录制。
  Ctrl+C 结束。输出到 ~/ros2_ws/.runtime/follow-bags/<标签>-<时间>/
TEXT
}
label=''; camera=0; depth=0; color=0
for argument in "$@"; do
  case "$argument" in
    -h|--help) usage; exit 0 ;;
    --camera) camera=1 ;;
    --depth) depth=1 ;;
    --color) color=1 ;;
    -*) echo "未知参数：$argument" >&2; usage >&2; exit 2 ;;
    *) [[ -z "$label" ]] || { echo '只能有一个标签' >&2; exit 2; }; label="$argument" ;;
  esac
done
if [[ ! "$label" =~ ^[A-Za-z0-9_.-]+$ ]]; then usage >&2; exit 2; fi
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"

directory="$workspace/.runtime/follow-bags"
mkdir -p "$directory"
free_gb=$(( $(df --output=avail -k "$directory" | tail -1) / 1024 / 1024 ))
if (( free_gb < 10 )); then echo "磁盘只剩 ${free_gb} GB，不录制" >&2; exit 1; fi

planner="$(timeout 5 ros2 param get /person_follow local_planner 2>/dev/null | sed -n 's/^String value is: //p' || true)"
motion="$(timeout 5 ros2 param get /person_follow enable_motion 2>/dev/null | sed -n 's/^Boolean value is: //p' || true)"
case "$label:${planner:-none}" in
  rla:neupan|mpc:mpc) ;;
  rla:*|mpc:*) echo "标签是 $label，但后端规划器是 ${planner:-未运行或未设置}，不录制" >&2; exit 1 ;;
esac

topics=(
  /r680/person_follow/status /r680/person_follow/command_candidate /r680/person_follow/cmd_vel_candidate
  /r680/person_follow/local_path /r680/person_follow/local_obstacles /r680/person_follow/depth_obstacles
  /r680/person_follow/reid_status /r680/perception/relative_objects /r680/perception/detections /r680/perception/target_mask
  /r680_nav/perception/status /r680_nav/perception/timing
  /r680_base/cmd_vel /r680_nav/chassis_cmd_vel /r680_base/control_state
  /wheel/odom_50hz /tf /tf_static /r680/d455/color/camera_info
)
(( depth )) && topics+=(/r680/d455/aligned_depth_to_color/image_raw /r680/d455/aligned_depth_to_color/camera_info)
(( color )) && topics+=(/r680/d455/color/image_raw)
(( camera )) && topics+=(/r680/debug/color/compressed)

output="$directory/$label-$(date +%Y%m%d-%H%M%S)"
note="$output.json"
python3 - "$note" "$label" "$planner" "$motion" "$(git -C "$workspace" rev-parse --short HEAD)" "${topics[@]}" <<'PY'
import json, sys
path, label, planner, motion, commit, *topics = sys.argv[1:]
json.dump(dict(label=label, local_planner=planner or None, enable_motion=motion or None, commit=commit, topics=topics),
          open(path, 'w'), indent=1, ensure_ascii=False)
PY
echo "录制到 $output（规划器 ${planner:-未知}，Ctrl+C 结束）"
jpeg_pid=''
if (( camera )) && ! pgrep -f 'wla_r680_navigation/kalo_jp[e]g' >/dev/null; then
  "$workspace/install/wla_r680_navigation/lib/wla_r680_navigation/kalo_jpeg" --ros-args -r __node:=kalo_jpeg >/dev/null 2>&1 &
  jpeg_pid=$!
fi
nice -n 10 ionice -c 3 ros2 bag record -s mcap -o "$output" "${topics[@]}" &
bag_pid=$!
# Background jobs of a script ignore SIGINT; SIGTERM also closes the bag properly.
stop() { kill -TERM "$bag_pid" 2>/dev/null || true; }
trap stop INT TERM
# wait returns early on a signal; keep waiting until the bag is closed.
while kill -0 "$bag_pid" 2>/dev/null; do wait "$bag_pid" || true; done
if [[ -n "$jpeg_pid" ]]; then kill -TERM "$jpeg_pid" 2>/dev/null || true; wait "$jpeg_pid" 2>/dev/null || true; fi
