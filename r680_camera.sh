#!/usr/bin/env bash
# The single D455 owner. r680_nav.sh and r680_perception.sh only reuse it.
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
  cat <<'EOF'
用法: r680_camera.sh run   [--infra] [--imu] [--emitter on|off|alternate] [--no-mount]
      r680_camera.sh check [--infra] [--imu]
      r680_camera.sh status
  run     独立终端启动 D455：彩色 + 对齐深度 640x480@30 常开，红外双目和 IMU 默认关闭；
          同时发布安装 TF r680_mapping_floor -> d455_link（已有其他发布者时自动不发）。
  check   检查本 ROS 域里的相机是否提供所需数据流（导航启动前调用）；0=就绪，3=无相机，4=缺数据流。
          报告中 mount 表示安装 TF 是否已存在，导航/建图据此决定是否自己发布。
  status  打印相机数据流、发射器与安装 TF 状态。
  --infra        打开红外双目 infra1/infra2（导航 --cuvslam / --openvins 需要）
  --imu          打开 D455 IMU（OpenVINS D455 IMU 标定需要，使用 IMU 修复版 realsense2_camera）
  --emitter off  关闭红外发射器（双目 VIO 更干净，深度避障变差）；默认 on
  --emitter alternate  发射器逐帧交替：亮帧给深度，暗帧红外双目发到 infra*_dark（导航加 --dark-infra）
  --no-mount     不发布安装 TF（由导航或标定脚本 publish_r680_d455_floor.py 发布时使用）
EOF
}
operation="${1:-status}"
if [[ $# -gt 0 ]]; then shift; fi
INFRA=false IMU=false EMITTER=on MOUNT=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --infra) INFRA=true; shift ;;
    --imu) IMU=true; shift ;;
    --no-mount) MOUNT=false; shift ;;
    --emitter)
      [[ $# -ge 2 && ( "$2" == on || "$2" == off || "$2" == alternate ) ]] || { echo '--emitter 需要 on、off 或 alternate' >&2; exit 2; }
      EMITTER="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done
case "$operation" in run|check|status) ;; -h|--help|help) usage; exit 0 ;; *) usage >&2; exit 2 ;; esac

source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"

# Prints a JSON report. Exit: 0 ready, 3 no camera in this domain, 4 missing
# required streams, 5 a RealSense process exists without topics here, 6 several RGB owners.
probe() {
  python3 - "$@" <<'PY'
import json, os, sys, time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener
need = set(sys.argv[1:])
topics = {'color': '/r680/d455/color/image_raw', 'depth': '/r680/d455/aligned_depth_to_color/image_raw',
          'infra1': '/r680/d455/infra1/image_rect_raw', 'infra2': '/r680/d455/infra2/image_rect_raw',
          'imu': '/r680/d455/imu'}
required = {'color', 'depth'} | ({'infra1', 'infra2'} if 'infra' in need else set()) | ({'imu'} if 'imu' in need else set())
process = False
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try: target = os.readlink(entry / 'exe')
    except OSError: continue
    process = process or target.endswith('/realsense2_camera_node')
rclpy.init()
node = rclpy.create_node('r680_camera_check', enable_rosout=False, start_parameter_services=False)
tf = Buffer(); listener = TransformListener(tf, node)
mounted = lambda: bool(tf.can_transform('r680_mapping_floor', 'd455_link', rclpy.time.Time()))
streams = {}
until = time.monotonic() + 3.0
while time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.1)
    streams = {k: len(node.get_publishers_info_by_topic(t)) for k, t in topics.items()}
    if streams['color'] and all(streams[k] for k in required): break
# The static mount edge is latched; give it a short window once the streams are known.
until = time.monotonic() + 1.0
while not mounted() and time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.1)
emitter = None
if streams['color']:
    client = node.create_client(GetParameters, '/r680/d455/get_parameters')
    if client.wait_for_service(timeout_sec=1.0):
        future = client.call_async(GetParameters.Request(names=['depth_module.emitter_enabled']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if future.done() and future.result() and future.result().values:
            emitter = 'on' if future.result().values[0].integer_value else 'off'
missing = sorted(k for k in required if not streams.get(k))
report = dict(camera=bool(streams['color']), process=process, streams={k: bool(v) for k, v in streams.items()},
              emitter=emitter, mount=mounted(), required=sorted(required), missing=missing)
print(json.dumps(report, ensure_ascii=False))
node.destroy_node(); rclpy.shutdown()
if streams['color'] > 1: sys.exit(6)
if not streams['color']: sys.exit(5 if process else 3)
sys.exit(4 if missing else 0)
PY
}

need=()
[[ "$INFRA" == true ]] && need+=(infra)
[[ "$IMU" == true ]] && need+=(imu)
case "$operation" in
  check) probe "${need[@]}" ;;
  status) probe "${need[@]}" || true ;;
  run)
    mkdir -p "$workspace/.runtime"
    exec 9>"$workspace/.runtime/r680-camera-${ROS_DOMAIN_ID}.lock"
    flock -n 9 || { echo '本 ROS 域已有 r680_camera.sh 在运行' >&2; exit 1; }
    set +e; report="$(probe)"; code=$?; set -e
    if [[ $code -ne 3 ]]; then
      echo "已有 D455 驱动在运行，不再重复打开：$report" >&2
      echo '如需改变红外/IMU/发射器设置，请先在其所在终端关闭它。' >&2
      exit 1
    fi
    if [[ "$IMU" == true ]]; then
      # Same RealSense 4.58.1 IMU dispatch fix that OpenVINS requires.
      fix="$workspace/.runtime/realsense_openvins/install/local_setup.bash"
      [[ -f "$fix" ]] || { echo "缺少 IMU 修复版驱动：bash $workspace/src/wla_openvins_navigation/scripts/build_realsense_imu_fix.sh" >&2; exit 1; }
      source "$fix"
      [[ "$(ros2 pkg prefix realsense2_camera)" == "$workspace/.runtime/realsense_openvins/install/realsense2_camera" ]] || {
        echo "Unexpected RealSense package: $(ros2 pkg prefix realsense2_camera)" >&2; exit 1; }
    fi
    if [[ "$MOUNT" == true && "$report" == *'"mount": true'* ]]; then
      echo '安装 TF 已由其他进程发布，本次不重复发布。'; MOUNT=false
    fi
    echo "[R680 CAMERA] D455：彩色+对齐深度 640x480@30；红外双目=$INFRA；IMU=$IMU；发射器=$EMITTER；安装 TF=$MOUNT；ROS 域 $ROS_DOMAIN_ID"
    exec ros2 launch wla_r680_navigation d455_camera.launch.py infra:="$INFRA" imu:="$IMU" emitter:="$EMITTER" mount:="$MOUNT"
    ;;
esac
