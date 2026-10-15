#!/usr/bin/env bash
set -Ee -o pipefail

usage() {
  cat <<'EOF'
用法: r680_nav.sh [选项]

  --map NAME             选择 maps 下的地图目录；默认读取 storage.yaml 的 active_map
  --storage-config PATH  指定存储配置文件
  --initial-pose "X Y Z R P Y"  给 RTAB-Map 提供出生点初值（米、弧度）
  --agent-control        Agent task gateway owns navigation; RViz/Web goal bridge disabled
  --enable-motion        显式开放真实底盘导航输出（默认仅定位、规划和预览）
  --openvins             OpenVINS 双目 VIO，默认 D455 内置 IMU 标定候选
  --openvins-motion-init 无需手柄的一次实车初始化：静置 5 秒，0.2m/s + 0.5rad/s 最多 1 秒，停稳回 MANUAL；配合 --enable-motion 可随后导航
  --openvins-imu d455|wheel  选择成套 IMU 标定（默认 d455）
  --openvins-config PATH  使用指定的完整 OpenVINS 配置目录
  --cuvslam              使用 cuVSLAM 双目 + 车身 IMU（试验前端，初始化前禁止运动）
  --no-degraded-odom      禁用短时轮速+IMU降级，恢复视觉失效即停车的行为
  --auto-vio-init        VIO 启动受限运动初始化（仅已确认空旷的起步区；无 --enable-motion 则仅预览）
  --rgbd                 使用原 RGB-D VO + EKF（默认，回退入口）
  --mpc                  显式使用 MPC（当前默认；全局静态地图，局部点云避障）
  --mppi                 切回 nav2.yaml/MPPI
  --scan-obstacles       局部代价地图试用 D455 点云生成的 LaserScan（默认仍用点云）
  --test-scan-only       使用独立 nav2_test.yaml：MPC + 局部 Scan，关闭全局 D455 动态层
  --test-recovery        使用 nav2_recovery_test.yaml：MPC + 局部 Scan + 有时限的全局动态障碍
  --no-rviz              不启动 RViz
  -h, --help             显示帮助

示例:
  ./r680_nav.sh
  ./r680_nav.sh --map map-2026-09-20-1
  ./r680_nav.sh --map map-2026-09-20-1 --initial-pose "0 0 0 0 0 0"
  ./r680_nav.sh --map map-2026-09-20-1 --enable-motion
  ./r680_nav.sh --map map-2026-09-20-1 --scan-obstacles
  ./r680_nav.sh --map map-2026-09-20-1 --test-scan-only
EOF
}

MAP_NAME=""
STORAGE_CONFIG=""
INITIAL_POSE=""
ENABLE_MOTION=false
USE_MPC=true
ODOM_SOURCE=rgbd
OPENVINS_CONFIG=""
OPENVINS_IMU=d455
OPENVINS_ALLOW_COMMISSIONING=false
USE_D455_IMU=false
AUTO_VIO_INIT=false
OPENVINS_MOTION_INIT=false
DISABLE_DEGRADED_ODOM=false
SCAN_OBSTACLES=false
TEST_SCAN_ONLY=false
TEST_RECOVERY=false
START_RVIZ=true
AGENT_CONTROL=false
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
    --agent-control) AGENT_CONTROL=true; shift ;;
    --enable-motion) ENABLE_MOTION=true; shift ;;
    --auto-vio-init) AUTO_VIO_INIT=true; shift ;;
    --openvins) ODOM_SOURCE=openvins; shift ;;
    --openvins-motion-init) OPENVINS_MOTION_INIT=true; shift ;;
    --openvins-config)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo '--openvins-config requires a directory' >&2; exit 2; }
      OPENVINS_CONFIG="$2"; shift 2 ;;
    --openvins-imu)
      [[ $# -ge 2 && ( "$2" == d455 || "$2" == wheel ) ]] || { echo '--openvins-imu must be d455 or wheel' >&2; exit 2; }
      OPENVINS_IMU="$2"; shift 2 ;;
    --cuvslam) ODOM_SOURCE=cuvslam; shift ;;
    --no-degraded-odom) DISABLE_DEGRADED_ODOM=true; shift ;;
    --rgbd) ODOM_SOURCE=rgbd; shift ;;
    --mpc) USE_MPC=true; shift ;;
    --mppi) USE_MPC=false; shift ;;
    --scan-obstacles) SCAN_OBSTACLES=true; shift ;;
    --test-scan-only) TEST_SCAN_ONLY=true; SCAN_OBSTACLES=true; shift ;;
    --test-recovery) TEST_RECOVERY=true; SCAN_OBSTACLES=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ "$TEST_RECOVERY" == true && ( "$TEST_SCAN_ONLY" == true || "$USE_MPC" == false ) ]]; then
  echo '--test-recovery 不能与 --test-scan-only 或 --mppi 同时使用' >&2; exit 2
fi

if [[ "$AUTO_VIO_INIT" == true && "$ODOM_SOURCE" != cuvslam && "$ODOM_SOURCE" != openvins ]]; then
  echo '--auto-vio-init 需要同时指定 --cuvslam 或 --openvins' >&2; exit 2
fi
if [[ -n "$OPENVINS_CONFIG" && "$ODOM_SOURCE" != openvins ]]; then
  echo '--openvins-config requires --openvins' >&2; exit 2
fi
if [[ "$OPENVINS_MOTION_INIT" == true ]]; then
  if [[ "$ODOM_SOURCE" != openvins || "$AUTO_VIO_INIT" == true || "$AGENT_CONTROL" == true ]]; then
    echo '--openvins-motion-init requires --openvins and cannot combine with --auto-vio-init or --agent-control' >&2; exit 2
  fi
  systemctl is-active --quiet r680-manual-base.service || { echo 'One-shot initialization requires the independent manual-base service' >&2; exit 2; }
fi
if [[ "$ODOM_SOURCE" == openvins && "$ENABLE_MOTION" == true ]]; then
  [[ "$AGENT_CONTROL" == false ]] || { echo 'OpenVINS standalone motion session cannot own Agent control' >&2; exit 2; }
  systemctl is-active --quiet r680-manual-base.service || { echo 'OpenVINS navigation requires the independent manual-base service' >&2; exit 2; }
fi
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$WORKSPACE/install/setup.bash"
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  CUVSLAM_SETUP="$HOME/ros2_ws/.runtime/cuvslam_navigation/install/setup.bash"
  [[ -f "$CUVSLAM_SETUP" ]] || { echo 'cuVSLAM 导航前端未构建，使用 --rgbd 回退' >&2; exit 1; }
  source "$CUVSLAM_SETUP"
  ros2 pkg prefix wla_cuvslam_navigation >/dev/null
fi
if [[ "$ODOM_SOURCE" == openvins ]]; then
  ros2 pkg prefix wla_openvins_navigation >/dev/null
  if [[ -z "$OPENVINS_CONFIG" ]]; then
    OPENVINS_CONFIG="$(ros2 pkg prefix wla_openvins_navigation)/share/wla_openvins_navigation/config/r680_${OPENVINS_IMU}_20261013"
  fi
  OPENVINS_CONFIG=$(realpath -e -- "$OPENVINS_CONFIG")
  OPENVINS_FLAGS=$(ros2 run wla_openvins_navigation inspect_config.py "$OPENVINS_CONFIG" --hardware-output "$ENABLE_MOTION")
  read -r OPENVINS_ALLOW_COMMISSIONING USE_D455_IMU <<<"$OPENVINS_FLAGS"
  echo "[OpenVINS] 配置: $OPENVINS_CONFIG；候选标定=$OPENVINS_ALLOW_COMMISSIONING；D455 IMU=$USE_D455_IMU"
fi
set -u
# A second driver cannot share this serial port. Do not terminate operator sessions.
USE_MANUAL_BASE=false
START_CHASSIS=true
if systemctl is-active --quiet r680-manual-base.service; then
  if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != 73 ]]; then
    echo '常驻底盘位于 ROS 域 73；当前域不同，拒绝静默切域。测试请直接使用隔离 launch。' >&2
    exit 2
  fi
  USE_MANUAL_BASE=true; START_CHASSIS=false; export ROS_DOMAIN_ID=73
  echo '[R680 NAV] 复用常驻底盘，手柄可随时接管；自动模式需显式授权。'
  echo '[R680 NAV] 导航预览/失败不关闭独立手柄；停稳后可调用 /r680_base/set_automatic 显式切换。'
fi
if [[ "$USE_MANUAL_BASE" != true ]]; then
python3 - <<'PYCHECK'
from pathlib import Path
import os
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        target = os.readlink(entry / 'exe')
    except OSError: continue
    if target.endswith('/wheeltec_robot_node'):
        raise SystemExit('底盘驱动已在运行；请先停止独立底盘/手柄启动会话，再运行导航，避免串口冲突。')
PYCHECK
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/ros2_ws/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"

# Navigation and mapping own the same TF/SLAM/control graph.
source "$WORKSPACE/src/wla_r680_navigation/scripts/r680_mode_lock.sh"
wla_acquire_navigation_lock

CONFIG_DIR=$(ros2 pkg prefix wla_r680_navigation)/share/wla_r680_navigation/config
NAV2_PARAMS_FILE="$CONFIG_DIR/nav2.yaml"
if [[ "$USE_MPC" == true ]]; then
  NAV2_PARAMS_FILE="$CONFIG_DIR/nav2_mpc.yaml"
  [[ -f "$NAV2_PARAMS_FILE" ]] || { echo "MPC 参数文件不存在: $NAV2_PARAMS_FILE" >&2; exit 1; }
  ros2 pkg prefix wla_diff_mpc >/dev/null || { echo '未构建 wla_diff_mpc 插件' >&2; exit 1; }
  export LD_LIBRARY_PATH="$HOME/.local/wla_mpc_deps/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ "$TEST_SCAN_ONLY" == true ]]; then
  [[ "$USE_MPC" == true ]] || { echo '--test-scan-only 仅支持 MPC，不能与 --mppi 同时使用' >&2; exit 2; }
  NAV2_PARAMS_FILE="$CONFIG_DIR/nav2_test.yaml"
  [[ -f "$NAV2_PARAMS_FILE" ]] || { echo "测试参数文件不存在: $NAV2_PARAMS_FILE" >&2; exit 1; }
fi
if [[ -z "$STORAGE_CONFIG" ]]; then
  STORAGE_CONFIG="$CONFIG_DIR/storage.yaml"
fi
if [[ "$TEST_RECOVERY" == true ]]; then
  NAV2_PARAMS_FILE="$CONFIG_DIR/nav2_recovery_test.yaml"
  [[ -f "$NAV2_PARAMS_FILE" ]] || { echo "测试参数文件不存在: $NAV2_PARAMS_FILE" >&2; exit 1; }
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
if [[ "$SCAN_OBSTACLES" == true && "$TEST_SCAN_ONLY" == false && "$TEST_RECOVERY" == false ]]; then
  python3 - "$NAV2_PARAMS_FILE" "$NAV_RUN_DIR/nav2_scan.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    params = yaml.safe_load(stream)
layer = params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
original = layer.pop('d455_obstacles')
sources = layer['observation_sources'].split()
if sources != ['d455_obstacles', 'd455_clearing']:
    raise SystemExit(f'unexpected local obstacle sources: {sources}')
layer['observation_sources'] = 'd455_scan d455_clearing'
layer['d455_scan'] = dict(original, topic='/r680_nav/d455/scan', data_type='LaserScan')
with open(sys.argv[2], 'w', encoding='utf-8') as stream:
    yaml.safe_dump(params, stream, sort_keys=False, allow_unicode=True)
PY
  NAV2_PARAMS_FILE="$NAV_RUN_DIR/nav2_scan.yaml"
fi
# Derive metadata and memory-node ownership from the selected costmap file.
# Recovery tests use an in-costmap timed layer, without points_confirmed.
GLOBAL_OBSTACLE_MODE=$(python3 - "$NAV2_PARAMS_FILE" <<'PYGLOBAL'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    params = yaml.safe_load(stream)['global_costmap']['global_costmap']['ros__parameters']
layers = [params[name] for name in params['plugins']
          if params[name].get('enabled', True) and params[name].get('plugin') in (
              'nav2_costmap_2d::ObstacleLayer', 'nav2_costmap_2d::VoxelLayer',
              'wla_r680_navigation::TimedObstacleLayer')]
memory = any(layer.get(source, {}).get('topic') == '/r680_nav/d455/points_confirmed'
             for layer in layers for source in layer.get('observation_sources', '').split())
print(str(bool(layers)).lower(), str(memory).lower())
PYGLOBAL
)
read -r GLOBAL_DYNAMIC_OBSTACLES_ENABLED START_DYNAMIC_OBSTACLES <<<"$GLOBAL_OBSTACLE_MODE"
SOURCE_DB_PATH="$DB_PATH"
DB_PATH="$NAV_RUN_DIR/rtabmap.db"
cp --reflink=auto "$SOURCE_DB_PATH" "$DB_PATH"
printf '%s\n' "$MAP_JSON" > "$NAV_RUN_DIR/selected_map.json"
if [[ "$ODOM_SOURCE" == openvins ]]; then
  cp -a -- "$OPENVINS_CONFIG" "$NAV_RUN_DIR/openvins_config"
  OPENVINS_CONFIG="$NAV_RUN_DIR/openvins_config"
fi
cat > "$NAV_RUN_DIR/navigation.json" <<EOF
{
  "selected_map": "$MAP_DIR",
  "map_yaml": "$MAP_YAML",
  "static_map_source": "map_server",
  "static_map_topic": "/r680/d455/map",
  "rtabmap_debug_map_topic": "/d455_slam/localization_map",
  "source_database": "$SOURCE_DB_PATH",
  "working_database": "$DB_PATH",
  "initial_pose": "$INITIAL_POSE",
  "odom_source": "$ODOM_SOURCE",
  "auto_vio_init": $AUTO_VIO_INIT,
  "openvins_motion_init": $OPENVINS_MOTION_INIT,
  "openvins_config_dir": "$OPENVINS_CONFIG",
  "hardware_output_enabled": $ENABLE_MOTION,
  "local_obstacle_input": "$(if [[ "$SCAN_OBSTACLES" == true ]]; then echo scan; else echo pointcloud; fi)",
  "global_dynamic_obstacles_enabled": $GLOBAL_DYNAMIC_OBSTACLES_ENABLED,
  "recovery_test": $TEST_RECOVERY,
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
BRINGUP_PID=""; PERMISSION_PID=""; RVIZ_PID=""; INIT_MOTION_PID=""; STARTED=false
READONLY_INPUTS_ACTIVE=false

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -TERM "$pid" 2>/dev/null || true
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
  trap '' INT TERM HUP
  set +e
  echo
  echo '[R680 NAV] 正在撤销运动授权并停止导航节点……'
  if [[ "$AUTO_VIO_INIT" == true && "$STARTED" == true ]]; then
    timeout 2 ros2 topic pub --once /r680_nav/vio_init_cancel std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
  fi
  stop_group "$INIT_MOTION_PID"
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
  fi
  for unit in "${PAUSED_UNITS[@]}"; do
    systemctl --user start "$unit" >/dev/null 2>&1 || true
  done
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
if [[ "$ODOM_SOURCE" == cuvslam || "$ODOM_SOURCE" == openvins ]]; then
  for unit in r680-d455-observation.service r680-readonly-inputs.service; do
    if systemctl --user is-active --quiet "$unit"; then
      PAUSED_UNITS+=("$unit")
      systemctl --user stop "$unit"
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

if [[ "$ODOM_SOURCE" == cuvslam || "$ODOM_SOURCE" == openvins ]]; then
  python3 - <<'PYCAMERA'
from pathlib import Path
import os
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        target = os.readlink(entry / 'exe')
    except OSError: continue
    if target.endswith('/realsense2_camera_node'):
        raise SystemExit('仍有独立 RealSense 驱动运行；请先关闭该相机会话，避免重复打开 D455。')
PYCAMERA
fi

echo "[R680 NAV] 地图：$MAP_DIR"
echo "[R680 NAV] 二维栅格：$MAP_YAML"
echo "[R680 NAV] RTAB-Map 只读源库：$SOURCE_DB_PATH"
echo "[R680 NAV] RTAB-Map 本次工作副本：$DB_PATH"
echo "[R680 NAV] 本次日志：$NAV_RUN_DIR"
echo "[R680 NAV] Nav2 参数：$NAV2_PARAMS_FILE"
echo "[R680 NAV] 全局动态障碍：$GLOBAL_DYNAMIC_OBSTACLES_ENABLED；局部避障保持开启"
echo "[R680 NAV] 局部障碍标记：$(if [[ "$SCAN_OBSTACLES" == true ]]; then echo '/r680_nav/d455/scan'; else echo '/r680_nav/d455/points'; fi)"
cp "$NAV2_PARAMS_FILE" "$NAV_RUN_DIR/nav2.yaml"
echo "[R680 NAV] 里程计前端：$ODOM_SOURCE"
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  cp "$CONFIG_DIR/cuvslam.yaml" "$NAV_RUN_DIR/cuvslam.yaml"
  cp "$CONFIG_DIR/degraded_odometry.yaml" "$NAV_RUN_DIR/degraded_odometry.yaml"
  if [[ "${DISABLE_DEGRADED_ODOM:-false}" == true ]]; then
    python3 - "$NAV_RUN_DIR/degraded_odometry.yaml" <<'PY'
import sys, yaml
path = sys.argv[1]
with open(path) as stream:
    config = yaml.safe_load(stream)
config['r680_degraded_odometry']['ros__parameters']['enabled'] = False
with open(path, 'w') as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
PY
  fi
  python3 - "$NAV_RUN_DIR/degraded_odometry.yaml" <<'PY'
import sys, yaml
with open(sys.argv[1]) as stream:
    p = yaml.safe_load(stream)['r680_degraded_odometry']['ros__parameters']
print(f"[R680 NAV] 短时降级：{'启用' if p['enabled'] else '禁用'}；"
      f"预算 {p['max_degraded_s']}s / {p['max_degraded_travel_m']}m / "
      f"{p['max_degraded_turn_rad']}rad；配置：{sys.argv[1]}")
PY
  if [[ "$AUTO_VIO_INIT" == true ]]; then
    cp "$CONFIG_DIR/vio_initializer_cuvslam.yaml" "$NAV_RUN_DIR/vio_initializer.yaml"
    echo "[R680 NAV] 自动初始化：最大请求 0.20m/s、0.50rad/s，运动最长 2s，路径预算 0.25m；真实输出=$ENABLE_MOTION"
    echo '[R680 NAV] 此选项用于已确认空旷的起步区；初始化失败将停车锁定，不自动重试。'
  else
    echo '[R680 NAV] cuVSLAM 初始化前运动锁定；需纹理场景和适量真实运动激励，预览时可手推。'
  fi
elif [[ "$ODOM_SOURCE" == openvins ]]; then
  if [[ "$OPENVINS_MOTION_INIT" == true ]]; then
    echo '[R680 NAV] 一次性实车初始化已显式开启：请保持静止，程序等待有效输入后静置 5 秒，再前进左转最多 1 秒。'
    echo '[R680 NAV] 目标 0.2m/s、0.5rad/s；估算前进约 0.18m、转向约 0.45rad。此实验由操作者确认地面和空间，无接地/障碍检测。'
    echo '[R680 NAV] 手柄输入/急停、传感器失效、1秒时限或轮速里程越界均结束本次尝试；失败不自动再走。'
  else
    echo '[R680 NAV] OpenVINS：先静置至少 3 秒，再给出运动激励；未初始化时不发布 odom。'
  fi
  echo '[R680 NAV] 当前成套标定为联调候选；--enable-motion 明确授权导航，标定验收状态保持原样。'
  if [[ "$AUTO_VIO_INIT" == true ]]; then
    cp "$CONFIG_DIR/vio_initializer.yaml" "$NAV_RUN_DIR/vio_initializer.yaml"
    echo '[R680 NAV] 自动初始化仅预览；还需真实接地状态、有效深度和安全链就绪。'
  fi
else
  echo '[R680 NAV] 前 5 秒保持车辆静止，正在估计车身 IMU 零偏……'
fi

LAUNCH_ARGS=(
  mode:=localization database_path:="$DB_PATH" navigation_map_yaml:="$MAP_YAML" web_map_yaml:="$MAP_YAML"
  nav_params_file:="$NAV2_PARAMS_FILE"
  obstacle_scan:="$SCAN_OBSTACLES"
  start_dynamic_obstacles:="$START_DYNAMIC_OBSTACLES"
  start_d455:="$(if [[ "$ODOM_SOURCE" == cuvslam || "$ODOM_SOURCE" == openvins ]]; then echo true; else echo false; fi)" start_chassis:="$START_CHASSIS" start_nav2:=true start_navigation_servers:=true
  start_state_estimation:=true use_d455_imu:="$USE_D455_IMU" use_chassis_imu:=true
  odom_source:="$ODOM_SOURCE" cuvslam_statistics_path:="$NAV_RUN_DIR/cuvslam_statistics.json"
  auto_vio_init:="$AUTO_VIO_INIT" vio_init_result_path:="$NAV_RUN_DIR/vio_init.json"
  publish_mount_tf:=true enable_hardware_output:="$(if [[ "$ODOM_SOURCE" == openvins ]]; then echo false; else echo "$ENABLE_MOTION"; fi)" agent_control:="$AGENT_CONTROL"
  start_semantics:=false semantic_output:="$MAP_DIR/semantic.geojson"
  semantic_map_id:="$(basename "$MAP_DIR")" semantic_mark_home:=false
)
# ROS 2 CLI rejects name:= with an empty value. Other frontends use launch defaults.
if [[ "$ODOM_SOURCE" == openvins ]]; then
  LAUNCH_ARGS+=(openvins_config_dir:="$OPENVINS_CONFIG" openvins_allow_commissioning:="$OPENVINS_ALLOW_COMMISSIONING")
fi
if [[ "$ODOM_SOURCE" == cuvslam ]]; then
  LAUNCH_ARGS+=(degraded_odom_params_file:="$NAV_RUN_DIR/degraded_odometry.yaml"
    enable_degraded_odom:="$(if [[ "$DISABLE_DEGRADED_ODOM" == true ]]; then echo false; else echo true; fi)")
fi
if [[ -n "$INITIAL_POSE" ]]; then
  LAUNCH_ARGS+=(initial_pose:="$INITIAL_POSE")
fi
setsid ros2 launch wla_r680_navigation bringup.launch.py "${LAUNCH_ARGS[@]}" \
  >"$NAV_RUN_DIR/logs/bringup.log" 2>&1 </dev/null &
BRINGUP_PID=$!
STARTED=true
if [[ "$ODOM_SOURCE" == cuvslam && "$AUTO_VIO_INIT" == true && "$ENABLE_MOTION" == true ]]; then
  setsid ros2 run wla_r680_navigation navigation_motion_session.py --cuvslam-init \
    >"$NAV_RUN_DIR/logs/navigation_motion_session.log" 2>&1 </dev/null &
  PERMISSION_PID=$!
  echo '[R680 NAV] 初始化授权：停稳后先切入底盘 AUTO/NAV，再允许受限激励；失败或手柄接管即撤销。'
fi
if [[ "$OPENVINS_MOTION_INIT" == true ]]; then
  setsid ros2 run wla_r680_navigation openvins_motion_init.py --execute \
    --config "$OPENVINS_CONFIG" --result "$NAV_RUN_DIR/init_motion.json" >"$NAV_RUN_DIR/logs/init_motion.log" 2>&1 </dev/null &
  INIT_MOTION_PID=$!
fi
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

WLA_BRINGUP_PID="$BRINGUP_PID" WLA_INIT_MOTION_RESULT="$NAV_RUN_DIR/init_motion.json" WLA_OPENVINS_MOTION_INIT="$OPENVINS_MOTION_INIT" WLA_BRINGUP_LOG="$NAV_RUN_DIR/logs/bringup.log" WLA_REQUIRE_SCAN="$SCAN_OBSTACLES" WLA_ODOM_SOURCE="$ODOM_SOURCE" WLA_AUTO_VIO_INIT="$AUTO_VIO_INIT" python3 - <<'PY'
import json
import math
import os
import time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from wla_r680_navigation_py.startup_evidence import StartupEvidence

rclpy.init()
node = Node('r680_navigation_ready_waiter')
ready_count = 0
last_ready = 0.0
map_ok = False
require_scan = os.environ['WLA_REQUIRE_SCAN'] == 'true'
last_scan = 0.0
init_state = 'waiting_inputs'
auto_init = os.environ['WLA_AUTO_VIO_INIT'] == 'true'
motion_init = os.environ['WLA_OPENVINS_MOTION_INIT'] == 'true'
estimator_state = '等待前端状态'
frontend_status = '等待传感器' if os.environ['WLA_ODOM_SOURCE'] in ('cuvslam', 'openvins') else 'rgbd'

def init_cb(msg):
    global init_state
    init_state = msg.data

def frontend_cb(msg):
    global frontend_status
    frontend_status = msg.data

def ready_cb(msg):
    global ready_count, last_ready
    ready_count = ready_count + 1 if msg.data else 0
    last_ready = time.monotonic()

def map_cb(msg):
    global map_ok
    map_ok = msg.info.width > 0 and msg.info.height > 0 and len(msg.data) > 0

def scan_cb(msg):
    global last_scan
    if msg.header.frame_id == 'r680_mapping_floor' and msg.ranges:
        last_scan = time.monotonic()

node.create_subscription(Bool, '/r680_nav/localization_ready', ready_cb, 10)
if os.environ['WLA_ODOM_SOURCE'] == 'openvins':
    from nav_msgs.msg import Odometry
    def estimator_cb(msg):
        global estimator_state, frontend_status
        estimator_state = 'official_odom_received'
        frontend_status = 'official_run_subscribe_msckf'
    node.create_subscription(Odometry, '/openvins/odomimu', estimator_cb, qos_profile_sensor_data)
if motion_init:
    node.create_subscription(String, '/r680_nav/init_motion_state', init_cb, 10)
if os.environ['WLA_ODOM_SOURCE'] == 'cuvslam':
    node.create_subscription(String, '/r680_nav/vo_watchdog_status', frontend_cb, 10)
if auto_init:
    node.create_subscription(String, '/r680_nav/vio_init_state', init_cb, 10)
map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(OccupancyGrid, '/r680/d455/map', map_cb, map_qos)
if require_scan:
    node.create_subscription(LaserScan, '/r680_nav/d455/scan',
                             scan_cb, qos_profile_sensor_data)
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
names = ['controller_server', 'planner_server', 'bt_navigator',
         'velocity_smoother', 'collision_monitor']
clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in names}
startup_timeout = 450.0 if os.environ['WLA_ODOM_SOURCE'] == 'openvins' else 120.0
deadline = time.monotonic() + startup_timeout
last_report = 0.0
bringup_log = Path(os.environ['WLA_BRINGUP_LOG'])
startup = StartupEvidence(bringup_log, os.environ['WLA_ODOM_SOURCE'],
                          bringup_pid=int(os.environ['WLA_BRINGUP_PID']))
while time.monotonic() < deadline:
    manager_active = startup.read()
    rclpy.spin_once(node, timeout_sec=0.2)
    if auto_init and init_state == 'failed':
        raise SystemExit('自动 VIO 初始化失败并已锁定；查看本次 vio_init.json，退出后可用 --rgbd 回退')
    states = {name: 0 for name in names}
    # OpenVINS STARTUP must have returned success before polling lifecycle services.
    # The static-map manager's success cannot authorize navigation readiness.
    if manager_active:
        for name, client in clients.items():
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
            states[name] = future.result().current_state.id if future.done() and future.result() else 0
    tf_ok = tf_buffer.can_transform('map', 'r680_mapping_floor', rclpy.time.Time(),
                                    timeout=Duration(seconds=0.05))
    scan_ok = not require_scan or time.monotonic() - last_scan < 0.6
    motion_ok = not motion_init
    if motion_init:
        result_path = Path(os.environ['WLA_INIT_MOTION_RESULT'])
        if result_path.exists():
            try:
                motion_result = json.loads(result_path.read_text())
            except (OSError, ValueError):
                motion_result = {}
            if motion_result.get('state') == 'failed':
                raise SystemExit('一次性初始化已停止: ' + motion_result.get('reason', 'unknown') + '; 查看 logs/init_motion.log')
            motion_ok = motion_result.get('state') in ('succeeded', 'already_initialized')
    interface_ok = ready_count >= 5 and time.monotonic() - last_ready < 0.5
    if motion_ok and interface_ok and map_ok and tf_ok and scan_ok and (not auto_init or init_state == 'succeeded') and all(v == 3 for v in states.values()):
        # A failure may have arrived while the blocking lifecycle queries ran.
        if not startup.read():
            continue
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    if time.monotonic() - last_report > 10.0:
        print('[R680 NAV] 等待就绪:', f'接口={interface_ok}', f'地图={map_ok}',
              f'TF={tf_ok}', f'Scan={scan_ok}', f'前端诊断={frontend_status}', f'自动运动初始化={init_state if auto_init or motion_init else "关闭，需手柄激励"}',
              f'估计器={estimator_state}',
              '生命周期=' + ','.join(f'{k}:{v}' for k, v in states.items()),
              flush=True)
        last_report = time.monotonic()
node.destroy_node()
rclpy.shutdown()
raise SystemExit(f'{startup_timeout:g} 秒内导航未就绪；检查本次 logs/bringup.log')
PY

if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
  echo '[R680 NAV] 导航主进程在就绪检查后已退出。' >&2
  exit 1
fi

if [[ "$ENABLE_MOTION" == true && "$ODOM_SOURCE" == openvins ]]; then
  if [[ -n "$INIT_MOTION_PID" ]]; then wait "$INIT_MOTION_PID"; INIT_MOTION_PID=""; fi
  setsid ros2 run wla_r680_navigation navigation_motion_session.py \
    >"$NAV_RUN_DIR/logs/navigation_motion_session.log" 2>&1 </dev/null &
  PERMISSION_PID=$!
  echo '[R680 NAV] 初始化与 Nav2 启动检查通过；正在确认当前健康状态并交接 AUTO/NAV。状态暂未就绪时保持零速等待，不重复运动初始化。'
  handoff_reports=0
  while kill -0 "$PERMISSION_PID" 2>/dev/null && kill -0 "$BRINGUP_PID" 2>/dev/null; do
    grep -q 'navigation_enabled;' "$NAV_RUN_DIR/logs/navigation_motion_session.log" && break
    if (( handoff_reports % 10 == 0 )); then
      tail -1 "$NAV_RUN_DIR/logs/navigation_motion_session.log"
    fi
    handoff_reports=$((handoff_reports + 1))
    sleep 1
  done
  if ! grep -q 'navigation_enabled;' "$NAV_RUN_DIR/logs/navigation_motion_session.log"; then
    tail -10 "$NAV_RUN_DIR/logs/navigation_motion_session.log" >&2
    echo '[R680 NAV] 导航控制交接失败，退出清理。' >&2; exit 1
  fi
  echo '[R680 NAV] navigation_enabled：已接管 AUTO/NAV，现在可以发送新的导航点。'
elif [[ "$ENABLE_MOTION" == true && "$ODOM_SOURCE" == cuvslam && "$AUTO_VIO_INIT" == true ]]; then
  handoff_deadline=$((SECONDS + 30))
  while (( SECONDS < handoff_deadline )) && kill -0 "$PERMISSION_PID" 2>/dev/null && kill -0 "$BRINGUP_PID" 2>/dev/null; do
    grep -q 'navigation_enabled;' "$NAV_RUN_DIR/logs/navigation_motion_session.log" && break
    sleep 0.2
  done
  if ! grep -q 'navigation_enabled;' "$NAV_RUN_DIR/logs/navigation_motion_session.log"; then
    tail -10 "$NAV_RUN_DIR/logs/navigation_motion_session.log" >&2
    echo '[R680 NAV] 初始化后的导航授权交接失败。' >&2; exit 1
  fi
  echo '[R680 NAV] 初始化完成且 AUTO/NAV 已确认，现在可以发送新的导航点。'
elif [[ "$ENABLE_MOTION" == true ]]; then
  setsid ros2 topic pub -r 10 /r680_nav/mission_motion_allowed \
    std_msgs/msg/Bool '{data: true}' >"$NAV_RUN_DIR/logs/motion_permission.log" 2>&1 </dev/null &
  PERMISSION_PID=$!
  echo '[R680 NAV] 导航侧速度输出已允许；此步骤未切换底盘 AUTO 模式。'
  echo '[R680 NAV] 停稳后调用 /r680_base/set_automatic（data: true），确认 control_state 为 AUTO/NAV，再下发新的导航点。'
else
  echo '[R680 NAV] 当前为安全预览：定位、规划和碰撞链已启动，真实底盘输出保持关闭。'
  if [[ "$ODOM_SOURCE" == openvins ]]; then
    echo '[R680 NAV] 下次加 --enable-motion 可在初始化完成后自动接管导航；可与 --openvins-motion-init 合用。'
  else
    echo '[R680 NAV] 需要实车运动时停止本脚本，再显式加 --enable-motion 启动。'
  fi
fi
echo '[R680 NAV] 就绪检查只证明传感器、地图、TF 与 Nav2 正常，不等同于视觉重定位已匹配正确。'
echo '[R680 NAV] 按 Ctrl+C 将撤销运动授权、发送零速、关闭本次节点并恢复原定位服务。'

while kill -0 "$BRINGUP_PID" 2>/dev/null; do
  if [[ -n "$PERMISSION_PID" ]] && ! kill -0 "$PERMISSION_PID" 2>/dev/null; then
    echo '[R680 NAV] 导航授权进程已结束，退出清理。' >&2
    exit 1
  fi
  sleep 1
done
echo '[R680 NAV] 导航主进程意外退出。' >&2
exit 1
