#!/bin/bash
# ════════════════════════════════════════════════════════════
# Gamepad Teleop 启动脚本
# 用法: ./start.sh [选项]
# ════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="humble"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}  Gamepad Teleop 手柄遥控 启动脚本${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"

# ── 检查 ROS2 ──
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo -e "${GREEN}[✓] ROS2 ${ROS_DISTRO} 已加载${NC}"
else
    echo -e "${RED}[✗] 未找到 ROS2 ${ROS_DISTRO}，请检查安装${NC}"
    exit 1
fi

# ── 检查编译 ──
if [ ! -d "${SCRIPT_DIR}/install" ]; then
    echo -e "${YELLOW}[!] 未检测到编译产物，正在编译...${NC}"
    cd "${SCRIPT_DIR}"
    colcon build --symlink-install
    echo -e "${GREEN}[✓] 编译完成${NC}"
fi

source "${SCRIPT_DIR}/install/setup.bash"
echo -e "${GREEN}[✓] 工作空间已加载${NC}"

# ── 检查手柄设备 ──
DEVICE="/dev/input/js0"
if [ -e "${DEVICE}" ]; then
    echo -e "${GREEN}[✓] 检测到手柄设备: ${DEVICE}${NC}"
else
    echo -e "${YELLOW}[!] 未检测到手柄设备 ${DEVICE}${NC}"
    echo -e "${YELLOW}    请插入手柄后重试，或指定设备路径:${NC}"
    echo -e "${YELLOW}    ./start.sh --device /dev/input/js1${NC}"
    echo -e "${YELLOW}    节点启动后会自动等待设备连接...${NC}"
fi

# ── 解析命令行参数 ──
EXTRA_ARGS=""
for arg in "$@"; do
    case "$arg" in
        --no-deadman)
            EXTRA_ARGS="${EXTRA_ARGS} -p deadman_enabled:=false"
            echo -e "${YELLOW}[!] 死人开关已禁用${NC}"
            ;;
        --single-stick)
            EXTRA_ARGS="${EXTRA_ARGS} -p single_stick_mode:=true"
            echo -e "${GREEN}[✓] 单摇杆模式${NC}"
            ;;
        --low)
            EXTRA_ARGS="${EXTRA_ARGS} -p max_linear_speed:=0.3 -p max_angular_speed:=0.5"
            echo -e "${GREEN}[✓] 低速安全模式${NC}"
            ;;
        --device=*)
            DEV="${arg#*=}"
            EXTRA_ARGS="${EXTRA_ARGS} -p device:=${DEV}"
            echo -e "${GREEN}[✓] 使用设备: ${DEV}${NC}"
            ;;
        --device)
            # 下一个参数是设备路径，跳过（由下方处理）
            ;;
        --topic=*)
            TOPIC="${arg#*=}"
            EXTRA_ARGS="${EXTRA_ARGS} -p cmd_vel_topic:=${TOPIC}"
            echo -e "${GREEN}[✓] 发布话题: ${TOPIC}${NC}"
            ;;
        --help|-h)
            echo ""
            echo "用法: ./start.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --no-deadman       禁用死人开关（不需要按住LT）"
            echo "  --single-stick     单摇杆模式（左摇杆控制前后和转弯）"
            echo "  --low              低速安全模式（限速 0.3m/s）"
            echo "  --device=PATH      指定手柄设备路径（默认 /dev/input/js0）"
            echo "  --topic=TOPIC      指定 cmd_vel 话题名（默认 cmd_vel）"
            echo "  -h, --help         显示帮助"
            echo ""
            echo "示例:"
            echo "  ./start.sh                          # 默认启动"
            echo "  ./start.sh --no-deadman              # 不需要按LT"
            echo "  ./start.sh --single-stick --low      # 单杆 + 低速"
            echo "  ./start.sh --device=/dev/input/js1   # 指定设备"
            echo "  ./start.sh --topic=/robot/cmd_vel    # 指定话题"
            exit 0
            ;;
    esac
done

# ── 处理 --device /dev/input/jsX 格式（空格分隔） ──
PREV=""
for arg in "$@"; do
    if [ "$PREV" = "--device" ]; then
        EXTRA_ARGS="${EXTRA_ARGS} -p device:=${arg}"
        echo -e "${GREEN}[✓] 使用设备: ${arg}${NC}"
    fi
    PREV="$arg"
done

echo -e "${GREEN}────────────────────────────────────────${NC}"
echo -e "${GREEN}  正在启动手柄遥控节点...${NC}"
echo -e "${GREEN}  按 Ctrl+C 退出${NC}"
echo -e "${GREEN}────────────────────────────────────────${NC}"

# ── 启动节点 ──
# 使用 ros2 run + 参数文件 + 命令行覆盖
ros2 run gamepad_control gamepad_teleop --ros-args \
    --params-file "${SCRIPT_DIR}/config/gamepad_params.yaml" \
    ${EXTRA_ARGS}
