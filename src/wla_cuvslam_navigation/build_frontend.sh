#!/usr/bin/env bash
set -eo pipefail
WLA_WS="${WLA_WS:-$HOME/ros2_ws}"
WLA_SDK="${WLA_SDK:-$WLA_WS/.runtime/cuvslam_validation}"
test -f "$WLA_SDK/sdk_build/bin/libcuvslam.so" || { echo '缺少已验证的 cuVSLAM SDK 构建' >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
cd "$WLA_WS"
colcon --log-base "$WLA_WS/.runtime/cuvslam_navigation/log" build \
  --packages-select wla_cuvslam_navigation \
  --build-base "$WLA_WS/.runtime/cuvslam_navigation/build" \
  --install-base "$WLA_WS/.runtime/cuvslam_navigation/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCUVSLAM_SOURCE_DIR="$WLA_SDK/cuVSLAM" -DCUVSLAM_BUILD_DIR="$WLA_SDK/sdk_build"
colcon build --packages-select wla_r680_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release
install -m 755 "$WLA_WS/src/wla_r680_navigation/scripts/r680_nav.sh" "$WLA_WS/r680_nav.sh"
