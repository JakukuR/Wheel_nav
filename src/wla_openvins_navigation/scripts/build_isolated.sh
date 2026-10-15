#!/usr/bin/env bash
# Usage: build_isolated.sh /absolute/path/to/new-workspace
set -euo pipefail
task_ws=${1:?Specify an isolated workspace directory}
case "$task_ws" in /*) ;; *) echo 'Workspace must be absolute' >&2; exit 2;; esac
task_package=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p -- "$task_ws/src"
task_ws=$(cd -- "$task_ws" && pwd)
if [ "$task_ws" = /home/orin/ros2_ws ]; then
  echo 'Use a separate trial workspace, not the active robot workspace' >&2; exit 2
fi
if [ ! -d "$task_ws/src/open_vins/.git" ]; then
  git clone https://github.com/rpng/open_vins.git "$task_ws/src/open_vins"
fi
task_commit=69488123ed9362dd44b6f28e7f4680abbff1442b
task_current=$(git -C "$task_ws/src/open_vins" rev-parse HEAD)
if [ "$task_current" != "$task_commit" ]; then
  test -z "$(git -C "$task_ws/src/open_vins" status --porcelain)"
  git -C "$task_ws/src/open_vins" checkout --detach "$task_commit"
fi
if git -C "$task_ws/src/open_vins" apply --unidiff-zero --check "$task_package/patches/openvins-jazzy-headers.patch"; then
  git -C "$task_ws/src/open_vins" apply --unidiff-zero "$task_package/patches/openvins-jazzy-headers.patch"
else
  git -C "$task_ws/src/open_vins" apply --unidiff-zero --reverse --check "$task_package/patches/openvins-jazzy-headers.patch"
fi
if ! git -C "$task_ws/src/open_vins" diff --quiet -- . \
  ':!ov_msckf/src/ros/ROS2Visualizer.h' ':!ov_msckf/src/ros/ROSVisualizerHelper.h'; then
  echo 'OpenVINS checkout contains non-header changes; use a clean official checkout' >&2
  exit 2
fi
case "$task_package/" in
  "$task_ws/src/"*) ;; # Already discoverable by colcon; do not add a duplicate symlink.
  *)
if [ ! -e "$task_ws/src/wla_openvins_navigation" ]; then
  ln -s -- "$task_package" "$task_ws/src/wla_openvins_navigation"
elif [ "$(readlink -f "$task_ws/src/wla_openvins_navigation")" != "$task_package" ]; then
  echo 'Existing package path points elsewhere; refusing to overwrite' >&2; exit 2
fi
  ;;
esac
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd -- "$task_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
export MAKEFLAGS=${MAKEFLAGS:--j2}
# Build the official bridge against the same OpenCV as OpenVINS. Keep this
# dependency private, since the rest of the robot can use the system bridge.
task_runtime="$task_ws/.runtime/openvins_official"
task_bridge="$task_runtime/dependencies/vision_opencv"
task_bridge_commit=f5b738d9694f0cee5904440d03912fc249943f8a
if [ ! -d "$task_bridge/.git" ]; then
  mkdir -p -- "$task_runtime/dependencies"
  git clone --depth 1 --branch 4.1.0 https://github.com/ros-perception/vision_opencv.git "$task_bridge"
fi
test "$(git -C "$task_bridge" rev-parse HEAD)" = "$task_bridge_commit"
test -z "$(git -C "$task_bridge" status --porcelain)"
task_opencv=${OpenCV_DIR:-/usr/lib/cmake/opencv4}
colcon --log-base "$task_runtime/cv_bridge_log" build \
  --base-paths "$task_bridge/cv_bridge" --packages-select cv_bridge \
  --build-base "$task_runtime/cv_bridge_build" --install-base "$task_runtime/cv_bridge_install" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DOpenCV_DIR="$task_opencv"
colcon build --executor sequential --packages-up-to wla_openvins_navigation \
  --cmake-args -DENABLE_ARUCO_TAGS=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="$task_opencv" \
  -Dcv_bridge_DIR="$task_runtime/cv_bridge_install/cv_bridge/share/cv_bridge/cmake" \
  -DCMAKE_INSTALL_RPATH="$task_runtime/cv_bridge_install/cv_bridge/lib"
