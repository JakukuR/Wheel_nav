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
if git -C "$task_ws/src/open_vins" apply --check "$task_package/patches/openvins-v1.patch"; then
  git -C "$task_ws/src/open_vins" apply "$task_package/patches/openvins-v1.patch"
else
  git -C "$task_ws/src/open_vins" apply --reverse --check "$task_package/patches/openvins-v1.patch"
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
colcon build --executor sequential --packages-up-to wla_openvins_navigation \
  --cmake-args -DOV_BUILD_ROS_VISUALIZER=OFF -DENABLE_ARUCO_TAGS=OFF -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0'
set +u
source install/setup.bash
set -u
colcon test --packages-select wla_openvins_navigation --event-handlers console_direct+
colcon test-result --verbose
