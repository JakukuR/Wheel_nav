#!/usr/bin/env bash
# Build only the pinned ROS wrapper; do not replace /opt/ros or the sensor SDK.
set -eo pipefail
pkg="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$(cd -- "$pkg/../.." && pwd)"
runtime="$workspace/.runtime/realsense_openvins"
archive="$runtime/realsense-ros-4.58.1.verified.tar.gz"
source_dir="$runtime/src/realsense-ros-4.58.1"
mkdir -p "$runtime/src"
if [[ ! -f "$archive" ]]; then
  curl -fL --connect-timeout 15 --max-time 120 \
    https://codeload.github.com/realsenseai/realsense-ros/tar.gz/refs/tags/4.58.1 \
    -o "$archive"
fi
echo "d48d0591f5a575a546521cf1fed50f240f2a4c9d5705b6d836ad00a8b689e651  $archive" | sha256sum -c -
[[ -d "$source_dir" ]] || tar xzf "$archive" -C "$runtime/src"
patch_file="$pkg/patches/realsense-4.58.1-imu-mode.patch"
if patch -d "$source_dir" -p1 --dry-run --forward < "$patch_file" >/dev/null; then
  patch -d "$source_dir" -p1 --forward < "$patch_file"
else
  patch -d "$source_dir" -p1 --dry-run --reverse < "$patch_file" >/dev/null
fi
# Jetson's system ROS cv_bridge uses OpenCV 4.6 while development OpenCV is 4.8.
# Reuse the matching private bridge already built for OpenVINS, scoped by RPATH.
bridge="$workspace/.runtime/openvins_official/cv_bridge_install/cv_bridge"
[[ -f "$bridge/lib/libcv_bridge.so" ]] || {
  echo "Missing OpenVINS private cv_bridge: $bridge" >&2; exit 1;
}
source /opt/ros/jazzy/setup.bash
export MAKEFLAGS="-j${REALSENSE_BUILD_JOBS:-2}"
export CMAKE_BUILD_PARALLEL_LEVEL="${REALSENSE_BUILD_JOBS:-2}"
cd "$runtime"
nice -n 15 colcon build --base-paths "$source_dir/realsense2_camera" \
  --packages-select realsense2_camera --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DOpenCV_DIR=/usr/lib/cmake/opencv4 \
  -Dcv_bridge_DIR="$bridge/share/cv_bridge/cmake" \
  -DCMAKE_INSTALL_RPATH="$bridge/lib" \
  -DCMAKE_SHARED_LINKER_FLAGS=-Wl,--disable-new-dtags \
  -DCMAKE_EXE_LINKER_FLAGS=-Wl,--disable-new-dtags
source "$runtime/install/local_setup.bash"
python3 - "$runtime/install/realsense2_camera/lib/librealsense2_camera.so" "$bridge" <<'CHECK_LINKS'
import subprocess,sys
links=subprocess.check_output(['ldd',sys.argv[1]],text=True)
assert 'not found' not in links, links
assert 'libopencv_core.so.408' in links and 'libopencv_core.so.406' not in links, links
assert sys.argv[2]+'/lib/libcv_bridge.so' in links, links
print('PASS: installed driver resolves private cv_bridge and only OpenCV 4.8')
CHECK_LINKS
python3 "$pkg/test/test_realsense_imu_mode.py" "$source_dir"
