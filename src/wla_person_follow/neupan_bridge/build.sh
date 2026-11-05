#!/usr/bin/env bash
# Builds the NeuPAN process into ~/ros2_ws/.runtime/neupan (not in git, per machine).
# usage: build.sh <neupan_cpp checkout at 074718d> <prefix with OSQP v1.0.0 + osqp-eigen v0.11.2> <car_50x40.bin>
set -euo pipefail
source_dir="$(cd "$1" && pwd)"; prefix="$(cd "$2" && pwd)"; weights="$3"
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
out="$HOME/ros2_ws/.runtime/neupan"; build="$out/build"
mkdir -p "$out/lib"
cmake -S "$here" -B "$build" -DNEUPAN_SOURCE="$source_dir" -DCMAKE_PREFIX_PATH="$prefix" >/dev/null
nice -n 10 cmake --build "$build" -j4 --target neupan_bridge
cp "$build/neupan_bridge" "$out/neupan_bridge"
cp -P "$prefix"/lib/libosqp.so* "$prefix"/lib/libOsqpEigen.so* "$out/lib/"
cp "$weights" "$out/car_50x40.bin"
cp "$here/r_la.yaml" "$out/r_la.yaml"
git -C "$source_dir" rev-parse --short HEAD 2>/dev/null > "$out/neupan_cpp_commit" || echo 074718d > "$out/neupan_cpp_commit"
echo "installed to $out"
