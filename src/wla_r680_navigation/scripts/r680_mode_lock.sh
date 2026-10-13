#!/usr/bin/env bash
# Source this before starting or stopping any navigation/mapping processes.
wla_acquire_navigation_lock() {
  local runtime_dir="${WLA_RUNTIME_DIR:-$HOME/ros2_ws/.runtime}"
  local domain="${ROS_DOMAIN_ID:-73}"
  [[ "$domain" =~ ^[0-9]+$ ]] || { echo 'Invalid ROS_DOMAIN_ID' >&2; return 2; }
  mkdir -p -- "$runtime_dir"
  exec 9>"$runtime_dir/nav-mapping-${domain}.lock"
  if ! flock -n 9; then
    echo "ROS 域 $domain 已有导航或建图会话；请先正常停止当前会话。" >&2
    return 1
  fi
  # Keep FD 9 open until the owner and its shutdown cleanup have finished.
}
