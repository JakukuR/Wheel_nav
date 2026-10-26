#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/orin/r680_chassis_candidate_ws/install/setup.bash
source /home/orin/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix wla_manual_control)/share/wla_manual_control/config/cyclonedds-local.xml"
exec 9>/home/orin/.cache/r680-manual-base.lock
flock -n 9 || exit 1
task_port=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
while [[ ! -e "$task_port" ]]; do sleep 2; done
if fuser -s "$task_port"; then echo 'Chassis serial port already owned' >&2; exit 1; fi
exec ros2 launch wla_manual_control manual_base.launch.py mux_impl:="${R680_MUX_IMPL:-python}"
