#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/r680_chassis_candidate_ws/install/setup.bash"
source "$HOME/ros2_ws/.runtime/cuvslam_validation/ros_install/setup.bash"
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml"
exec ros2 run wla_cuvslam_validation run_validation.py --prepare-camera --start-chassis "$@"
