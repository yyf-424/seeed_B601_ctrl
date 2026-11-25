#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/ros_env.sh
b601_source_workspace
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/ros2_ws/log/ros}"

exec ros2 launch b601_demos target_web.launch.py "$@"
