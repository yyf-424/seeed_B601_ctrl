#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

exec ros2 launch b601_demos target_web.launch.py "$@"
