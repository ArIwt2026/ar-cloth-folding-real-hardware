#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source "$HOME/Documents/ar/install/setup.bash"
source "$HOME/Documents/ar/teleop_kuka_iiwa7/ros2_ws/install/setup.bash"

exec ros2 launch lbr_bringup global_rviz.launch.py "$@"
