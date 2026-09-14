#!/usr/bin/env bash
set -e

# Match the system CycloneDDS configuration so all camera topics (Docker and host) are reachable.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file:///home/iwtros/Documents/ar/cyclonedds.xml"

# Ensure DISPLAY is valid for GUI rendering
display_number="${DISPLAY#:}"
if [ -z "${DISPLAY:-}" ] || [ ! -S "/tmp/.X11-unix/X${display_number%%.*}" ]; then
  for socket in /tmp/.X11-unix/X*; do
    if [ -S "$socket" ]; then
      export DISPLAY=":${socket##*X}"
      break
    fi
  done
fi

source /opt/ros/humble/setup.bash
if [ -f "$HOME/Documents/ar/install/setup.bash" ]; then
  source "$HOME/Documents/ar/install/setup.bash"
fi
if [ -f "$HOME/Documents/ar/teleop_kuka_iiwa7/ros2_ws/install/setup.bash" ]; then
  source "$HOME/Documents/ar/teleop_kuka_iiwa7/ros2_ws/install/setup.bash"
fi

exec python3 /home/iwtros/Documents/ar/camera_grid_viewer.py "$@"
