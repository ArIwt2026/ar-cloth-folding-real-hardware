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

MODE="${1:-all}"

case "$MODE" in
  --cameras|-c|cameras)
    shift || true
    exec python3 /home/iwtros/Documents/ar/camera_grid_viewer.py "$@"
    ;;
  --rviz|-r|rviz)
    shift || true
    exec ros2 launch lbr_bringup global_rviz.launch.py "$@"
    ;;
  --all|all|*)
    GRID_PID=""
    RVIZ_PID=""

    cleanup() {
      trap - INT TERM HUP EXIT
      if [[ -n "$GRID_PID" ]] && kill -0 "$GRID_PID" 2>/dev/null; then
        kill -TERM "$GRID_PID" 2>/dev/null || true
      fi
      if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill -TERM "$RVIZ_PID" 2>/dev/null || true
      fi
      for _ in {1..10}; do
        if (! kill -0 "$GRID_PID" 2>/dev/null) && (! kill -0 "$RVIZ_PID" 2>/dev/null); then
          break
        fi
        sleep 0.1
      done
      if [[ -n "$GRID_PID" ]] && kill -0 "$GRID_PID" 2>/dev/null; then
        kill -KILL "$GRID_PID" 2>/dev/null || true
      fi
      if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill -KILL "$RVIZ_PID" 2>/dev/null || true
      fi
      wait "$GRID_PID" "$RVIZ_PID" 2>/dev/null || true
    }

    trap cleanup INT TERM HUP EXIT

    python3 /home/iwtros/Documents/ar/camera_grid_viewer.py &
    GRID_PID=$!

    ros2 launch lbr_bringup global_rviz.launch.py "$@" &
    RVIZ_PID=$!

    # Wait for EITHER window to close, then cleanly shut down both
    wait -n "$GRID_PID" "$RVIZ_PID" 2>/dev/null || true
    cleanup
    ;;
esac
