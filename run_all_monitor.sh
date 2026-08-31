#!/usr/bin/env bash
set -Eeuo pipefail

AR_ROOT="/home/iwtros/Documents/ar"
PANDA_DIR="$AR_ROOT/teleop_panda"
LUCID_DIR="$AR_ROOT/lucid_cameras"
IIWA_WS="$AR_ROOT/teleop_kuka_iiwa7/ros2_ws"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
umask 077
LOCK_FILE="/tmp/ar-run-all-monitor-${UID}.lock"
STATE_FILE="/tmp/ar-run-all-monitor-${UID}.state"
BOOT_ID=$(< /proc/sys/kernel/random/boot_id)
RUN_ID=""
MONITOR_PID=""
MONITOR_PGID=""
CLEANUP_STARTED=0

# Only one supervisor may own this robot/camera stack. FD 9 is explicitly
# closed in the ROS child so a crashed supervisor cannot leave the lock held.
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "Another run_all_monitor.sh supervisor is already running." >&2
  exit 1
fi

remove_stale_project_containers() {
  # Remove only containers created by this project/supervisor. Do not use a
  # broad ancestor filter because the image may be used by another workflow.
  local pattern containers
  for pattern in '^/panda_monitor$' 'teleop_panda-teleop_panda-run-' \
      'lucid_cameras-lucid_ros2-' 'lucid_ros2-'; do
    containers=$(docker ps -aq --filter "name=$pattern" 2>/dev/null || true)
    if [[ -n "$containers" ]]; then
      docker rm -f $containers >/dev/null
    fi
  done
}

process_group_alive() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v wanted="$pgid" \
    '$1 == wanted && $2 !~ /^Z/ { found=1 } END { exit !found }'
}

process_group_owned() {
  local pgid="$1" run_id="$2" pid
  while read -r pid; do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | \
        grep -Fqx "AR_MONITOR_RUN_ID=$run_id"; then
      return 0
    fi
  done < <(ps -eo pid=,pgid=,stat= | awk -v wanted="$pgid" \
    '$2 == wanted && $3 !~ /^Z/ { print $1 }')
  return 1
}

stop_launch_group() {
  local pgid="$1" signal i own_pgid
  [[ "$pgid" =~ ^[0-9]+$ ]] && (( pgid > 1 )) || return 0
  own_pgid=$(ps -o pgid= -p $$ | tr -d ' ')
  if [[ "$pgid" == "$own_pgid" ]]; then
    echo "Refusing to signal the supervisor's own process group $pgid." >&2
    return 1
  fi
  process_group_alive "$pgid" || return 0

  for signal in INT TERM KILL; do
    kill -"$signal" -- "-$pgid" 2>/dev/null || true
    # Give ROS ten seconds for INT, five for TERM, then verify KILL.
    local attempts=20
    [[ "$signal" == INT ]] && attempts=40
    [[ "$signal" == KILL ]] && attempts=8
    for ((i=0; i<attempts; i++)); do
      process_group_alive "$pgid" || return 0
      sleep 0.25
    done
  done

  echo "Process group $pgid still exists after forced shutdown." >&2
  return 1
}

recover_interrupted_launch() {
  local old_pid old_pgid old_boot_id old_run_id
  [[ -r "$STATE_FILE" ]] || return 0
  read -r old_pid old_pgid old_boot_id old_run_id < "$STATE_FILE" || true
  if [[ "${old_boot_id:-}" == "$BOOT_ID" ]] && \
      [[ "${old_pgid:-}" =~ ^[0-9]+$ ]] && \
      process_group_alive "$old_pgid" && \
      process_group_owned "$old_pgid" "${old_run_id:-missing}"; then
    echo "Recovering ROS process group $old_pgid from an interrupted run..."
    stop_launch_group "$old_pgid"
  elif [[ "${old_pgid:-}" =~ ^[0-9]+$ ]] && process_group_alive "$old_pgid"; then
    echo "Ignoring stale state: process group $old_pgid is not owned by this monitor run." >&2
  fi
  rm -f -- "$STATE_FILE"
}

record_launch_group() {
  local pid="$1" pgid i
  for i in {1..40}; do
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ "$pgid" =~ ^[0-9]+$ ]]; then
      MONITOR_PID="$pid"
      MONITOR_PGID="$pgid"
      printf '%s %s %s %s\n' "$MONITOR_PID" "$MONITOR_PGID" "$BOOT_ID" "$RUN_ID" > "$STATE_FILE"
      return 0
    fi
    sleep 0.05
  done
  echo "Could not determine the ROS launch process group." >&2
  return 1
}

cleanup() {
  (( CLEANUP_STARTED == 0 )) || return 0
  CLEANUP_STARTED=1
  trap - INT TERM HUP QUIT EXIT
  echo "Stopping the complete host ROS process group..."
  if [[ -n "$MONITOR_PGID" ]]; then
    stop_launch_group "$MONITOR_PGID" || true
  fi
  if [[ -n "$MONITOR_PID" ]]; then
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  rm -f -- "$STATE_FILE"
  echo "Stopping Panda monitor..."
  docker compose -f "$PANDA_DIR/docker-compose.yml" down --remove-orphans || true
  echo "Stopping Lucid cameras..."
  docker compose -f "$LUCID_DIR/docker-compose.yml" down --remove-orphans || true
}

trap cleanup INT TERM HUP QUIT EXIT

echo "Starting Panda monitor and D455 camera..."
recover_interrupted_launch
remove_stale_project_containers
docker compose -f "$PANDA_DIR/docker-compose.yml" up -d --build
echo "Starting Lucid cameras..."
docker compose -f "$LUCID_DIR/docker-compose.yml" up -d --build
echo "Starting iiwa 7 FRI monitor..."
# ROS setup scripts expect to initialize some variables that may initially be
# unset, so temporarily disable nounset while sourcing them.
set +u
source /opt/ros/humble/setup.bash
source "$IIWA_WS/install/setup.bash"
source "$AR_ROOT/install/setup.bash"
set -u
echo "Starting master ROS launch..."
RUN_ID="$BOOT_ID-$$-$(date +%s%N)"
AR_MONITOR_RUN_ID="$RUN_ID" setsid ros2 launch \
  "$AR_ROOT/master_system.launch.py" "launch_rviz:=$LAUNCH_RVIZ" 9>&- &
MONITOR_PID=$!
# setsid makes the child PID the process-group ID. Record it immediately so an
# abrupt supervisor failure cannot occur between launch and crash recovery.
MONITOR_PGID=$MONITOR_PID
printf '%s %s %s %s\n' "$MONITOR_PID" "$MONITOR_PGID" "$BOOT_ID" "$RUN_ID" > "$STATE_FILE"
record_launch_group "$MONITOR_PID"
wait "$MONITOR_PID"
