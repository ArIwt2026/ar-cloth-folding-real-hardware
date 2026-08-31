#!/usr/bin/env bash
set -Eeuo pipefail

AR_ROOT="/home/iwtros/Documents/ar"
PANDA_DIR="$AR_ROOT/teleop_panda"
LUCID_DIR="$AR_ROOT/lucid_cameras"
STATE_FILE="/tmp/ar-run-all-monitor-${UID}.state"
BOOT_ID=$(< /proc/sys/kernel/random/boot_id)

group_alive() {
  ps -eo pgid=,stat= | awk -v wanted="$1" \
    '$1 == wanted && $2 !~ /^Z/ { found=1 } END { exit !found }'
}

group_owned() {
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

stop_group() {
  local pgid="$1" signal i attempts own_pgid
  [[ "$pgid" =~ ^[0-9]+$ ]] && (( pgid > 1 )) || return 0
  own_pgid=$(ps -o pgid= -p $$ | tr -d ' ')
  [[ "$pgid" != "$own_pgid" ]] || return 1
  for signal in INT TERM KILL; do
    group_alive "$pgid" || return 0
    kill -"$signal" -- "-$pgid" 2>/dev/null || true
    attempts=20
    [[ "$signal" == INT ]] && attempts=40
    [[ "$signal" == KILL ]] && attempts=8
    for ((i=0; i<attempts; i++)); do
      group_alive "$pgid" || return 0
      sleep 0.25
    done
  done
  return 1
}

if [[ -r "$STATE_FILE" ]]; then
  read -r supervisor_pid monitor_pgid state_boot_id run_id < "$STATE_FILE" || true
  if [[ "${state_boot_id:-}" == "$BOOT_ID" ]] && \
      [[ "${monitor_pgid:-}" =~ ^[0-9]+$ ]] && \
      group_owned "$monitor_pgid" "${run_id:-missing}" && \
      [[ "${supervisor_pid:-}" =~ ^[0-9]+$ ]] && \
      tr '\0' ' ' < "/proc/$supervisor_pid/cmdline" 2>/dev/null | \
      rg -q 'run_all_monitor\.sh'; then
    echo "Requesting graceful shutdown from supervisor PID $supervisor_pid..."
    kill -INT "$supervisor_pid" 2>/dev/null || true
    for i in {1..160}; do
      kill -0 "$supervisor_pid" 2>/dev/null || break
      sleep 0.25
    done
  fi
  if [[ "${state_boot_id:-}" == "$BOOT_ID" ]] && \
      [[ "${monitor_pgid:-}" =~ ^[0-9]+$ ]] && \
      group_alive "$monitor_pgid" && group_owned "$monitor_pgid" "${run_id:-missing}"; then
    echo "Stopping remaining ROS process group $monitor_pgid..."
    stop_group "$monitor_pgid"
  elif [[ "${monitor_pgid:-}" =~ ^[0-9]+$ ]] && group_alive "$monitor_pgid"; then
    echo "Refusing to stop unowned process group $monitor_pgid." >&2
  fi
  rm -f -- "$STATE_FILE"
else
  echo "No monitor state file found; no owned host ROS group to stop."
fi

echo "Stopping Panda monitor containers..."
docker compose -f "$PANDA_DIR/docker-compose.yml" down --remove-orphans 2>/dev/null || true

echo "Stopping Lucid camera containers..."
docker compose -f "$LUCID_DIR/docker-compose.yml" down --remove-orphans 2>/dev/null || true

echo "Removing remaining project containers..."
for pattern in '^/panda_monitor$' 'teleop_panda-teleop_panda-run-' \
    'lucid_cameras-lucid_ros2-' 'lucid_ros2-'; do
  containers=$(docker ps -aq --filter "name=$pattern" 2>/dev/null || true)
  if [[ -n "$containers" ]]; then
    docker rm -f $containers >/dev/null 2>&1 || true
  fi
done

echo "All owned monitor processes and containers stopped."
