#!/usr/bin/env bash
set -uo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

PASS=0
WARN=0
FAIL=0

ok()   { printf '[ OK ] %s\n' "$1"; PASS=$((PASS + 1)); }
warn() { printf '[WARN] %s\n' "$1"; WARN=$((WARN + 1)); }
fail() { printf '[FAIL] %s\n' "$1"; FAIL=$((FAIL + 1)); }

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

check_topic() {
  local topic="$1" label="$2"
  if ! topic_exists "$topic"; then
    fail "$label: topic missing ($topic)"
    return
  fi
  local info pubs type
  info=$(ros2 topic info "$topic" 2>/dev/null || true)
  pubs=$(awk '/Publisher count:/ {print $3; exit}' <<< "$info")
  type=$(awk '/Type:/ {print $2; exit}' <<< "$info")
  if [[ "${pubs:-0}" =~ ^[1-9][0-9]*$ ]]; then
    ok "$label: $topic [$type], publishers=$pubs"
  else
    fail "$label: topic exists but has no publisher ($topic)"
  fi
}

check_rate() {
  local topic="$1" label="$2"
  local output
  output=$(timeout 4 ros2 topic hz --window 3 --spin-time 0.5 "$topic" 2>&1 || true)
  if grep -q 'average rate:' <<< "$output"; then
    local rate
    rate=$(awk '/average rate:/ {print $3; exit}' <<< "$output")
    ok "$label: ${rate} Hz"
  else
    fail "$label: no messages received ($topic)"
  fi
}

echo '=== ROS 2 topic health check ==='
echo "Host: $(hostname)    ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo

echo '--- Required publishers ---'
check_topic /tf 'TF'
check_topic /tf_static 'Static TF'
check_topic /iiwa7/joint_states 'KUKA joint states'
check_topic /panda/joint_states 'Panda joint states'
check_topic /lucid/triton/image_color 'Lucid Triton color image'
check_topic /lucid/triton/camera_info 'Lucid Triton camera info'
check_topic /lucid/helios/image_raw 'Lucid Helios image'
check_topic /iiwa7/d455/color/image_raw 'KUKA D455 color image'
check_topic /iiwa7/d455/color/camera_info 'KUKA D455 camera info'

echo
echo '--- All discovered camera/image, joint, and gripper topics ---'
mapfile -t discovered < <(ros2 topic list 2>/dev/null | grep -Ei 'image|camera_info|joint_states|gripper|wsg' | sort -u)
if [[ "${#discovered[@]}" -eq 0 ]]; then
  fail 'No camera, joint-state, or gripper topics discovered'
else
  for topic in "${discovered[@]}"; do
    info=$(ros2 topic info "$topic" 2>/dev/null || true)
    pubs=$(awk '/Publisher count:/ {print $3; exit}' <<< "$info")
    type=$(awk '/Type:/ {print $2; exit}' <<< "$info")
    if [[ "${pubs:-0}" =~ ^[1-9][0-9]*$ ]]; then
      ok "$topic [$type], publishers=$pubs"
    else
      warn "$topic [$type] has no publisher"
    fi
  done
fi

echo
echo '--- Message rates (5-second samples) ---'
rate_topics=(
  '/tf|TF' \
  '/iiwa7/joint_states|KUKA joint states' \
  '/lucid/triton/image_color|Lucid Triton image' \
  '/lucid/triton/camera_info|Lucid Triton camera info' \
  '/iiwa7/d455/color/image_raw|KUKA D455 color image' \
  '/iiwa7/d455/depth/image_rect_raw|KUKA D455 depth image' \
  '/iiwa7/d455/color/camera_info|KUKA D455 camera info' \
  '/panda/d455/color/image_raw|Panda D455 color image' \
  '/panda/d455/depth/image_rect_raw|Panda D455 depth image' \
  '/panda/d455/color/camera_info|Panda D455 camera info'
)
rate_dir="${TMPDIR:-/tmp}/verify_ros_topics.$$.d"
mkdir -p "$rate_dir"
trap 'rm -rf "$rate_dir"' EXIT
rate_index=0
for item in "${rate_topics[@]}"; do
  topic="${item%%|*}"; label="${item#*|}"
  if topic_exists "$topic"; then
    check_rate "$topic" "$label" >"$rate_dir/$rate_index" 2>&1 &
    rate_index=$((rate_index + 1))
  fi
done
wait
for result in "$rate_dir"/*; do
  [[ -f "$result" ]] && cat "$result"
done

echo
echo '--- Critical TF transforms ---'
check_tf() {
  local parent="$1" child="$2"
  local output
  output=$(timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)
  if grep -q 'At time\|Translation:' <<< "$output"; then
    ok "TF $parent -> $child is available"
  else
    fail "TF $parent -> $child is unavailable"
  fi
}

check_tf iiwa7_link_0 iiwa7_link_ee
check_tf panda_link0 panda_hand_tcp

echo
echo "Summary: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "$FAIL" -eq 0 ]]
