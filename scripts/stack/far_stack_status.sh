#!/usr/bin/env bash
set -Eeo pipefail

RUN_DIR="/tmp/far_stack_${USER}"
STATE_FILE="$RUN_DIR/state"

# shellcheck disable=SC1091
source /home/nvidia/hric_tools/hric_env.sh
unset ROS_DOMAIN_ID

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置}"
if [[ -s "$STATE_FILE" ]]; then
  echo "受管进程："
  while IFS='=' read -r name value; do
    if [[ "$name" == "SESSION_DIR" ]]; then
      echo "  日志：$value"
    elif [[ "$value" =~ ^[0-9]+$ ]]; then
      if kill -0 "$value" 2>/dev/null; then
        echo "  OK    $name PID=$value"
      else
        echo "  EXIT  $name PID=$value"
      fi
    fi
  done <"$STATE_FILE"
else
  echo "没有活动的受管 A–F 会话。"
fi

echo
for topic in \
  /livox/lidar \
  /fastlio2/livox_custom \
  /fastlio2/world_cloud \
  /my_nav/odom \
  /far_eval/terrain_map \
  /far_eval/terrain_map_ext \
  /far_eval/way_point; do
  output="$(timeout 3 ros2 topic info "$topic" 2>/dev/null || true)"
  count="$(awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$output")"
  printf '  publishers=%s  %s\n' "$count" "$topic"
done

echo
ros2 param get /far_planner world_frame 2>/dev/null || true
ros2 param get /far_planner local_planner_range 2>/dev/null || true

