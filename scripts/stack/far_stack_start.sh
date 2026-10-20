#!/usr/bin/env bash
set -Eeo pipefail

RUN_DIR="/tmp/far_stack_${USER}"
STATE_FILE="$RUN_DIR/state"
LOCK_FILE="$RUN_DIR/lock"
LOG_ROOT="/home/nvidia/logs/far_stack"
STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="$LOG_ROOT/$STAMP"

mkdir -p "$RUN_DIR" "$SESSION_DIR"
exec 9>"$LOCK_FILE"
flock -n 9 || { echo "另一项 FAR 栈启动/停止操作正在进行。" >&2; exit 1; }

# shellcheck disable=SC1091
source /home/nvidia/hric_tools/hric_env.sh
unset ROS_DOMAIN_ID

declare -a STARTED_NAMES=()
declare -a STARTED_PIDS=()

publisher_count() {
  local topic="$1"
  local output
  output="$(timeout 3 ros2 topic info "$topic" 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$output"
}

wait_for_publisher() {
  local topic="$1"
  local seconds="$2"
  local deadline=$((SECONDS + seconds))
  while (( SECONDS < deadline )); do
    if (( $(publisher_count "$topic") > 0 )); then
      echo "  OK: $topic"
      return 0
    fi
    sleep 1
  done
  echo "等待 $topic 发布者超时。" >&2
  return 1
}

start_component() {
  local name="$1"
  local command="$2"
  local log_file="$SESSION_DIR/${name}.log"
  echo "启动 $name ..."
  # Do not let long-running children inherit the orchestration lock (fd 9).
  setsid bash -lc "$command" 9>&- >"$log_file" 2>&1 < /dev/null &
  local pid=$!
  STARTED_NAMES+=("$name")
  STARTED_PIDS+=("$pid")
  printf '%s=%s\n' "$name" "$pid" >>"$STATE_FILE"
  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "$name 启动后立即退出，日志：$log_file" >&2
    tail -n 30 "$log_file" >&2 || true
    return 1
  fi
}

stop_started_on_error() {
  local status=$?
  trap - ERR
  if (( status == 0 )); then
    return
  fi
  echo "启动失败，只停止本脚本本次创建的进程。" >&2
  local index
  for ((index=${#STARTED_PIDS[@]}-1; index>=0; index--)); do
    kill -INT -- "-${STARTED_PIDS[$index]}" 2>/dev/null || true
  done
  echo "日志保留在：$SESSION_DIR" >&2
  exit "$status"
}
trap stop_started_on_error ERR

if [[ -s "$STATE_FILE" ]]; then
  while IFS='=' read -r old_name old_pid; do
    [[ "$old_pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$old_pid" 2>/dev/null; then
      echo "已有受管进程 $old_name (PID $old_pid)；请先运行 far_stack_status.sh 或 far_stack_stop.sh。" >&2
      exit 1
    fi
  done <"$STATE_FILE"
fi
printf 'SESSION_DIR=%s\n' "$SESSION_DIR" >"$STATE_FILE"

echo "检查机器人已有 Livox 驱动..."
wait_for_publisher /livox/lidar 8
wait_for_publisher /livox/imu 8

start_component A_pc2_to_custom \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/fastlio2_ws/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 run fastlio2 pc2_to_custom'
wait_for_publisher /fastlio2/livox_custom 12

start_component B_fastlio2 \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/fastlio2_ws/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 launch fastlio2 lio_launch.py'
wait_for_publisher /fastlio2/world_cloud 30
wait_for_publisher /fastlio2/lio_odom 15

# lio_launch.py also starts RViz. Stop only rviz2 processes in B's process group;
# FAST-LIO2 and the launch parent remain running. RViz is started manually later.
B_PID="${STARTED_PIDS[1]}"
for _ in {1..10}; do
  mapfile -t rviz_pids < <(
    ps -eo pid=,pgid=,comm= | awk -v pgid="$B_PID" '$2 == pgid && $3 == "rviz2" {print $1}'
  )
  if (( ${#rviz_pids[@]} > 0 )); then
    echo "关闭 FAST-LIO2 launch 自动打开的 RViz：${rviz_pids[*]}"
    for rviz_pid in "${rviz_pids[@]}"; do
      kill -INT "$rviz_pid" 2>/dev/null || true
    done
    break
  fi
  sleep 1
done

start_component C_nav_adapter \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/fastlio2_ws/install/setup.bash; source /home/nvidia/tiangong_nav_ws/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 launch fastlio_nav_adapter adapter.launch.py'
wait_for_publisher /my_nav/odom 20

start_component D_terrain_analysis \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 run terrain_analysis terrainAnalysis --ros-args -r /state_estimation:=/my_nav/odom -r /registered_scan:=/fastlio2/world_cloud -r /terrain_map:=/far_eval/terrain_map'
wait_for_publisher /far_eval/terrain_map 25

start_component E_terrain_analysis_ext \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 run terrain_analysis_ext terrainAnalysisExt --ros-args -r /state_estimation:=/my_nav/odom -r /registered_scan:=/fastlio2/world_cloud -r /terrain_map:=/far_eval/terrain_map -r /terrain_map_ext:=/far_eval/terrain_map_ext'
wait_for_publisher /far_eval/terrain_map_ext 25

start_component F_far_planner \
  'source /home/nvidia/hric_tools/hric_env.sh; source /home/nvidia/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash; source /home/nvidia/far_planner_eval_ws/far_planner/install/setup.bash; unset ROS_DOMAIN_ID; exec ros2 run far_planner far_planner --ros-args -r __node:=far_planner --params-file "$(ros2 pkg prefix --share far_planner)/config/default.yaml" -p world_frame:=fastlio_odom -p is_opencv_visual:=false -p is_pub_boundary:=false -r /odom_world:=/my_nav/odom -r /terrain_cloud:=/far_eval/terrain_map_ext -r /scan_cloud:=/far_eval/terrain_map -r /terrain_local_cloud:=/fastlio2/world_cloud -r /goal_point:=/far_eval/goal_point -r /way_point:=/far_eval/way_point'
wait_for_publisher /far_eval/way_point 25

trap - ERR
echo
echo "A–F 已启动，ROS_DOMAIN_ID 未设置。"
echo "日志目录：$SESSION_DIR"
echo "状态检查：/home/nvidia/hric_tools/far_stack_status.sh"
echo "手动 RViz：/home/nvidia/hric_tools/far_stack_rviz.sh"
echo "安全停止：/home/nvidia/hric_tools/far_stack_stop.sh"

