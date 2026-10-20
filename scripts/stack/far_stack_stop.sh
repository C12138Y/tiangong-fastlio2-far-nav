#!/usr/bin/env bash
set -Eeo pipefail

RUN_DIR="/tmp/far_stack_${USER}"
STATE_FILE="$RUN_DIR/state"
LOCK_FILE="$RUN_DIR/lock"
mkdir -p "$RUN_DIR"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "警告：锁可能被旧版后台子进程继承；将只按状态文件停止受管进程。" >&2
fi

if [[ ! -s "$STATE_FILE" ]]; then
  echo "没有找到受管 A–F 进程记录；不会停止其他机器人进程。"
  exit 0
fi

declare -a NAMES=()
declare -a PIDS=()
SESSION_DIR=""
while IFS='=' read -r name value; do
  if [[ "$name" == "SESSION_DIR" ]]; then
    SESSION_DIR="$value"
  elif [[ "$value" =~ ^[0-9]+$ ]]; then
    NAMES+=("$name")
    PIDS+=("$value")
  fi
done <"$STATE_FILE"

for ((index=${#PIDS[@]}-1; index>=0; index--)); do
  pid="${PIDS[$index]}"
  name="${NAMES[$index]}"
  if kill -0 "$pid" 2>/dev/null; then
    echo "停止 $name (进程组 $pid) ..."
    kill -INT -- "-$pid" 2>/dev/null || true
  else
    echo "$name 已退出。"
  fi
done

deadline=$((SECONDS + 12))
while (( SECONDS < deadline )); do
  alive=0
  for pid in "${PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && alive=1
  done
  (( alive == 0 )) && break
  sleep 1
done

still_alive=0
for index in "${!PIDS[@]}"; do
  if kill -0 "${PIDS[$index]}" 2>/dev/null; then
    echo "${NAMES[$index]} 仍在运行；未强制终止，请检查：${PIDS[$index]}" >&2
    still_alive=1
  fi
done

if (( still_alive == 0 )); then
  : >"$STATE_FILE"
  echo "A–F 已安全停止。日志保留在：${SESSION_DIR:-未知}"
else
  echo "存在未退出进程，状态文件已保留。" >&2
  exit 1
fi

