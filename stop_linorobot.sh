#!/usr/bin/env bash

set -euo pipefail

readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}"
readonly GAZEBO_PID_FILE="${RUNTIME_DIR}/simulation.pid"
readonly NAV_PID_FILE="${RUNTIME_DIR}/navigation.pid"
readonly EXPLORE_PID_FILE="${RUNTIME_DIR}/exploration.pid"

session_ids=()
pid_files=()
invalid_session_found=false

session_pids() {
  local session_id="$1"
  ps -s "${session_id}" -o pid= 2>/dev/null | tr -d ' '
}

session_is_alive() {
  [[ -n "$(session_pids "$1")" ]]
}

session_commands() {
  local session_id="$1"
  ps -s "${session_id}" -o args= 2>/dev/null || true
}

signal_session_members() {
  local signal_name="$1"
  local session_id="$2"
  local member_pid

  while read -r member_pid; do
    [[ -n "${member_pid}" ]] && kill "-${signal_name}" "${member_pid}" 2>/dev/null || true
  done < <(session_pids "${session_id}")
}

register_session() {
  local pid_file="$1"
  local expected_pattern="$2"
  local process_name="$3"
  local session_id
  local commands

  [[ -f "${pid_file}" ]] || return 0

  read -r session_id < "${pid_file}" || true
  if [[ ! "${session_id:-}" =~ ^[0-9]+$ ]]; then
    rm -f "${pid_file}"
    echo "${process_name} 的 PID 文件无效，已清理。" >&2
    return 0
  fi

  if ! session_is_alive "${session_id}"; then
    rm -f "${pid_file}"
    return 0
  fi

  # 即使顶层 ros2 launch 已退出，也可通过会话中的 gzserver 等成员识别残留进程。
  commands="$(session_commands "${session_id}")"
  if ! grep -Eq "${expected_pattern}" <<< "${commands}"; then
    echo "拒绝终止：会话 ${session_id} 已不属于 ${process_name}。" >&2
    invalid_session_found=true
    return 0
  fi

  session_ids+=("${session_id}")
  pid_files+=("${pid_file}")
}

register_session "${GAZEBO_PID_FILE}" \
  "linorobot2_gazebo|gazebo|gzserver|gzclient" "Gazebo"
register_session "${NAV_PID_FILE}" \
  "linorobot2_navigation|nav2|slam_toolbox|rviz2" "Nav2/SLAM"
register_session "${EXPLORE_PID_FILE}" \
  "pudu_coverage_planner|explore_lite|autonomous_mapping|/explore" "自动探索"

if (( ${#session_ids[@]} == 0 )); then
  if [[ "${invalid_session_found}" == true ]]; then
    exit 1
  fi
  echo "Linorobot2 仿真未运行。"
  exit 0
fi

echo "正在终止 Linorobot2 Gazebo、Nav2/SLAM、自动探索和 RViz……"

# 先只通知 ros2 launch 会话首进程，让 launch 正常关闭其节点。
for session_id in "${session_ids[@]}"; do
  if kill -0 "${session_id}" 2>/dev/null; then
    kill -INT "${session_id}" 2>/dev/null || true
  fi
done

for _ in {1..50}; do
  any_alive=false
  for session_id in "${session_ids[@]}"; do
    if session_is_alive "${session_id}"; then
      any_alive=true
    fi
  done
  [[ "${any_alive}" == false ]] && break
  sleep 0.2
done

# Gazebo 的包装进程可能让 gzserver/gzclient 创建新的进程组，但仍属于同一会话。
for session_id in "${session_ids[@]}"; do
  if session_is_alive "${session_id}"; then
    echo "正常关闭超时，正在结束会话 ${session_id} 的剩余进程……"
    signal_session_members TERM "${session_id}"
  fi
done

for _ in {1..25}; do
  any_alive=false
  for session_id in "${session_ids[@]}"; do
    if session_is_alive "${session_id}"; then
      any_alive=true
    fi
  done
  [[ "${any_alive}" == false ]] && break
  sleep 0.2
done

for session_id in "${session_ids[@]}"; do
  if session_is_alive "${session_id}"; then
    signal_session_members KILL "${session_id}"
  fi
done

rm -f "${pid_files[@]}"
echo "Linorobot2 仿真已终止。"
