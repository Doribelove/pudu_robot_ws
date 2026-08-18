#!/usr/bin/env bash

set -euo pipefail

readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}"
readonly PID_FILE="${RUNTIME_DIR}/arena.pid"

session_pids() {
  local session_id="$1"
  ps -s "${session_id}" -o pid= 2>/dev/null | tr -d ' '
}

session_is_alive() {
  [[ -n "$(session_pids "$1")" ]]
}

signal_session_members() {
  local signal_name="$1"
  local session_id="$2"
  local member_pid

  while read -r member_pid; do
    [[ -n "${member_pid}" ]] && kill "-${signal_name}" "${member_pid}" 2>/dev/null || true
  done < <(session_pids "${session_id}")
}

if [[ ! -f "${PID_FILE}" ]]; then
  mapfile -t unmanaged_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
  if (( ${#unmanaged_launches[@]} > 0 )); then
    echo "检测到 Arena4 正在其他终端运行，但它不是由 PUDU 统一入口启动："
    ps -o pid=,ppid=,sid=,stat=,etime=,args= -p "$(IFS=,; echo "${unmanaged_launches[*]}")"
    echo "为避免误杀，请在原终端按 Ctrl+C 停止。"
  else
    echo "Arena4 未运行。"
  fi
  exit 0
fi

read -r session_id < "${PID_FILE}" || true
if [[ ! "${session_id:-}" =~ ^[0-9]+$ ]]; then
  rm -f "${PID_FILE}"
  echo "Arena4 PID 文件无效，已清理。" >&2
  exit 1
fi

if ! session_is_alive "${session_id}"; then
  rm -f "${PID_FILE}"
  echo "Arena4 已停止，残留 PID 文件已清理。"
  exit 0
fi

commands="$(ps -s "${session_id}" -o args= 2>/dev/null || true)"
if ! grep -Eq 'arena_bringup|arena.launch.py|task_generator|gz sim' <<< "${commands}"; then
  echo "拒绝终止：会话 ${session_id} 已不属于 Arena4。" >&2
  exit 1
fi

echo "正在终止 Arena4（会话：${session_id}）……"
if kill -0 "${session_id}" 2>/dev/null; then
  kill -INT "${session_id}" 2>/dev/null || true
fi

for _ in {1..60}; do
  session_is_alive "${session_id}" || break
  sleep 0.2
done

if session_is_alive "${session_id}"; then
  echo "正常关闭超时，正在结束会话剩余进程……"
  signal_session_members TERM "${session_id}"
fi

for _ in {1..25}; do
  session_is_alive "${session_id}" || break
  sleep 0.2
done

if session_is_alive "${session_id}"; then
  signal_session_members KILL "${session_id}"
fi

rm -f "${PID_FILE}"
echo "Arena4 已终止。"
