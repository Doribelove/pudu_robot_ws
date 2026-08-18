#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"

if [[ ! -d "${ARENA4_WS}/src" || ! -x "${ARENA4_WS}/colcon_build" ]]; then
  echo "错误：Arena4 工作区不完整：${ARENA4_WS}" >&2
  exit 1
fi

mapfile -t arena_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
if (( ${#arena_launches[@]} > 0 )); then
  echo "错误：Arena4 正在运行，拒绝同时重编其 install 空间。" >&2
  ps -o pid=,ppid=,sid=,stat=,args= -p "$(IFS=,; echo "${arena_launches[*]}")" >&2 || true
  echo "请先在原终端按 Ctrl+C，或对统一入口启动的实例运行 ./stop_arena4.sh。" >&2
  exit 1
fi

echo "==> 独立增量编译 Arena4: ${ARENA4_WS}"
(
  unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
  cd "${ARENA4_WS}"
  set +u
  source ./arena.bash
  set -u
  ./colcon_build "$@"
)
