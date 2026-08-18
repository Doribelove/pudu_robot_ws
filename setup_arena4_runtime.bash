#!/usr/bin/env bash

# This file must be sourced so the Arena environment remains in the caller.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source ${BASH_SOURCE[0]} 加载 Arena4 环境。" >&2
  exit 1
fi

_arena4_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_arena4_project_root}/stack_paths.bash"

if [[ ! -f "${ARENA4_WS}/arena.bash" ]]; then
  echo "错误：找不到 Arena4 环境脚本：${ARENA4_WS}/arena.bash" >&2
  unset _arena4_project_root
  return 1
fi

# Arena's upstream source script derives ARENA_WS_DIR from the current working
# directory. Enter the canonical integrated directory just for the source step.
_arena4_previous_dir="${PWD}"
_arena4_nounset=false
[[ $- == *u* ]] && _arena4_nounset=true
set +u
cd "${ARENA4_WS}" || return 1
source ./arena.bash
_arena4_source_status=$?
cd "${_arena4_previous_dir}" || return 1
[[ "${_arena4_nounset}" == true ]] && set -u

unset _arena4_project_root _arena4_previous_dir _arena4_nounset
if (( _arena4_source_status != 0 )); then
  unset _arena4_source_status
  return 1
fi
unset _arena4_source_status
