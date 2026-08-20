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

# Follow the isolated ROS domain selected by start_arena4.sh. Without a
# managed instance, arena.bash's upstream Domain 1 remains in effect.
_arena4_domain_file="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}/ros_domain_id"
if [[ -r "${_arena4_domain_file}" ]]; then
  read -r _arena4_domain_id < "${_arena4_domain_file}" || true
  if [[ "${_arena4_domain_id:-}" =~ ^[0-9]+$ ]] && (( _arena4_domain_id <= 232 )); then
    export ROS_DOMAIN_ID="${_arena4_domain_id}"
  fi
fi

# Join the running Gazebo transport partition too, so commands such as
# `gz model --list` address the managed Arena instance instead of timing out.
_arena4_partition_file="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}/gazebo_partition"
if [[ -r "${_arena4_partition_file}" ]]; then
  read -r _arena4_partition < "${_arena4_partition_file}" || true
  if [[ "${_arena4_partition:-}" =~ ^arena4_[0-9]+_[0-9]+$ ]]; then
    export GZ_PARTITION="${_arena4_partition}"
  fi
fi

unset _arena4_project_root _arena4_previous_dir _arena4_nounset
unset _arena4_domain_file _arena4_domain_id
unset _arena4_partition_file _arena4_partition
if (( _arena4_source_status != 0 )); then
  unset _arena4_source_status
  return 1
fi
if [[ -d "${ARENA4_G2O_PREFIX:-}" ]]; then
  export CMAKE_PREFIX_PATH="${ARENA4_G2O_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${ARENA4_G2O_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
unset _arena4_source_status
