#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"

readonly CLEAN_CMAKE_CACHE="${PUDU_CLEAN_CMAKE_CACHE:-false}"
cmake_cache_args=()
if [[ "${CLEAN_CMAKE_CACHE}" == "true" ]]; then
  cmake_cache_args+=(--cmake-clean-cache)
fi

require_workspace() {
  local workspace="$1"
  if [[ ! -d "${workspace}/src" ]]; then
    echo "缺少工作区：${workspace}" >&2
    exit 1
  fi
}

build_workspace() {
  local label="$1"
  local workspace="$2"
  shift 2
  echo "==> 编译 ${label}: ${workspace}"
  (
    cd "${workspace}"
    colcon build --symlink-install "${cmake_cache_args[@]}" "$@"
  )
}

for workspace in \
  "${NAV2_REFERENCE_WS}" \
  "${LINOROBOT_SIM_WS}" \
  "${EXPLORATION_REFERENCE_WS}" \
  "${SCURM_REFERENCE_WS}"; do
  require_workspace "${workspace}"
done

set +u
source /opt/ros/humble/setup.bash
local_ros_prefix="${NAV2_REFERENCE_WS}/local_deps/rootfs/opt/ros/humble"
if [[ -f "${local_ros_prefix}/local_setup.bash" ]]; then
  source "${local_ros_prefix}/local_setup.bash"
fi
set -u
export PATH="/home/robot/.local/bin:${PATH}"
if [[ -d "${local_ros_prefix}/lib/python3.10/site-packages" ]]; then
  export PYTHONPATH="${local_ros_prefix}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi

build_workspace "Nav2 参考栈" "${NAV2_REFERENCE_WS}"

set +u
source "${NAV2_REFERENCE_WS}/setup_reference.bash"
set -u
if [[ -d "${LINOROBOT_SIM_WS}/.python_deps" ]]; then
  export PYTHONPATH="${LINOROBOT_SIM_WS}/.python_deps${PYTHONPATH:+:${PYTHONPATH}}"
fi
build_workspace "Linorobot2 仿真" "${LINOROBOT_SIM_WS}"
set +u
source "${LINOROBOT_SIM_WS}/install/local_setup.bash"
set -u

build_workspace "探索功能" "${EXPLORATION_REFERENCE_WS}"
set +u
source "${EXPLORATION_REFERENCE_WS}/install/local_setup.bash"
set -u

PUDU_CLEAN_CMAKE_CACHE="${CLEAN_CMAKE_CACHE}" "${PUDU_WS}/build_scurm_reference.sh"
set +u
source "${SCURM_REFERENCE_WS}/install/local_setup.bash"
set -u

build_workspace "PUDU 主项目" "${PUDU_WS}"

echo "全部工作区编译完成。"
