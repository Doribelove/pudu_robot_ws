#!/usr/bin/env bash

_linorobot_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_linorobot_project_root}/stack_paths.bash"

export ROS_DOMAIN_ID="${LINOROBOT_ROS_DOMAIN_ID:-42}"

source "${LINOROBOT_SIM_WS}/setup_sim.bash"

if [[ -d "${NAV2_REFERENCE_WS}/local_deps/rootfs/usr/bin" ]]; then
  export PATH="${NAV2_REFERENCE_WS}/local_deps/rootfs/usr/bin:${PATH}"
fi

if [[ -f "${EXPLORATION_REFERENCE_WS}/install/local_setup.bash" ]]; then
  source "${EXPLORATION_REFERENCE_WS}/install/local_setup.bash"
fi

if [[ -f "${_linorobot_project_root}/install/local_setup.bash" ]]; then
  source "${_linorobot_project_root}/install/local_setup.bash"
fi

unset _linorobot_project_root
