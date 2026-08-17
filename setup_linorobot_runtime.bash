#!/usr/bin/env bash

_linorobot_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID="${LINOROBOT_ROS_DOMAIN_ID:-42}"

source /home/robot/linorobot_sim_ws/setup_sim.bash

if [[ -f /home/robot/exploration_reference_ws/install/local_setup.bash ]]; then
  source /home/robot/exploration_reference_ws/install/local_setup.bash
fi

if [[ -f "${_linorobot_project_root}/install/local_setup.bash" ]]; then
  source "${_linorobot_project_root}/install/local_setup.bash"
fi

unset _linorobot_project_root
