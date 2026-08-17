#!/usr/bin/env bash

_pudu_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /home/robot/nav2_reference_ws/setup_reference.bash

if [[ -f /home/robot/exploration_reference_ws/install/local_setup.bash ]]; then
  source /home/robot/exploration_reference_ws/install/local_setup.bash
fi

if [[ -f "${_pudu_workspace_root}/install/local_setup.bash" ]]; then
  source "${_pudu_workspace_root}/install/local_setup.bash"
fi

unset _pudu_workspace_root
