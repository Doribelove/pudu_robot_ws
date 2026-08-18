#!/usr/bin/env bash

_pudu_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_pudu_workspace_root}/stack_paths.bash"

source "${NAV2_REFERENCE_WS}/setup_reference.bash"

if [[ -f "${EXPLORATION_REFERENCE_WS}/install/local_setup.bash" ]]; then
  source "${EXPLORATION_REFERENCE_WS}/install/local_setup.bash"
fi

if [[ -f "${_pudu_workspace_root}/install/local_setup.bash" ]]; then
  source "${_pudu_workspace_root}/install/local_setup.bash"
fi

unset _pudu_workspace_root
