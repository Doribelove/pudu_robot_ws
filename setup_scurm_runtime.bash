#!/usr/bin/env bash

_pudu_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_pudu_workspace_root}/stack_paths.bash"
_scurm_workspace_root="${SCURM_REFERENCE_WS}"

source "${NAV2_REFERENCE_WS}/install/setup.bash"

if [[ ! -f "${_scurm_workspace_root}/install/local_setup.bash" ]]; then
  echo "SCURM underlay is not built: ${_scurm_workspace_root}" >&2
  return 1 2>/dev/null || exit 1
fi
source "${_scurm_workspace_root}/install/local_setup.bash"

if [[ -f "${_pudu_workspace_root}/install/local_setup.bash" ]]; then
  source "${_pudu_workspace_root}/install/local_setup.bash"
fi

unset _pudu_workspace_root _scurm_workspace_root
