#!/usr/bin/env bash

# Central path registry for the integrated PUDU simulation stack.
_pudu_stack_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PUDU_ROBOT_WS="${PUDU_ROBOT_WS:-${_pudu_stack_root}}"
export PUDU_EXTERNAL_WS_ROOT="${PUDU_EXTERNAL_WS_ROOT:-${PUDU_ROBOT_WS}/external}"
export NAV2_REFERENCE_WS="${NAV2_REFERENCE_WS:-${PUDU_EXTERNAL_WS_ROOT}/nav2_reference_ws}"
export LINOROBOT_SIM_WS="${LINOROBOT_SIM_WS:-${PUDU_EXTERNAL_WS_ROOT}/linorobot_sim_ws}"
export EXPLORATION_REFERENCE_WS="${EXPLORATION_REFERENCE_WS:-${PUDU_EXTERNAL_WS_ROOT}/exploration_reference_ws}"
export SCURM_REFERENCE_WS="${SCURM_REFERENCE_WS:-${PUDU_EXTERNAL_WS_ROOT}/scurm_sentry_ws}"
export ARENA4_WS="${ARENA4_WS:-${PUDU_EXTERNAL_WS_ROOT}/arena4_ws}"

unset _pudu_stack_root
