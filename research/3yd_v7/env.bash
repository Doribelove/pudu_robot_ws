#!/usr/bin/env bash
# Relocatable source delivery overlay; source in a separate shell for checks.
task_3yd_v7_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -n "${PUDU_ROS_UNDERLAY:-}" ]]; then
  source "${PUDU_ROS_UNDERLAY}/setup.bash"
fi
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${task_3yd_v7_root}/src:${task_3yd_v7_root}/tools:${task_3yd_v7_root}/external/arena4_ws/src/arena/three_d_v1:${task_3yd_v7_root}/external/arena4_ws/src/arena/evaluation/arena_evaluation:${task_3yd_v7_root}/external/arena4_ws/src/arena/three_d_v1_nav2:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-189}
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${task_3yd_v7_root}/config/fastdds_64m.xml"
export THREEYD_SNAPSHOT=locked
export THREEYD_R9_OPTIMIZE=${THREEYD_R9_OPTIMIZE:-1}
export THREEYD_R9_MEMORY_PROFILE=${THREEYD_R9_MEMORY_PROFILE:-balanced}
export THREEYD_R10_MODE=${THREEYD_R10_MODE:-ABC}
export THREEYD_R11_MODE=${THREEYD_R11_MODE:-AB}
unset task_3yd_v7_root


# Default L1 search shared with r14; legacy can be selected explicitly.
export THREEYD_L1_SEARCH=${THREEYD_L1_SEARCH:-astar}
export THREEYD_L1_FIRST_BUDGET=${THREEYD_L1_FIRST_BUDGET:-50000}
export THREEYD_L1_ALTERNATE_BUDGET=${THREEYD_L1_ALTERNATE_BUDGET:-10000}
export THREEYD_L1_LEGACY_COMPAT=${THREEYD_L1_LEGACY_COMPAT:-1}

export THREEYD_R13_START_ACCESS_TUBE=${THREEYD_R13_START_ACCESS_TUBE:-1}
export THREEYD_R13_NORTH_EXIT_GUIDE=${THREEYD_R13_NORTH_EXIT_GUIDE:-1}
export THREEYD_L1_GOAL_POTENTIAL=${THREEYD_L1_GOAL_POTENTIAL:-1}

# Query-time L1 comparison. No precompiled raster index is used in V6-r2.
export THREEYD_V6_LAZY_RESUME=${THREEYD_V6_LAZY_RESUME:-1}
export THREEYD_V6_DOMINANCE_BUCKETS=${THREEYD_V6_DOMINANCE_BUCKETS:-1}
export THREEYD_V6_SINGLE_LANE_CERTIFICATE=${THREEYD_V6_SINGLE_LANE_CERTIFICATE:-1}
export THREEYD_V6_COPY_ON_DELETE=${THREEYD_V6_COPY_ON_DELETE:-1}
export THREEYD_V6_TRANSITION_TABLE=${THREEYD_V6_TRANSITION_TABLE:-1}
export THREEYD_V6_DIRECTED_POTENTIAL=${THREEYD_V6_DIRECTED_POTENTIAL:-1}
export THREEYD_V6_PREFIX_SUFFICIENCY=${THREEYD_V6_PREFIX_SUFFICIENCY:-1}
export THREEYD_V6_PRECOMPUTED_POTENTIAL=${THREEYD_V6_PRECOMPUTED_POTENTIAL:-1}
export THREEYD_V6_SKIP_RELAXED_POTENTIAL=${THREEYD_V6_SKIP_RELAXED_POTENTIAL:-1}

# V7-r1 independent controls: A=goal-stop map-only CSR search;
# B=shared immutable prefixes/coordinates in semantic legacy alternatives.
export THREEYD_V7_GOAL_STOP=${THREEYD_V7_GOAL_STOP:-1}
export THREEYD_V7_COMPACT_LEGACY=${THREEYD_V7_COMPACT_LEGACY:-1}
