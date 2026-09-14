#!/usr/bin/env bash
# Source this file; only this shell and its children receive the research overlay.
task_3yd_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -n "${PUDU_ROS_UNDERLAY:-}" ]]; then
  source "${PUDU_ROS_UNDERLAY}/setup.bash"
fi
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${task_3yd_root}/external/arena4_ws/src/arena/three_d_v1:${task_3yd_root}/external/arena4_ws/src/arena/evaluation/arena_evaluation:${task_3yd_root}/external/arena4_ws/src/arena/three_d_v1_nav2:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${task_3yd_root}/config/fastdds_large_map.xml"
unset task_3yd_root
