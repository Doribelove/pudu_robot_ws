#!/usr/bin/env bash
# Public packaging adapter (2026-10-19): replace the host Arena overlay with
# the system Humble underlay. Algorithm sources retain their accepted bytes.
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/three_d_v1:/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/evaluation/arena_evaluation:/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/three_d_v1_nav2:/venv/lib/python3.10/site-packages:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/config/fastdds_large_map.xml

# Keep the accepted Nav2 1.1.19 server/plugin ABI together.
export LD_LIBRARY_PATH=/sdk/lib:${LD_LIBRARY_PATH}
export AMENT_PREFIX_PATH=/sdk:${AMENT_PREFIX_PATH}
