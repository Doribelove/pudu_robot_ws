#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/three_d_v1:/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/evaluation/arena_evaluation:/home/robot/workspaces/3yd_v1_semantic_ack_20260915/external/arena4_ws/src/arena/three_d_v1_nav2:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/config/fastdds_large_map.xml
