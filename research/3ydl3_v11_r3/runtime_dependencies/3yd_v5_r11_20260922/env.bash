#!/usr/bin/env bash
source /home/robot/workspaces/3yd_v1_semantic_ack_20260915/env.bash
export PYTHONPATH=/home/robot/workspaces/3yd_v5_r11_20260922/src:/home/robot/workspaces/3yd_v5_r11_20260922/tools:/home/robot/workspaces/3yd_v1_semantic_ack_20260915/tools:${PYTHONPATH:-}
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=189
export THREEYD_SNAPSHOT=locked
export LD_LIBRARY_PATH=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/install/nav2_snapshot_local/lib:${LD_LIBRARY_PATH:-}
export AMENT_PREFIX_PATH=/home/robot/workspaces/3yd_v1_semantic_ack_20260915/install/nav2_snapshot_local:${AMENT_PREFIX_PATH:-}

# r9 only: private transport profile; loopback-only discovery remains explicit.
export THREEYD_R9_OPTIMIZE=${THREEYD_R9_OPTIMIZE:-1}
if [ "$THREEYD_R9_OPTIMIZE" = "1" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robot/workspaces/3yd_v5_r11_20260922/config/fastdds_64m.xml
  export ROS_LOCALHOST_ONLY=0
  export THREEYD_R9_MEMORY_PROFILE=${THREEYD_R9_MEMORY_PROFILE:-balanced}
  if [ "$THREEYD_R9_MEMORY_PROFILE" = "compact" ]; then
    export MALLOC_ARENA_MAX=2
    export MALLOC_TRIM_THRESHOLD_=131072
  elif [ "$THREEYD_R9_MEMORY_PROFILE" = "balanced" ]; then
    unset MALLOC_ARENA_MAX MALLOC_TRIM_THRESHOLD_
  else
    echo "Invalid THREEYD_R9_MEMORY_PROFILE: $THREEYD_R9_MEMORY_PROFILE" >&2
    return 2
  fi
fi

# r10 independent ablations: control, A(sync), B(preflight), C(repair), ABC.
export THREEYD_R10_MODE=${THREEYD_R10_MODE:-ABC}

export THREEYD_R11_MODE=${THREEYD_R11_MODE:-AB}

if [[ "$THREEYD_R11_MODE" == *A* ]]; then
  export LD_LIBRARY_PATH=/home/robot/workspaces/3yd_v5_r11_20260922/install/nav2_snapshot_local/lib:$LD_LIBRARY_PATH
  export AMENT_PREFIX_PATH=/home/robot/workspaces/3yd_v5_r11_20260922/install/nav2_snapshot_local:$AMENT_PREFIX_PATH
fi
