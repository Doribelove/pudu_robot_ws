#!/usr/bin/env bash
source /home/robot/workspaces/3yd_v5_r11_20260922/env.bash
export PYTHONPATH=/home/robot/workspaces/3ydl3_v11_r3_requestcore_k_20261016/src:/home/robot/workspaces/3ydl3_v11_r3_requestcore_k_20261016/tools:${PYTHONPATH:-}

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

# V8-r3 validated static candidate. Direct module imports remain conservative
# (A); the registered run.sh entry selects the tested A+B configuration.
export THREEYD_V8_R3_MODE=${THREEYD_V8_R3_MODE:-AB}

# V8-r4 independent storage experiments.
export THREEYD_V8_R4_MODE=${THREEYD_V8_R4_MODE:-ABC}

export THREEYD_V8_R5_REPAIR=${THREEYD_V8_R5_REPAIR:-1}

export THREEYD_V8_R6_MODE=${THREEYD_V8_R6_MODE:-ABC}
if [[ "$THREEYD_V8_R6_MODE" == *B* ]]; then
  export LD_LIBRARY_PATH=/home/robot/workspaces/3ydl3_v11_r3_requestcore_k_20261016/install/nav2_snapshot_local/lib:$LD_LIBRARY_PATH
fi

export THREEYD_V8_FULLROUTE_REUSE=${THREEYD_V8_FULLROUTE_REUSE:-1}

export THREEYD_V8_FIRST_MODE=${THREEYD_V8_FIRST_MODE:-AB}

export THREEYD_V8_CONTINUATION=${THREEYD_V8_CONTINUATION:-1}

export THREEYD_V8_R11_MODE=${THREEYD_V8_R11_MODE:-AB}

export THREEYD_V9_MODE=${THREEYD_V9_MODE:-control}

export PYTHONPATH="/home/robot/workspaces/3ydl3_v11_r3_requestcore_k_20261016/deps:${PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
