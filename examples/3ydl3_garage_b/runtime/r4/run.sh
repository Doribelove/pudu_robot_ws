#!/usr/bin/env bash
set -eo pipefail
source /home/robot/workspaces/3ydl3_v11_r4_tailflow_i_20261016/env.bash
export THREEYD_BENCH_SOURCE=/home/robot/workspaces/3ydl3_v11_r4_tailflow_i_20261016
export THREEYD_BENCH_CPU=14
export THREEYD_V10_R3_MODE=${THREEYD_V10_R3_MODE:-ABRTHI}
export THREEYD_V10_R4_MODE=${THREEYD_V10_R4_MODE:-PHFBDJNTV}
export THREEYD_V10_R5_MODE=${THREEYD_V10_R5_MODE:-MLSEKU}
export THREEYD_V10_R6_MODE="${THREEYD_V10_R6_MODE:-DCGXY}"
export THREEYD_V10_R7_MODE="${THREEYD_V10_R7_MODE:-ABCD}"
export THREEYD_V10_R8_MODE="${THREEYD_V10_R8_MODE:-STPBE}"
exec python3 /home/robot/workspaces/3ydl3_v11_r4_tailflow_i_20261016/tools/run_crossmap_v10.py --workload fixed --l3-backend independent --v9-mode A --review-mode S --r2-mode D --r3-mode control --r5-mode C --r6-mode H --r7-mode NT --r8-mode OGR --r9-mode PLMX "$@"
