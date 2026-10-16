#!/usr/bin/env bash
set -euo pipefail
cd /home/robot/workspaces/3yd_v8_r7_fullroute_20261008
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 THREEYD_V8_FULLROUTE_REUSE=1
taskset -c 12 ./run.sh --mode semantic --map-id mengjia --repeats 3 --output results/final/mengjia --domain 220
taskset -c 12 ./run.sh --mode free-space --map-id arena_large_250x300_2d_005 --repeats 3 --output results/final/arena_large_250x300_2d_005 --domain 220
taskset -c 12 ./run.sh --mode semantic --map-id 3yd_synth_multiloop_a_v1 --repeats 1 --output results/additional/3yd_synth_multiloop_a_v1 --domain 220
taskset -c 12 ./run.sh --mode free-space --map-id arena_medium_200x200_2d_005 --repeats 1 --output results/additional/arena_medium_200x200_2d_005 --domain 220
taskset -c 12 ./run.sh --mode semantic --map-id mengjia --queries Q01,Q06,Q09 --interleave --repeats 2 --output results/interleaved/mengjia --domain 220
