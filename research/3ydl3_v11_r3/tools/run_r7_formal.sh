#!/usr/bin/env bash
set -euo pipefail
cd /home/robot/workspaces/3yd_v8_r7_fullroute_20261008
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
for arm in control fullroute; do
 if [[ "$arm" == control ]]; then export THREEYD_V8_FULLROUTE_REUSE=0; else export THREEYD_V8_FULLROUTE_REUSE=1; fi
 taskset -c 12 ./run.sh --mode semantic --map-id mengjia --repeats 3 --output "results/$arm/mengjia" --domain 220
 taskset -c 12 ./run.sh --mode free-space --map-id arena_large_250x300_2d_005 --repeats 3 --output "results/$arm/arena_large_250x300_2d_005" --domain 220
done
