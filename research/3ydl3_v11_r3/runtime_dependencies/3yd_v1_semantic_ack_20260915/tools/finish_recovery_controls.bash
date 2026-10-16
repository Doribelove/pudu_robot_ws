#!/usr/bin/env bash
set -eo pipefail
cd /home/robot/workspaces/3yd_v1_static_recovery_20260914
source env.bash
for scene in two_routes_optimistic_edge return_bulb return_bulb_rotated; do
 for arm in topology_015 recovery; do
  suffix=""
  if [ "$scene" = two_routes_optimistic_edge ] && [ "$arm" = topology_015 ]; then suffix="_startup_retry2"; fi
  /usr/bin/python3 tools/static_benchmark.py --output "$PWD/results/control_${scene}_${arm}${suffix}" --arm "$arm" --scene "$scene" --repeats 3 --domain 224
 done
done
