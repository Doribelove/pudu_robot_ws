#!/usr/bin/env bash
set -eo pipefail
cd /home/robot/workspaces/3yd_v1_static_recovery_20260914
source env.bash
/usr/bin/python3 tools/static_benchmark.py --output "$PWD/results/formal_recovery_final" --arm recovery --indices 0:20 --repeats 3 --domain 201
recovery_domain=202
for scene in calibration rotated narrow two_routes two_routes_optimistic_edge return_bulb return_bulb_rotated; do
 /usr/bin/python3 tools/static_benchmark.py --output "$PWD/results/control_final_${scene}_recovery" --arm recovery --scene "$scene" --repeats 3 --domain "$recovery_domain"
 recovery_domain=$((recovery_domain+1))
done
