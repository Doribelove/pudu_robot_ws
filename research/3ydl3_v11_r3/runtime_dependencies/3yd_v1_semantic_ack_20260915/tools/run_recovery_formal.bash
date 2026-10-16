#!/usr/bin/env bash
set -eo pipefail
RECOVERY_ROOT=/home/robot/workspaces/3yd_v1_static_recovery_20260914
V1_ROOT=/home/robot/workspaces/3yd_multires_static_20260914
cd "$RECOVERY_ROOT"
source env.bash
/usr/bin/python3 tools/static_benchmark.py --output "$RECOVERY_ROOT/results/formal_recovery" --arm recovery --indices 0:20 --repeats 3 --domain 224
(
 cd "$V1_ROOT"
 source env.bash
 /usr/bin/python3 tools/static_benchmark.py --output "$RECOVERY_ROOT/results/formal_v1" --arm topology_015 --indices 0:20 --repeats 3 --domain 224
)
for scene in calibration rotated narrow two_routes two_routes_optimistic_edge return_bulb return_bulb_rotated; do
 for arm in topology_015 recovery; do
  /usr/bin/python3 tools/static_benchmark.py --output "$RECOVERY_ROOT/results/control_${scene}_${arm}" --arm "$arm" --scene "$scene" --repeats 3 --domain 224
 done
done
