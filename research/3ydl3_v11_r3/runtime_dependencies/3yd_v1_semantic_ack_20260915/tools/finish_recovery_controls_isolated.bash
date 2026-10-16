#!/usr/bin/env bash
set -eo pipefail
cd /home/robot/workspaces/3yd_v1_static_recovery_20260914
source env.bash
/usr/bin/python3 tools/static_benchmark.py --output results/control_two_routes_optimistic_edge_recovery_isolated --arm recovery --scene two_routes_optimistic_edge --repeats 3 --domain 226
/usr/bin/python3 tools/static_benchmark.py --output results/control_return_bulb_topology_015 --arm topology_015 --scene return_bulb --repeats 3 --domain 227
/usr/bin/python3 tools/static_benchmark.py --output results/control_return_bulb_recovery --arm recovery --scene return_bulb --repeats 3 --domain 228
/usr/bin/python3 tools/static_benchmark.py --output results/control_return_bulb_rotated_topology_015 --arm topology_015 --scene return_bulb_rotated --repeats 3 --domain 229
/usr/bin/python3 tools/static_benchmark.py --output results/control_return_bulb_rotated_recovery --arm recovery --scene return_bulb_rotated --repeats 3 --domain 230
