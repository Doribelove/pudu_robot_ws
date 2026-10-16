#!/usr/bin/env bash
set -eo pipefail
cd /home/robot/workspaces/3yd_multires_static_20260914
source env.bash
set -u
for arm in baseline topology_005 topology_015; do
 /usr/bin/python3 tools/static_benchmark.py --output "results/formal_01_${arm}" --arm "$arm" --indices 0:20 --repeats 3 --domain 223
done
for scene in calibration rotated; do
 for arm in baseline topology_005 topology_015; do
  /usr/bin/python3 tools/static_benchmark.py --output "results/control_formal_01_${scene}_${arm}" --arm "$arm" --scene "$scene" --repeats 3 --domain 223
 done
done
/usr/bin/python3 tools/static_benchmark.py --output results/narrow_formal_01_topology_015 --arm topology_015 --scene narrow --repeats 3 --domain 223
for arm in baseline topology_005 topology_015; do
 /usr/bin/python3 tools/static_preparation.py --output "results/prep_formal_01_${arm}" --arm "$arm" --phase build
 /usr/bin/python3 tools/static_preparation.py --output "results/prep_formal_01_${arm}" --arm "$arm" --phase restore
done
/usr/bin/python3 tools/summarize_static.py --tag formal_01
