#!/usr/bin/env bash
set -eo pipefail
cd /home/robot/workspaces/3yd_multires_static_20260914
source env.bash
for arm in topology_005 topology_015; do
 /usr/bin/python3 tools/static_preparation.py --output "results/prep_formal_02_${arm}" --arm "$arm" --phase build
 /usr/bin/python3 tools/static_preparation.py --output "results/prep_formal_02_${arm}" --arm "$arm" --phase restore
done
/usr/bin/python3 tools/validate_restored_static.py
/usr/bin/python3 tools/summarize_static.py --tag formal_01
