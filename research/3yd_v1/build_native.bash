#!/usr/bin/env bash
set -euo pipefail
task_3yd_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p -- "${task_3yd_root}/build"
g++ -std=c++17 -O2 -shared -fPIC "${task_3yd_root}/native/swept_rect.cpp" -o "${task_3yd_root}/build/libsemantic_sweep.so"
task_eval_root="${task_3yd_root}/external/arena4_ws/src/arena/evaluation/arena_evaluation"
read -r -a task_python_includes <<< "$(/usr/bin/python3-config --includes)"
task_python_suffix="$(/usr/bin/python3-config --extension-suffix)"
g++ -std=c++17 -O3 -shared -fPIC "${task_python_includes[@]}" "${task_eval_root}/src/nav2_effective_costmap.cpp" -o "${task_eval_root}/arena_evaluation/_nav2_effective_costmap${task_python_suffix}"
g++ -std=c++17 -O3 -ffp-contract=off -shared -fPIC "${task_python_includes[@]}" "${task_eval_root}/src/endpoint_geometry_r3.cpp" -o "${task_eval_root}/arena_evaluation/_endpoint_geometry_r3${task_python_suffix}"
