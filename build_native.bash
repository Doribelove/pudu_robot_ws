#!/usr/bin/env bash
set -euo pipefail
task_core_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_core_cxx="${CXX:-g++}"
mkdir -p -- "${task_core_root}/build"
for task_core_name in goal_dijkstra cold_dstar pose_batch; do
  "${task_core_cxx}" -std=c++17 -O3 -shared -fPIC -Wall -Wextra \
    "${task_core_root}/native/${task_core_name}.cpp" \
    -o "${task_core_root}/native/lib${task_core_name}.so"
done
"${task_core_cxx}" -std=c++17 -O2 -shared -fPIC \
  "${task_core_root}/native/swept_rect.cpp" \
  -o "${task_core_root}/build/libsemantic_sweep.so"
"${task_core_cxx}" -std=c++17 -O3 -shared -fPIC \
  "${task_core_root}/src/three_yd_v4/vehicle_search.cpp" \
  -o "${task_core_root}/build/libv4_vehicle_search.so"
task_core_python_config="${PYTHON_CONFIG:-python3-config}"
read -r -a task_core_python_includes <<< "$("${task_core_python_config}" --includes)"
task_core_python_suffix="$("${task_core_python_config}" --extension-suffix)"
"${task_core_cxx}" -std=c++17 -O3 -shared -fPIC \
  "${task_core_python_includes[@]}" \
  "${task_core_root}/native/extensions/nav2_effective_costmap.cpp" \
  -o "${task_core_root}/src/arena_evaluation/_nav2_effective_costmap${task_core_python_suffix}"
"${task_core_cxx}" -std=c++17 -O3 -ffp-contract=off -shared -fPIC \
  "${task_core_python_includes[@]}" \
  "${task_core_root}/native/extensions/endpoint_geometry_r3.cpp" \
  -o "${task_core_root}/src/arena_evaluation/_endpoint_geometry_r3${task_core_python_suffix}"
printf '%s\n' 'Algorithm kernels built locally. Generated libraries are ignored by Git.'
