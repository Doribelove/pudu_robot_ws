#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-layered}"
if [[ "${mode}" == "layered" ]]; then
  planner="navfn"
  variant="product"
  query_id="${2:-all}"
  cycle_interval="${3:-2.0}"
else
  if [[ "${mode}" == "baseline" ]]; then
    shift
  fi
  mode="baseline"
  planner="${1:-navfn}"
  variant="${2:-product}"
  query_id="${3:-all}"
  cycle_interval="${4:-1.0}"
fi

usage() {
  cat <<'EOF'
Usage:
  ./view_arena4_baseline.sh [layered] [all|q00..q09] [interval_seconds]
  ./view_arena4_baseline.sh baseline [navfn|smac] [product|normalized] [all|q00..q09] [interval_seconds]
  ./view_arena4_baseline.sh [navfn|smac] [product|normalized] [all|q00..q09] [interval_seconds]

Examples:
  ./view_arena4_baseline.sh                         # L1/L2/L3 replay, q00..q09, 2 seconds
  ./view_arena4_baseline.sh layered q03 2.0        # one layered query
  ./view_arena4_baseline.sh baseline navfn normalized all 1.0
  ./view_arena4_baseline.sh smac normalized all 1.5

The default layered mode replays the recorded static Stage 6/8 result and
shows L1 topology, L2 grid, and L3 kinematic paths together. It cycles q00..q09
every two seconds. Baseline mode retains the live Nav2 planner visualization.
No dynamic obstacle or vehicle controller is started.
EOF
}

if [[ "${planner}" == "-h" || "${planner}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${planner}" != "navfn" && "${planner}" != "smac" ]]; then
  echo "Invalid planner: ${planner}" >&2
  usage >&2
  exit 2
fi
if [[ "${variant}" != "product" && "${variant}" != "normalized" ]]; then
  echo "Invalid variant: ${variant}" >&2
  usage >&2
  exit 2
fi
if [[ "${query_id}" != "all" && ! "${query_id}" =~ ^q0[0-9]$ ]]; then
  echo "Invalid query ID: ${query_id}; expected all or q00..q09" >&2
  exit 2
fi
if [[ ! "${cycle_interval}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
   ! awk -v value="${cycle_interval}" 'BEGIN { exit !(value > 0.0) }'; then
  echo "Invalid interval: ${cycle_interval}; expected a positive number" >&2
  exit 2
fi

if [[ "${planner}" == "navfn" ]]; then
  stage_dir="stage5_navfn_${variant}"
  params_name="stack_params_navfn_${variant}.yaml"
else
  stage_dir="stage5_smac_${variant}"
  params_name="stack_params_smac_hybrid_${variant}.yaml"
fi

map_yaml="${workspace_dir}/experiments/maps/hospital_005/map.yaml"
queries_file="${workspace_dir}/experiments/planner_benchmark/hospital_005/queries_v2.yaml"

stage8_dir="${workspace_dir}/experiments/layered_planner_benchmark/hospital_005/stage8a_hard_radius_l3_v2"
stage6_dir="${workspace_dir}/experiments/layered_planner_benchmark/hospital_005/stage6_l1_l2"
topology_dir="${workspace_dir}/experiments/topology_benchmark/hospital_005/stage5_full_v2/topology"
params_file="${workspace_dir}/experiments/planner_benchmark/hospital_005/${stage_dir}/logs/${params_name}"

for required_file in "${map_yaml}" "${queries_file}" "${params_file}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required Stage 5 file is missing: ${required_file}" >&2
    exit 2
  fi
done

# shellcheck disable=SC1091
source "${workspace_dir}/setup_arena4_runtime.bash"

echo "Opening static baseline: planner=${planner}, variant=${variant}, query=${query_id}, interval=${cycle_interval}s"
echo "Close RViz or press Ctrl+C here to stop all visualization processes."

exec ros2 launch arena_evaluation planner_baseline_visualization.launch.py \
  map_yaml:="${map_yaml}" \
  params_file:="${params_file}" \
  queries_file:="${queries_file}" \
  query_id:="${query_id}" \
  cycle_interval:="${cycle_interval}" \
  visualization_mode:="${mode}" \
  planner_label:="${planner}/${variant}" \
  stage8_directory:="${stage8_dir}" \
  stage6_directory:="${stage6_dir}" \
  topology_directory:="${topology_dir}" \
  stage_label:="Stage 8A L1/L2/L3 replay" \
  start_rviz:=true
