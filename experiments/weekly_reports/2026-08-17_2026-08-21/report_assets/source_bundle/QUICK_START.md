# Quick Data Guide

## Baseline planner timing, CPU, and memory

- `source_data/14_stage3_hospital_010_cross_planner_summary/`
- `source_data/34_stage5_hospital_010_summary/`
- `source_data/35_stage5_hospital_005_summary/`
- `source_data/36_stage5_resolution_comparison/`

Use `planner_runs.csv` for request-level wall/planning/CPU/RSS/PSS values and
`path_metrics.csv` for path quality. Use only rows where `run_mode=measured`
for primary statistics.

## Topology precomputation and L1/L2 ablation

- `source_data/40_stage5_hospital_005_topology/precompute_metrics.csv`
- `source_data/50_stage6_l1_l2_ablation/summary_by_mode.csv`
- `source_data/50_stage6_l1_l2_ablation/stage6_comparison.csv`
- `source_data/50_stage6_l1_l2_ablation/topology_amortization.csv`

## Hard kinematic validity

- `source_data/70_stage8a_hard_radius_l3/stage8_acceptance_summary.csv`
- `source_data/70_stage8a_hard_radius_l3/stage8a_performance_summary.csv`
- `source_data/70_stage8a_hard_radius_l3/kinematic_runs.csv`

Accepted candidate paths have `final_valid_success=true`; failed and reference
paths must not be mixed into hard-line collision or radius acceptance counts.

## Lateral preference

- `source_data/80_stage8b_lateral_preference/weight_scan.csv`
- `source_data/80_stage8b_lateral_preference/stage8b_selected_comparison.csv`
- `source_data/80_stage8b_lateral_preference/stage8b_acceptance_summary.csv`

## Presentation figures

All final PNG files are copied into `figures/` using unique names such as:

```text
50_stage6_l1_l2_ablation__plots__query_time_by_mode.png
70_stage8a_hard_radius_l3__plots__stage8a_planning_time.png
80_stage8b_lateral_preference__plots__preference_online_time.png
```

The exact renamed-to-source relationship is recorded in
`RENAMED_ASSET_MAP.tsv`.
