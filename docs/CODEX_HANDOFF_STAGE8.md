# Codex Handoff: Arena4 Static A2B Planner, Stage 8 Complete

Updated: 2026-08-24 (Asia/Singapore)

## 1. Workspace and Git state

- Workspace: `/home/robot/pudu_robot_ws`
- Root branch: `report/2026-08-17_2026-08-21`
- Root HEAD: `27d83af` (`docs: add PLN-02 completion status`)
- Root remote tracking branch: `origin/report/2026-08-17_2026-08-21`
- `main` remains at `21dcaee` (`Add static layered planner benchmarks through stage 8`), which is an ancestor of the current report branch.
- Root worktree was clean before this handoff document was added.
- Nested evaluation repository: `/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation`
- Nested repository branch: `humble`, base/upstream HEAD `9476242` (`adjust recorder`), remote `https://github.com/voshch/arena-evaluation.git`.
- Nested evaluation worktree is intentionally dirty and contains the Stage 2-8 source/config/test changes. Do not reset, checkout, or discard it.
- Root `.gitignore` excludes `external/`, `experiments/`, build/install/log caches. The reproducible Arena evaluation source is captured in the root patch `dependencies/patches/arena4-evaluation.patch` in commit `21dcaee`.

## 2. Scope constraints

Read and obey:

`/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/AGENTS.md`

The only research scope is static global A2B planning on occupancy grids. Do not add dynamic obstacles, actors, HuNav, Pedsim, Gazebo navigation, TEB/MPPI/DWB control, cmd_vel tracking, localization, mapping, RRT*, or multi-resolution planning. Keep `dynamic_obstacles: false`. Do not modify Hospital original maps or overwrite Stage 3-8 output directories.

Formal vehicle protocol for Stage 8 is a project constraint, not a claim about the Jackal physical limit:

```yaml
allow_in_place_rotation: false
minimum_turning_radius: 0.40
maximum_curvature: 2.50
allow_reverse: true
reverse_penalty: 2.0
motion_model: REEDS_SHEPP
dynamic_obstacles: false
```

## 3. Completed implementation

The evaluation package includes:

- Stage 2-3 planner microbenchmark and Nav2 `ComputePathToPose` client;
- Stage 4 static topology extraction and topology-guided grid A*;
- Stage 5 fixed `0.05 m` Hospital map derivation and resolution comparison;
- Stage 6 L1 topology + L2 grid ablation/fallback;
- Stage 7 rotation-enabled L3 validator baseline;
- Stage 8A hard-radius L3 validator/local Smac REEDS_SHEPP repair;
- Stage 8B center/right-edge lateral preference scan and selected runs;
- unit tests and report/visualization CLIs.

Last verified before this handoff:

- Python tests: `81 passed`;
- `colcon build --symlink-install --packages-select arena_evaluation`: succeeded;
- no planner/map/lifecycle/Gazebo/dynamic processes remained after experiments.

## 4. Final experiment artifacts

The final packaged handoff is:

`/home/robot/pudu_robot_ws/experiments/deliverables/arena4_static_a2b_experiment_bundle_stage8_v1_20260821`

Archive:

`/home/robot/pudu_robot_ws/experiments/deliverables/arena4_static_a2b_experiment_bundle_stage8_v1_20260821.tar.gz`

Archive SHA-256:

`96444df49029606b091cfb6962ec9af1056e06946da1a0356f4a9444f291adbb`

The package contains renamed source directories, CSV/YAML/JSON tables, PNG figures, 1,524 compressed raw paths, topology files, protocols, manifests, logs, `SOURCE_MAP.tsv`, `RENAMED_ASSET_MAP.tsv`, `FILE_INDEX.csv`, and `SHA256SUMS`. Its README and QUICK_START explain the statistical caveats.

Important final source directories:

```text
experiments/planner_benchmark/hospital_005/stage5_navfn_product
experiments/planner_benchmark/hospital_005/stage5_navfn_normalized
experiments/planner_benchmark/hospital_005/stage5_smac_product
experiments/planner_benchmark/hospital_005/stage5_smac_normalized
experiments/topology_benchmark/hospital_005/stage5_full_v2
experiments/layered_planner_benchmark/hospital_005/stage6_l1_l2
experiments/layered_planner_benchmark/hospital_005/stage7_l3_kinematic
experiments/layered_planner_benchmark/hospital_005/stage8a_hard_radius_l3_v2
experiments/layered_planner_benchmark/hospital_005/stage8b_lateral_preference_v2
experiments/resolution_benchmark/hospital/stage5_strict_v2
```

## 5. Key measured results

### Stage 6 L1/L2

From `stage6_l1_l2/stage6_acceptance_summary.csv`:

- `full_grid`: 9/9 reachable query success;
- `topology_guided_grid`: 6/9 reachable query success;
- `topology_guided_grid_fallback`: 9/9 reachable query success;
- q04 remains the conservative static semantics failure.

From `stage6_l1_l2/stage6_comparison.csv`:

- direct topology-guided node reduction: `42.55%` mean;
- direct topology-guided online speedup: `1.255x` mean on paired valid runs;
- fallback node reduction: `3.84%` mean;
- fallback online speedup: `0.608x` (therefore slower than full-grid);
- fallback path-length ratio P95: `1.132066`.

Topology precompute from `precompute_metrics.csv`:

- wall: `7763.684 ms`;
- CPU: `8588.354 ms`;
- peak RSS: `251772928 bytes`;
- graph: `224 nodes`, `195 edges`, `35 components`;
- persisted topology size: `879577 bytes`.

### Stage 8A hard-radius L3

From `stage8a_hard_radius_l3_v2/stage8_acceptance_summary.csv`:

- candidate rows: `50`;
- final valid successes: `35/50`;
- all-query success rate: `0.70`;
- reachable-query success rate: `0.777778` (`7/9`);
- successful collision paths: `0`;
- successful rotate-in-place segments: `0`;
- successful hard-radius violations: `0`;
- Hybrid calls: `100`, successful calls: `75`;
- action-success/static-invalid rows: `10`;
- q04 code: `STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH`.

L3 repair failed for q00 and q08; q04 must not enter L3. Do not hide these failures or force a 9/9 claim.

### Stage 8B lateral preference

Selected weight for both `center` and `right_edge` was `1.0` under the fixed selection rule. L2 weight scans are in `weight_scan.csv`; selected comparisons are in `stage8b_selected_comparison.csv`.

- center selected path-length ratio P95: `1.059699`;
- center preference error mean: `0.077778 m`;
- right-edge selected path-length ratio P95: `1.110439`;
- right-edge target-side error mean: `0.239983 m`;
- both selected L2 groups had 9/9 reachable-query success before hard-radius L3;
- Stage 8B hard-L3 outputs still have 35/50 final valid rows per preference mode and zero collision rows in successful paths.

## 6. What is not complete

Do not describe the current data as a complete map-scale baseline. Only Hospital 0.1/0.05 resolution comparison exists; there is no validated multi-map scale curve yet. Missing items include:

1. fixed map set by physical/grid scale;
2. fixed query sets and protocols for every map;
3. full NavFn/Smac measured runs for those maps;
4. a cross-map report with time/memory P50/P95/P99 against grid cells, free cells, and physical area;
5. one unified L1/L2/L3 switch ablation table;
6. same-process end-to-end composed timing and peak memory (current composed L3 time is explicitly an estimate);
7. a theory-backed shortest-path optimum (current shortest reference is only the shortest observed valid path).

## 7. Safe continuation plan

The next bounded task is the scale-baseline package, not a new planner algorithm:

1. Read `AGENTS.md` and inspect current worktrees.
2. Freeze a static map set, preferably all maps at `0.05 m`: `ignc_005`, `house17_005`, `factory_005`, `hospital_005`.
3. Derive maps by exact nearest-neighbor replication where needed; preserve origin, extent, hashes, and `dynamic_obstacles: false`.
4. Freeze one versioned query YAML per map and validate all endpoints with the same footprint/clearance semantics.
5. Run only the existing NavFn/Smac planner benchmark, with measured-only primary statistics and independent output directories.
6. Add a read-only cross-map report keyed by `map_id`, `grid_cells`, `free_grid_cells`, `physical_area_m2`, planner/config, and measured run mode.
7. Plot P50/P95/P99 timing, CPU, planner/stack RSS/PSS, success, and path validity versus grid cells and physical area.
8. Do not rerun or modify Stage 3-8 directories.

## 8. First checks for the next Codex

```bash
cd /home/robot/pudu_robot_ws
git status --short
git branch -vv
test -f external/arena4_ws/src/arena/evaluation/AGENTS.md
sha256sum -c experiments/deliverables/arena4_static_a2b_experiment_bundle_stage8_v1_20260821.tar.gz.sha256
source ./setup_arena4_runtime.bash
python3 -m pytest external/arena4_ws/src/arena/evaluation/arena_evaluation/test -q
```

