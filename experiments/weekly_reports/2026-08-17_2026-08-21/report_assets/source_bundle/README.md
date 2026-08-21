# Arena4 Static A2B Experiment Bundle

Bundle version: `stage8-v1-20260821`

This package contains the final, potentially reusable static A2B planning
artifacts produced through Stage 8. Original experiment directories are not
modified. Each source directory is copied under a stable, descriptive name in
`source_data/`; `tables/` and `figures/` provide flattened, uniquely named
entry points for analysis and presentation.

## Scope

Included:

- Stage 3 Hospital 0.1 m NavFn and Smac planner baselines;
- Stage 4 Hospital 0.1 m topology benchmark;
- Stage 5 Hospital 0.05 m planner baselines and 0.1/0.05 comparison;
- Stage 5 topology artifacts for both resolutions;
- Stage 6 L1+L2 ablation and fallback measurements;
- Stage 7 rotation-enabled L3 ablation baseline;
- Stage 8A hard-radius L3 results;
- Stage 8B center/right-edge preference sweep and selected results;
- maps, fixed queries, protocols, manifests, raw paths, logs, CSV tables,
  topology arrays, and report figures contained in those final directories.

Excluded deliberately:

- smoke, debug, archived intermediate attempts, and duplicate weekly reports;
- `teb_hospital` navigation/controller data, which is outside the static
  global A2B benchmark scope;
- build/install/log caches outside the selected final experiment directories.

## Important interpretation

- `action_success` is not equivalent to `final_valid_success`.
- Stage 8 accepted paths enforce no in-place rotation and a project-level hard
  minimum turning radius of 0.40 m.
- q04 retains `STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH`.
- `composed_online_time_ms` is an explicitly marked layered estimate, not a
  same-process end-to-end measurement.
- The Hospital 0.1/0.05 comparison is a resolution comparison, not a
  multi-map scale curve.
- `length_over_shortest_observed_valid` is not a theoretical global optimum.

## Indexes

- `SOURCE_MAP.tsv`: renamed directory to original source mapping.
- `FILE_INDEX.csv`: size and SHA-256 for every packaged file.
- `SHA256SUMS`: standard checksum list.
- `tables/`: flattened CSV/YAML/JSON analysis files.
- `figures/`: flattened PNG figures with stage prefixes.

