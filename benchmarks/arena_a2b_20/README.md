# Arena A2B benchmark: 20 fixed pairs

This benchmark contains 20 deterministic start/goal pose pairs for each of
ten Arena maps: 200 tasks in total. The eight generated scale maps share one
normalized task layout. The mentor warehouse map and its exact 4x-area variant
share a second normalized layout fitted to their connected corridor network.

The files are:

- `arena_a2b_benchmark_20.json`: complete manifest with all coordinates,
  headings, clearances, scale thresholds, preferences, and feature tags.
- `arena_a2b_benchmark_20.csv`: flat table for experiment runners.
- `external/arena4_ws/src/arena/simulation-setup/worlds/<world>/scenarios/a2b_benchmark_20.json`:
  Arena-compatible scenario file for each map.

Every endpoint is selected on a free PGM cell with a measured clearance above
0.80 m (the required lower bound is 0.60 m). The generator validates free-space connectivity and rejects a pair
whose direct start-goal distance is below the scale threshold:

| map band | minimum direct distance |
| --- | --- |
| small | 0.18 x min(width, height) |
| medium | 0.20 x min(width, height) |
| large | 0.22 x min(width, height) |
| xlarge | 0.25 x min(width, height) |

The task mix is 8 center-preference, 8 edge-preference, and 4 no-preference
tasks per map. Feature tags cover east-west and north-south corridors, central
transfer zones, wide cross aisles, rooms, doors, and warehouse shelf aisles as
applicable to each map family. The metadata is intended for the
topology/grid/kinematic layer ablation and for grouping results by preference
and map scale.

Regenerate and validate after changing a map with:

```bash
python3 tools/generate_a2b_benchmark.py \
  --source-root external/arena4_ws/src/arena/simulation-setup/worlds \
  --manifest-root benchmarks/arena_a2b_20
```

For Arena's scenario mode, select one map and set
`task.scenario.file:=a2b_benchmark_20.json`. The scenario contains all 20
records; a single Jackal run consumes the selected record, so an evaluator
should iterate by task id and use the corresponding `start` and `goal` pose.
