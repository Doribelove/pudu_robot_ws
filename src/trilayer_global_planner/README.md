# TriLayer Global Planner

Executable entry to the actual P6 `IncrementalService` H-I planner, not a metapackage.
It starts a source-built Nav2 planning worker in an explicitly chosen local DDS
domain, initializes the real map-only L1/L2/L3 pipeline, returns a real path or
failure, and cleans up the worker. No robot driver/controller is started.

```sh
ros2 run trilayer_global_planner plan --demo obstacle --domain-id 179 --output /tmp/my-trilayer-plan
ros2 run trilayer_global_planner plan --map /absolute/map.yaml \
  --start 2 4 0 --goal 5 4 0 --domain-id 179 --output /tmp/my-other-plan
```

Choose an unused DDS domain. Output must be a new directory. The CLI explicitly
sets localhost-only discovery; do not reuse a domain belonging to another task.
The demo generator produces a new 240×240 map at 0.05 m/cell with a 0.5×1.5 m
obstacle. No private map is included. `straight` and `small_budget` demos are also
available; the latter intentionally exercises the original spatial-budget failure.

Positions are metres and yaw radians in fixed `map` frame. The original vehicle
is fixed at half-length 0.265 m, half-width 0.225 m, minimum turning radius 0.40 m.
The original P6 algorithm settings are not exposed as arbitrary CLI tuning knobs.
Custom maps require explicit start and goal. Semantic traffic inputs are not yet
accepted by this entry, even though separate semantic L1 primitives are available.

`request.json` records inputs, `result.json` contains runtime results/path and
validation evidence, `worker.log` contains diagnostics, and `execution.json`
records exit/timeout/cleanup status. A successful exit requires runtime success
and clean process exit. Exit 1 means planner/runtime failure, 124 means the whole
worker timed out, and 130 means cancellation. Default whole-worker bound is 60s,
followed by bounded cleanup (up to 20s). This is not an algorithm performance claim.

Solver packages must be provisioned in the installed environment. For explicit
source-built dependency testing, `--solver-root /absolute/site-packages` may be
provided; the runtime records imported solver origins. No solver binary is
vendored into this entry package. No apt publication has been made. Placeholder
maintainer/dependency release gates remain before Bloom.

## Verified entry milestone

Five CTest entries pass: four CLI unit contracts in one test, plus real obstacle,
small-map rejection, custom start/goal, and forced whole-worker timeout. The
obstacle returns 253 path poses from (2,4,0) to (5,4,0), executes actual Clarabel
SQP and passes the runtime's independent audit. The custom request returns 44
poses from (3,6,0) to (7,6,0). The small map reports `L3_SPATIAL_BUDGET` with exit 1.
A 1-second bound produces `WORKER_TIMEOUT`, exit 124, and no remaining tracked
process. Compact result hashes/evidence are in `test/evidence/ENTRY_MILESTONE.json`.

Configure `TRILAYER_ENTRY_TEST_SOLVER_ROOT` to register real runtime integration
tests; they reserve DDS domain 181 and run serially. CTest is bounded at 90s each.
The subprocess tests cover cleanup of a nested process that starts its own session.
`runtime_result.json` preserves the API response; `result.json` adds supervisor
acceptance/timeout state and never publishes a successful accepted path on timeout.
This validates the entry on synthetic map-only cases, not full P6 equivalence or
hardware suitability.

## Nav2 installation entry

This package also depends on `trilayer_nav2_planner`, which brings the persistent snapshot service. The existing `plan` CLI remains map-only; use the installed Nav2 package configuration for actual planner_server/ComputePathToPose integration. This dependency does not announce an available apt release.
