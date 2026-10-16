# 3D-V1 stable runtime

`3D-V1-r2-stable` is the only default production implementation. It reuses
the accepted `r2-production-acceptance` controller and cache-only lifecycle;
there is no duplicated search implementation.

The fixed chain is:

```text
versioned observation + two-frame confirmation
  -> relevance/optimality-safe scheduler
  -> deterministic Graph A* L1
  -> topology-turn adaptive 2/4 m corridor
  -> verified-cache-only selective persistent D* Lite L2
       small relevant blocking increase -> bounded D*
       miss/reject/large/recovery/not-ready/timeout -> deterministic grid A*
       L2 no-route -> exclude blocked topology edges and rerun L1
  -> exact old/new dirty ROI union + effective-content ACK
  -> settle=0, 48-bin Smac Hybrid DUBIN
  -> one canonical PathAudit result
```

There is no pure-D* production mode and no online synchronous D* state build.
A route cache is used only when the stable bundle header, route entry, endpoint
positions and yaws, all binding fields, schemas, sizes, and hashes validate.
Otherwise the request uses deterministic grid A*.

## Production API

```python
from arena_3d_v1 import create_controller

controller = create_controller(
    l1_plan,
    cache_bundle=cache_bundle,
    start_pose=(start_x, start_y, start_yaw),
    goal_pose=(goal_x, goal_y, goal_yaw),
)
```

No `revision` argument resolves to `3D-V1-r2-stable`. Production mode rejects
explicit r0/r1/r2 research revisions. Benchmark or reproduction code must use
an explicit revision and `mode="benchmark"`.

## Cache workflow

The offline deployment workflow creates a verified L1 plan bundle, runs
`three_d_v1_cache_prebuild`, then runs `three_d_v1_cache_verify --deep`.
Writes use temporary files, `fsync`, and atomic rename. `--purge-obsolete-report`
only reports candidates and never removes them.

The runtime telemetry always includes `architecture_id`,
`production_baseline_id`, `source_revision`, `selected_backend`,
`fallback_reason`, and `cache_status`.

Current dynamic evidence is a realistic synthetic workload on the real 4×
map because no map/coordinate/time-aligned cleaning log was available. Stable
means the stable global-planning architecture; it does not claim that a full
Nav2/TEB vehicle stack has been completed.
