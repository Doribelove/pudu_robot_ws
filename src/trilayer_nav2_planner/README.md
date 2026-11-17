# TriLayer Humble Nav2 planner adapter

This package exports `trilayer_nav2_planner/Planner`, a real `nav2_core::GlobalPlanner` plugin. It does not manufacture paths. A separately started `trilayer_planner_service` worker must run real P6 and return a successful, context-bound path, otherwise createPlan throws `nav2_core::PlannerException`.

## Interface boundary

Stock installed Humble `nav2_core/global_planner.hpp` has configure/activate/deactivate/cleanup and `createPlan(start,goal)`. It has **no cancellation callback**. Thus planner_server ComputePathToPose cancellation cannot be promised to interrupt an in-progress call immediately. Polling transport checks deactivate every 50 ms, and each call has a bounded timeout (default/max 60 seconds). Closing the socket alone is not proof that the worker stopped computation; worker supervision owns its computation/process bound.

Configure `<plugin_name>.worker_socket` to an explicit absolute Unix socket and optional `<plugin_name>.timeout_seconds` in (0,60]. Parent and poses must use `map`. Only .05 m, <=1M cells, nonrolling costmaps and a four-corner .265 m by .225 m half-extent robot footprint including padding are supported. No TF is guessed. Unknown cells are conservatively hard obstacles. Configure footprint padding=0 with the exact rectangle.

Each call copies the caller's own Nav2 master under its lock, releases the lock, and sends the full uint8 bytes base64 in bottom-row-first order. `context` binds dimensions, origin, resolution, frame, SHA256 full bytes and SHA256 of hard mask (cost>=253 becomes byte1, otherwise0). The worker projects this exact snapshot into its real P6 map, reuses its session only for the same context, and closes/reopens cold on changes. This is **not incremental reuse across host map changes**. Worker DDS must be explicitly isolated from caller domain to avoid its internal Nav2 nodes/services colliding. The plugin never queries a global costmap service from an unrelated graph.

Before returning, the plugin takes a second caller snapshot and rejects any full-byte/geometry change. It does not hold a costmap lock while waiting for P6. Non-success, wrong request ID/context/frame, missing/nonfinite path or empty path fail closed. No command-velocity output exists.

Protocol: Unix stream, one JSON line per connection, v=1, op=plan, id, start/goal [x,y,yaw], context, snapshot base64. Response v/id/ok/frame_id/context/path plus optional diagnostics. Response cap8MiB. Source includes no algorithm changes.

## Verification status

Build and live planner_server acceptance must be separately reported. A plugin registration or transport-negative test is not proof of actual P6 planning through ComputePathToPose. Live tests use isolated caller domain185 and a distinct worker domain.

Returned paths additionally require `audit_valid=true` and integer matching dynamic/trajectory versions. Endpoint acceptance follows the frozen P6 `endpoint_tolerance.json` maximum, 0.2 m Euclidean and 30 degrees wrapped yaw at each endpoint. This is not an exact-endpoint promise: no connector is invented from requested to resolved endpoints. Footprint is revalidated on both before/after snapshots, so a runtime body change fails closed.

## Verified integration

Stock Humble 1.1.20 build and actual pluginlib load/inactive-rejection pass.
`test/planner_server_smoke.py` runs real planner_server lifecycle and
ComputePathToPose actions against the real P6 socket worker, with no fabricated
path provider. The .05 m 240x240 fixture plans (2,4,0)→(5,4,0): initial33points,
repeat same snapshot, then a new crossing host obstacle yields253points through
real cold-rebuilt P6. Caller domain185; worker187. A separate explored longer
obstacle fixture returned L3_SPATIAL_BUDGET and correctly aborted with empty path;
bounded failure is not physical infeasibility proof.

Configure `TRILAYER_NAV2_TEST_SOLVER_ROOT` to the installed source-built solver
Python directory to enable the self-contained real action CTest. The supervisor
checks same-snapshot native PID reuse, changed-snapshot cold rebuild and worker
cleanup. The same integration also verifies mid-request host mutation rejection after a
real successful P6 computation, invalid quaternion rejection and unavailable
worker rejection. Exhaustive timeout/cancellation/malformed-response coverage is
not claimed.

Projection scope: costs below253 are not optimized as Nav2 soft penalties; their
bytes are still bound in the snapshot hash. This initial plugin uses map-only P6,
not semantic-scene input. These limitations are not hidden by the plugin name.

## Minimal bringup (no robot controller)

Source the Humble and TriLayer install setup. In one terminal start the broker
with a **new**, private output directory and a DDS domain distinct from the host:

```
ros2 run trilayer_planner_service serve --output /tmp/my-trilayer-worker --domain-id 187 --lifetime 0
```

Pass `--solver-root` only when explicitly selecting an alternate verified solver
installation. Wait for `ready.json` in that directory. Copy the installed
`share/trilayer_nav2_planner/config/planner_server.yaml`, replace the socket path
with `/tmp/my-trilayer-worker/service.sock`, and provide a .05 m `/map` plus valid
`map -> base_link` TF from your existing setup. The plugin does not start drivers,
a controller, map server or a robot localization system. Use the selected host
domain (185 in the isolated tests):

```
ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1 ros2 run nav2_planner planner_server --ros-args --params-file /absolute/path/to/copied.yaml
```

In another terminal with the same host ROS environment, configure then activate:

```
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

The action `/compute_path_to_pose` now selects planner_id `TriLayer` and accepts
map-frame poses. On shutdown deactivate then cleanup the planner and stop the
owned broker. Explicit lifetime 0 runs until stopped; per-request watchdogs remain
active. Without that option the default lifetime is 300 seconds. The broker
refuses new planning below its output-filesystem free-space floor (512 MiB minimum)
and safely closes its owner; it never deletes audit logs automatically. The installed
`persistent_service.launch.py` provides user-owned launch without automatic respawn.
This is not a system service installation or a long-duration soak certification.
Nav2 integration is currently **map-only**; semantic planning remains available
through the Python API. Host map changes rebuild cold, not incremental. Unknown255
and costs>=253 project as hard obstacles; allow_unknown=true is refused.

## Independent pre-install testing

CTest creates a package-private `build/test_install` fixture with `cmake --install
--prefix`, leaving the configured production install prefix untouched. Tests
requiring installed resources/plugins depend on that fixture and resolve it
before the declared dependency underlay. An existing fixture directory is
replaced only when its ownership marker exists; an unowned directory fails.
This does not disable installed-layout tests or require an older installed copy
of this package. Optional solver-root integration tests retain their existing
enabling flags and use the same private stage when enabled.
