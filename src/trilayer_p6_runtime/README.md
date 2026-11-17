# TriLayer P6 runtime source envelope

This package imports the actual `IncrementalService` and `CurveSession` from a
private source tree, using the original ordered P6 overlays:
`profiles/p6/src`, `base/src`, `base/runtime/base/src`, `base/runtime`.
It does not install legacy namespaces globally or source historical env scripts.

Call `activate(RuntimeConfig(master_service=..., cache_root=...))` explicitly.
It locates installed resources with ament_index, applies a captured P6 algorithm
profile, rejects conflicting already-loaded namespaces/settings, and returns the
original class objects plus module/hash provenance. It does not start ROS or
instantiate a planner. The service endpoint must be explicitly supplied and should
point to `trilayer_master_snapshot`'s locked master service.

Source binding adaptations are enumerated with original and patched hashes in
`SOURCE_PROVENANCE.json`. They bind the endpoint/resource root, L2 helpers and
semantic sweep to installed packages, add the read-only master snapshot layer,
and preserve caller-selected transport isolation. Algorithm source/profile
priority is unchanged. The 67 scalar algorithm flags are recorded explicitly;
historical paths and CPU affinity are excluded.

## Current validation and limits

The package now exposes real single-request `plan_map` and `plan_semantic` APIs.
The latter accepts only a strict, validated static rectangular semantic subset,
not arbitrary legacy scene documents or complete P6 functionality. The companion
CLI remains map-only. The public persistent P6Session API is documented below. Both APIs start a
private stock-Nav2 worker and require a reserved localhost DDS domain and an
external process-tree timeout, not an existing robot control stack.

Two Python extensions, 17 ctypes helpers and the original native Nav2 worker
are built from provenance-recorded C++ source. Semantic sweep comes from the
source-built L3 geometry package; the original optimized P6 L3 implementation is
retained, not replaced by that package's base-profile Python solver. Numerical
solver dependencies must be source-built and explicitly installed. No historical
binary wheel or .so is copied into this runtime package.

The effective-costmap/endpoint extensions match the original frozen references
on 120 cases. Public API tests cover actual map-only and semantic service calls,
SQP/certification, real-master ACKs, expected traffic/parking refusals, different
query endpoints, invalid inputs, input snapshots and interruption cleanup.
These bounded synthetic cases are not complete P6 equivalence, hardware/ARM
coverage or performance benchmarks. Full license/rosdep/release auditing and
matched-input broader regression remain release gates. No apt release was made.

The conservative Python envelope contains research benchmark defaults, inventoried
in `resources/unresolved_historical_paths.json`. Explicit runtime guards block
unsupported benchmark/storage-crop entrypoints before their private-path I/O;
the shared adaptive-corridor geometry function remains unchanged.

## Real service milestone (2026-11-17)

`test/test_service_request.py` generates a 240×240 map (0.05 m, occupied border)
then uses the original map-only `FreeSpaceTopology`, `IncrementalService` H-I and
`CurveSession`. It runs the source-built native Nav2 worker with stock Humble
libraries and the independent locked-master snapshot plugin. It does not load
robot drivers. Run only with explicit `ROS_LOCALHOST_ONLY=1`, isolated domain
179, an external 60-second timeout, new output directory, and installed package
setup files. Never source a historical workspace environment. Solver dependency
paths must be explicitly supplied until vendor packaging is finished.

Observed: 3 m straight request succeeds; independent swept-body audit valid,
endpoint position/yaw error zero. Bootstrap ACK checks 256 cells over two stable
reads; request exact-effective-master ACK checks 6844 cells over two stable
reads with zero mismatch. Native worker exits 0 and cleanup has no errors.
The original certified quadratic initializer accepts this case without SQP
iterations, so this is **not a Clarabel numerical-solver integration test**.
No semantic-traffic scenario, obstacle update sequence, full P6 equivalence,
ARM build, or performance claim follows from it.

An earlier 160×160 map is correctly rejected by the unmodified L3 spatial-budget
check (2684 required versus 2560 allowed cells). The map was enlarged rather
than relaxing algorithm flags. Evidence is in `/tmp/trilayer-p6-request-1` and
`/tmp/trilayer-p6-request-2` on the test host, not a shipped runtime dependency.

17 additional original ctypes helpers and the native Nav2 worker are now built
from provenance-recorded C++ sources. The strict Smac YAML is installed. Resource
patches bind semantic sweep to the source-built L3 package, add a read-only
snapshot plugin, and remove the historical FastDDS default/localhost override.
The caller's transport configuration remains explicit. Full license/rosdep and
release metadata audit remains a release gate.

### Original-profile obstacle SQP coverage

The next generated fixture adds a 0.5×1.5 m rectangular obstacle across the
straight route. With the **unchanged original configuration** it naturally
rejects quadratic fast-start certification and performs three real Clarabel SQP
iterations (12/12/14 inner QP iterations in the recorded run). All report Solved
and accepted. The 4.1806 m path has a certified curvature upper bound 2.47484/m
(limit 2.5/m), valid independent swept-body audit over 411 dense poses, and zero
endpoint error. Both real-master ACK stages and worker exit 0 are asserted.
This closes the SQP branch for one synthetic map-only example, not a full P6
regression suite, semantic traffic validation, or hardware benchmark.

Configure `-DTRILAYER_P6_TEST_SOLVER_ROOT=/absolute/source-built/site-packages`
to register the three service tests in addition to the import contract:
`p6_service_straight_contract`, `p6_service_small_budget_contract`, and
`p6_service_obstacle_contract`. Tests print and validate all OSQP, QDLDL and
Clarabel module origins beneath that supplied directory. The obstacle test
requires actual Clarabel iterations and certification. Each uses fresh temporary
output, a 60-second interrupt bound (5-second forced-termination backstop),
CTest timeout 70 seconds, and serial/domain resource locking. Run in a clean
environment containing only stock Humble and these new package prefixes; no
historical Python paths or environment scripts. CTest passed 4/4 in 6.84 seconds
on the development x86_64 host. Explicit isolated domain 179 must be reserved
for these tests; do not run alongside another owner of that DDS domain.

`test/evidence/` preserves compact original straight-success, original small-map
budget rejection, and original obstacle-SQP results, including source result
hashes. Original result files were not overwritten. An optional `forced_sqp`
mode is available only as a clearly labeled test-resource copy with
`certified_quadratic_start=false`; it was **not needed or run** for this milestone.

## Public single-request map-only API

```python
from pathlib import Path
from trilayer_p6_runtime.api import MapOnlyRequest, plan_map
request = MapOnlyRequest(
    map_yaml=Path('/absolute/map.yaml'),
    start=(2.0, 4.0, 0.0), goal=(5.0, 4.0, 0.0),
    output_dir=Path('/absolute/new-output-directory'))
evidence = plan_map(request)  # in caller-owned, isolated, time-bounded subprocess
```

`start`/`goal` are actual query inputs, not fixed demo constants. `path` is a list
of `[x,y,yaw]`, frame `map`; `plan_result` retains full planner metadata;
`result`, ACK events, solver sources, exception and cleanup records are returned
and written to the new output directory. Existing outputs are rejected. `success`
is false if runtime/cleanup fails or the worker does not exit 0. Optional
`solver_root` selects an explicit source-built dependency directory and validates
all three solvers' origins; without it installed environment hooks are used.

Supported contract is deliberately narrow: original fixed vehicle half-length
0.265 m, half-width 0.225 m, turning radius 0.40 m; 0.05 m trinary map, zero map
origin rotation, dimensions at least 16×16 and at most one million cells; finite
in-bounds poses; fixed owned snapshot endpoint. Unsupported inputs are explicitly
rejected before starting ROS. A separate nondefault DDS domain and localhost are
required; caller must reserve it and enforce timeout/process-tree cleanup.
This API starts a private static-map planning stack, not an existing robot stack,
and does not send control commands. Use a fresh interpreter for each request.

The integration tests call this public API, including shifted start/goal
(3,6,0)→(7,6,0), and assert actual returned endpoints. A validation contract checks
seven invalid input forms before output/process creation.

## Experimental semantic service coverage

`test/test_semantic_service.py` exercises original SemanticWorld and DefaultTopology
on a generated west-lane → junction → east-lane scene. Actual IncrementalService
selects regions [1,2,3], audits expected/visited roads [1,3], performs real Clarabel
SQP, and returns a certified 15.26 m path with zero endpoint error. The real master
ACK checks 31860 cells, two stable observations, no mismatches; worker exits 0.
This scene has no one-way or parking-access constraints, so those remain untested.
This first probe preceded the strict static semantic API documented below; it did not establish arbitrary semantic-schema support.

### Semantic rules and parking regression fixtures

The experimental semantic tests now cover the original source contracts:
`traffic_rules.one_way[{semantic_id,yaw}]`,
`forbidden_transitions[[from,junction,to]]`, and query/endpoint-specific parking
bindings with ownership, anchor tolerance, allowed road, travel side and local
cross-road permission. No production default was modified.

Positive real-service cases: eastbound one-way traversal and explicitly bound
parking exit both produce certified paths with real-master ACKs and traffic/body
audits. Negative real-service cases assert **no delivered path**, normal cleanup,
and exact final failure codes:

- Reverse one-way: `WRONG_TRAVEL_DIRECTION`.
- Forbidden west→junction→east: `V12_FULL_L2_TRAFFIC_REJECT:['FORBIDDEN_TURN']`.
- Missing parking ownership: `PARKING_OWNERSHIP_UNRESOLVED`.
- Parking anchor mismatch: `PARKING_BINDING_POSITION_MISMATCH`.
- Disallowed parking travel side: `PARKING_ENDPOINT_TRAVEL_SIDE`.

These are original P6 service results, not merely standalone L1 tests. In several
negative cases original recovery/degradation proceeds beyond the first
`NO_LEGAL_SEMANTIC_ROUTE` and then rejects at a downstream traffic gate; do not
claim that L1 always terminates the request immediately. Fixtures preserve
original result hashes and compact evidence under `test/evidence/`; all raw run
outputs remain in their distinct temporary directories. The eight semantic
CTest cases each retain the same isolated-domain and 60-second bound.

This expands synthetic regression coverage but does not qualify arbitrary semantic
input documents for public use or establish complete parking/traffic coverage.
The public static semantic API below accepts only the explicitly validated subset; the CLI remains map-only and the public persistent P6Session API has the bounded contract documented below.

### Explicitly unsupported research entrypoints

Activation rejects `StorageCropSession` at construction, before a delegate or
ROS worker can start. Its historical `native_snapshot.record` dependency is not
part of the portable API. No fake recorder is injected.

Historical benchmark entrypoints in `two_layer_v1_formal_benchmark`,
`l1_l3_corridor_hybrid_validity`, and `layered_architecture_paired_benchmark`
that access private map/report defaults are rejected with
`UnsupportedResearchFeature`. Use `plan_map`/`plan_semantic` with explicit paths.
The production `build_adaptive_corridor_mask` function is unchanged, including
its identity. Frozen source-file hashes are unchanged; only wrapper activation
installs these explicit unsupported-feature guards.


## Public static semantic API

`SemanticRequest` extends `MapOnlyRequest` with keyword-only absolute paths
`semantic_json`, `topology_json`, and `parking_access_json`.
`plan_semantic(request, solver_root=None)` returns the common path/result/audit/
ACK/cleanup schema with `mode="semantic"`, `schema="trilayer-p6-semantic-v1"`,
and `input_snapshot` hashes. It does not silently accept a map-only request.

Supported inputs are deliberately restricted to non-overlapping, nondegenerate
axis-aligned rectangular lane/junction/parking regions in map frame, at most 128
regions, topology edges on shared boundaries, lane-aligned one-way rules and
lane→junction→lane forbidden transitions. Parking bindings identify this query's
start/goal, an explicit parking/road ID, anchor and tolerance up to 0.25 m,
travel sides, and a boolean local-cross-road permission. Other rules, features,
unknown fields, ambiguous square lanes, duplicate IDs/keys, invalid references,
nonfinite coordinates, metadata/hash mismatch and malformed files are rejected
before output creation or ROS launch. Existing fixed vehicle/map limits apply.
Geometry/ownership refusals from valid inputs still use the original P6 algorithm.

The adapter retains source bytes, writes a private snapshot of the map YAML,
image and three JSON documents, adjusts only the snapshot's image reference,
and records both source and snapshot SHA256. Original files are never edited.
The snapshot is rechecked before returning success; changing an input snapshot
is a cleanup/integrity failure. A fresh output directory is mandatory. Errors
retain diagnostic artifacts rather than deleting user input. Validation tests
cover invalid inputs and prove later edits/deletion of originals cannot alter
retained snapshot bytes. All eight semantic integration cases now call this API.
A real SIGINT test after observing native nodes active verifies captured failure,
worker exit 0 and no surviving descendants, without mocking lifecycle behavior.

Always use a fresh interpreter per request, isolated localhost DDS and an external
wall-clock/process-tree limit. Continuous dynamic requests use the separately owned P6Session lifecycle below,
not a loop around these one-shot functions.


### Public path fail-closed rule

Only the top-level `success=true` response exposes an executable `path`. Any
runtime, snapshot-integrity, cleanup or worker-exit failure forces public `path=[]`,
even if planning had already produced a valid candidate. `plan_result` and nested
`result` are retained solely as diagnostic records; their candidate points or
nested success flags are **not executable authorization**. A regression test runs
real successful planning and real close, then injects a test-only cleanup error:
it verifies retained diagnostic points, public success=false/path empty, worker
exit 0 and no surviving child processes.


## Public persistent P6Session API

```python
from trilayer_p6_runtime.session import P6Session
with P6Session(request, solver_root=source_built_solver_directory) as session:
    first = session.plan()
    changed = session.update(1, upsert={"box": [[3.25,3.25],[3.75,3.25],
                                               [3.75,4.75],[3.25,4.75]]},
                             current_pose=[2.0,4.0,0.0])
    current = session.current_trajectory()
```

`request` is the same MapOnlyRequest or strict SemanticRequest used by one-shot
APIs. Shared RuntimeOwner initializes one original IncrementalService, world,
topology and native worker. Events do **not** restart the worker. APIs are
`open`, `plan(start=None, goal=None, query_id=None, seed=None)`,
`update(sequence, upsert=None, clear=(), observed_free=False, current_pose=None)`,
`current_trajectory`, `map_context`, and idempotent `close`. All concurrent
operations and calls after close are rejected. Reopen is unsupported. One
interpreter owns one generation; changed base snapshots require a new interpreter,
not a new cache binding hidden inside an existing interpreter. Context-manager
exit closes the owned worker and reports cleanup failure.

Upserts map object IDs to finite, in-bounds simple polygons. Sequence is an
integer (older values retain the original stale-sequence handling), clear IDs unique, observed_free strictly boolean. Malformed
input withdraws public trajectory exposure before raising. Valid events are sent
to original `IncrementalService.update`: same-sequence same-payload DUPLICATE,
stale-sequence status, conflicting same-sequence failure, and observed-free clear
rules retain original behavior. No adapter-generated ACK or sequence acceptance
is substituted. Failed events expose no executable path even when original state
retains diagnostic candidates. A new original successful plan/update is required
to restore exposure. Trajectory availability additionally requires original
current_trajectory and matching dynamic/trajectory version.

Map-only plan endpoints can change within the supported map. Persistent semantic
start/goal/query_id remain fixed to the validated input request, conservatively
avoiding unverified parking-binding rebinding; other endpoints need a fresh
validated generation. The existing per-event P6 request budget applies, but the
caller still owns wall-clock/process-tree supervision and an isolated DDS domain.
No controller or velocity command is issued.

`map_context()` hashes bottom-row-first 0/1 uint8 masks. `hard_mask_sha256` is an
alias of **static_hard_mask_sha256**, for immutable base-map binding.
`effective_hard_mask_sha256` covers the current static|dynamic mask. It changes
on accepted obstacle insertion and restores on observed clear; static hash does
not change. Neither mask includes Nav2-added inflation. Geometry/frame/version
fields accompany the hashes, and snapshot integrity is checked before exposure.

Persistent map and semantic tests exercise a real same-PID plan→upsert→duplicate→
stale→gap→observed clear→unobserved clear refusal→conflict refusal→replan sequence,
malformed polygon/sequence withdrawal, static/effective hashes, concurrent-operation
rejection, semantic endpoint override rejection and idempotent close. This is
bounded synthetic coverage, not proof of all P6 dynamic or real-robot behavior.

For a supervised persistent JSON worker, use serialized operations with unique
request IDs, one owner process per generation, explicit open inputs, and replies
carrying operation result, executable path, dynamic/trajectory versions and real
ACK evidence. Close/EOF must close the owner; outer supervision must clean the
whole process tree on timeout. Do not implement an undocumented reset or silently
reopen per event. A broker may persist across clearly reported cold generations.

### ACK portability patch (ARM execution still unvalidated)

`request_ack.cpp` now guards x86 AVX2 includes, definitions and dispatch at
compile time. Other architectures retain the original scalar implementation.
The original hash is preserved in SOURCE_PROVENANCE.json with a separate patched
hash and distributable patch. No Python ACK policy was changed: original
`request_ack.initialize` returns false without vector availability, selecting
the existing non-fused caller fallback, not a new fused default.

The portable scalar CTest compares exact flags, five counts, dirty bounding box
and equality for 128 combinations against NumPy (contiguous, column-strided,
negative-strided, row-strided; widths around vector boundaries; previous absent
and unchanged). Local amd64 testing also compared freshly compiled original
source: original/vector/scalar all matched. Forced scalar on amd64 is not ARM
compilation or an end-to-end ARM ACK result. See test/evidence/ACK_PORTABILITY.json.

## Independent pre-install testing

CTest creates a package-private `build/test_install` fixture with `cmake --install
--prefix`, leaving the configured production install prefix untouched. Tests
requiring installed resources/plugins depend on that fixture and resolve it
before the declared dependency underlay. An existing fixture directory is
replaced only when its ownership marker exists; an unowned directory fails.
This does not disable installed-layout tests or require an older installed copy
of this package. Optional solver-root integration tests retain their existing
enabling flags and use the same private stage when enabled.
