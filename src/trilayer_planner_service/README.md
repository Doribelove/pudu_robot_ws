# TriLayer persistent local planner service

Experimental localhost-only P6 session service; no robot controller or velocity
commands. This is not yet an apt release. A host Nav2 planner plugin supplies a
complete, explicitly bound costmap snapshot for each plan.

```
ros2 run trilayer_planner_service serve --output /tmp/trilayer-worker-unique \
  --domain-id 186 --solver-root /absolute/installed/solver/path
```

Reserve a **different**, unused child DDS domain from the caller's planner_server.
The output must not exist and must have a short enough path for Unix sockets.
The private directory (0700) contains `service.sock` (0600). No TCP listener.
Defaults: request deadline 55s, whole service lifetime 300s; hard maxima 60s/3600s.
A supervisor tracks PID/create-time descendants, including native processes in
separate sessions, and cleans them after timeout or shutdown. All artifacts are
new files under the explicit output. The supervisor must remain running.

For explicitly user-managed long-running operation, pass `--lifetime 0`.
This disables only the overall lifetime, **not** the per-request hard deadline.
Default behavior remains bounded to 300 seconds; positive limits remain capped
at 3600 seconds. Persistent mode waits while idle, exits on close/SIGTERM, and
never implicitly restarts a failed or timed-out owner. SIGTERM requests graceful
cleanup; a stuck owner is killed within the supervisor cleanup bounds. Caller
failures must be surfaced, not retried against state whose completion is unknown.

A user-started ROS launch recipe is provided (no controller nodes, no respawn):

```
ros2 launch trilayer_planner_service persistent_service.launch.py \
  output:=/tmp/tri-user-session-new domain_id:=186 lifetime:=0 \
  solver_root:=/absolute/installed/solver/path
```

Launch owns the process and shutdown; this is process-lifecycle supervision,
not a ROS LifecycleNode or automatic Nav2 lifecycle-manager participant. Start
it before configuring the plugin, wait for `ready.json`, and supply that socket
to the plugin. On shutdown deactivate the caller plugin first, then shut down
launch. A failed broker requires explicit restart with a **new** output directory.
Before each new plan/update request, `shutil.disk_usage(output)` checks only that
filesystem. `--min-free-mib` (launch `min_free_mib`) defaults to 512 MiB and may
be raised, not disabled or reduced. Below that floor the worker returns
`DISK_SPACE_LOW`, accepts no new work, closes its owner, and exits nonzero after
cleanup. Status and close are not blocked by the free-space guard. No files are deleted and the request watchdog stays enabled. This is a
preflight floor, not a reservation or maximum request size: concurrent writers
can still exhaust storage. Error/cleanup records are best-effort if space is
already exhausted; cleanup is attempted even if those writes fail.

All request/native audit files remain in that output, with no automatic deletion
or rotation: long-running users must monitor disk use and retain/archive evidence.
Persistent mode is verified with short idle, planning, request-timeout and signal
shutdown tests; no multi-day stability or unattended production readiness claim.

## Wire v1

One AF_UNIX stream connection per request; one JSON object plus newline. Maximum
request 4 MiB, response 8 MiB. Duplicate keys, unknown fields, nonfinite/bool
coordinates, invalid IDs/frames and unsupported operations are rejected. `id`
is 1..64 safe ASCII characters (letters, digits, underscore, hyphen, single dots; starts alphanumeric; no double dots), `v` is integer 1. `plan` requires `start`/`goal` each
[x,y,yaw], `snapshot` standard base64 of full uint8 cost cells in bottom-row-first
order, and `context` containing exactly:

- frame_id: `map`; width/height; resolution: 0.05; origin: [x,y]
- costmap_sha256: SHA256 of full cost bytes
- hard_mask_sha256: SHA256 of bytes 0/1 produced by cost >= 253 (unknown included)

The worker validates both hashes then conservatively projects hard cells into a
trinary PGM, flips row order, and starts original P6 on that map. Soft host costs
are **not** reproduced; only hard occupancy is preserved. The plugin must reject
rolling maps, incompatible footprint, and any changed host snapshot before
accepting a response. This is not shared-host-costmap optimization equivalence.

Same context/snapshot reuses the same owner interpreter/native P6 Session.
Changed snapshot closes and verifies prior cleanup, then starts a new owner
interpreter with a new generation/output. This is explicit **cold rebuild**, not
incremental obstacle inference or a claimed dynamic reset. Original activation
binds global state per interpreter, hence generation isolation is required.

Responses include v/id, ok, path, frame_id/context, audit_valid, dynamic and
trajectory versions, generation/reuse and native PID. Failure never returns a
stale path. Native results/audits are stored as request JSON. `status` gives no
path. `close` closes the session and listener. `reset` is unsupported.

`update` is a separate experimental original-P6 operation: sequence/upsert/clear/
observed_free/current_pose with the currently bound context. It is not automatic
conversion of host costmap changes; do not mix externally updated host snapshots
and this explicit overlay API without managing a new cold generation. It follows
the original service sequence/free-observation rules. Any integer sequence,
including negative stale values, is forwarded unchanged; booleans are rejected.
Upserts/clears have at most 64 safe-ASCII IDs (64 characters each), polygons have
at most 128 vertices, and numeric int/float identity is preserved for original
same-sequence payload hashing. Disconnect does not imply
instant cancellation: current work is bounded, response discarded if disconnected.
The Humble action interface has no immediate cancellation callback to this worker.
