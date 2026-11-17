# Bounded single-session dynamic integration contract

`test_dynamic_session.py` runs the real frozen P6 `IncrementalService` and one
`CurveSession`/native Nav2 process, using a generated map and reserved localhost
DDS domain 184. The supervisor allows 60 seconds plus bounded cleanup; CTest
allows 70 seconds and serializes access. Run only when domain 184 is unused.
This is a self-contained runtime integration test with its own synthetic map
and local test-only subprocess supervisor; it does not import the entry package.
`dynamic_test_process.py` is a snapshot of the audited entry supervisor, kept
local to avoid a reverse dependency. It tracks descendants by PID/create-time
and cleans separate-session children without broad process killing.
It is not a public dynamic API, a controller, or a complete P6 equivalence test.

Original contracts:
- `base/src/three_yd_v13/service.py::_p4_guarded_update` accepts sequence,
  upsert, clear, observed_free and current_pose, then performs L2/L3 and
  certification in the existing session. Another process per update is not
  equivalent.
- `base/runtime/base/src/three_yd_v3/dynamic.py::apply` rejects stale sequence;
  identical same-sequence payload is a duplicate; conflicting same-sequence
  payload raises. Sequence gaps are permitted. Missing observations do not
  clear obstacles: there is no TTL or stream-timeout clearing contract.
- `base/runtime/base/src/three_yd_v5/task_progress.py::update` withdraws the
  executable trajectory on exceptions, including unobserved clear rejection.
  Valid no-change responses can retain the previous certified trajectory and
  do not require new ACKs. Clear with a controller initially built under a
  dynamic obstacle may trigger cold restoration; this test's particular
  baseline-built controller exercises incremental restoration instead.

Assertions cover a straight initial route, a real rectangle intersecting it,
341 changed cells and certified detour, duplicate/stale/gapped sequence,
explicit observed-free restoration, and rejected clear/conflicting sequence.
Each changed successful route requires new ACK evidence, zero mismatch,
exact-effective-master classification, two stable observations, a valid native
continuous swept-body independent audit, and matching dynamic/trajectory
versions. The center-point polygon check is additional evidence, not a
replacement for the continuous swept-body audit. Session identity and native
PID must remain the same across all stages.

Raw events retain full results, trajectories, dynamic/trajectory versions and
ACK publication sequences, hashes, observation counts and server timestamps.
The compact evidence file records raw-evidence and test hashes. No real robot
hardware, command-velocity topic, private map, TTL or invented reset is used.

CTest passes the explicit source-built solver root. Before P6 imports, the worker
prepends it and verifies OSQP, QDLDL and Clarabel module origins are all below
that root. The exact origins are retained in result and compact evidence.
