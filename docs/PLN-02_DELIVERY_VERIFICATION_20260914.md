# Source-delivery verification — 2026-09-14

This records checks run on the isolated delivery checkout, not a new formal navigation
experiment or production promotion. Production defaults remain unchanged.

## Tests

| Suite | Passed |
| --- | ---: |
| 2A-V1-r3 affected service, cache, endpoint and ACK tests | 320 |
| Frozen 3YD semantic/multiresolution and r1 lifecycle tests | 47 |
| STL synthetic regression | 26 |
| Existing production 3D-V1 test directory | 56 |
| Total test executions across the four suites | 449 |

The production and research snapshots contain related baseline tests; these counts describe
test executions, not 449 distinct navigation scenarios. No formal map runs were repeated.

## Build and checks

- The root arena_evaluation endpoint, OMPL and effective-costmap native extensions built.
- The research sweep, effective-costmap and endpoint extensions built using the supplied wrapper.
- The static planner service and research static benchmark CLI help commands passed.
- Delivered Python files compiled in memory; delivery shell wrappers passed `bash -n`.
- Missing research native dependencies were included as exact source snapshots and rebuilt.
- The r3 test dependency scikit-image was installed only in an isolated temporary virtualenv;
  no system Python packages were changed.
- Staged-content checks found no excluded private map/query/image/cache/binary/log payloads
  and no high-confidence secret-pattern matches. Pattern scanning is not proof of all-secret absence.
- Whitespace checks passed for modified project and newly authored delivery files. A global
  diff check still reports historical whitespace in byte-exact frozen research dependencies;
  these files were not reformatted to preserve source hashes.
- Production 3D-V1 source/config defaults and the root `.gitignore` were not changed.
- No ROS/Nav2/Gazebo processes or formal experiments were launched by this delivery task.

Source hashes and the scope exclusions are recorded in
[the delivery manifest](PLN-02_DELIVERY_20260914.json). Dataset-dependent scripts require the
privately held inputs described in [the research README](../research/3yd_v1/README.md).
The original dirty worktree and ongoing recovery research were left untouched.
