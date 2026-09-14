# Project source snapshot — 2026-09-14

This update delivers source code and regression tests; it is not production promotion or a new
claim of full-map/vehicle acceptance. Production 3D-V1 continues to resolve to r2-stable.

## Included

- 2A-V1-r3 bounded-memory preparation, endpoint/ROI handling, exact ACK, static planner service
  and related regression tests. The previously ignored Python modules referenced by tracked
  code are included explicitly so the remote checkout does not contain dangling imports.
- STL-V0-r2 independent research code, synthetic scenes, tests and text reports.
- [3YD-V1](../research/3yd_v1/README.md): latest completed lightweight semantic topology and
  0.15/0.05 m static reference-planning research snapshot; includes its 3YD-V0-r0 base modules.

## Not included or promoted

- Private maps, semantic annotations, real query data, actual-site figures, caches, binaries,
  videos, experiment trees and full run logs remain local. Historical report image references
  may point to local-only evidence; synthetic test scenes remain available.
- The ongoing `3yd_v1_static_recovery_20260914` work is not included and is not called accepted.
- No new real-map campaign, ROS/Nav2/Gazebo run, long soak or physical-vehicle test is implied.
- Earlier benchmark versions, private data and frozen baseline directories are not changed.

The delivery is prepared in a separate Git worktree so the original working directory and
other tasks' unfinished changes are preserved. Source provenance and current verification are
recorded in the accompanying delivery manifest and verification report.
