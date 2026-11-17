# TriLayer L3 base-profile optimizer and numerical core

Relocatable ROS 2 package retaining frozen cubic B-spline trust-region SQP,
Clarabel adapter and continuous curve certification. Includes source-built
native corridor rectangles and swept-rectangle collision checks. No ROS node,
robot control, stock Nav2 override, or hidden old-workspace environment sourcing.

```python
from trilayer_l3_core import optimize, certify
from trilayer_l3_core import basis, differential, footprint_constraints
from trilayer_l3_core import clarabel_setup, clarabel_solve
```

## Verified scope

The unchanged installed `config/l3.json` base path is runnable: synthetic 5 m
straight guide → corridor → spline optimization → Bernstein derivative bounds
→ enlarged continuous native rectangle sweep. Tested both the original certified
quadratic-start branch and actual one-iteration Clarabel/SQP. The certificate is
recomputed from returned coefficients and matches the returned path. A new
obstacle across the curve is rejected. A native sweep also rejects an obstacle
between two individually collision-free endpoints. No collision mocks are used.

**This is not the complete P6 L1/L2/L3 global planner or historical P6 profile.**
Environment-injected recovery/seed modes, tiled sweep and full semantic/traffic/
Nav2 ACK integration remain outside this package. It does not claim the previous
300-query results or a new full benchmark. Do not enable undocumented historical
configuration flags; missing strategy modules have not been silently substituted.
The caller supplies the actual boolean occupied grid, allowed-region mask, map
resolution/origin and vehicle half-length/half-width/minimum-turn-radius.

## Build and tests

Build with colcon and source the install. Config and native libraries resolve
through ament_index. See PROVENANCE.md and GEOMETRY_PROVENANCE.json for original
hashes and precise import/resource-location changes.

- `python3 test/test_numeric.py [--solver-deps PATH]`
- `python3 test/test_optimize.py [--solver-deps PATH]`

CTest registers both contracts. Optional CMake cache
`TRILAYER_L3_SOLVER_DEPS` explicitly supplies frozen historical solver libraries
for validation only. No wheel is copied into this package. Such tests are not
clean-OS dependency resolution or solver source-build proof. Numerical tests also
cover all spline derivative orders against SciPy and a finite-difference
curvature Jacobian. See DEPENDENCIES.md for solver release blockers.

The maintainer email remains a placeholder requiring approval before release.

## Independent pre-install testing

CTest creates a package-private `build/test_install` fixture with `cmake --install
--prefix`, leaving the configured production install prefix untouched. Tests
requiring installed resources/plugins depend on that fixture and resolve it
before the declared dependency underlay. An existing fixture directory is
replaced only when its ownership marker exists; an unowned directory fails.
This does not disable installed-layout tests or require an older installed copy
of this package. Optional solver-root integration tests retain their existing
enabling flags and use the same private stage when enabled.
