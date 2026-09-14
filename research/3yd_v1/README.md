# 3YD-V1 source delivery

This is the completed 2026-09-14 **static research snapshot**, not a new production default.
It includes the 3YD-V0-r0 semantic modules, the 3YD-V1 lightweight topology/multiresolution modules,
their frozen Python dependencies, the native continuous-footprint checker, and synthetic tests.
`COLCON_IGNORE` isolates it from the existing ROS workspace. Production still uses 3D-V1-r2-stable.

The implementation files are copied byte-for-byte from `3yd_multires_static_20260914`.
Only this delivery README and relocatable environment/build wrappers are new; the original
research description is retained as [UPSTREAM_README.md](UPSTREAM_README.md).

## Scope and latest completed result

- L1: lightweight directed semantic-interface topology, with downstream feasibility checks.
- L2: 0.15 m reference search, with recorded 0.05 m fallback when necessary.
- L3: original 0.05 m Smac, finite reference costs, exact content ACK and path audits.
- Same-run static comparison: 60/60 accepted in both baseline and V1; query P50
  4.711 s to 1.792 s. V1 uses fine L2 fallback in 18/60 attempts.
- These are historical reported results, **not new experiments run for this Git delivery**.
  See [performance report](report/performance.md) for timing and quality limitations.
- `3yd_v1_static_recovery_20260914` remains separate, ongoing work and is not included.

## Public/private boundary

This public snapshot contains **no private site maps, semantic annotations, real query poses,
real-site images, caches, binary libraries or full experiment logs**. Existing absolute local
paths in historical reports identify the original evidence, not downloadable repository assets.
Do not add such assets merely to make old report links resolve.

The historical benchmark tools are preserved, not advertised as data-free reproduction.
Real-map experiments need authorized local `config/acceptance.yaml`, `config/queries.yaml`,
maps and annotations, the original environment/provenance, and a separate output directory.
Some historical tools refer to the original workspace/cache paths; adapt those paths in a new
experiment and retain the original frozen source/hash records. Synthetic unit tests do not
require the private data or those configurations.

## Data-free checks

Prerequisites: Linux, g++, Python 3 development headers with NumPy, SciPy, OpenCV, Pillow, PyYAML, psutil, pytest,
and the existing ROS 2 Humble Python dependencies. No Nav2/Gazebo process is started by the tests.

```bash
cd research/3yd_v1
bash build_native.bash
source env.bash
/usr/bin/python3 -m pytest -q -p no:cacheprovider \
  external/arena4_ws/src/arena/three_d_v1/test/test_semantic_dual_map.py \
  external/arena4_ws/src/arena/three_d_v1/test/test_multires_static.py
```

To run ROS/Smac separately, source a compatible pinned ROS underlay before `env.bash`, or set
`PUDU_ROS_UNDERLAY` to its install directory. This Git update does not alter pinned Nav2 or
change the production factory, vehicle limits or any historical experiment.
