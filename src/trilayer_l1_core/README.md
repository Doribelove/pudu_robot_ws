# TriLayer L1 map-only search core

This package extracts four unchanged methods from the P6 snapshot's
`FreeSpaceTopology`: coarse-mask generation, sparse grid graph construction,
endpoint snapping, and grid guide search. Method text hashes are recorded and
checked in the test suite. The new `GridGuideSearch` wrapper validates inputs and
creates fresh per-request graph state.

```python
import numpy as np
from trilayer_l1_core import GridGuideSearch
free = np.ones((20, 30), dtype=bool)
result = GridGuideSearch().search(free, (1, 1), (18, 28))
```

The default algorithm is the original bounded weighted A* (weight 1.5), intended
as a soft L1 guide rather than an optimality claim. Optional `mode='dijkstra'`
uses the original SciPy full-component sparse graph branch; this mode is not
bounded by the A* expansion budget. Both prevent diagonal corner cutting. A
failed bounded search returns `cells=None`; it does not prove disconnection.
Inputs use row/column grid coordinates and True for traversable cells.

Tests use synthetic grids to check connectivity, disconnection, an alternative
channel after blocking the first route, repeated-map changes, diagonal safety,
budget exhaustion, and exact extracted-method hashes. Runtime dependencies are
NumPy and SciPy; no historic workspace, private maps, ROS nodes or binary helpers
are loaded.

## Scope

This is the map-only L1 search kernel, not the entire P6 L1 subsystem. It excludes
semantic traffic rules, parking access, route/task serialization, world-space
geometry, and the native goal-stop Dijkstra branch. L2 remains responsible for
final geometric planning. The manifest's placeholder maintainer address must be
replaced before ROS release.

## Semantic L1 milestone

`trilayer_l1_core.semantic.SemanticPlanner` now provides a portable input adapter
for the original V5 semantic topology constructor and default A* route generator.
Both full modules are preserved apart from explicit package-local import rewrites.
Inherited parking binding/direction checks, right-boundary construction, polygon
rasterization, and route dataclass functions are extracted without body changes.
The source hashes and import rewrites are in `SEMANTIC_PROVENANCE.json`.

Input is the original three-file graph/semantic/parking contract plus explicit
grid dimensions, origin, resolution, and optional hard mask. Output includes the
original route, serialized right-reference polyline task, and endpoint contexts.
The constructor genuinely rasterizes polygons and detects parking contacts.

Six additional synthetic tests run the real constructor/search: regular route
and task generation (A* and legacy), disconnected graph, one-way restrictions,
forbidden junction transitions, two alternative geometric corridors, parking
ownership/anchor/travel-side rules, and endpoints outside semantic regions.
No query or route result is mocked.

This supersedes the earlier map-only scope paragraph for this milestone. Full P6
L1/L2 integration, traffic trajectory auditing, graph extraction from arbitrary
maps, native goal-stop branches, and large-map performance remain separate work.
The wrapper supports zero-yaw maps up to one million cells and uses the inherited
environment-selected A* optimization switches (tests explicitly clear them).
See `THIRD_PARTY_SCOPE.md` for the inherited helper-license release gate.
