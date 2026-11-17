# TriLayer Global Planner

Source packages for ROS 2 Humble on Ubuntu 22.04. This is a release-preparation branch, **not an available ROS apt release**. Native Ubuntu 22.04 amd64 and arm64 source builds and synthetic tests have passed independent GitHub Actions runs. This does not establish board-specific or real-robot validation.

TriLayer preserves the original P6 three-layer planning pipeline: semantic/topological L1 guidance, native L2 planning, and L3 optimize–certify with independent continuous trajectory auditing. It does not replace failed P6 plans with a different algorithm or declare failed paths executable.

## Packages and interfaces

- `trilayer_global_planner`: installation entry and bounded map-only planning CLI.
- `trilayer_nav2_planner`: actual `nav2_core::GlobalPlanner` plugin for stock Humble `planner_server` / `ComputePathToPose`.
- `trilayer_planner_service`: private Unix-socket broker retaining real P6 sessions; identical caller snapshots reuse a worker, changed snapshots cold-rebuild in a fresh interpreter.
- `trilayer_p6_runtime`: original algorithm source closure and `plan_map`, validated `plan_semantic`, and persistent `P6Session` APIs.
- `trilayer_l1_core`, `trilayer_l3_core`, `trilayer_native_helpers`: independently testable semantic/topology, optimization/certification and native helpers.
- `trilayer_master_snapshot`, `trilayer_delta_codec`, `trilayer_costmap_bridge`: locked master snapshots, original delta encoding, and a separate validated topic cache. Topic caches are not master synchronization barriers.
- `trilayer_osqp_vendor`, `trilayer_clarabel_vendor`: pinned offline source builds of OSQP/QDLDL/Clarabel. No historical binary wheels are shipped.

The entry package pulls its planning dependency closure, not necessarily every optional package above. See package READMEs, dependency declarations and license inventories.

## Build and verify

Install stock Humble and the dependencies declared in the package manifests. Four rosdep rules currently need upstream acceptance: `python3-pybind11`, `python3-setuptools-scm`, `rustc-1.85`, `cargo-1.85`. `release/rosdep/` contains proposals; `release/ci/rosdep-ci.yaml` is only a temporary disposable-CI override, not an official rule source.

The reproducible build/test procedure is `release/ci/build-and-test.sh`. It builds both solver vendors from source, then the remaining packages, configures real solver-backed tests and runs them sequentially. It expects official Jammy Rust 1.85 and all manifest dependencies already installed. The dependency installer in the same directory deliberately only runs on disposable GitHub Actions runners; do not bypass that guard on a workstation.

A fresh isolated local amd64 build and the subsequent persistent-service regression each passed 42 CTests across 12 packages; the latest regression with pre-install fixtures and exact scalar/vector ACK comparisons passes all 47 CTests. These are synthetic source-level and real ROS process/action tests, not official buildfarm acceptance or board/robot-safety certification. Raw machine-local logs are intentionally not part of this public source tree. CI will publish its own run evidence when executed.

## Nav2 bringup

See [Nav2 integration instructions](src/trilayer_nav2_planner/README.md) and [service ownership and limits](src/trilayer_planner_service/README.md). The plugin never starts robot drivers or a controller. It snapshots its caller's actual master costmap and rejects a successful worker result if the caller map or footprint changed in flight.

The adapter currently supports map-only planning on nonrolling 0.05 m grids in frame `map`, with the documented fixed rectangular P6 body. Costs >=253, including unknown255, are hard obstacles; lower costs are integrity-bound but not used as soft optimization penalties. Semantic inputs and genuine incremental obstacle events are exposed through the Python API, not inferred from Nav2 costmap differences. Endpoint tolerances retain original P6 limits; no unverified connector segment is synthesized.

The broker defaults to 300 seconds. Explicit `--lifetime 0` or its installed persistent launch enables user-owned indefinite operation, while each request remains bounded. A minimum 512 MiB output-filesystem free-space guard fails closed; audit logs are never automatically deleted. Humble's synchronous planner interface has no immediate cancellation callback. Shutdown/timeout tests do not establish long-duration endurance.

## Provenance, license and release status

The source baseline is `Doribelove/pudu_robot_ws` main `a98a639a8ac9b7b50c14f8a69c393706db4cc3ae`; frozen algorithm provenance and exact source changes are documented in individual packages. Public synthetic fixtures replace no historical private benchmark dataset. Unsupported research-only loaders remain explicitly guarded.

Original additions use the licenses declared by each package. Preserve all bundled third-party notices, including the SciPy-adapted BSD heap and the mixed OSQP/QDLDL/Clarabel source inventories. Do not interpret a wrapper's Apache-2.0 license as relicensing every dependency.

Remaining release gates include official rosdep resolution, clean package/deb installation, official build-target scheduling/exclusion decisions, a dedicated Bloom release repository, rosdistro review, official jobs and apt synchronization. A source branch or green CI alone does not satisfy those gates.

Maintainer: Doribelove <doribelove@gmail.com>.

## Native architecture evidence

Independent [amd64 CI](https://github.com/Doribelove/pudu_robot_ws/actions/runs/44068616506) and [arm64 CI](https://github.com/Doribelove/pudu_robot_ws/actions/runs/44069917303) each built all 12 packages and passed 47 CTests from fresh native runners. Exact commits and scope are recorded in `release/ci/ARCHITECTURE_VALIDATION.json`. The consolidated workflow now tests both architectures for future changes; use the latest Actions result for the current revision. ARM64 here does not mean Jetson, Horizon J6M/BPU, or physical-robot validation.
