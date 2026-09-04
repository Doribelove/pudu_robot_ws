# PLN-02 2D-V3 r0 final qualification report

## Decision

- Architecture: `2D-V3`
- Implementation: `r0-hybrid-tail-bounded-v1`
- Parent: `2D-V2-r0`
- Final predefined verdict: **C — reject promotion**.
- Production recommendation: keep deterministic Graph A* on the 2D L1 path. Keep D* Lite research on the 3D-V0 L2 grid layer, where graph size and locality may amortize persistent state more effectively.

The candidate is correct, but it misses both the held-out dynamic latency gates and the static P95 gate. Dynamic ROS Stage 6 was therefore not started.

## Workload provenance

No real cleaning trace aligned to `mentor_map_20260825_005` or `mentor_map_20260825_005_4x_area` exists in the audited repository, rosbag, log, or scenario inputs. Generic Arena pedestrian and factory actor scenarios use different maps and lack a compatible cleaning trace, so they were rejected as source data.

The experiment uses an explicitly labelled `realistic_synthetic_cleaning_workload`. It covers single and multiple people crossing, crowds, corridor motion, slow carts, doorway persistence, simultaneous channel changes, transient obstacles, jitter/false positives, disappearance/recovery, bridge/min-cut no-route, many off-path obstacles, and few/many on-path obstacles. At 2 Hz, two-observation block/recovery confirmation adds 0.5 s of deterministic confirmation delay before planning time; no extra debounce delay was enabled.

Across held-out dynamic snapshots, changed-edge count has P25/P50/P75/P95/P99/max of 1/1/10/100/100/100. The corresponding ratio P25/P50/P75/P95/P99/max is 0.0219%/0.0219%/0.2192%/2.192%/2.192%/2.192% of the 4,562 mapped topology edges.

## Frozen selector

Calibration considered 0.5, 1, 2, 4, 6, and 8 ms wall budgets plus OPEN-pop, `update_vertex`, OPEN-size, and inconsistent-state caps. No candidate passed all calibration gates. The predeclared fallback selection rule chose the best P95/P99 candidate:

- wall: 0.5 ms;
- OPEN pops: 64;
- `update_vertex`: 1,024;
- OPEN size: 4,096;
- inconsistent states: 2,048;
- changed edges: at most 2;
- changed-edge ratio: at most 0.092%;
- current-route intersections: at most 1;
- post-fallback cooldown: 1 update.

Exact Tarjan bridges are a direct Graph A* risk feature. Corridor relevance in pure L1 uses a conservative one-hop topology support set; the V2 raster mask would replace it in ROS. Large changes, bridge risk, multiple route intersections, unavailable D* state, recent fallback, or any runtime budget exhaustion select Graph A*. An unconverged D* route is never extracted. After fallback, the current response uses A* and low-priority D* resync is separately charged as CPU/accounted compute.

## 4x map audit

The map is 6,574 × 3,024 = 19,879,776 cells. It has 9,703,692 raw-free cells (24,259.23 m²), 6,544,617 topology-traversable cells (16,361.54 m²), 4,376 nodes, 4,562 edges, and 157 connected components. Route-edge P50/P95 is 84/142.2. Relative to the base map, total/raw-free area is 4.00×, topology nodes/edges are 2.07×/2.10×, and traversable area is 6.22×.

## Held-out pure-L1 result

All 17,640 arm/snapshot rows pass the fresh Graph A* oracle: reachability and failure code match, maximum path-cost error is zero, no output contains a `BLOCKED` or `RECOVERING` edge, no snapshot/status hash mismatches occur, and no unconverged D* result is returned. No-route and recovery parity is 100%.

| Arm | Response L1 P50 | P95 | P99 | Expanded P50 | P95 | P99 |
|---|---:|---:|---:|---:|---:|---:|
| cold Graph A* | 7.753 ms | 22.229 ms | 22.738 ms | 3,022 | 3,961 | 4,034 |
| pure persistent D* | 7.333 ms | 50.800 ms | 68.556 ms | 424 | 5,138 | 5,591 |
| V3 hybrid | 7.772 ms | 23.886 ms | 24.476 ms | 2,957 | 3,961 | 4,034 |

Hybrid minus Graph A* is +0.018/+1.658/+1.737 ms at P50/P95/P99. Thus P50 is 0.24% slower instead of at least 10% faster; P95 is 7.46% slower (limit 5%); P99 is 7.64% slower (limit 10%, pass in isolation). Stage 4 fails.

Pure D* shows the intended median node reduction, but its queue tail dominates: P95 queue pops/pushes/stale entries are 7,646/7,642/2,541 and P95 `update_vertex` calls are 13,823. Search P50/P95/P99 is 3.641/44.509/49.891 ms. This is an algorithm/implementation tail, not ROI, Smac, or PathAudit.

S0 first-plan P50/P95/P99 is 6.968/9.297/9.744 ms for Graph A*, 30.056/121.313/180.672 ms for pure D*, and 30.384/149.559/191.675 ms for hybrid. No-route hybrid response is 1.317/9.383/10.063 ms; recovery is 7.796/8.758/9.106 ms, with full oracle parity.

## Selector outcomes and accounting

Of 5,600 held-out dynamic hybrid rows, 1,260 (22.5%) are scheduler skips. These are system scheduling gains and are not attributed to D*. There are 4,340 real L1 calls:

- bounded D* selected: 1,320 (30.4%);
- bounded D* converged within budget: 400 (30.3% of attempts; 9.2% of L1 calls);
- D* budget fallback: 920 (69.7% of attempts);
- direct/fallback Graph A*: 3,940 (90.8% of L1 calls);
- resync: 3,940 (90.8% of L1 calls).

Resync consumes 146.77 s CPU across the formal held-out run, 37.25 ms per resync. Including resync, hybrid accounted-compute P50/P95/P99 is 37.719/143.302/194.742 ms. The fallback bounds response latency but does not produce an acceptable total engineering cost.

## Break-even

Absolute workload points:

| Changed edges | Pure D*/A* wall ratio | Hybrid/A* wall ratio | D*/A* expanded ratio |
|---:|---:|---:|---:|
| 1 | 0.940 | 0.997 | 0.182 |
| 2 | 0.718 | 1.010 | 0.088 |
| 5 | 0.912 | 1.016 | 0.142 |
| 20 | 2.177 | 1.034 | 0.652 |
| 100 | 1.696 | 1.073 | 0.674 |

Ratio-matched supplemental points:

| Nominal ratio | Edges | Pure D*/A* wall ratio | Hybrid/A* wall ratio | D*/A* expanded ratio |
|---:|---:|---:|---:|---:|
| 0.046% | 2 | 0.508 | 1.029 | 0.034 |
| 0.092% | 4 | 0.613 | 1.010 | 0.032 |
| 0.230% | 11 | 0.647 | 1.019 | 0.071 |
| 0.921% | 42 | 1.899 | 1.045 | 0.572 |
| 4.604% | 210 | 1.642 | 1.079 | 0.851 |

Pure D* crosses from faster to slower between 11 and 42 edges (0.230%–0.921%). Hybrid is not faster at any ratio-matched point once selection/fallback overhead is included.

## Static regression

The fresh 20-query, 3-warmup + 5-measured run returns 95/100 Final-valid. A2B-07 is 5/5 valid; A2B-16 remains five truthful `L1_NO_ROUTE` failures; A2B-19 is 5/5 valid with online P50 620.982 ms and Smac reported planning P50 422.072 ms. Smac expanded/generated state counters are unavailable in this runtime and are not estimated.

Success online P50/P95/P99 is 290.790/532.801/636.825 ms. P50 passes the 331.827 ms limit; P95 fails the 487.706 ms limit. ACK is 95/95 with zero mismatch, repair, or full fallback. All accepted paths pass footprint/kinematic/no-reverse/no-in-place-rotation/curvature checks. Normal settle and costmap clear are zero, each routed request calls L3 once, and canonical PathAudit is reused.

## Soak, memory, and cache

The final 20-cycle soak contains 17,640 correct arm/snapshot rows. It records 1,260 scheduler skips, including 200 stable no-route skips, and no implicit reset, unsafe path, or recovery failure. Route-hash churn totals 2,020 transitions over 280 hybrid episodes (P50/max 10/15 per episode).

Pure D* state memory P50/P95/max is about 1.339/1.422/1.431 MB in held-out runs. The final soak harness peaks near 964 MB RSS, but that includes three independent 4x cell indexes and the oracle harness; it is not algorithm state.

The V3 code provides compressed per-edge/turn/endpoint ROI primitives plus a configurable 128 MiB route-level LRU, binding validation, invalidation, and eviction accounting. Equivalence/invalidation/eviction behavior is unit-tested. Dynamic ROS was forbidden by the failed Stage-4 gate, so the route-mask LRU was not materialized in a formal ROS run. The unchanged static V2 cache used by Stage 5 is 94,428,936 bytes with 29.040 s cold build; compressed edge primitives are 2,933,676 bytes. No dynamic-cache saving is claimed as measured.

Cleaning controller task time, coverage, repeat coverage, controller avoidance triggers, and path-invalid exposure are `not_available`: the environment contains neither compatible real cleaning data nor an executable cleaning-controller replay. Planner-side route churn, calls, no-route/recovery, CPU, and memory are the only reported proxies.

## Formal artifacts

- Calibration: `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v3_calibration_4x_area_r0_20260903_183307`
- Held-out Stage 4: `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v3_dynamic_4x_area_r0_20260903_183611`
- Static Stage 5: `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v3_static_mentor_map_005_r0_20260903_174043`
- Final soak: `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v3_cleaning_replay_r0_20260903_184825`
- Ratio-matched supplement: `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v3_ratio_break_even_4x_r0_20260903_190301`

Each formal directory contains its reproduction command, source snapshot and hash manifest. The five pre-existing frozen baseline directory hashes match their task-start values after all runs.

## Qualification commands

```bash
PYTHONPATH=external/arena4_ws/src/arena/evaluation/arena_evaluation python3 -m pytest -q \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_2d_v3_dynamic_benchmark.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_dynamic_incremental_value.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_layered_2d_v0_pipeline.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_2d_v1_dynamic_incremental_benchmark.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_2d_v1_4x_dynamic_incremental_benchmark.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_layered_2d_v2_pipeline.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_2d_v2_roi_ack_dynamic.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_2d_v2_pathaudit_parity.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_2d_v2_benchmark.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_v1_r2_roi_pathaudit.py

python3 -m compileall -q \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test

source /opt/ros/humble/setup.bash
cd /home/robot/pudu_robot_ws/external/arena4_ws
colcon build --packages-select arena_evaluation --symlink-install

cd /home/robot/pudu_robot_ws
git diff --check
```

Final results: 65 tests pass, compileall passes, `arena_evaluation` builds, both new CLI help commands pass, and `git diff --check` passes. No V3 task process remains after qualification.
