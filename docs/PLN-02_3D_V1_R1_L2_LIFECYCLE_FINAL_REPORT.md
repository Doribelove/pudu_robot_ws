# PLN-02 3D-V1-r1 L2 状态生命周期与动态负载工程化验证最终报告

## 1. 最终判定

本轮结论为 **B：生命周期优化有效，但证据不足，不晋升为生产基线**。

3D-V1-r1 在冻结 held-out 上完成 12 条查询、每条 10 次、三臂严格配对验证：正确性 3,600/3,600，canonical cost error 为 0，BLOCKED/RECOVERING 入路为 0，partial D* 为 0，hidden reinitialize 为 0；相对 cold grid A*，P50 改善 55.99%，P95/P99 比值为 1.005/1.002；warm activation P95 为 152.81 ms。10,000-snapshot soak 也通过，无 oracle mismatch、partial result、blocked path、timeout 或本轮残留子进程。

但冻结 held-out 的最大单 active state resident 为 **35,335,905 B**，超过冻结目标 **35,000,000 B** 共 335,905 B（0.96%）。虽然相对冻结 57,974,008 B 基线仍下降 39.05%，满足至少下降 30% 的条件，但本轮在 held-out 前已把 35 MB 目标纳入 Stage-A hard gate，因此不得看过 held-out 后调整。Stage B 按协议记录为 `NOT_RUN_L2_GATE_FAILED`，没有启动新的 ROS/Nav2/Smac 进程。

因此：保留 r0 的 deterministic grid A* fallback 主导安全链路；r1 compact lifecycle 可作为下一轮父实现，但当前不得宣称总体端到端收益或生产晋升。

## 2. 版本、审计与冻结基线

- `architecture_id`: `3D-V1`
- `revision_id`: `r1-l2-state-lifecycle-soak`
- `protocol_id`: `PLN-02-3D-V1-R1-L2-LIFECYCLE-V1`
- 工作区：`/home/robot/pudu_robot_ws`
- branch：`codex/2a-v1-r2-roi-pathaudit-delivery`
- HEAD：`337fed6f9d3b9e27f16e87c21ed3557e6a14834a`
- Python：`/usr/bin/python3 3.10.12`；pyenv `3.10.12`
- ROS：Humble；RMW `rmw_fastrtps_cpp`
- 地图：`mentor_map_20260825_005_4x_area`，`3024 × 6574`，`0.05 m/cell`
- map hash：`7226bba2392bd0986adce55b06974174e8952d84196ec7c6122237d0e08385f6`
- map YAML hash：`e0355183ca126ab7b1f7172d667aa157b49f84f12c7eb77d5d8ebe87b4fc0849`
- query JSON hash：`c9126b7cec978f64843e1a04bb873eb07004c2ef067afd15cb148e2fd59da1c0`

完整 Stage-0 记录位于：

`experiments/layered_planner_benchmark/3d_v1_r1_stage0_audit_20260904_120828`

本轮未执行 reset/checkout，未修改 `.gitignore`，未 stage、commit 或 push。开始时已经存在的 dirty changes 均保留。`three_d_v1` 位于被 `.gitignore` 的 `external/*` 下，普通 `git status` 看不到本轮新增源码，属于明确交付风险；本轮未经授权未改变 ignore 规则。

冻结历史证据在结束时重新计算，未发生变化：

| 历史基线 | 结束时 tree/file hash |
|---|---|
| r0 设计实现报告 | `253f0d065db886bc9beb8cc54b5500bcb7cc7da1a0b7c4eba7be156ebc211c50` |
| r0 source bundle（12 files） | `8f189bbb6361b6604898f80c76ebc9594b1a909cc8e89db2b27d4873e4ee15f5` |
| r0 selective preflight | `98d3b5beb4899c988daf98b32ae3542a7b95a4a17b4c70770985519f765bba42` |
| r0 real 4x Stage A | `1328df374b6825416af907124f44f578b1331d91230a024b84ef4b8c3266750f` |
| r0 Stage B smoke | `4ff9839249c856cea9ae3523e299d3bb9acfddd0dfb4e6ac7a2c352e43281e05` |

误发任务留下的 `3d_v1_r1_r0_profile_20260904_01` 被标为非权威诊断参考并排除；本报告不依赖其源码或结果。

## 3. Stage 1：r0 profile

权威 profile：

`experiments/layered_planner_benchmark/3d_v1_r1_r0_profile_20260904_02`

A2B-07 的 cropped full-resolution L2 ROI 为 `2196 × 441 = 968,436` cells，其中 statically safe state 为 195,826。首次 D* 求解 expanded 176,252，update-vertex/predecessor visits 为 1,410,016。

主要内存来源：

- Python `g/rhs/open` 对象估算约 50.2 MB；
- NumPy/索引数组约 10.7 MB；
- r0 报告的 state memory 为 57,974,008 B；
- profile 进程 RSS 增量约 94.8 MB。

instrumented cold profile 为 35.84 s，但它包含 `tracemalloc` 和逐调用 hook，不作为时延校准。时延基线仍采用 r0 正式数据中的约 12.4–12.9 s cold build、eligible D* P50 268.038 ms。

profile 结论是：首建和内存问题来自 Python per-cell 状态、稠密 bbox 索引与重复邻接工作，而不是“D* 必须成为唯一算法”。pure D* synthetic P95 约 1.3 s 的既有反例保持有效，本轮未把 pure D* 作为晋升臂。

## 4. r1 实现

### 4.1 Compact corridor geometry

只对静态 footprint-safe corridor cells 分配 compact state ID；动态障碍写入 bool overlay，不修改静态占用。

最终 v2 格式保存：

- row-major `int32 state_cells_linear`，可逆映射到 ROI cell；
- `int16 neighbor_deltas[N,8]`，离线构建并验证目标范围；
- 对角边只有在两个静态侧边均可通行时存在；运行时再检查动态侧边 blocked bit；
- 不保存 `ROI bbox × int32` 的稠密 cell→state 网格。

该改动将最大 calibration ROI 的 resident 从 67.6 MB 降到约 27.1 MB。cost、g、rhs 均保持 float64；没有使用会改变代价或 tie-break 的 float32/量化。

### 4.2 Immutable geometry 与 mutable D* state 分层

geometry key 绑定：map hash/shape/origin/resolution、topology hash、route edges、corridor hash、footprint hash、安全策略、邻接规则和格式版本。

mutable state key 额外绑定：geometry hash、start/goal endpoints、dynamic baseline version、algorithm version 和格式版本。

geometry/state cache 均具有 schema、manifest、binding hash、array/content hash、atomic temporary write + `fsync` + `replace`。key/hash/schema/shape 任一不符均 fail closed。动态 dirty state 不会以 empty baseline key 保存。

held-out 的 12 路线 cache 实际磁盘占用：geometry 6,164,204 B，mutable state 13,254,654 B；单 payload 范围分别为 156,098–924,033 B、196,377–2,119,490 B。

### 4.3 有界 LRU 与 telemetry

默认最多 1 个 active mutable state，hard max 为 2。逐出释放 planner/geometry/state 引用并执行 GC；不会让所有 query state 常驻。telemetry 包含 cache hit/miss/reject reason、build/serialize/restore/activate/evict ms、resident、RSS 和 active count。

10,000-snapshot soak 中完成 20 次路线 activation、21 次 eviction，peak active count 为 1；最后 clear 后 active=0、resident=0。

### 4.4 r0 契约保持

实现保持：

- 两次观测动态确认与版本/时间/地图绑定；
- relevance scheduler safe skip；
- deterministic Graph A* L1 与自适应 2/4 m corridor；
- cropped 0.05 m ROI；
- corner-safe 8 邻接；
- D* timeout/invalid extraction 不返回 partial，改走原 deterministic grid A*；
- large change、recovery、not-ready 强制 grid A*；
- recovery 永不 skip；
- exact old/new dirty ROI union 与 server content ACK；
- 48-bin Smac Hybrid DUBIN、no reverse/no in-place rotation contract；
- canonical PathAudit 单实例复用。

## 5. Stage 3：测试与交付验证

新增测试覆盖：

- 8 seeds 的 compact graph 与 grid oracle reachability/cost/path-cell exact parity；
- static/dynamic diagonal corner cutting 禁止；
- no-route→recovery；
- D* timeout/invalid extraction 无 partial 且 fallback parity；
- geometry/state key 的全部 binding 字段敏感；
- 截断、损坏、schema mismatch 和 binding mismatch cache 拒绝；
- LRU admission/hit/eviction、weakref release 与 hard bound；
- route/endpoint/corridor state 不串；
- dynamic confirmation、scheduler、ROI/ACK、48-bin runtime contract 与 canonical PathAudit。

最终验证：

- `/usr/bin/python3 -m pytest ...`：**30 passed**；
- `/usr/bin/python3 -m compileall ...`：通过；
- `r1_profile`、`r1_stage_a`、`r1_soak`、`r1_stage_b_gate` 四个新 CLI `--help`：通过；
- `colcon build --packages-select arena_3d_v1 --symlink-install`：1 package finished；
- root repo、evaluation nested repo 和 ignored r1 files 的 whitespace/diff check：通过。

cache corruption/schema/binding mismatch 测试 6/6 拒绝。

## 6. Calibration 与冻结

权威 calibration：

`experiments/layered_planner_benchmark/3d_v1_r1_calibration_20260904_05`

查询为 A2B-02/07/11/15，每条 3 repetitions。冻结配置为：

`external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_r1_l2_lifecycle.yaml`

配置于 held-out 前冻结，threshold adjustment 为 0。其 SHA-256 为 `8a24aafff96e95e988a9c2ec04a15f55ac168ce775a565ca7f7210f5b01d539b`。

calibration 结果：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| correctness | 360/360 | 100% |
| C vs A P50 reduction | 51.33% | ≥20% |
| C/A P95 | 1.0217 | ≤1.05 |
| C/A P99 | 1.0384 | ≤1.10 |
| warm activation P95 | 149.11 ms | ≤1,000 ms |
| max resident | 27,127,859 B | ≤35,000,000 B |
| resident reduction vs 57,974,008 | 53.21% | ≥30% |

calibration hard gate 通过；P95/P99“不慢于 A*”目标门槛未通过，已在冻结前如实记录，没有转成更宽松门槛。

calibration 四查询 cold build P50 从 r0 11,984.65 ms 降到 r1 6,080.64 ms，下降 49.26%；最大查询从 32,085.27 ms 降到 16,008.49 ms。

## 7. Stage 5：12-query held-out 三臂结果

权威 held-out：

`experiments/layered_planner_benchmark/3d_v1_r1_heldout_20260904_01`

查询：A2B-01/03/04/05/06/08/09/10/12/13/17/18；每条 10 repetitions。A2B-16/19 仅进入 `classification_diagnostics.csv`，未污染成功性能总体。A2B-16 的 known classification 为 full-map all-variants failure 待地图/Smac 排查；A2B-19 为已知 L3 long-tail。

未找到能验证 map/坐标/时间版本的真实清扫动态占用日志。本轮负载必须称为 **realistic synthetic workload on real 4x map**，不得称为真实障碍分布。

每个 query/repetition 覆盖 unconfirmed、duplicate、off-corridor、off-path cost increase、1/2-source eligible change、large fallback、no-route 和 recovery；三臂共享 snapshot/hash/map/query/seed。

三臂 all-invoked（每臂 n=600）：

| Arm | P50 ms | P95 ms | P99 ms | Mean ms |
|---|---:|---:|---:|---:|
| A cold grid A* | 717.782 | 2,157.206 | 2,233.528 | 978.081 |
| B r0 selective | 317.002 | 2,157.965 | 2,259.901 | 585.259 |
| C r1 optimized selective | 315.877 | 2,167.674 | 2,238.302 | 576.407 |

关键分桶：

| Bucket | A P50/P95/P99 ms | B P50/P95/P99 ms | C P50/P95/P99 ms |
|---|---|---|---|
| eligible | 988.226 / 2,175.064 / 2,250.591 | 43.581 / 102.968 / 105.900 | 35.695 / 79.931 / 81.059 |
| large fallback | 1,007.867 / 2,147.247 / 2,195.269 | 1,039.146 / 2,191.651 / 2,254.806 | 1,036.971 / 2,211.987 / 2,240.400 |
| no-route | 587.952 / 993.128 / 1,020.383 | 557.826 / 1,030.521 / 1,043.742 | 548.210 / 1,026.378 / 1,099.224 |
| recovery | 999.313 / 2,175.352 / 2,246.815 | 1,059.844 / 2,249.630 / 2,328.020 | 1,036.526 / 2,215.459 / 2,316.527 |

selective 策略的净收益边界很清楚：在 eligible bucket，C 的 P50 相对 A 改善约 96.4%，相对 B 改善约 18.1%；large/recovery 仍由 deterministic A* 主导，C 相对 A 存在约 2%–3% 包装开销。把 40% eligible 与 60% fallback/no-route/recovery 的 invoked workload 汇总后，C 的 P50 相对 A 改善 55.99%，P95/P99 仅慢 0.49%/0.21%，通过 hard gate，但没有通过“不慢于 A*”目标门槛。

held-out cold build P50 从 r0 18,461.94 ms 降到 r1 9,160.61 ms，下降 50.38%；P95 从 38,782.42 ms 降到 19,347.92 ms。所有 120 次 warm activation cache hit，P95 152.81 ms。

正确性：3,600/3,600；max raw floating accumulation difference 为 `1.2733e-11`，按冻结 `1e-9` canonicalization 后 cost error 为 0；blocked/recovering path、partial、hidden reinitialize 均为 0。r0 与 r1 的 exact path-cell parity 均为 1,560/1,800；240 个 eligible 行采用与 forward cold A* 不同但等成本的确定性 D* 最优路径。该差异没有隐藏，compact A* 自身与 grid A* 的 exact path-cell property test 仍为 8/8 seeds 通过。

唯一 hard failure：最大路线 A2B-03 的 warm resident 35,335,905 B，比 35 MB 冻结目标多 335,905 B。held-out 最大 r0 state 为 179,908,640 B；r1 对大路线已经显著缩减，但不能事后修改统一目标。

## 8. Stage 6：高动态 soak

权威 soak：

`experiments/layered_planner_benchmark/3d_v1_r1_soak_20260904_01`

- 10,000 snapshots，动态段 1,535.34 s，达到 snapshot 上限后停止；
- A2B-07/11/17，每 500 snapshots 路线切换；
- 障碍移动、聚集、消失、20 次 no-route、820 次 recovery；
- 2,807 次 deterministic fallback，7,173 次 scheduler skip，40 次显式 resync；
- 20 route activations、21 evictions；peak active state=1；
- oracle mismatch=0，partial=0，blocked path=0，timeout=0；
- 8 次故意错误 ACK 全部被拒绝，unexpected ACK acceptance=0；
- 结束 clear 后 active=0、resident=0；本轮 child PID 残留为空。

趋势：

| 指标 | 前 10% | 后 10% |
|---|---:|---:|
| RSS mean | 902,959,911 B | 939,582,882 B |
| latency P50 | 6.536 ms | 5.552 ms |
| latency P95 | 698.530 ms | 672.496 ms |
| latency P99 | 709.971 ms | 682.424 ms |

RSS 前后均值增加 36.62 MB，全序列线性斜率 3,201.48 B/snapshot，peak 944,791,552 B。路线循环中多次从约 932–942 MB 回落至约 923–935 MB，active resident 随路线在约 3.78–8.57 MB 间回落，不呈 query-state 常驻或逐快照单调累积。按 soak runner 的判定规则，RSS 无界增长门槛通过；但 CPython/NumPy allocator 不归还全部 RSS 页，36.62 MB 平台抬升仍应在下一轮更长真实进程观测中继续监控。

## 9. Stage 7：条件式 Stage B

权威 disposition：

`experiments/layered_planner_benchmark/3d_v1_r1_stage_b_not_run_20260904_03`

状态为 `NOT_RUN_L2_GATE_FAILED`。唯一 hard failure 是 `resident_target_pass`；`p95_target_pass` 和 `p99_target_pass` 仅作为非阻断目标 miss 单列。Stage-B process started=0，Smac query executed=0。

系统进程审计发现 4 个本轮开始前已经存在、引用 `20260826...planner_visualization` 的 ROS/Nav2 进程（PIDs 773996/774006/774008/774014）。它们不是本轮子进程，本轮没有终止或修改这些用户现场进程。soak 自身结束时 child PID 为空。

`stage_b_not_run_20260904_01` 和 `_02` 为记录字段逐步澄清后的 superseded 目录，保留但不作为权威 disposition。

## 10. 失败、中断与排除目录

以下目录均保留，不纳入最终统计：

- `3d_v1_r1_r0_profile_20260904_01`：误发任务，非权威；
- `3d_v1_r1_calibration_smoke_20260904_01`：早期单查询诊断；
- `3d_v1_r1_calibration_20260904_01`：scheduler 路径相关性负载设计暴露分歧后中断；
- `3d_v1_r1_calibration_smoke_20260904_02`、`_03` 与 `calibration_20260904_02`：原 ROI barrier 未构成稳定 no-route；
- `3d_v1_r1_calibration_smoke_20260904_04`：修正负载后的 1-query 冒烟；
- `3d_v1_r1_calibration_20260904_03`：稠密 bbox index 导致最大 resident 67.5 MB；
- `3d_v1_r1_calibration_20260904_04`：compact v2 首版 fallback path mapping 循环导致 P95/P99 退化；
- `3d_v1_r1_stage_b_not_run_20260904_01`、`_02`：由 `_03` supersede。

失败目录的存在说明本轮是先 profile、再以门槛驱动优化；没有删除不利结果或覆盖目录。

## 11. 证据限制

1. 没有找到可验证 map/坐标/时间版本的真实清扫动态障碍日志，Stage A/soak 都是 real-map realistic synthetic workload。
2. held-out Stage A 保存了 scheduler decision/count 和 L2 response，但没有把 `pipeline_response_ms` 独立列写入 CSV；因此不能从正式目录重建 scheduler wall 与完整端到端 wall 分布。本报告不声称总体端到端收益。
3. production L1 diagnostics 的 `corridor_route_length_m` 在这些 query 中为 0；query 分层使用 route-edge count、turn count、ROI/safe-state size 和 endpoint identity 作为长度/复杂度代理。
4. Stage B 未运行，48-bin Smac/footprint/kinematic/curvature 在 r1 上只有单测与继承契约，没有新的三查询集成证据。
5. A2B-03 resident 目标仅差 0.96%，但冻结规则禁止事后优化再复用同一 held-out 作为晋升证据；下一轮必须重新冻结新 held-out。

## 12. 精确修改文件

实现与配置：

- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/l2_state_lifecycle.py` — `a189b8ae68e30a24b00fad397e33c10da6999fb1ba9124b81836e9357e362b9d`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_profile.py` — `20ab13ad525d22148a5ef671934c3e9b4dd9ce7c04c0fd4af8b918e59fa09667`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_pipeline.py` — `e4cf9ad21f5d8819e8757ee27d8339d5aaf44384a6ff93bf68bb8360ad043e7a`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_stage_a.py` — `e3850e56025c30a8e26d63e2dd8d6992d21fe9e92119580f0cda9fd09f264f65`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_soak.py` — `d904fa0e02ae67b2d11259c47bab1453b5574f35c174b326b2b54ca89aeb81f3`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_stage_b_gate.py` — `4f7007b0bc807c7279aeba141dbdfcfbd6e9e546104fa37a4a902d4f12c7044b`
- `external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_r1_l2_lifecycle.yaml` — `8a24aafff96e95e988a9c2ec04a15f55ac168ce775a565ca7f7210f5b01d539b`
- `external/arena4_ws/src/arena/three_d_v1/test/test_l2_state_lifecycle.py` — `0ba2da6629a66b5e41a7127ba080cc526c24435bc8e2e712376f0dd63cf112a6`
- `external/arena4_ws/src/arena/three_d_v1/test/test_r1_pipeline.py` — `9027993e7eed0eddb59422ae08bb9b902440ec686468b78b81050d47f10e2a14`
- `external/arena4_ws/src/arena/three_d_v1/setup.py` — `19b08dc532d10742f05aa3a3c2cde38dbd29cf44ca57394bbc20173993114b3f`
- `docs/PLN-02_3D_V1_R1_L2_LIFECYCLE_FINAL_REPORT.md`

新实验目录：Stage-0 audit、权威 r0 profile、calibration `_05`、held-out `_01`、soak `_01`、Stage-B not-run `_03`，以及上节列出的保留失败/诊断目录。

权威 manifest hashes：

| 证据 | manifest SHA-256 |
|---|---|
| r0 profile `_02` | `ecc2bacd362a2b1d7866e12523f824bece282549262b4c49e7d95a5dbb6d4877` |
| calibration `_05` | `5c457825ca44b6e5979a1d95eef06da32ced38203e9328d190a3359e3c91342e` |
| held-out `_01` | `4c241e1339be95206b88ad5fe651116edb65b63d02c3c58f49cd101375fcceb5` |
| soak `_01` | `d0bd749895f384cd2c55b22baa04b5886c589ebc5320aaf9401ca8fa30d02f36` |
| Stage-B not-run `_03` | `3ba72a950bdf5ff9dc6589bfc5503b0d91f5fbfed1ecb343c5c7c3ba5ad82d39` |

## 13. 下一步约束

若启动下一 revision，应先在不查看新 held-out 的情况下，把 A2B-03 resident 至少再降低 335,905 B 并补齐 scheduler/pipeline/end-to-end wall telemetry；随后使用新的冻结 held-out 验证，不能复用本轮 held-out 做晋升判定。只有新的 Stage A 全部 hard gates 通过，才能启动至少三条 query 的 48-bin Smac Stage B。

在此之前，生产判断保持：**不晋升 r1；保留 deterministic grid A* fallback 主导链路，不恢复 pure-D* 主线。**
