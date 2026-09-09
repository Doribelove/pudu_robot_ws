# PLN-02 2A-V3 r14 停车引导、适用性分派、ACK 与缓存工程化报告

## 结论

- `architecture_id`: `2A-V3`
- `implementation_revision`: `r14-parking-dispatch-ack-cache`
- 算法协议: `PLN-02-2A-V3-R14-PARKING-DISPATCH-ACK-CACHE-V1`
- 扩样协议: `PLN-02-2A-V3-R14-EXPANDED-PAIRED-V1`
- 最终判定: **C / NOT PROMOTABLE**
- Nav2 plugin: **NOT_RUN_FORMAL_GATE_FAILED**

r14 的缓存、内存、显式 E0 分派、成功路径安全和中位时延通过了冻结门槛，但 320 个正式请求中只有 275 个完成 exact ACK 并生成 final-valid 路径，45 个请求在 3 s exact ACK 截止前 fail-closed。正式 P95/P99 也超过冻结上限。因此 2A-V3 仍是研究实现，不能生产使用，不能注册成 Nav2 planner plugin。

停车中心线只完成了“同停车实例内连续、二维可达”的几何构造。冻结停车查询的中心带覆盖率只有 0.134--0.395，且折线几何曲率诊断为 7.56--11.99 1/m；没有任何停车查询获得下游 48-bin Dubins SE(2) witness。故本报告不声称“停车曲率可行引导已在线实现”。这些查询全部按冻结适用性规则分派到 E0。

## 父基线与不可变边界

r13 是唯一算法父基线：

- 报告 SHA-256: `1518935cbfa94bee34baff7bf13b33c7e8a1a893704d71eed869aefabc9048b2`
- r13 route-phase 配置 SHA-256: `c6cf6d098d90ad85bc4029345d12660a971ada06c1638fe73eb2e32dc9fb8824`
- selected8 文件 SHA-256: `b3307d4578447131e71db16156cd2f72e9f5042f98bdce0d75d21fe81300738d`
- selected8 query hash: `7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e`
- 交接文件 SHA-256: `0a87f51ecc56d815c07bcb9bfa674d6a8eea263e0c4eff12e87931b37dbbfa0a`

保持 0.05 m 地图、48-bin DUBIN、仅前进、禁止原地旋转、最小转弯半径 0.40 m、最大曲率 2.50 1/m、完整 padded footprint、硬语义与 no-stopping 约束。未修改 pinned Nav2；其 HEAD 为 `656ae8d4c56978efbdd446fe85582f2bcd06e920`，既有 dirty 文件 `nav2_bringup/launch/navigation_launch.py` 未触碰。

## r14 实现内容

### 1. 停车连续参考

`semantic_parking_reference_v3.py` 在同一停车 connected component 和 route tube 内执行确定性 8 邻接加权 A*，禁止 diagonal corner cutting，以停车 normalized deviation 为中心代价，按 route station 将连续几何参考拼回有向 L1 route。每个参考绑定 map、semantic map、query、route 与 policy hash。

该几何参考不是车辆轨迹。只有下游显式 48-bin forward-only Dubins 搜索逐 primitive 完整 footprint 重放、曲率不超过 2.50 1/m 且通过 canonical/semantic audit 时，才可称为 SE(2) certified。正式结果中没有停车 reference 获得该证书，所以本项为**部分完成**。

### 2. 适用性分派

- 语义 endpoint attachment 超过 0.75 m：`SEMANTIC_ROUTE_ENDPOINT_ATTACH_INAPPLICABLE`，进入 E0。
- 停车连续参考中心带覆盖率不大于 0.50：`PARKING_TARGET_CONTINUOUS_COVERAGE_NOT_APPLICABLE`，进入 E0。
- 有限有向 SE(2) 图中没有 start-reachable 且 goal-coreachable 的 target state：`NO_REACHABLE_SEMANTIC_TARGET`，进入 E0。
- 只有明确适用的查询才执行 `E5_R14_EXPLICIT_SE2`；适用后 E5 搜索失败必须 fail-closed，不能改写成 E0 成功。
- E0 fallback 不计 semantic success。

正式 32 查询中，ADAPTIVE 的成功行有 124 个 E0 分派和 14 个 E5 语义成功；失败的 22 行没有被回退掩盖。

### 3. deterministic ACK 流程

内容改变时的固定事务是：

1. 发布 bounded dirty ROI tiles；
2. 同步调用 public `ClearEntireCostmap`；
3. 一次重放完整当前 source grid（8,594,760 cells）；
4. 两次稳定 `GetCostmap` 观测，对 hard、soft、stale、ordinary affected cells 和 full hash 做 exact 比较；
5. sequence、policy、source、expected-master、server-content、ROI 全绑定后才允许规划。

超时不会触发 full repair，任何 mismatch 都 fail-closed。ACK 已证明 server 与 expected-master 逐栅格相等后，E5 使用 `EXPECTED_MASTER_PROVEN_EQUAL_BY_EXACT_ACK`，不再额外发起第三次 `GetCostmap`。192-request 预检证实 192/192 exact、`post_ack_get_costmap_requests=0`；但 320-request 正式长跑仍出现 45 个 fail-closed，说明 public full-costmap RPC 生命周期尚不够稳定。

### 4. 有界缓存与紧凑 query state

- 静态 raster、lane labels、edge annotations：schema/hash-bound immutable disk cache，`.npy` mmap，磁盘最多 2 项，活动项最多 1。
- 静态 cache key: `848b5358c775e15c4243e449c98e4a7b85309e851d4ecac681ba1bd6dc4702b7`。
- cache 文件 120,638,318 B，mapped arrays 120,326,640 B。
- query directional/search arrays 只 materialize 于 route 可达 lateral probe 6.0 m 加 0.75 m 支撑边界；publication ROI 仍保留完整 route-lane ROI，不缩小 E5 可达状态集。
- dispatch LRU 容量 64、1 MiB；正式运行活动 32 项、431,882 B、0 eviction。

r13 的约 10.87 s 静态准备降到 warm activation 0.484 s；首次 cache build 仍约 9.4 s，明确属于离线/冷构建成本，没有藏入在线数值。正式峰值 RSS 为 1,702,756,352 B，较 r13 的约 2.12 GB 下降约 20%；前后 10% RSS P50 从 877,207,552 B 到 886,353,920 B，增长 9,146,368 B，没有单调无界增长。

## 冻结扩样设计

query set 为 `pudu_wanda_3f_2a_v3_r14_expanded32_v2`：32 条，query hash `42464f9e99d3a017e63dde3c9559944698deccf732b2d03a0f56110d814dfd02`，文件 SHA-256 `d581763605e7830d1b72d2503d75a996646a7155b698bdedd0dd10f9f41b83e2`。

它对 selected8 的每条查询使用固定 `[0,1]`、`[.04,.96]`、`[.08,.92]`、`[.12,.88]` route windows，仅依据 map、语义类别、footprint、connected component 和 neutral topology route 验证，不读取 E0/E5 outcome。覆盖 8 类，每类 4 条。局限是 32 条仍来自 8 条父查询的相关窗口，不等价于 32 条独立场景，也未补足 forbidden/unlabelled/narrow 的广泛泛化证据。

每条先 1 次 warmup，再 5 次 measured；两臂各 160 个正式样本。顺序按 query ordinal 和 repetition 确定性 AB/BA counterbalance，每轮 16 条 E0-first、16 条 ADAPTIVE-first，避免固定 E0-first 给 fallback/no-op 人为优势。

冻结门槛：

| 门槛 | 阈值 |
|---|---:|
| query 数 | >=30 |
| 每臂 measured | >=100 |
| final-valid / exact ACK / safety | 全部通过 |
| E5/E0 P50 | <=2.0 |
| E0 P95 / P99 | <=3.0 / 3.5 s |
| ADAPTIVE P95 / P99 | <=5.0 / 6.0 s |
| static cache activation | <=1.0 s 且 HIT |
| peak RSS | <=1.8 GB |
| 稳态 RSS 增长 | <=128 MiB |
| E5 semantic coverage | >=4 queries 且 >=10 rows |

## 实验结果

### 通过的 192-request 生命周期预检

目录：`2a_v3_r14_ack_proof_reuse_preflight_v2_20260908T174500Z`

- 192/192 final-valid、exact ACK 和安全通过；timeout full repair=0。
- 64 measured/arm，因此不冒充正式 P99。
- E0 P50/P95/P99 = 1.623/2.023/2.174 s。
- ADAPTIVE P50/P95/P99(debug) = 1.674/4.098/4.853 s。
- ADAPTIVE/E0 P50 = 1.032。

### 冻结正式 32-query paired run

权威目录：`2a_v3_r14_expanded32_formal_v3_20260908T180500Z`

| 指标 | E0 | ADAPTIVE |
|---|---:|---:|
| measured | 160 | 160 |
| final-valid | 137 | 138 |
| fail-closed | 23 | 22 |
| semantic success | 0 | 14 |
| P50 | 1.760 s | 1.777 s |
| P95 | 4.448 s | 4.870 s |
| P99 | 5.358 s | 5.654 s |

ADAPTIVE/E0 P50=1.010，通过 2.0 门槛。E0 和 ADAPTIVE 的正式 P95/P99 均超过冻结门槛。320 行中 275 行 exact ACK 成功，45 行 fail-closed，timeout-driven full repair 始终为 0。ACK failure log 有 35 条明确记录 `global costmap readback timed out`；最早在 repetition 3 出现，随后在 repetition 4/5 形成队列压力。失败不能算作路径成功或安全成功。

成功生成的 275 条路径全部满足 collision=0、kinematic=0、hard semantic=0、no-stopping endpoint=0、reverse distance=0、rotate-in-place=0、maximum curvature<=2.50 1/m。

显式 E5 成功只来自 4 条不同查询：

| query | 成功 repetitions | correct-side | lateral error P50 | 路径最大曲率 |
|---|---:|---:|---:|---:|
| x32-05 lane-south | 3 | 1.000 | 0.050 m | 2.494 1/m |
| x32-17 junction-turn | 4 | 0.975 | 0.350 m | 2.494 1/m |
| x32-18 junction-turn | 4 | 0.995 | 0.269 m | 2.494 1/m |
| x32-20 junction-turn | 3 | 0.962 | 0.379 m | 2.494 1/m |

其余语义不适用查询使用 E0。特别是 12 条 parking-related query 中 6 条 endpoint attachment 不适用，另外 6 条中心参考覆盖不足；没有停车 E5 witness。因此 r14 没有完成停车在线语义收益证明。

### Gate 总表

| Gate | 结果 |
|---|---|
| 32 query / 每臂 160 measured | 通过 |
| AB/BA 公平顺序 | 通过 |
| all safety（已有路径） | 通过 |
| P50 ratio | 通过 |
| semantic query coverage | 通过 |
| static cache activation | 通过 |
| peak / steady RSS | 通过 |
| all exact ACK | **失败** |
| all final-valid | **失败** |
| formal P95/P99 | **失败** |
| promotion prerequisites | **失败** |

`runs.csv` SHA-256 为 `0890a9383e7dc2e12b2c41e17f57f9b089dce873de9c298d55d4fd57e0203ec8`，`final_result.json` SHA-256 为 `278dc5aa55280923cd1735935d6ac1fea8eae22a93cc589bb9c7c14e0b0e9401`。

## 失败与排除目录

- `2a_v3_r14_expanded32_formal_v1_20260908T163000Z`: `EXCLUDED_CONFIG_SERIALIZATION_PRE_RUN`，0 planner request。
- `2a_v3_r14_expanded32_formal_v2_20260908T164000Z`: 完成但 gate failed；重复 post-ACK GetCostmap 引起 152/320 fail-closed。
- `2a_v3_r14_ack_snapshot_reuse_preflight_v1_20260908T171500Z`: 完成但 gate failed；易失 server buffer cache hash mismatch，56 fail-closed。
- `2a_v3_r14_ack_proof_reuse_preflight_v2_20260908T174500Z`: 通过预检适用门槛，正式样本不足是预期。
- `2a_v3_r14_expanded32_formal_v3_20260908T180500Z`: 最终权威正式失败结果，不能筛除失败行后重算晋升结论。

所有目录唯一且未复用；历史 r0--r13 结果只读。

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash
export PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation:$PYTHONPATH
/usr/bin/python3 -m pytest -q external/arena4_ws/src/arena/evaluation/arena_evaluation/test
/usr/bin/python3 -m compileall -q external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation external/arena4_ws/src/arena/evaluation/arena_evaluation/test
colcon build --packages-select arena_evaluation --symlink-install
/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r14_expanded --help
git diff --check
git -C external/arena4_ws/src/arena/evaluation diff --check
```

结果：658 tests passed（6 个既有 warning）；compileall、三个 r14 CLI `--help`、`colcon build arena_evaluation`、主仓及嵌套仓 `git diff --check` 全部通过。

正式复现命令已保存在权威目录 `reproduction_command.txt`。权威运行使用 `ROS_DOMAIN_ID=167`，退出码 2 是 gate failure，不是进程崩溃；stderr 为空。任务结束时 r14 ROS/Nav2/Smac 残留进程为 0，既有其他任务 PID 未终止。

## 修改文件

新增 r14 实现：

- `semantic_parking_reference_v3.py`
- `semantic_static_cache_v3.py`
- `semantic_v3_ack_r14.py`
- `semantic_route_phase_compact_r14.py`
- `semantic_query_expansion_v3.py`
- `two_layer_v3_semantic_r14_benchmark.py`
- `two_layer_v3_semantic_r14_online.py`
- `two_layer_v3_semantic_r14_expanded.py`
- 三个 `two_layer_v3_semantic_r14_*.yaml`
- `pudu_wanda_3f_v3_r14_expanded32_v2.yaml`
- `test_two_layer_v3_semantic_r14.py`

更新 `arena_evaluation/setup.py` 仅增加 r14 CLI entry points。未 stage、commit 或 push。

## 下一步

不应继续增加 ACK timeout 或重复刷同一 32 条查询。下一 revision 应先替换当前“Python 端反复拉取 8.6 MB full master”的 ACK transport：在不修改 pinned Nav2 的边界下，使用独立、可版本化的 costmap observation adapter，以一次服务响应/共享内存快照绑定 publication sequence 和 full content hash，并提供显式 completion event；同时必须保留逐栅格 exact proof 和 fail-closed。该生命周期在 >=384 连续请求预检全过后，才能重新冻结正式实验。

停车侧必须先在独立离线同原语 oracle 中生成至少一条中心带占比>0.50、normalized deviation P50<=0.25、完整 footprint、曲率<=2.50 1/m 的连续 SE(2) reference；如果不存在，应将对应 query 明确判为语义/运动学不适用，而不是把高曲率二维折线叫作可行 reference。

在 ACK 与停车两项均通过之前，不注册 Nav2 plugin，也不讨论 A/生产晋升。
