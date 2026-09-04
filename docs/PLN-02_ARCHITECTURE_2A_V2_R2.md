# PLN-02 2A-V2 r2：方向、exact ACK 与冷路径时延

## 结论先行

- `architecture_id`: `2A-V2`
- `implementation_revision`: `r2-direction-ack-latency`
- `protocol_id`: `PLN-02-2A-V2-R2-DIRECTION-ACK-LATENCY-V1`
- 父基线：`2A-V2-r1`
- 最终判定：**C——停止扩样，生产侧不得据此晋升。**

三项任务的判定是：

| 项目 | 判定 | 证据 |
|---|---|---|
| reverse 方向/可行几何 | **未通过** | 离线方向场正确，但在线 reverse 仍为 `0.795795 / 2.10 m`，没有把 0.796 四舍五入成 0.80 |
| exact effective-content ACK | **通过** | 冻结 8-query 共 53/53 attempts；hard/soft/stale/hash/sequence mismatch 全为 0 |
| E4 冷请求时延 | **部分通过** | 独立进程 P50 `3.204 s / 2.029 s = 1.579x`，通过硬门槛 2.0x，未达到目标 1.5x |

此外，E0 成功的 `real-lane-junction-lane` 在 E4 到 R4 仍是
`SMAC_MAX_ITERATIONS`。因此即使 exact ACK 和性能硬门槛通过，Stage 5 仍不满足
Stage 6 启动条件。正式 30–50 query / ≥100 有效样本扩样标记为
`NOT_RUN_STAGE5_GATE_FAILED`，没有用调 Smac 或增加迭代数掩盖 L2/L3 问题。

## 冻结与边界

工作区 branch 为 `codex/2a-v1-r2-roi-pathaudit-delivery`，起始 HEAD 为
`337fed6e…`。Python 为 `/usr/bin/python3 3.10.12`，ROS 为 Humble。父基线及历史结果
均只读；没有 reset、checkout、clean、stage、commit 或 push，也没有修改 pinned
Nav2 核心。

主要冻结哈希：

| 对象 | SHA-256 / Git |
|---|---|
| r0 run12 `runs.csv` | `48069a641e…` |
| r0 run12 `summary.json` | `715456ad…` |
| r0 run12 `protocol.json` | `c5984b45…` |
| r1 报告 | `fa282334a75acae1a46eca8c7e4cfb7540daeddad61806a78b1ea383214d4f15` |
| r1 v20 目录流哈希 | `13a2fa…` |
| r1 v21 冷性能目录流哈希 | `3992ee…` |
| 地图 | `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a` |
| SemanticMapV1 | `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553` |
| query set | `9daf9b5ddaf682cf844a2845d4b9bc1abb827506d14dce333cdfd9916409c67a` |
| topology | `73ce811afd6083c0cd5b4eb3eefb33353bdfbbc18da6e83ec7316c21e6b90fb2` |
| pinned Nav2 HEAD | `656ae8d…` |

完整 branch/status、父结果哈希和逐 attempt profile 见
`docs/r2_root_cause_report.md`。工作区本来就有大量 dirty/untracked 内容，本轮只增量
修改清单末尾列出的文件。

## Stage 1：先 profile，再修改

冻结 r1 复现目录：

- `r2_stage1_r1_reproduction_20260904_01`：E0/E1 `5/8`、E2/E3 `4/8`、E4
  `5/8`，与 r1 冻结事实一致；
- `r2_stage1_ack_root_cause_20260904_01`：捕获每层 expected→actual 迁移、空间
  mismatch 和 ACK 时序；
- `docs/r2_root_cause_report.md`：在行为修改前写定根因。

ACK profile 共捕获 35 次内容观测。r1 的 interval ACK 会在 InflationLayer 尚未稳定时
提前通过；后续观测仍有大量 exact mismatch。即使强制 full reset 和可靠 QoS，仍有
少量稳定差异。逐栅格回溯证明这些差异来自 pinned Humble InflationLayer 的整数距离
桶、行优先 lethal seed、`seen_` 首次传播顺序，而不是消息丢失。

r1 冷路径的主要在线成本则是全图/大 crop 的方向场临时数组和重复 compose；9 秒级
semantic edge precompute 是独立 cold-start 成本，未藏入 request latency。

## Stage 2：方向实现与真实几何证据

实现保证：

1. L1 polyline 在进入方向场前按 query start→goal 定向；反向时 route、node/edge
   顺序和 edge annotations 同步反转，缓存源对象不被修改；
2. 右侧由同一 lane feature instance 内的真实左右边界距离 `d_right <= d_left`
   判定，不使用任意骨架线符号；
3. 切线和边界最近点只在同一 semantic lane instance 内传播，junction/parking/
   相邻 lane 不互相污染；
4. 方向 float grid 只保存 active crop，保留方向稳定度和每 lane instance 诊断；
5. query validity 规则在在线调参前固定为 endpoint footprint/hard/no-stopping、起终点
   4-connected、以及存在可达 ≤0.5 m target band。

冻结离线 8-query 的 forward/reverse target band 分别为：

| query | preferred correct-side | target error P50 | 离线场 |
|---|---:|---:|---|
| real-lane-forward | 0.999829 | 0.210 m | 通过 |
| real-lane-reverse | 0.999858 | 0.240 m | 通过 |

原 reverse query 不能重分类为无效：端点均合法，起终点位于同一自由连通分量，目标带
有 49,148 个可行栅格。但真实静态障碍把目标带切成 108 个分量，最大分量 25,725
cells；L1 route 到目标带的距离 P50 为 3.114 m。这解释了“离线场正确、在线长距离
Hybrid 搜索不能兑现”的差异，但不改变 query denominator。

校准没有扫参碰运气：以 r1 5 m 过弱和首个 1 m 过强 profile 为边界，只做了有记录的
中点/损失形状/启发式诊断。1 m（64 cap）有 42.329% 软代价饱和，2 m 有 33.227%，
24/40 cap、sqrt loss 和 cost-penalty 3.5 均在不变的 1,000,000 iterations 处失败；
linear 5 m 虽有路径，仍只有 `0.792 / 2.15 m`。这些目录均带 `EXCLUDED.md`。最终冻结
保留 r1 的温和 Huber 5 m / cap 64，避免用更高软代价破坏正常规划。

## Stage 3：exact effective-content ACK

新增的 C++ expected-effective mapper逐项复现 pinned Humble InflationLayer，包括地图
上下行序、整数距离 level、lethal seed 行优先顺序、source propagation、`seen_` 和
unknown max-combination。输入仍是标准 OccupancyGrid；观测仍来自 Smac 实际消费的
master costmap。

ACK key 完整绑定：publication version/sequence、policy hash、source grid hash、
expected master hash、ROI bbox、server content hash。dirty 区域为 source changed、
old/new expected-master dirty、hard 和 soft 的并集；要求两个稳定的 exact 观测，timeout
或任意 mismatch 都 fail closed。no-op 只有在完整 key 对应的既有 exact ACK 证据存在时
复用，缓存容量为 1。

冻结 8-query 结果：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| attempts / acknowledged | 53 / 53 | 全部 |
| hard exact mismatch | 0 | 0 |
| soft exact mismatch / checked | 0 / 16,962,691（0.000%） | 0 |
| stale ROI cells | 0 | 0 |
| ACK hash mismatch | 0 | 0 |
| ACK sequence mismatch | 0 | 0 |

因此 r2 可以称 **exact effective-content ACK verified**；不再使用 r1 的“区间 ACK”表述。

## Stage 4：延迟与内存

冷性能采用两个全新目录、两个独立 Python/Nav2 进程和 ROS domain 143/144。两臂共享
相同 map/query/topology/seed/hash，但不共享任何进程内缓存。下表是 8 个冻结 query
的 debug P50；n=8，P95/P99 不用于正式宣称。

| 累计阶段 P50 | E0 cold | E4 cold |
|---|---:|---:|
| request wall | 2,028.991 ms | 3,203.779 ms |
| L1 | 1.493 ms | 3.836 ms |
| ROI build | 96.547 ms | 393.477 ms |
| field build/audit field | 252.522 ms | 981.357 ms |
| compose | 396.310 ms | 467.564 ms |
| publish | 6.247 ms | 10.166 ms |
| exact ACK wait | 720.735 ms | 1,135.839 ms |
| Smac | 457.110 ms | 552.642 ms |
| audit | 124.931 ms | 140.705 ms |

`E4/E0 = 1.579x`，通过 ≤2.0x 硬门槛，未达到 ≤1.5x 目标门槛。相对冻结 r1 E4
P50 6.181 s，r2 为 3.204 s，下降 48.16%；r1 的 2.16x 降为 1.58x。

cold-start 单独报告：E0 9,113.595 ms，E4 9,126.910 ms，主要仍是约 8.23 s 的
semantic edge precompute。若把 cold-start 与 P50 request 相加，则 E0/E4 分别约
11.143/12.331 s；本报告没有用缓存后的 L1 数字替代完整首启成本。

composer 的 base/class/inflation 缓存有显式 1–2 entry 上限，单组实测 resident bytes
为 34,379,040；unit test 覆盖 hit、key 改变和 eviction。冻结五臂进程 peak RSS 为
2.720 GB，低于 r1 v20 的 3.192 GB（约 14.8%）。独立 E4 冷进程 peak 为 2.872 GB，
低于 r1 v21 的 3.131 GB（约 8.3%）。重复 forward 10 次的 current RSS 并非逐次单调：
`1.817, 2.023, 2.100, 1.976, 2.079, 2.105, 2.009, 2.087, 1.991, 2.120 GB`；
这支持“未观察到逐 request 单调泄漏”，但 10 次仅是开发诊断，不能替代 soak。

warm-process forward（1 warmup + 3 measured）P50 为 E0 0.983 s、E4 1.764 s；
E4 composer inflation cache hit 且 build=0，exact no-op ACK P50=0。query field 仍按有界
生命周期重建，所以这不是把无界缓存换来的速度。

## Stage 5：冻结 8-query 五臂结果

| arm | final-valid | R0 success | relaxation | request P50（同进程诊断） |
|---|---:|---:|---:|---:|
| E0 | 5/8 | 5 | 0% | 2.130 s |
| E1 | 5/8 | 5 | 0% | 0.821 s |
| E2 | 4/8 | 4 | 0% | 0.854 s |
| E3 | 4/8 | 4 | 0% | 3.126 s |
| E4 | 5/8 | 4 | 50% | 1.372 s |

同进程数字只说明缓存/复用行为，不作为 cold gate；cold gate 使用上一节的独立目录。
E4 触发放宽的查询是 forbidden、junction、unlabelled、narrow，只有 forbidden 在 R1
获得有效路径。

逐 query 的 E0/E4 配对如下。长度和曲率只在同 query 且两臂都成功时比较；由于 E0
与 E4 selector 不同，这些配对不能被解释为单模块因果。

| query | E0 | E4 | E4 level | length E0/E4 (m) | curvature P95 E0/E4 | E4 side / error | 结论 |
|---|---|---|---|---:|---:|---:|---|
| lane-forward | pass | pass | R0 | 155.873 / 157.491 | 0.2793 / 0.2798 | 0.9886 / 0.45 m | direction pass |
| lane-reverse | pass | pass | R0 | 155.660 / 156.191 | 0.2993 / 0.2781 | 0.7958 / 2.10 m | direction fail |
| lane-junction-lane | pass | fail | R4 | N/A | N/A | N/A | E0 retention/R0 fail |
| lane-to-parking | fail | pass | R0 | N/A | N/A | 0.9945 / 0.55 m | E4 gain |
| parking-internal | pass | pass | R0 | 168.149 / 168.019 | 0.5564 / 0.4927 | 0.9229 / 1.30 m | paired valid |
| forbidden-detour | pass | pass | R1 | 111.583 / 14.220 | 0.5904 / 1.0899 | 0.0392 / 5.55 m | selector differs; not L3 attribution |
| unlabelled | fail | fail | R4 | N/A | N/A | N/A | no regression, no success |
| narrow-lane | fail | fail | R4 | N/A | N/A | N/A | no regression, no success |

`lane-to-parking` 的 E2 与 E3 route hash 都是
`0144fac55a29775b529cc74c22cc2bad0f7082321ae7efc94732ac11fa3ff953`；E2 失败、E3 R0
成功，因此该条收益可以归因于固定 L1 route 下的 L3/route-lane ROI，而不是 selector。

所有成功路径的 collision、kinematic、hard semantic、no-stopping goal 违规均为 0；
失败行的 `hard_constraints_held` 保持 null / `NOT_APPLICABLE`。

## 测试与复现

权威验证：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash
export PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation:$PYTHONPATH
/usr/bin/python3 -m pytest -q /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/test
/usr/bin/python3 -m compileall -q /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation
/usr/bin/python3 -m arena_evaluation.two_layer_v2_semantic_r2_benchmark --help
cd /home/robot/pudu_robot_ws/external/arena4_ws && colcon build --packages-select arena_evaluation --symlink-install
cd /home/robot/pudu_robot_ws && git diff --check
```

结果：361 tests passed；compileall、CLI help、colcon build、git diff --check 通过。
每个冻结 r2 目录内含复现命令、manifest、verification、runner stdout/stderr、源码快照与
哈希；ROS/Nav2 的完整进程日志在各目录 `logs/`。

## 权威结果目录

- root cause：`private_data/pudu_wanda_3f/results/r2_stage1_r1_reproduction_20260904_01`
- ACK root cause：`private_data/pudu_wanda_3f/results/r2_stage1_ack_root_cause_20260904_01`
- synthetic：`private_data/pudu_wanda_3f/results/synthetic_smoke_r2_v1`
- frozen offline direction：`private_data/pudu_wanda_3f/results/offline_direction_r2_frozen8_v1`
- frozen Stage 5：`private_data/pudu_wanda_3f/results/real_ablation_r2_frozen8_v1`
- isolated E0 cold：`private_data/pudu_wanda_3f/results/real_ablation_r2_e0_cold_isolated_v1`
- isolated E4 cold：`private_data/pudu_wanda_3f/results/real_ablation_r2_e4_cold_isolated_v1`
- warm forward：`real_ablation_r2_e0_warm_forward_v1`、`real_ablation_r2_e4_warm_forward_v1`
- 10× RSS diagnostic：`real_ablation_r2_e4_warm_forward_10x_v1`
- 最终验证与进程审计：`private_data/pudu_wanda_3f/results/r2_final_verification_20260904_01`

失败和校准目录全部保留。没有复用目录；失败 exact-ACK probe、错误 ROS 环境启动和所有
未冻结的方向损失/权重诊断均带 `EXCLUDED.md` 或 `STATUS.md`。误发的 3D-V1 任务只留下
既有的中断 profile 目录，本轮没有继续任何 3D 实现或实验。

## 精确修改文件

- `docs/r2_root_cause_report.md`
- `docs/PLN-02_ARCHITECTURE_2A_V2_R2.md`
- `arena_evaluation/arena_evaluation/regional_preference_r2.py`
- `arena_evaluation/arena_evaluation/semantic_costmap_r2.py`
- `arena_evaluation/arena_evaluation/semantic_smac_session_r2.py`
- `arena_evaluation/arena_evaluation/two_layer_v2_semantic_r2_root_cause.py`
- `arena_evaluation/arena_evaluation/two_layer_v2_semantic_r2_benchmark.py`
- `arena_evaluation/src/nav2_effective_costmap.cpp`
- `arena_evaluation/config/two_layer_v2_semantic_r2.yaml`
- `arena_evaluation/test/test_two_layer_v2_semantic_r2.py`
- `arena_evaluation/arena_evaluation/two_layer_v2_semantic_r1_benchmark.py`（仅增加 r2 可观测字段/current RSS；r1 逻辑不变）
- `arena_evaluation/setup.py`（增加 r2 CLI 和本地 expected-effective extension）

其中 `arena_evaluation` 均位于
`/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/`。

本轮没有修改 pinned Nav2；其历史 dirty 文件和所有用户已有修改保持不动。
