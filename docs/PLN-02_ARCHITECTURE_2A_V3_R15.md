# PLN-02 2A-V3 r15 快速迭代与硬停止报告

## 结论

- `architecture_id`: `2A-V3`
- `implementation_revision`: `r15-fast-iteration-single-read-ack`
- `protocol_id`: `PLN-02-2A-V3-R15-FAST-ITERATION-V1`
- 本 revision 判定：**C / REJECTED**。
- 这不是把 r13 已有的 B 级研究证据降级，而是 r15 没有解决 r14 暴露的两个晋升阻塞点；2A-V3 仍不得注册 plugin 或生产晋升。
- 快速迭代目标达成：两个权威 focused gate 共用约 207.2 s（parking 35.25 s，ACK 171.90 s）即给出停止结论，没有再运行 4–5 小时的全量实验。

## r15 实现了什么

1. 新增 full-resolution parking reference oracle。它在同一 parking component 和安全区域内，按 `(中心带外长度, normalized deviation 积分, 几何长度)` 做确定性字典序搜索，避免 r14 标量权重用较短边界路径换掉中心覆盖。它只生成 guide，最终仍由冻结的 48-bin、forward-only DUBIN、完整 Jackal footprint route-phase 搜索和 canonical audit 判定。
2. 新增 single-observation exact ACK。保留 r14 的 ROI publish → clear service → full current source replay → full master byte/hash 比对，只把每次变更所需的相同 full-master 观察从 2 次减到 1 次。timeout、hard/soft/stale/hash/sequence mismatch 仍全部 fail closed，无 timeout full repair。
3. 新增快速 gate runner：parking oracle 无 ROS；ACK stress 无 Smac path search；首个硬失败立即停止；只有 parking、ACK、sentinel8、medium32 全通过才允许一次 formal32。

这不是用二维 raster 冒充 SE(2)，没有修改 pinned Nav2，也没有放宽曲率、倒车、原地旋转、footprint 或语义统计门槛。

## Gate 1：parking 同原语 oracle

权威目录：

`private_data/pudu_wanda_3f/results/2a_v3_r15_parking_oracle_v4_20260908T215000Z`

第一条冻结查询 `x32-21-cmp2-06-lane-to-parking` 已触发首错硬停止：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| full-resolution reference target ratio | 0.3894 | > 0.50 |
| reference polygonal curvature（仅诊断） | 10.518 1/m | downstream witness ≤ 2.50 1/m |
| SE(2) goal candidates | 48 | — |
| strict SE(2) witness | 0 | 必须存在 |
| candidate parking center-band ratio 最大值 | 0.3325 | > 0.50 |
| candidate normalized deviation P50 最小值 | 0.4068 | ≤ 0.25 |
| `GEOMETRIC_REVISIT` | 48/48 | 0 |
| `NATURALNESS_SCREEN` | 48/48 | 0 |
| semantic gate failure | 48/48 | 0 |
| wall | 33.73 s query / 35.25 s run | focused stage |
| peak RSS | 1.513 GB | ≤ 1.8 GB |

有限图中存在 start-reachable/goal-coreachable parking target states，且搜索没有达到 resource limit；但现有 phase/reference 接口生成的 48 条完整路径全有不必要回访并且中心统计不合格。因此这是**当前绑定 route-phase state lattice/interface 的无 witness 结果**，不是对连续空间绝对不可行性的证明。按冻结 gate，另两条 parking 查询未运行。

## Gate 2：single-observation exact ACK

4-transition smoke 目录：

`private_data/pudu_wanda_3f/results/2a_v3_r15_ack_smoke_v3_20260908T221000Z`

4/4 exact，通过；P50 1.084 s，P99 debug 1.270 s，峰值 1.477 GB。

权威 stress 目录：

`private_data/pudu_wanda_3f/results/2a_v3_r15_ack_stress_384_v1_20260908T221500Z`

计划 384 次，在第 148 次首错硬停：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| exact successes | 147 | 384/384 |
| first failure | transition 148, `cmp2-05-junction-turn` | 无失败 |
| successful P50 / P95 / P99 debug | 1.083 / 1.342 / 1.366 s | P99 ≤ 2.5 s |
| failure request wall | 3.602 s | ACK timeout 3.0 s |
| peak RSS | 1.478 GB | ≤ 1.8 GB |
| first/last 10% RSS median growth | 96,256 B | ≤ 128 MiB |
| timeout full repair | 0 | 0 |

第 148 次服务日志显示 `global costmap readback timed out`。超时时 master 仍有 272,878 个 affected mismatch，其中 hard 92、soft 9,614、stale 272,816；server full hash 与 expected hash 不同。实现正确地 fail closed，没有继续规划，也没有全图超时修复。

因此 r14 的长期缺陷不是“第二次相同 readback 太多”这么简单，而是反复 clear/full replay 后 StaticLayer/InflationLayer 和 GetCostmap 服务本身会出现未完成恢复与服务超时。把观察从两次改为一次只能推迟/减少负载，不能建立确定性 transaction。

## 为什么不运行 selected8 / expanded32

- `sentinel8`: `NOT_RUN_PARKING_AND_ACK_GATE_FAILED`
- `medium32`: `NOT_RUN_PARKING_AND_ACK_GATE_FAILED`
- `formal32`: `NOT_RUN_PARKING_AND_ACK_GATE_FAILED`
- Nav2 plugin：`NOT_REGISTERED`

继续运行这些阶段既不能改变前置硬失败，又会重新引入数小时成本，并违反本 revision 的 hard-stop protocol。

## 验证

- r13/r14/r15 回归：35 passed in 0.42 s。
- r15 专项：11 passed in 0.26 s。
- `/usr/bin/python3 -m compileall -q arena_evaluation/arena_evaluation`：通过。
- r15 CLI `--help`：通过。
- `git diff --check`：通过。
- `colcon build --packages-select arena_evaluation`：独立临时 build/install 树通过。原 install 树尝试因已有配置 symlink 返回 `EEXIST`，未删除或覆盖用户构建产物。
- r15 ROS/Nav2/Smac 残留进程：0。预先存在的 visualization process group 773996 保持未动。
- 未 stage、commit 或 push。

## 下一版应做什么

不应回到 2A-V2，也不应继续调 parking 标量权重或 ACK timeout。最合适的 r16 是两个明确的接口变更：

1. **phase-owned parking SE(2) trajectory**：在 parking component 内直接搜索单调 progress 的 `(station, lateral, yaw_bin)` trajectory，把中心覆盖、Dubins primitive、完整 footprint 和 no-revisit 同时放进状态/边约束；不要先生成锯齿二维中心线，再让 route-phase 搜索追踪它。
2. **query-invariant master lifecycle**：停止每个 query 对 Nav2 master 做 soft-cost clear/replay。会影响路径选择的 lane/parking reference-deviation 留在显式 E5 state lattice；Nav2 server 只对不可变 static/hard master 在 session activation 时做一次 exact ACK。E0 fallback 使用独立、冻结、未经语义改写的 native Smac base session。这样 ACK 不再是逐请求异步 transaction。

r16 仍可归入 2A-V3，因为核心仍是静态语义、显式 forward-only SE(2) global planner；但它是接口/状态空间重构，不是 r15 参数微调。r16 继续沿用本报告的快速 gate：3-query oracle → ACK activation soak → sentinel8 → candidate-only 32 → 最多一次 formal32。

## 文件与哈希

新增：

- `semantic_v3_ack_r15.py`
- `semantic_parking_reference_r15.py`
- `two_layer_v3_semantic_r15_fast.py`
- `config/two_layer_v3_semantic_r15_fast.yaml`
- `test/test_two_layer_v3_semantic_r15.py`
- 本报告

修改：`arena_evaluation/setup.py`，仅注册 r15 CLI。

当前主要 r15 SHA-256：

- ACK：`0487a698e0486df4d539cf29736b4c7bcccf607e7eecff13d5ffbe14609cd5ea`
- parking：`fb6e6d7792e89ff1002b6a20d1f782bdbe5e8b921a3f5fe40a10c9e717df4612`
- runner：`13e962481c1655eeed88092106831ec0c8dbd80df28a47194761e213447226f8`
- config：`4c4202c7c68693e70192907bff1459133d20475aea42cfe35f4453d592744dab`
- test：`8d066a9e89a7e54af96ce59063b47f2c81f15ac4bad0e9164fb0533b25a6f06a`

冻结父基线：

- r14 report：`a2774ccedd6816d296fbece62b48bd4a622076acce663e28c4425aa401dd3ddf`
- r14 algorithm config：`51a690266e00374abd0f54746544d7813262b2c1993f4fbab847278ea5a74128`
- r14 expanded config：`503f7c7419cac0c17f3b5f44f91e988beebb12b71bd4978ad677652ad02b0da6`
- r14 formal artifact manifest：`87492651c5f66efe75f07764dbbc600c4c0f12a162f1e55caa0e1486bd6d0ea3`

排除目录均保留且有 `EXCLUDED.md`：parking v1/v2、ACK smoke v1/v2。parking v3 的 gate 有效但候选 breakdown 不完整，目录有 `STATUS.md`；v4 为权威替代。所有新权威目录都有 protocol、reproduction command、CSV/JSON、source snapshot 和 artifact hash manifest。
