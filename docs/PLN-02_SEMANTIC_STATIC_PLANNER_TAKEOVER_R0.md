# PLN-02 导师语义地图静态全局规划接手与严格收口报告 R0

日期：2026-09-07

状态：**C，未达到真实 B，禁止晋升或使用**

本轮候选：`UNNAMED_SEMANTIC_PLANNER_CANDIDATE`
历史父架构：`2A-V2`（保持历史名称，不把本轮研究候选冒充 2A-V2 的新可用版本）

## 1. 结论先行

本轮没有把旧的 `gate_passed=true` 当成成功，而是补齐了“终点后折返收集语义分数”的失败关闭审计。最终三条冻结真实离线查询严格结果为 **2/3**：

| query | 请求/证书绑定 | 硬安全 | R2 语义 | 自然性 | 严格结果 |
|---|---:|---:|---:|---:|---:|
| `r3-mirror-1-positive` | 通过 | 通过 | 通过 | **失败** | **失败** |
| `r3-mirror-2-negative` | 通过 | 通过 | 通过 | 通过 | 通过 |
| `cmp2-02-lane-south` | 通过 | 通过 | 通过 | 通过 | 通过 |

因此：

- 当前系统仍是 **C**，没有达到 B；
- 在线新架构、同轮三臂、exact server ACK、冷请求时延和默认 selected8 全部按 gate **未启动**；
- 不能把 297 秒生成的 positive 候选缓存或读取为在线规划结果；
- 本轮没有命名、晋升或生产化任何新架构；
- 继续调二维 soft cost、再扩大搜索或换一个优化器，不会解决已暴露的“查询目标与可达语义带不一致”问题。

权威聚合结果：

- `private_data/pudu_wanda_3f/results/transition_naturalness_triad_takeover_final_20260907T090000Z/`
- 总证据包：`private_data/pudu_wanda_3f/results/semantic_static_takeover_r0_final_20260907T090000Z/`

## 2. 冻结契约与边界

本轮完整读取并沿用 `PLN-02_SEMANTIC_HANDOFF_20260907.md` 及其引用内容，没有重新询问用户已经批准的 R2 补充契约：

- 路径 `L <= 12 m`：全程统计；
- 路径 `L > 12 m`：仅统计 `[6 m, L - 6 m]`；
- lane 靠右：correct-side ratio `>= 0.80`，目标带占比 `> 0.50`，横向误差 P50 `<= 0.50 m`；
- parking 靠中：normalized deviation P50 `<= 0.25`，中心带占比 `> 0.50`；
- 静态/lethal、完整 padded footprint、禁止倒车、禁止原地旋转、`Rmin >= 0.40 m`、最大曲率 `<= 2.50 1/m` 均未放宽。

本轮 targeted 三条均是 lane 查询，因此 parking 阈值未被触发，但被冻结保留，没有删除或改写。

冻结输入：

- map hash：`05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`
- semantic map hash：`2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`
- targeted query content hash：`66212b05ef6c4d16eaedafc1c27866387c3d74aa85bf5ac69bca92487254667d`
- selected8 query content hash：`7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e`
- R2 config SHA-256：`4e63f5af208bc70909104a110ad1175efe6488ca5d7d1557de3451404f3cc994`

## 3. 本轮实际实现

### 3.1 路径必要性/自然性审计

新增的 `semantic_path_necessity_audit.py` 不以“无自交”代替“无无意义往返”，而是同时审计：

1. 按 query start→goal 定向的 L1 route 终端切线与目标法平面；
2. 路径越过目标平面的最大距离；
3. 沿 route 的连续及累计后退进度；
4. 间隔至少 `pi * Rmin` 的反向姿态完整 padded-footprint 实面积重叠；
5. 每个有效目标带样本位于目标平面前、平面上还是平面后；
6. 缺 route、缺 mask、端点绑定失败、非法数值等均失败关闭。

该屏障是保守筛查器：若发现可疑往返，则要求额外的硬障碍/运动学绕行必要性证据；本轮没有实现一个可以任意豁免它的“detour justification”接口。

### 3.2 独立、写一次、抗漂移的验证链

新增 `semantic_transition_naturalness_verify.py`，把以下门槛明确拆开：

- `hard_safety_gate_passed`；
- `targeted_binding_gate_passed`；
- `r2_semantic_gate_passed`；
- `naturalness_gate_passed`；
- 四者严格合取的 `strict_acceptance_gate_passed`。

旧优化器内部的 `bound_length` 不属于用户批准的 R2 验收规则，现只作为 research diagnostic，不再混入 hard safety。

验证器在计算前后、写盘前再次校验候选 path/control、query JSON/NPZ、semantic map、审计源码及所有 gate dependency 的 SHA-256；冻结 R2 YAML 本身也校验固定 SHA。任何 source/input/hash 漂移直接失败，不输出可用结论。

新增 `semantic_transition_naturalness_aggregate.py` 对三条结果再次独立检查：

- query 顺序、位姿内容和 targeted hash；
- input artifact manifest；
- candidate path/control；
- verifier、必要性审计、revisit 审计及 gate dependencies；
- candidate 内层 gates 与顶层 gates 一致；
- aggregate 读取到写盘之间的全输入 snapshot 不变。

### 3.3 终点平面与语义统计真实性

`semantic_transition_goal_plane_postprocess.py` 没有重跑已经完成的 1 cm / 360 yaw 昂贵投影，而是严格验证原 certificate/result/artifact manifest 哈希后，重新按稳定终端切线分类。它还把全部 raw target 子栅格拆分为：

- sampled full-footprint 不可行；
- sampled free 且属于起点分量；
- sampled free 但属于其他连通分量。

所有字段明确使用 `sampled projection` 命名，删除了会被误读为连续空间定理的 “necessary topology gate” 表述。

### 3.4 请求现场生成的快速有限走廊探针

`semantic_transition_ordered_corridor.py` 只读取当前 query/map/semantic/route，构建单 lane instance 内、station 单调、不过目标平面、48 yaw-bin、forward-only Dubins 的有限图；不读取历史 witness。

该探针用于验证“地图派生走廊能否把 297 秒压到请求级时间”，不是连续完备算法。其最终 path evaluation 已接入同一 hard/R2/revisit/naturalness 审计。

### 3.5 被主动降级且未运行的 SE(2) reachability 草案

代码审查发现最初的 48-bin 双向 reachability 草案每个 `(cell,yaw_bin)` 只保留首个连续代表元，是欠近似，并且缺少 Smac analytic expansion 和可重放父链证书。直接运行会制造假阴性或误导性成功。

因此该模块已改名义为 `NON_DECISION_EXPLORATORY`：

- `hard_stop_eligible=false`；
- `acceptance_evidence=false`；
- `c1_infeasibility_evidence=false`；
- CLI 固定返回 3；
- **没有在真实 positive 上运行**。

这符合“不重复无明确新假设的探索”和“不得伪造 B”的要求。

## 4. positive 候选为什么必须拒绝

297 秒生成的 candidate 数值如下：

- path length：`31.153514 m`；
- lane correct-side ratio：`0.835724`；
- lane target-band ratio：`0.547588`；
- lane lateral-error P50：`0.425988 m`；
- hard safety：通过；
- exact control replay、endpoint、trace 和 lane-instance binding：通过；
- 几何自交筛查：通过。

如果只看这组数字，它似乎满足 R2。但新的审计证明这些分数来自不真实的路径行为：

- 最大目标平面超调：`9.495705 m`；
- 累计 route 后退：`9.639804 m`；
- 非局部反向 footprint 重叠：`11,416` 对；
- active target samples：`420`；
- 其中目标平面后：`420/420 = 100%`；
- 目标平面前或平面上：`0`。

同时，冻结约束下存在一条 start→goal 的 hard-safe `LSL` 直达路径：

- length：`8.481812 m`；
- 比 candidate 短：`22.671702 m`；
- 自然性：通过；
- R2 soft semantics：失败（side `0`、band `0`、P50 `4.0 m`）。

这不表示“短路径必须被选中”，也不表示任何更长语义路径都非法；它只证明 22.67 m 的额外绕行不是静态碰撞或端点运动学所必需。candidate 是为了进入终点外的目标带并返回，从而抬高统计窗口，而不是为完成 A→B 请求。

![positive 候选的折返与语义窗口](../private_data/pudu_wanda_3f/results/transition_candidate_independent_replay_20260907T064200Z/path_and_semantics.png)

## 5. 三条严格离线结果

| query | 长度 | correct-side | target-band | error P50 | 自然性证据 | 严格结果 |
|---|---:|---:|---:|---:|---|---|
| positive | 31.154 m | 0.836 | 0.548 | 0.426 m | 超调、后退、反向 footprint 返访、100% credit 在终点后 | **FAIL** |
| negative | 10.554 m | 1.000 | 0.731 | 0.250 m | 无目标超调、无反向 footprint 返访、credit 全在终点前 | PASS |
| long south | 59.465 m | 1.000 | 0.738 | 0.150 m | 无目标超调、无 route 后退、credit 全在终点前 | PASS |

三条的 hard safety、target binding 和 R2 semantics 均通过；唯一 blocker 是 positive 的自然性真实性。权威 verifier 目录：

- `transition_naturalness_verify_positive_takeover_final_20260907T090000Z/`
- `transition_naturalness_verify_negative_takeover_final_20260907T090000Z/`
- `transition_naturalness_verify_south_takeover_final_20260907T090000Z/`

## 6. 地图/查询几何到底说明了什么

冻结的 1 cm position / 360 yaw sampled any-yaw full-footprint projection 共含 `628,125` 个 raw target 子栅格：

| 分类 | 终点前 | 平面上 | 终点后 | 合计 |
|---|---:|---:|---:|---:|
| sampled footprint 不可行 | 101,610 | 48 | 14,630 | 116,288 |
| sampled free、其他分量 | 427,229 | 36 | 33,789 | 461,054 |
| sampled free、起点分量 | **0** | **0** | **50,783** | 50,783 |
| raw target 总计 | 528,839 | 84 | 99,202 | 628,125 |

起点分量内最近的 target 仍位于目标平面后 `4.952305 m`，最远位于后方 `10.471364 m`。

这说明 positive 的语义目标带并非数值上不存在；问题是终点前的目标带在该高分辨率、乐观 any-yaw 投影中要么放不下完整 footprint，要么与起点分量断开，而与起点连通的目标带全部位于请求终点之后。它高度支持“query/语义适用性冲突”，而不是“优化器权重没调好”。

边界必须保留：1 cm center 和 1° yaw 仍是离散采样，且 any-yaw 2D union 对 yaw continuity 是乐观投影；因此该结果不是连续空间不可行性的数学证明，不能据此写 C1。

## 7. 快速算法假设的结果

新的 ordered-corridor probe：

- wall：`8.081895 s`，相对 297.1125 s 探索约快 `36.8x`；
- state：`225`；
- edges：`1,829`；
- complete candidates：`18`；
- peak RSS：`525,414,400 B`；
- 最佳 correct-side：`0.719647`；
- 最佳 target-band：`0`；
- 最佳 error P50：`2.150 m`；
- 结果：`FINITE_ORDERED_GRAPH_NO_R2_WITNESS`。

它证明“请求现场、地图派生、秒级构图”方向可以显著降低生成时间，但当前有限候选族没有 positive witness。由于 station/lateral/yaw 采样和 label 数有界，不能把失败升级成连续空间不可行证明，也不能拿它做在线 B 证据。

## 8. 为什么没有继续在线与 selected8

交接契约明确要求三条真实离线门槛全部成立后才能实施在线接口。当前 positive 失败，因此以下阶段均是：

- online new architecture：`NOT_RUN_OFFLINE_GATE_FAILED`；
- same-round E0 / historical semantic arm / new arm：`NOT_RUN_OFFLINE_GATE_FAILED`；
- exact server ACK：`NOT_RUN_OFFLINE_GATE_FAILED`；
- isolated-process cold latency：`NOT_RUN_OFFLINE_GATE_FAILED`；
- default selected8 warmup/repeated measurements：`NOT_RUN_OFFLINE_GATE_FAILED`。

历史 r3 的 exact ACK/性能通过，不能替代一个尚不存在的新在线架构的 ACK 和时延证据。离线 exact replay 也不是 server effective-content ACK。

## 9. 架构判断与下一步

### 当前是否需要改架构

**现在首先不是架构问题，而是语义适用性定义问题。** 在 positive 的冻结 query/端点/R2 统计下，真正可达且位于终点前的目标带没有被 sampled projection 找到。换成 state-lattice、连续优化或更强 Hybrid A*，都不能合理地把“终点后走 9 m 再回来”变成一次真实 A→B 成功。

因此当前结论是：

- 2A-V2 保持历史 rejected/C，不继续调 soft cost；
- 不立即定义 2A-V3；
- 先冻结一个导师认可的 **semantic applicability / query-validity 规则**。

建议的新规则只改变“某个 soft preference 是否适用于该 query”，不改变安全约束，也不把 raw target 偷偷映射到最近可达格：

1. raw semantic target 原样保留；
2. 以同一 semantic feature instance、完整 footprint 静态可行域为基础；
3. 求 start-directed reachable 与 goal-directed coreachable 的交集；
4. 仅位于请求 start→goal 有向终端平面之前、且存在可重放 forward-only SE(2) chain 的 target 才计为 applicable；
5. 若交集为空，原 positive 保留为负例，soft preference 标 `NOT_APPLICABLE`，不能算通过，也不能删 query；
6. 另行预先冻结一条真实 applicable 的 mirror-positive query，作为三条方向验收中的正例。

这是一项验收含义变更，必须由用户/导师明确批准，不能由实现偷偷完成。用户已批准的 R2 数值阈值无需再问；需要批准的是 **适用性/分母规则**。

批准后，才建议定义新架构（可命名 `2A-V3`）：

- offline：feature-instance + full-footprint + directed reach/coreach 语义走廊；
- online：48-bin forward-only constrained Hybrid/state-lattice，以 path-level semantic resource 为目标，而不是二维逐栅格 soft ridge；
- 输出：可重放 primitive certificate、canonical PathAudit、明确 N/A/失败码；
- 集成：独立 adapter/plugin，不修改 pinned Nav2；
- 验收：先新冻结 triad 3/3，再同轮三臂、exact effective-content ACK、cold latency、selected8 24/24 measured，最后才讨论 B。

如果导师坚持原 positive 必须按现端点、现 raw target 和现自然性同时通过，则下一步不是继续工程扫参，而是做连续 SE(2) interval/branch-and-bound 的不可行性证明；这条路线成本高且短期不利于交付 B。

## 10. 测试、构建与收尾

- focused/new tests：通过；
- `/usr/bin/python3 pytest` 全包（正确 source ROS + workspace）：**508 passed / 0 failed / 92.69 s**；
- 首次环境不完整运行：507 passed / 1 failed，唯一失败为 local Smac launch 找不到 `arena_evaluation`；补 source workspace 后单例通过，已明确排除；
- `compileall`：通过；
- isolated `colcon build --packages-select arena_evaluation`：通过，1 package；仅既有 OMPL/C++ warning；
- build/install/log：`/tmp/pln02_semantic_takeover_build.TKe67h/`，没有覆盖 shared install；
- installed module + CLI `--help`：5/5 通过；冻结 R2 和 targeted YAML 均从 isolated install share 正确解析；
- root/evaluation `git diff --check`：最终复核见收尾记录；
- 本任务残留 semantic/pytest/colcon/ROS/Smac 进程：0；
- PID `773996/774006/774008/774014` 是 2026-08-26 已存在的同一组无关 visualization stack，保持不动；
- 未 reset、clean、checkout、stage、commit 或 push；
- pinned Navigation2 HEAD `656ae8d4c56978efbdd446fe85582f2bcd06e920` 未修改；其已有 dirty `nav2_bringup/launch/navigation_launch.py` 保持不动。

## 11. 本轮精确文件

实现：

- `semantic_path_necessity_audit.py`
- `semantic_transition_naturalness_verify.py`
- `semantic_transition_naturalness_aggregate.py`
- `semantic_transition_goal_plane_topology.py`
- `semantic_transition_goal_plane_postprocess.py`
- `semantic_transition_ordered_corridor.py`
- `semantic_transition_se2_chain_reachability.py`（非判定研究模块，真实运行未启动）
- `config/pudu_wanda_3f_semantic_naturalness_ordered_corridor_r0.yaml`

测试：

- `test_semantic_path_necessity_audit.py`
- `test_semantic_transition_naturalness_verify.py`
- `test_semantic_transition_naturalness_aggregate.py`
- `test_semantic_transition_goal_plane_topology.py`
- `test_semantic_transition_goal_plane_postprocess.py`
- `test_semantic_transition_ordered_corridor.py`
- `test_semantic_transition_se2_chain_reachability.py`

本报告与完整哈希、命令、验证记录：

- `docs/PLN-02_SEMANTIC_STATIC_PLANNER_TAKEOVER_R0.md`
- `private_data/pudu_wanda_3f/results/semantic_static_takeover_r0_final_20260907T090000Z/`

旧目录没有覆盖。`*_takeover_r0_20260907T084000Z` 是本轮在最终 source freeze 前生成的中间目录，已各自标 `EXCLUDED.md`；交接前旧目录仍只读，是否被后续结果替代只在本报告中说明。
