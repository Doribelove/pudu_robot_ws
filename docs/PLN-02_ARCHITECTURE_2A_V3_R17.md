# PLN-02 Architecture 2A-V3 r17

## 结论先行

`2A-V3 r17-parking-aisle-medial-reference` 完成了 route-local parking aisle 表示和连续中心 reference 的离线实现，但冻结的三查询 gate 只通过 2/3，因此 **r17 不进入在线 ROS、selected8 或 expanded32，不能替代当前 r13 的研究型 B 基线，也不能生产晋升**。

r17 的失败不是数值线过严：前两条查询在新 aisle 指标下远高于门槛；第三条的 aisle field 和几何 reference 也通过，但多 phase 的 SE(2) 搜索在冻结的 12,000-label 资源上限内没有到达终点。后续两个有针对性的连接诊断同样失败，已按快速迭代规则停止。

当前总状态仍为：**r13 B_RESEARCH_ACCEPTANCE 保持；r17 是部分有效、离线 gate 失败的研究 revision；2A-V3 仍不是 A/生产版本。**

## 版本与冻结边界

- `architecture_id: 2A-V3`
- `implementation_revision: r17-parking-aisle-medial-reference`
- `protocol_id: PLN-02-2A-V3-R17-PARKING-AISLE-V1`
- 主仓 HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- evaluation 嵌套仓 HEAD：`94762429bea19b84cab50a3d0910a736184738a0`
- pinned Nav2 HEAD：`656ae8d4c56978efbdd446fe85582f2bcd06e920`
- pinned Nav2 和 r0–r16 权威结果未修改；没有 reset、clean、stage、commit 或 push。

冻结输入：

- map hash：`05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`
- semantic map hash：`2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`
- expanded32 query hash：`42464f9e99d3a017e63dde3c9559944698deccf732b2d03a0f56110d814dfd02`
- r17 config hash：`dc99fe3a87e25871b3b151dab1dea757b3f5f9c27b8200353e9405742c7fc64c`

## r17 实现了什么

r17 没有继续调二维 soft cost。它在冻结 L1 route 的每个 parking station 上构造障碍分隔的横截面，只保留与 route seed 连通、属于同一 parking component、满足 footprint/master hard 条件的单元；该横截面的局部最大组合净空用于构造 aisle-relative deviation。不同 station 按纵向最近关系确定所有权，避免相邻更宽停车通道抬高当前通道的归一化分母。

该 field 只作为 guide：实际 witness 仍由 48-bin、forward-only DUBIN SE(2) 搜索生成，并继续执行完整 footprint、有效 master、硬语义、曲率、精确端点、控制重放、路线单调性与无往返审计。Rmin 0.40 m、最大曲率 2.50 1/m、禁止倒车和原地旋转均未改变。

非常重要：已批准 R2 的 parking 指标按整个 component 最大净空归一化，r17 没有悄悄改写它。新 field 的指标单独命名为 `parking-route-local-aisle-normalized-r3-research`；旧 R2 component 指标并列报告。若未来要将 aisle 指标作为正式 B/A 验收口径，必须先单独批准并冻结契约。

## 冻结三查询结果

| query | field 目标可用率 | reference 目标率 | aisle 中心带 | aisle P50 | 旧 R2 中心带 | 旧 R2 P50 | 曲率 | 结果 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| x32-21 lane-to-parking | 1.000 | 0.841 | 0.905 | 0.053 | 0.290 | 0.410 | 2.494 | strict witness 通过 |
| x32-25 parking-to-lane | 1.000 | 0.800 | 0.856 | 0.098 | 0.244 | 0.649 | 2.494 | strict witness 通过 |
| x32-29 parking-internal | 1.000 | 0.863 | N/A | N/A | N/A | N/A | N/A | 无 SE(2) candidate |

前两条同时满足：aisle normalized deviation P50 ≤0.25、中心带占比 >50%、canonical final-valid、full-footprint collision-free、hard semantic、no-stopping endpoint、精确端点、ordered progress、无 geometric revisit 和 exact primitive replay。它们的弧长分别为 56.143 m 和 57.421 m，均低于绑定路线的长度上限。

第三条不是 field 或 reference 失败：field station target availability 为 100%，reference target ratio 为 86.3%。失败发生在 route-phase SE(2) 搜索：116 个 layer、2,953 个 state，扩展达到冻结上限 12,000 labels，生成 59,429 labels，最远到第 114 层但没有 goal candidate；搜索分解出了 7 个交替的 parking/lane phase runs。

## 为什么不能把 2/3 写成通过

第一，冻结 gate 明确要求三条都存在在线前置的离线 strict witness，不能事后删掉 parking-internal。第二，新 aisle 指标与旧 R2 component 指标差异很大：前两条在 aisle 指标通过，但在旧 R2 下中心带只有 0.290/0.244、P50 为 0.410/0.649。因此本结果证明“局部 aisle 表示有工程信号”，不证明旧 R2 合同下三查询已通过。

第三，r16 证明 planning failure 不能反推语义不适用。虽然 E0 fallback 仍是非适用查询的正确行为，但 x32-29 不能仅因为此次搜索失败就在事后改分类；需要独立、规划前冻结的 route/semantic applicability 规则。

## 快速根因诊断与停止理由

冻结 r17 失败后只执行了三个单查询、明确排除的机制诊断：

1. r16 whole-primitive ranking：仍在 12,000 labels 停止，生成 140,836 labels，耗时 46.08 s，没有 candidate；明显更差。
2. 终点 yaw 的 1.20 m 切线调理：证明终点局部 Dubins 边存在，但搜索仍在 12,000 labels 停止，最远第 112 层，耗时 4.03 s。
3. adjacent-first 的空层/semantic-boundary 单层桥接：两个变体分别在第 83 层停止，耗时 1.89/1.96 s，没有 candidate。

这些结果排除了“简单扩大一次连接”这一快速修复。继续扫 station skip、连接长度、label budget 或 soft 权重既违反本轮冻结，也可能重新引入 r17 之前已经观察到的 hairpin shortcut/self-cross，因此按 hard stop 停止。

## 性能与未启动阶段

- 静态准备：596.4 ms；三查询离线总墙钟：14.47 s。
- query 墙钟：2.761 s、2.694 s、7.442 s。
- peak RSS：1.641 GB；本轮尚未解决长期内存生命周期门槛。
- 在线 ROS/E0-E5 公平对照：`NOT_RUN_OFFLINE_THREE_QUERY_GATE_FAILED`。
- selected8：`NOT_RUN_OFFLINE_THREE_QUERY_GATE_FAILED`。
- expanded32 和 30–50 query：`NOT_RUN_OFFLINE_THREE_QUERY_GATE_FAILED`。
- exact server ACK/冷时延：未进入在线阶段，不能沿用旧结果替代本轮证据。

## 下一步

下一 revision 不应再调 aisle 权重或连接半径。最有价值的方向是把当前“每层少量采样 + 多资源 Pareto labels”的接口替换为一个目标可达性优先的全局 SE(2) route-corridor search：终点 yaw 作为构图边界条件，先做反向 goal-coreachability 剪枝，再在可达子图内优化 lane/parking 资源；同时在搜索前冻结多 phase route 的语义适用性，确属不适用时明确调度 E0 且不计语义成功。

仍采用快速 gate：先只跑 x32-29；若不能在不增加 12,000-label 预算、不改变端点/地图/安全合同下产生 strict witness，就停止该搜索假设。通过后才复跑本次 3-query，随后才允许在线 targeted。这样一次失败迭代仍控制在分钟级，不回到 4–5 小时。

## 验证和权威产物

- `/usr/bin/python3` 完整 pytest：737 passed，6 个既有 warning；通过工作区已配置的 Python 3.10 dependency path 提供 `scikit-image`，未安装或修改系统环境。
- r13–r17 focused pytest：49 passed；r17/r18 新增测试 11 passed。
- compileall：passed；root/evaluation `git diff --check`：passed。
- isolated colcon：`arena_evaluation_msgs`、`arena_evaluation` passed，目录 `/tmp/2a_v3_r17_colcon_sOrpKM`。前三次构建因工作区依赖发现范围错误而失败，保留目录 `/tmp/2a_v3_r17_colcon_ICRN7h`、`/tmp/2a_v3_r17_colcon_qb3OEs`、`/tmp/2a_v3_r17_colcon_LtNxlQ`；第四次用显式 package base paths 后通过。
- r17 CLI `--help`：passed。
- r17/r18 本轮残留进程：0。既有 PID 3539521（`2a_v1_r3_reachable_endpoint` lifecycle manager）和 ROS domain 230 daemon 未终止、未接管；本轮未启动任何 3D-V1 工作。

本轮精确源码/文档改动：

- `docs/PLN-02_ARCHITECTURE_2A_V3_R17.md`
- `arena_evaluation/semantic_parking_aisle_r17.py`
- `arena_evaluation/two_layer_v3_semantic_r17_parking_aisle.py`
- `arena_evaluation/semantic_route_phase_r18.py`（仅对应已排除的后门控诊断）
- `config/two_layer_v3_semantic_r17_parking_aisle.yaml`
- `test/test_two_layer_v3_semantic_r17.py`
- `test/test_semantic_route_phase_r18.py`
- `setup.py` 仅增加 r17 CLI entry point；原有其他 dirty 内容保留。

权威 r17 gate：

- `private_data/pudu_wanda_3f/results/2a_v3_r17_parking_aisle_offline3_v1_20260908T111300Z`

排除诊断：

- `private_data/pudu_wanda_3f/results/2a_v3_r17_parking_internal_whole_primitive_diag_v1_20260908T111800Z`
- `private_data/pudu_wanda_3f/results/2a_v3_r17_parking_internal_terminal_pose_diag_v1_20260908T112500Z`
- `private_data/pudu_wanda_3f/results/2a_v3_r18_parking_internal_gap_bridge_diag_v1_20260908T113500Z`
- `private_data/pudu_wanda_3f/results/2a_v3_r18_parking_internal_boundary_bridge_diag_v2_20260908T114500Z`

最终判定：**R17_OFFLINE_GATE_FAILED_2_OF_3；ONLINE_AND_EXPANSION_NOT_RUN；R13_B_RESEARCH_ACCEPTANCE_PRESERVED；NOT_A；NOT_PRODUCTION_PROMOTED。**
