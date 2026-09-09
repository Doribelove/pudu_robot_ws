# PLN-02 静态语义全局规划：2A-V3 r13 最终报告

日期：2026-09-08  
`architecture_id: 2A-V3`  
`implementation_revision: r13-route-phase-multisemantic-state-lattice`  
`protocol_id: PLN-02-2A-V3-R13-ROUTE-PHASE-MULTISEMANTIC-ONLINE-V1`

## 结论

**2A-V3 r13 达到本轮冻结契约的真实 B 级研究验收（`B_RESEARCH_ACCEPTANCE`），但不是 A，也不批准直接生产晋升。**

核心结果：

- targeted 真正适用 mirror-positive、原 mirror-negative、long south：离线 3/3，在线 measured 9/9 严格语义、final-valid、R0、exact ACK、安全通过；
- 默认 selected8：E0 24/24、E4-r3 24/24、E5-r13 24/24 final-valid，E5 对 E0 无成功回归；
- E5 selected8 final exact ACK 24/24，hard/soft/stale/hash/sequence mismatch 均为 0；
- E5 selected8 measured P50 3.591 s，E0 1.916 s，比例 1.874×；独立冷进程为 3.582 s / 1.969 s = 1.819×，通过 ≤2.0× 硬门槛；
- 全部成功路径 collision、kinematic、hard semantic、no-stopping goal、reverse、in-place rotation 违规为 0，最大控制曲率 2.494 1/m ≤2.50 1/m；
- selected8 严格语义成功仅 2/8 query、6/24 measured；其余 6 条 query 是明确的安全 soft fallback，绝不计作语义成功。

因此，2A-V2 仍保持历史 C 结论并停止晋升；本轮可保留并继续扩样的是新架构 2A-V3。

## 1. 为什么必须从 2A-V2 变成 2A-V3

2A-V2 r3 已证明二维 guide 离线可行，但原生 48-bin Smac 在线不能稳定兑现。pinned Smac 的公开入口只有 start、goal 与二维 costmap，没有 per-state `(x,y,yaw_bin)` reference。完整证据见 [Smac 接口审计](/home/robot/pudu_robot_ws/docs/PLN-02_2A_V3_SMAC_INTERFACE_AUDIT.md)。

2A-V3 的实质变化不是继续调二维 soft cost，而是新增独立显式 SE(2) planner adapter：

- 48 yaw bins；
- DUBIN、仅前进；
- `Rmin=0.40 m`，实际 primitive radius 0.401 m；
- 禁止倒车和原地旋转；
- 完整 padded Jackal footprint `±0.265 m × ±0.225 m`；
- route station 绑定同一 lane instance、parking component 或必要 transfer phase；
- primitive 逐条读取 Nav2 实际 effective master，并检查 phase、硬语义、footprint 和有向 station；
- 路径最终执行 R2 语义统计、canonical PathAudit、硬语义、ordered progress、revisit/naturalness 审计；
- 结果通过真实 `nav_msgs/Path` 发布并回显验证。

E5 不是原生 Smac，也没有伪装成原生 Smac。E0/E4-r3 仍使用冻结的原生 48-bin Smac，作为公平对照。

## 2. 冻结 semantic applicability / query validity

用户已批准并冻结：

- 路径长度 ≤12 m：全程统计；长路径：两端各排除 6 m；采样 0.025 m；
- lane：correct-side ratio ≥0.80、target-band ratio >0.50、lateral error P50 ≤0.50 m；
- parking：normalized deviation P50 ≤0.25、center-band ratio >0.50；
- 必须同 feature instance、完整 footprint 可行、start 可达、goal 可共达、target 位于有向终点前，并能以相同 forward SE(2) primitive 精确重放；
- 空目标带为 negative/N/A，不能记作 semantic success；
- 禁止用自交、终点后折返、振荡或不必要加长路径换统计通过。

冻结 targeted query hash：`e8bc6434a2a5992ef0bf5e459931c2773b18989415cde7094681980ff9ec37e5`。真正适用 positive 为 `v3-applicable-mirror-positive`，旧 positive 继续保留为历史负证据，不重写历史。

默认 selected8 保持：

- query-set id：`pudu_wanda_3f_semantic_compare_selected8_r2_v2`；
- query hash：`7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e`；
- map hash：`05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`；
- semantic canonical hash：`2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`。

## 3. targeted 结果

冻结离线 fresh-request 复验：

| query | 结果 | planning wall |
|---|---:|---:|
| mirror-positive | 严格通过 | 3.719 s |
| mirror-negative | 严格通过 | 1.508 s |
| long south | 严格通过 | 3.856 s |

在线正式测量（每条 1 warmup + 3 measured）：

| query | E0 lateral P50 | E4-r3 lateral P50 | E5 side / band / lateral P50 | E5 measured |
|---|---:|---:|---:|---:|
| mirror-positive | 2.350 m | 1.100 m | 1.000 / 0.531 / 0.500 m | 3/3 strict |
| mirror-negative | 1.350 m | 0.950 m | 1.000 / 0.522 / 0.500 m | 3/3 strict |
| long south | 1.540 m | 0.534 m | 1.000 / 0.714 / 0.500 m | 3/3 strict |

0.534 m 没有四舍五入成通过。E5 的三个 lateral P50 是浮点实值 0.49999994、0.49999997、0.49999997 m，均按冻结 ≤0.50 m 规则通过。

targeted measured P50：E0 1.661 s，E4-r3 1.852 s，E5-r13 2.887 s，E5/E0=1.738×。冻结前、相同最终参数的独立冷预检 E5/E0=1.401×。

## 4. selected8 同轮三臂

每臂均为独立进程和独立 ROS domain；每条 1 warmup + 3 measured。所有臂使用同一 map/query/L1 policy/endpoint/footprint/48 bins/迭代预算。

| query | E0 | E4-r3 | E5 | E5 semantic | E5 P50 |
|---|---:|---:|---:|---|---:|
| lane-north | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 5.976 s |
| lane-south | 3/3 | 3/3 | 3/3 | 3/3 strict | 2.901 s |
| speed-bump | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 3.748 s |
| multi-junction | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 3.573 s |
| junction-turn | 3/3 | 3/3 | 3/3 | 3/3 strict | 4.458 s |
| lane-to-parking | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 3.595 s |
| parking-to-lane | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 3.288 s |
| parking-internal | 3/3 | 3/3 | 3/3 | 0/3，safe fallback | 3.178 s |

E5 在 selected8 的作用要分成两层理解：

1. **系统级 B 证据**：24/24 final-valid、R0、无 E0 成功回归、安全和 exact ACK 通过；
2. **语义效果证据**：只在 south 与 junction-turn 严格通过。其它 query 的路径可用，但不声称实现对应 lane/parking 偏好。

三个停车 query 仍未达到 approved parking gate；典型值为：

- lane-to-parking：center band 0.354，normalized deviation P50 0.388；
- parking-to-lane：0.324 / 0.515；
- parking-internal：0.092 / 0.598。

这说明下一步的算法重点是 parking phase 的连续 reference trajectory，而不是再增加二维代价。

## 5. exact effective-content ACK

最终 gate：

- targeted measured：9/9 exact ACK；
- selected8 measured：24/24 exact ACK；
- hard exact mismatch=0；
- soft exact mismatch=0；
- stale ROI cells=0；
- hash mismatch=0；
- sequence mismatch=0；
- Path content echo：targeted 9/9、selected8 24/24。

r13 还修复了 Nav2 Humble 的增量 inflation seam：128 KB 小块相邻重叠 32 行，并在全部主块后对每个 seam 及 ROI 上下边界重发 16 行半宽修复带。最终判断仍来自完整 server master 的逐字节读回，不以发布成功代替 ACK。

必须透明说明：正式 selected8 中 3/24、targeted 中 3/9 的第一次 ROI exact ACK 未通过，系统等满冻结 2 s 后执行全图重发；只有重发后的完整 master exact=0 才开始搜索。没有在 mismatch 状态下规划，但该偶发修复是 P95 尾时延来源之一。

## 6. 性能与内存

selected8 measured：

| arm | final-valid | request P50 | P95 | P99 |
|---|---:|---:|---:|---:|
| E0 | 24/24 | 1.916 s | 2.212 s | 2.256 s |
| E4-r3 | 24/24 | 2.252 s | 2.602 s | 2.648 s |
| E5-r13 | 24/24 | 3.591 s | 5.934 s | 6.010 s |

E5/E0 measured P50=1.874×。独立 frozen cold process：E0 1.969 s，E4-r3 2.241 s，E5-r13 3.582 s，E5/E0=1.819×。两者通过 2.0× 硬门槛，但没有达到 1.5×目标。

P95/P99 的每臂有效样本只有 24，全部标为 debug，不能作正式尾延迟声明。E5 静态 prepare 约 10.87 s、Nav2 session start 3.06 s，均独立报告，没有藏进在线 request，也没有用缓存后的 L1 冒充完整首建成本。

E5 进程 peak RSS 约 2.12 GB，与 E4-r3 同数量级、高于 E0 的约 1.54 GB。32 请求中 `ru_maxrss` 在不同大 ROI 首次出现时逐级抬升，不能据此证明长期稳定；本轮没有 soak，因此内存仍是生产化阻碍。

## 7. 安全与自然性

所有正式 E5 路径：

- collision violations=0；
- kinematic violations=0；
- hard semantic violations=0；
- no-stopping goal violations=0；
- reverse distance=0；
- rotate-in-place=0；
- maximum control curvature=2.493765586 1/m；
- ordered route progress 与 revisit screen 通过；
- 未再次出现旧 positive 的自交或“越过终点后沿相邻轨迹返回”。

逐 query 三臂叠图位于结果汇总目录的 `overlays/`；包含不同颜色、start/goal 和方向箭头。

## 8. B 判定与不能晋升生产的原因

自动 gate 文件的 9 个布尔项全部为 true，因此按已批准且冻结的接管契约，结论为 **B_RESEARCH_ACCEPTANCE**。

它仍不能直接生产晋升，原因不是安全失败，而是证据和覆盖边界：

1. selected8 只有 2/8 query 严格语义成功；停车语义仍未解决；
2. 还没有 30–50 条冻结 query 和 ≥100 个有效 measured，P99 无效；
3. E5 是独立 adapter，尚未完成长期 Nav2 GlobalPlanner plugin 生命周期工程；
4. ROI exact ACK 仍偶发需要 2 s 后全图修复；
5. peak RSS 约 2.12 GB，未做长时间内存稳定性试验；
6. measured 与 cold P50 通过 2×，但未达到 1.5×目标。

所以本轮是“可以进入独立扩样/工程化”的 B，不是 A，不是生产可用声明。

## 9. 下一步

优先级如下：

1. 把 parking phase 从稀疏 station 候选改成连续、曲率可行的中心 reference trajectory，并保持同 component 与 R2 统计；
2. 把 E0 Smac 作为不可适用 query 的明确生产 fallback，避免为了“统一 E5”强行使用无语义收益的 state-lattice；
3. 将 exact ROI 更新改成单次确定性 reinflation/reset 协议，消除 2 s timeout 后 full repair；
4. 将 route phase graph、canonical distance field 与 immutable semantic geometry 做有界缓存，降低 10.87 s 首建和约 2.12 GB peak RSS；
5. 冻结 30–50 query，分正反向、junction、parking、forbidden、unlabelled、narrow；每条 warmup + ≥5 paired，达到 ≥100 measured 后再评价 P99；
6. 通过扩样、尾延迟与生命周期门槛后，才讨论注册为 Nav2 plugin 和生产晋升。

## 10. 权威结果目录

- 自动同轮汇总：`private_data/pudu_wanda_3f/results/2a_v3_r13_frozen_paired_report_v3_20260908T052000Z`
- 当前冻结离线 targeted：`.../2a_v3_r13_frozen_targeted_offline_formal_v3_20260908T044000Z`
- targeted E0/E4/E5：
  - `.../2a_v3_r13_frozen_targeted_e0_formal_v2_20260908T014000Z`
  - `.../2a_v3_r13_frozen_targeted_e4_formal_20260908T015000Z`
  - `.../2a_v3_r13_frozen_targeted_e5_formal_v2_20260908T011000Z`
- selected8 E0/E4/E5：
  - `.../2a_v3_r13_frozen_selected8_e0_formal_20260908T025000Z`
  - `.../2a_v3_r13_frozen_selected8_e4_formal_20260908T031000Z`
  - `.../2a_v3_r13_frozen_selected8_e5_formal_20260908T021000Z`
- frozen cold E0/E4/E5：
  - `.../2a_v3_r13_frozen_selected8_e0_cold_20260908T035000Z`
  - `.../2a_v3_r13_frozen_selected8_e4_cold_20260908T040000Z`
  - `.../2a_v3_r13_frozen_selected8_e5_cold_20260908T034000Z`

权威机器可读结论：`same_round_comparison.json` SHA-256 `a400e71b76ca72046892bec8ed5f42af827536a250f4e4f1dbb7d5e408690c48`。

## 11. 验证与仓库状态

- `/usr/bin/python3 pytest`：549 passed，6 warnings；
- `compileall`：通过；
- r13 offline/online/report 三个 CLI `--help`：通过；
- 隔离 `colcon build --packages-select arena_evaluation`：通过；
- 主仓与 evaluation 嵌套仓 `git diff --check`：通过；
- 本任务 ROS/Nav2/Smac 残留进程：0；8 月 26 日已有 visualization 进程保持未动；
- 未 stage、commit、push；未 reset、clean、checkout；未修改 pinned Nav2；未修改飞书。

主仓起点：branch `codex/pln-02-2a-v2-3d-v1-r1`，HEAD `ed25b1767976ecb48086bfa429b2b5a3a49d7226`。evaluation 嵌套 HEAD `94762429bea19b84cab50a3d0910a736184738a0`。历史 `real_ab_r0_run12/runs.csv` SHA-256 `48069a641e2984a5c405d8f966671713b12a4cec522515203a00f419ab2d37ad`，保持只读。

排除目录均保留 `EXCLUDED.json`。其中包括首次 selector 错误、三次在线 ROI/master 接口校准失败、一次缺 ROS 环境、一次 CLI 参数错误、一次 YAML 时间序列化错误和一次输入路径错误；它们未计入正式统计。
