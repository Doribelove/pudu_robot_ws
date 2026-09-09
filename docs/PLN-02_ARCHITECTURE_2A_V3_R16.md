# PLN-02 Architecture 2A-V3 r16

## 结论先行

`2A-V3` 的真实状态是：**r13 已达到研究型 B（B_RESEARCH_ACCEPTANCE），r14/r15 的 C 是更严格的工程化压力门槛失败，并不撤销 r13 的 B。当前仍不是 A，也不批准生产晋升。**

r16 验证了用户对“验收是否过严”的怀疑中合理的部分：安全、运动学、exact ACK 和已批准的数值线并不应降低；错误在于 r15 把 3 条未经语义适用性验证的 parking 压力查询全部纳入 strict-witness 分母。新的地图/路线几何审计表明，这 3 条以及独立 held-out 的另外 9 条 parking 查询都不满足 E5 parking 语义适用前提，应由 E0 native Smac 安全回退，且回退不能计作语义成功。

## 版本与冻结边界

- `architecture_id: 2A-V3`
- `implementation_revision: r16-route-local-semantic-applicability`
- `protocol_id: PLN-02-2A-V3-R16-ROUTE-LOCAL-APPLICABILITY-V1`
- 主仓 HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- evaluation 嵌套仓 HEAD：`94762429bea19b84cab50a3d0910a736184738a0`
- pinned Nav2 HEAD：`656ae8d4c56978efbdd446fe85582f2bcd06e920`
- pinned Nav2 既有 dirty 文件 `nav2_bringup/launch/navigation_launch.py` 未修改。
- r0–r15 源码和结果保持只读；没有 reset、clean、stage、commit 或 push。

## 为什么 r13 已经是 B

冻结 r13 的权威事实保持不变：

- targeted 离线 3/3、在线 9/9；
- selected8 的 E0、E4-r3、E5-r13 均为 24/24 final-valid；
- E5 selected8 最终 exact ACK 24/24，hard/soft/stale/hash/sequence mismatch 均为 0；
- E5/E0 measured P50 为 1.874×，独立冷进程为 1.819×，通过 B 的 ≤2.0×门槛；
- collision、footprint、运动学、硬语义、no-stopping、倒车、原地旋转约束全部保持；
- selected8 的严格语义成功是 2/8 query、6/24 measured，其余为明确且不计语义成功的安全 soft fallback。

所以 B 的含义是“可运行、可复验、无安全回归的研究候选”，不是“所有语义类都已解决”，更不是生产 A。

## r16 没有降低什么

r16 未修改：

- parking normalized deviation P50 ≤0.25；
- parking 中心带占比 >50%；
- 长路径每端排除 6 m、短路径全程统计；
- 48-bin、forward-only DUBIN、禁止倒车和原地旋转；
- Rmin 0.40 m、最大曲率 2.50 1/m；
- full Jackal footprint、静态/硬语义/no-stopping 等安全门槛。

新增的是一个在规划前执行的必要条件：对冻结 L1 路线的每个 active parking station，只在与该路线相连的局部可通行横截面内检查是否存在满足原 0.25 目标且具有 footprint 净空的单元。只有超过 50% 的 active parking stations 存在这种目标，E5 parking 语义才适用。分类只读地图、语义、路线和 footprint，不读取历史 witness，也不使用规划成败倒推适用性。

## 校准 3-query 结果

| query | active parking stations | 局部目标可用 | 比例 | 冻结分类 |
|---|---:|---:|---:|---|
| x32-21 lane-to-parking | 94 | 42 | 0.447 | E0 fallback |
| x32-25 parking-to-lane | 54 | 19 | 0.352 | E0 fallback |
| x32-29 parking-internal | 75 | 15 | 0.200 | E0 fallback |

三条都低于原始的 >0.50，不是 0.49 被四舍五入之类的边界问题。校准总墙钟 5.20 s，静态缓存准备 0.591 s，peak RSS 1.556 GB。

## 独立 held-out 9-query 结果

规则和阈值在运行 held-out 前已写入独立 YAML，held-out 不包含上述 3 条：

- x32-22：0.459，route-local target 不适用；
- x32-28：0.038，route-local target 不适用；
- x32-32：0.245，route-local target 不适用；
- x32-23、x32-24、x32-26、x32-27、x32-30、x32-31：冻结语义 L1 route 的 endpoint attachment 超过 0.75 m，E5 route binding 不适用；
- 9/9 均明确调度 E0 native Smac fallback，fallback 语义成功计数为 false。

held-out 总墙钟 11.43 s，静态缓存准备 0.601 s，peak RSS 1.563 GB。第一次 held-out 尝试在发现 endpoint binding 错误时整体 fail-closed；该目录保留并标记 EXCLUDED。runner 随后改为逐 query 记录结构化失败并继续，重试使用新目录。

## 被快速淘汰的搜索假设

r16 还验证了“先 replay 整条 Dubins primitive，再按整条边的语义覆盖排序”的独立搜索臂。第一条查询的最好 parking 中心带覆盖仅由约 0.333 提升到 0.344，仍远低于 0.50；该假设按硬停止规则淘汰，未浪费时间跑后两条，也未启动 ROS 大实验。

这说明继续增加边数或调 soft 权重不是合适方向。根因在 parking 语义表示：同一个大连通 parking polygon 包含宽度不同、受墙体分隔的多条局部通道，而当前 deviation 用整个连通域的最大净空归一化。邻近宽通道会抬高分母，使当前窄通道的真实局部中心仍被记为 >0.25。

## 下一步最合适的架构工作

保持 2A-V3，不需要因为这次发现立刻改名为新架构。下一 revision 应只做一个实质变化：把 parking semantic instance 从“大连通 polygon”分解为由静态障碍和 medial branch 定义的 route-local aisle instance，并在 aisle 内构建连续、可达、曲率可行的中心 reference trajectory。数值门槛仍保持 0.25 / >50%，但归一化对象从混合多通道的整块 polygon 变为真实局部 aisle；这属于语义表示修正，需要在正式 held-out 前冻结并单独留痕。

同时保持：

1. 不适用查询使用 E0 native Smac，绝不冒充语义成功；
2. exact ACK 继续 fail-closed。r15 在第 148 次 transition 的 GetCostmap 超时说明无修复 ACK 生命周期尚未达生产门槛；
3. 先用 3 条校准 + 9 条 held-out 的秒级 applicability gate，再启动在线/扩样；
4. 只有 parking aisle 语义在新冻结适用查询上通过、ACK stress 和 30–50 query 内存/尾延迟通过后，才讨论 A/production plugin。

## 验证与产物

- r13–r16 focused pytest：42 passed；
- r16 focused pytest：7 passed；
- compileall：passed；
- isolated colcon：`arena_evaluation_msgs`、`arena_evaluation` passed；
- CLI `--help`：passed；
- `git diff --check`：passed；
- r16 残留进程：0；审计到的 PID 3539521 属于既有 `2a_v1_r3_reachable_endpoint` 任务，未终止。

权威新结果：

- calibration3：`private_data/pudu_wanda_3f/results/2a_v3_r16_route_local_applicability_v1_20260908T231500Z`
- held-out9：`private_data/pudu_wanda_3f/results/2a_v3_r16_route_local_applicability_heldout9_v2_20260908T234000Z`
- excluded first held-out：`private_data/pudu_wanda_3f/results/2a_v3_r16_route_local_applicability_heldout9_v1_20260908T233000Z`

最终判定：**B_RESEARCH_ACCEPTANCE_PRESERVED；R16_APPLICABILITY_FIX_VALIDATED；NOT_A；NOT_PRODUCTION_PROMOTED。**
