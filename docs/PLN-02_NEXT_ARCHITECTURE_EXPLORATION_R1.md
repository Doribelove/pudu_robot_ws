# PLN-02 下一代语义规划架构探索 r1

日期：2026-09-07。协议：`PLN-02-NEXT-ARCHITECTURE-EXPLORATION-R1-V1`。

## 结论

本轮最终判定仍为 **C1，未达到 B**。`mirror positive` 在三种本质不同且可运行的新原型中仍没有严格见证，因此按门禁没有定义 `2A-V3`，也没有启动在线 adapter、同轮 48-bin 三臂、selected8、服务端 exact ACK 或 cold latency 实验。

这不是因单次超时停止：资源约束 lattice 给出一个明确有限图内的资源上界失败；target-budget Hybrid A* 展开 411,863 个 SE(2) 状态；连续控制全局优化对 16,844 条候选逐条做 6 mm 密集审核，并产出一条完整安全、精确端点、确定性可重放的路径，但该路径仍同时违反三项语义指标。

当前 targeted 离线结果为 2/3：negative 与 south 沿用前一轮已独立重放通过的严格见证，positive 不通过。旧冻结契约尚未被证明在连续空间数学不可行，不能伪造 B，也不能未经用户同意更改契约。

## 冻结输入与边界

正式 Stage 0：`private_data/pudu_wanda_3f/results/next_arch_r1_stage0_20260907T113300Z`。

| 项目 | 值 |
|---|---|
| root HEAD | `ed25b1767976ecb48086bfa429b2b5a3a49d7226` |
| evaluation HEAD | `94762429bea19b84cab50a3d0910a736184738a0` |
| 地图 SHA-256 | `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a` |
| 语义地图 SHA-256 | `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553` |
| positive 输入 NPZ | `9275e38b26c3f8c9958f769297f794c00dfd9d4cfdd921108e74bb356e5b1685` |
| positive 起点 | `[-25.750999, -19.133104, 1.5707963268]` |
| positive 终点 | `[-25.950999, -10.683104, 2.3561944902]` |
| lane instance | `4` |

保持 0.05 m 原图、forward-only、禁止倒车和原地旋转、`Rmin=0.40 m`、最大曲率 `2.50 1/m`、完整 padded footprint `±0.265 × ±0.225 m`、精确端点、同 lane instance、R0 和 no-stopping。旧 constrained-feasibility 的 `40 m` positive 研究上限继续保留；没有利用新任务清单未重复列出该上限来改变旧合同。

## 三种候选架构

| 候选 | 路径级语义表达 | positive 实测 | 证明/失败范围 | 在线与集成判断 |
|---|---|---|---|---|
| 多标签资源 state-lattice | 每个标签精确保留长度、`5C-4N`、`2B-N`，同顶点做 Pareto 支配 | 3,744 节点、111,841 边；15.95 s；T 后缀上界为 `-343`，未展开即失败 | 只证明所列 station/lateral/yaw/Dubins DAG 内无合格路径 | 最可解释，能输出控制链并复用 canonical PathAudit；图构建和标签量不适合直接全程在线 |
| target-budget route-conditioned Hybrid A* | 原语代价显式惩罚 measured target-band debt、wrong-side 与 master cost，终点检查完整路径指标 | 120.11 s，展开 411,863 状态；最优拒绝候选 side `0.70677`、band `0`、P50 `2.55 m` | 48-bin、给定运动原语和时间预算内无 witness；不是连续空间证明 | 最接近 Nav2/Smac 接口，但长距离资源历史不能仅靠单状态最小 cost 表达，在线预算风险最高 |
| 连续 `(x,y,yaw)` 控制点全局优化 + 解析 Dubins | 直接以完整路径 side、band、P50、安全和长度作为目标/罚项 | 16,844 候选全部密集审核；120.59 s；找到 32.00 m 安全可重放路径，但 side `0.63545`、band `0.34056`、P50 `2.43483 m` | 只覆盖冻结参数族和 seed；不是全局最优或不可行证明 | 适合离线生成/平滑 reference，不适合作为单独的实时全局规划器 |

三者都有可运行代码和 write-once 数据，不是文字比较。所有 lattice/Hybrid 边和连续优化的全部 16,844 条候选均做密集 padded-footprint 检查；最终候选统一回到未放宽的 `ConstraintWorld.audit`：≤0.025 m、≤1° 密集 footprint，canonical PathAudit、精确端点、路径级语义和控制重放共同决定 gate。没有重复点、异常采样密度或无意义闭环；连续优化结果的倒车距离和原地旋转均为 0，最大曲率 `2.493765586 1/m`。

## 几何证据

新增 1 cm 位置、1° yaw 的完整矩形 footprint 穷举投影。每个 yaw 都单独对 lethal 障碍做旋转矩形相交，随后取 `any-yaw free` 并做 8 连通分量，因此二维投影对 yaw 连续性是乐观的。

结果为 11 个连通分量。起终点均在分量 1；分量 1 中有 50,783 个 target 子格，而最大的 target 分量有 401,185 个子格，位于分量 2，无法从起终点侧进入。位置采样覆盖半径为 7.07 mm，yaw 半档误差为 0.5°。该结果表明主目标带的分离不是 5 cm/48-bin 的明显量化伪影。

![1 cm / 1° 可行域与目标带](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/next_arch_r1_delivery_final_20260907T120300Z/positive_topology_1cm_1deg.png)

黄色/蓝灰区域为起终点侧 any-yaw 可行分量，绿色为 target band；顶部小片 target 可达，右侧长目标带被障碍带隔开。该图仍是离散覆盖，不足以声明连续空间不可行。

连续优化的最佳安全拒绝路径如下。它从西侧向北绕行进入唯一可达的北部目标区，再回到冻结终点；安全和运动学通过，但积累不到多数 target-band 路程。

![positive 最佳安全拒绝路径](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/next_arch_r1_delivery_final_20260907T120300Z/positive_best_safe_overlay.png)

## Targeted 门禁

| 查询 | 严格 witness | correct-side | P50 / m | target-band | 路径 / m |
|---|---:|---:|---:|---:|---:|
| `r3-mirror-1-positive` | 否 | 0.635449 | 2.434833 | 0.340557 | 32.003801，安全拒绝路径 |
| `r3-mirror-2-negative` | 是 | 1.0 | 0.250000 | 0.730679 | 10.553371 |
| `cmp2-02-lane-south` | 是 | 1.0 | 0.250000 | 0.625973 | 59.464220 |

positive 的三个要求分别为 side≥0.80、P50≤0.50 m、band>0.50，当前三项都失败。negative/south 见证的碰撞、硬语义、no-stopping、倒车和原地旋转违规均为 0，并通过完整 padded footprint、精确端点、R0 和确定性重放。

因此后续项目按协议全部标为 `NOT_RUN_OFFLINE_GATE_FAILED`：不能把离线原型冒充在线接口，也不能用历史 r3/r4 的 ACK、72-bin 在线结果或 selected8 结果补齐本轮门禁。

## 架构选择

本轮**不正式选择或命名 V3**。若旧契约将来获得 positive witness，推荐的架构方向是：离线/地图级多标签资源 corridor 生成器 + 在线受 corridor 约束的 Hybrid A* + 可选连续轨迹平滑。理由是只有多标签资源模型能无损保留路径级 side/band 历史；Hybrid 负责在线运动学与 Nav2 接口；连续优化只做有严格回退的后处理。exact effective-content ACK 与 canonical PathAudit 可直接复用 2A-V2 r2/r4 的绑定，不需要修改 pinned Nav2 核心，也不应以二维 costmap 冒充 SE(2) 接口。

当前不能继续做在线实现，因为 hardest-case 已说明问题首先在查询契约与可行走廊，而不是 adapter。继续扫 cost 权重只会增加搜索量，不能让分离的主目标带变得可达。

## 两条后续路径

保持旧契约：继续做带区间界的连续 SE(2) 分支定界，至少要对位置单元、yaw 区间、矩形 footprint 和曲率可达集给出保守外包；只有覆盖完整 `≤40 m` 简单路径类后，才可能把“未找到”提升为明确范围内不可行。当前 r1 没有做到这一步。

采用新契约候选：须先由用户/导师批准并另起协议。最小候选是将“端点过渡区和没有同侧 footprint 连通 target corridor 的区段”标为语义不适用，仅在稳定车道区段统计靠右；安全、footprint、端点、yaw、forward-only、曲率和禁行/禁停不变。另一个候选是重新冻结 positive 端点/yaw，使其与主目标带位于同一可行分量。两者都会改变验收含义，不能拿来回写旧结果或直接宣称 B。

## 验证

- 新旧全包最终复验：`387 passed in 83.92s`；新增聚焦测试与 constrained 测试：`10 passed`。
- 隔离 Release `colcon build arena_evaluation`：通过，4.05 s；只有系统 OMPL 头文件和既有扩展的编译警告。
- `compileall`、安装后 `semantic_architecture_explorer --help`、root/evaluation `git diff --check`：全部通过。
- 本轮离线进程已退出；没有残留新的 ROS/Nav2/Smac 进程。2026-08-26 的旧 visualization 栈保持不动。
- 未修改 pinned Nav2 源码，未 stage、commit、push、reset、clean 或覆盖旧结果；未修改飞书。

第一次全包验证因未 source ROS 而在 `nav_msgs` 收集阶段失败；第二次 source ROS 但未加载 workspace overlay，唯一 Smac 集成测试找不到 `arena_evaluation`。两次均保留为排除验证。加载完整 ROS + workspace 环境的第三次权威验证为 387/387。

## 权威产物

- 汇总：`private_data/pudu_wanda_3f/results/next_arch_r1_delivery_final_20260907T120300Z`
- Stage 0：`private_data/pudu_wanda_3f/results/next_arch_r1_stage0_20260907T113300Z`
- 1 cm/1° 拓扑：`private_data/pudu_wanda_3f/results/next_arch_r1_topology_positive_1cm_20260907T113000Z`
- 多标签 lattice：`private_data/pudu_wanda_3f/results/next_arch_r1_lattice_positive_20260907T113400Z`
- target-budget Hybrid：`private_data/pudu_wanda_3f/results/next_arch_r1_target_budget_hybrid_positive_20260907T114300Z`
- 连续优化：`private_data/pudu_wanda_3f/results/next_arch_r1_optimize_dense_all_positive_20260907T120000Z`
- 权威验证：`private_data/pudu_wanda_3f/results/next_arch_r1_verification_delivery_final_20260907T121300Z`
- 旧 negative/south 证据：`private_data/pudu_wanda_3f/results/constraint_stage1_delivery_final_20260907T030500Z`

`directory_roles.json` 明确列出五个不完整或被替代的目录；旧实验目录全部只读保留。汇总目录包含 `candidate_comparison.csv/json`、`per_query.csv/json`、`gate_results.json`、图件、复现命令和 artifact hashes。

最终判定：**C1，不定义 2A-V3，不达到 B，不可宣称生产可用。**
