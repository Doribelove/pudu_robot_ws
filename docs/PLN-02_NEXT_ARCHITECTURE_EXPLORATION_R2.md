# PLN-02 下一代语义地图规划架构探索 r2

日期：2026-09-07。基线仍为 `2A-V2`；本轮没有定义新的正式 architecture_id。

## 结论

本轮继续按冻结契约探索 `mirror positive`，最终仍为 **C1，未达到 B**。失败原因已经从“某个搜索器超时”收敛为：完整 padded Jackal footprint 形成的配置空间可行走廊，与主要靠右目标带分离。新的走廊/可见性架构和语义加权变体都生成了安全、精确端点、前向、曲率合规的路径，但没有把路径带入目标侧。

这仍不是连续空间不可行的数学证明。当前证据是带有明确分辨率和 yaw 采样范围的乐观配置空间投影，以及多个独立规划器的失败结果。它足以说明继续扫 2A-V2 soft cost 权重不能合理地把本轮 positive 推到 B，但不足以修改原冻结契约或宣称“所有连续轨迹均无解”。

## 为什么上一轮没有通过

`mirror positive` 的冻结输入为：

- start `[-25.750999, -19.133104, 1.5707963268]`；
- goal `[-25.950999, -10.683104, 2.3561944902]`；
- lane instance `4`；
- 0.05 m 栅格、完整 padded footprint `±0.265 m × ±0.225 m`；
- forward-only、禁止倒车和原地旋转、`Rmin=0.40 m`、最大曲率 `2.50 1/m`；
- exact endpoint、同 lane instance、R0；
- correct-side `≥0.80`、target-band `>0.50`、lateral-error P50 `≤0.50 m`。

上一轮最好的安全可重放路径已经满足碰撞、端点、运动学和重放要求，但只有：

| 指标 | 结果 | 门槛 |
|---|---:|---:|
| correct-side | `0.635449` | `≥0.80` |
| target-band | `0.340557` | `>0.50` |
| lateral P50 | `2.434833 m` | `≤0.50 m` |
| path length | `32.003801 m` | `≤40 m` |
| maximum curvature | `2.493766 1/m` | `≤2.50 1/m` |
| collision / reverse / in-place | `0 / 0 / 0` | `0 / 0 / 0` |

也就是说，安全和运动学不是本轮的失败点，失败点是目标侧语义无法在同一条可行路径上积累到验收比例。

## 本轮新增架构

新增并实际运行了第四类方法：

1. 先把完整 footprint 对障碍的旋转矩形膨胀投影到配置空间；
2. 在任意 yaw 乐观可行投影上生成 medial-axis/加权 A* 走廊；
3. 用 portal 切分走廊；
4. 用精确 Dubins 连接 portal，并统一送入原 `ConstraintWorld` 和 canonical PathAudit。

实现位于 `external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/semantic_corridor_visibility.py`。它不是只做文字比较，而是输出了可重放路径、配置空间投影、portal 证书和逐候选审计。

对 positive 使用 2.5 cm 投影、72/144 yaw 样本，并测试 geometric、target-weight 0.5/1/2/4 和三种 portal stride，共 15 个语义加权候选。结果为：

| 项目 | 结果 |
|---|---:|
| 安全且精确端点的候选 | 14/15 |
| safe candidate 路径长度 | `8.59–21.84 m` |
| correct-side | 全部 `0.0` |
| target-band | 全部 `0.0` |
| lateral P50 | 约 `3.75 m` |
| 最大曲率 | `2.493766 1/m` |
| reverse / in-place | `0 / 0` |

提高语义权重改变了走廊长度，但没有改变可连接的目标区域。对应结果见：

- `private_data/pudu_wanda_3f/results/next_arch_r1_corridor_semantic_positive_20260907T142000Z/`
- `private_data/pudu_wanda_3f/results/next_arch_r1_corridor_visibility_positive_20260907T132000Z/`

## 配置空间转移审计

独立审计 `semantic_positive_transition_audit.py` 在 0.01 m 位置分辨率、5° yaw 采样上，对每个 yaw 的完整矩形 footprint 做精确矩形-栅格相交膨胀，然后取 any-yaw union。结果：

| 项目 | 结果 |
|---|---:|
| 可行投影连通分量 | `16` |
| start / goal 分量 | 均为 `1` |
| start 分量 free subcells | `1,755,927` |
| start 分量 target subcells | `50,770` |
| start 分量 target 占比 | `2.891%` |
| 主 target 分量 | `400,894` subcells，位于 component `2` |
| 主 target bbox 最大跨度 | `52.74 m` |
| start 分量 target bbox 最大跨度 | `5.19 m` |

因此，主要靠右目标走廊不在起终点的 footprint 可行分量内。该投影对 yaw 连续性和曲率是乐观的：如果连乐观 union 都分离，真实 SE(2) 搜索只会更受限；但离散采样仍不能替代连续空间证明。

补充检查去掉 footprint 膨胀、只保留中心点的原始 `master/allowed/lane` 可行域后，多数 target 仍能与起终点中心点分量连通；这说明真正造成分离的关键因素是障碍间距相对于完整车体 footprint 的不足。该结果也解释了为什么中心线 A* 看起来可行，而完整 padded footprint 审核会失败。

结果见：

- `private_data/pudu_wanda_3f/results/positive_transition_audit_20260907T130500Z/transition_audit.json`
- `external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/semantic_positive_transition_audit.py`

## 与前面三类方法的合并判断

| 方法 | positive 结果 | 失败范围 |
|---|---|---|
| 多标签语义资源 state-lattice | 资源上界失败 | 所列 station/lateral/yaw/Dubins 图 |
| target-budget route-conditioned Hybrid A* | 411,863 states、120 s；最佳 side `0.70677`、band `0` | 48-bin 原语和预算内 |
| 连续航点优化 + Dubins replay | 16,844 候选全密集审核；最佳 side `0.635449`、band `0.340557` | 既定控制点参数族 |
| 配置空间 corridor + portal + Dubins | 15 个语义加权候选；side/band 全为 `0` | 2.5 cm 投影和有限 portal stride |

四种方法的搜索假设不同，但都在同一冻结地图、同一端点、同一 footprint 和同一 PathAudit 下复核。没有任何方法得到 positive strict witness，因此按协议不能定义 `2A-V3`，也不能启动在线 adapter、三臂公平对照、selected8 或 exact server ACK。

## 当前判定和后续边界

当前正式判定仍是 **C1**。不能把离线 prototype 叫作在线接口，也不能用历史 72-bin、旧 ACK 或 selected8 结果补齐本轮门禁。

如果坚持旧冻结契约，下一步只能继续做有明确覆盖范围的连续 SE(2) 分支定界或 interval reachability，并把证明对象限定为“给定位置单元、yaw 区间、footprint 外包和曲率上界的可行性范围”。在这类证明完成前，正确表述仍是“强离散证据支持 positive 难以满足”，不是“连续空间已证明无解”。

如果导师允许改验收契约，最小候选是把端点过渡区和没有同侧 footprint 连通 target corridor 的区段标记为语义不适用，只在稳定车道区段统计靠右；安全、footprint、精确端点、yaw、forward-only、曲率和禁行/禁停约束保持不变。该候选必须另起协议，不能回写成旧契约下的 B。

## 验证状态

本轮新增源码已通过 `py_compile`，所有新候选都保存了独立 write-once 目录。旧结果目录未覆盖，工作树既有用户修改保留；未修改飞书、未 stage、commit 或 push。
