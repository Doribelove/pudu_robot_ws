# PLN-02 语义端点过渡区契约 R1

日期：2026-09-07  
协议：`PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R1-V1`  
架构：`2A-V2`  
契约修订：`semantic-endpoint-transition-6m-r1`

当前复验状态（2026-09-07）：固定 6 m 契约下的三条离线 witness 为 **2/3，C1**；negative 长 10.553 m，按下述 `L>12 m` 规则为 `INVALID_CONTRACT_METRIC`。不得用其原契约通过替代本契约通过。在线与 selected8 尚未运行，详见 [统一契约复验及更正](PLN-02_SEMANTIC_TRANSITION_PREFLIGHT_R1.md)。本文规范性门槛保持不变。

## 1. 定位

本文件定义一个独立的语义统计契约候选。它保留 `2A-V2` 的架构编号，因为本轮改变的是靠右/靠中软偏好的适用区，不是规划器的层级、搜索器或 ROS 接口。旧的无过渡区契约继续只读保留，历史 C1 结论不被改写。

本契约的依据是 `PLN-02_NEXT_STAGE_CONTRACT_SENSITIVITY_R0`：在同一条安全路径上，从两端各排除 6 m 后，`mirror positive` 的靠右指标达到 `correct-side=0.856079`、`target-band=0.545906`、横向误差 P50 `0.387184 m`；排除长度从 0–8 m 扫描，6 m 是第一个同时通过候选门槛的长度。该结果只证明契约候选可行，尚未证明在线规划或 selected8 通过。

## 2. 语义采样定义

对一条已经通过完整 PathAudit 的路径，按路径弧长建立累计坐标 `s∈[0,L]`。使用既有语义审计的确定性采样规则：平移采样间隔不大于 `0.025 m`，相邻 yaw 插值不超过 `1°`。

当且仅当 `L > 12 m` 时，软语义统计区间为：

```text
s ∈ [6 m, L - 6 m]
```

两端窗口长度相等，按弧长计算，不按输入点数量计算。规划器不能通过重复点、删除点、改变采样密度或异常绕圈改变统计结果。报告必须同时保存完整路径指标和窗口内指标、两端排除长度、有效样本数及弧长范围。

窗口只影响软偏好指标：

- lane：`correct-side`、`target-band` 和 lateral error P50；
- parking_area：靠中偏差及其既有中心带指标。

每一种语义类别都使用该类别在活动区间内的样本作为自己的分母。`unlabelled`、`junction` 和过渡区样本单独报告，不能充当语义成功样本。固定采样后等弧长样本等权，避免通过点密度操纵比例。路径长度不超过 12 m，或活动区间内不存在适用语义样本时，该契约指标标记为 `INVALID_CONTRACT_METRIC`，不得记为通过。

## 3. 全路径安全边界

端点过渡区不是安全豁免区。以下检查始终覆盖完整 `[0,L]`：完整 padded Jackal footprint、未知区和障碍碰撞、禁行/禁停硬语义、显式方向规则、精确 start/goal x/y/yaw、forward-only、禁止倒车、禁止原地旋转、`Rmin=0.40 m`、最大曲率 `2.50 1/m`、R0 和 canonical PathAudit。

因此，靠右或靠中指标在窗口外未统计，不代表窗口外允许穿越障碍、逆向、倒车、原地旋转、停在禁停终点或违反显式单向规则。

## 4. 架构与版本规则

机器可读配置为：

`external/arena4_ws/src/arena/evaluation/arena_evaluation/config/pudu_wanda_3f_semantic_endpoint_transition_r1.yaml`

每次实验必须记录：

```text
architecture_id: 2A-V2
contract_revision: semantic-endpoint-transition-6m-r1
protocol_id: PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R1-V1
implementation_revision: <actual implementation revision>
```

在 targeted 离线、在线 adapter、同轮 E0/E4/新架构对照和 selected8 全部门槛通过前，不得创建 `2A-V3`，也不得宣称 B、A 或生产可用。

## 5. selected8 规则

导师语义地图的默认查询集仍为 `pudu_wanda_3f_semantic_compare_selected8_r2_v2`，即 8 组拓扑长度严格大于 50 m、端点净空至少 1.5 m 的筛选回归集。新契约不改变 query 文件、顺序、端点、yaw、地图或语义地图 hash。

所有常规对比、消融、性能和回归实验默认运行这 8 组，并在启动时校验 query/map/semantic-map hash、端点净空和拓扑长度。`--query-set` 只有在用户明确要求扩样、难例或其他查询时才能覆盖；覆盖原因必须写入 manifest。selected8 是筛选后的回归集，不能用于推断总体成功率。

窗口规则也不能用来筛掉失败查询：8 组必须全部执行；短路径、无适用语义区或完整路径失败都要记录结构化结果。

## 6. 分阶段门槛

第一阶段是契约重放：在不修改路径的条件下，用同一条 canonical path 重新计算完整区间和 6 m 窗口区间，确认 hash、采样规则和安全审计一致。

第二阶段是 targeted 离线门槛：`mirror positive`、`mirror negative`、`long south` 三条路径都必须满足端点、完整 footprint、安全、运动学和窗口语义指标。任何一条失败都停止在线阶段。

第三阶段才允许实现独立在线 planner adapter。adapter 必须输出 `nav_msgs/Path`，接入 canonical PathAudit；不能修改 pinned Nav2 核心，也不能用二维 costmap 结果冒充 SE(2) 规划接口。随后在相同 48 yaw bins、相同 query 和 session 条件下运行 E0、E4-compatible 和新架构三臂，exact effective-content ACK mismatch 必须为 0，冷请求 P50 比 E0 不得超过 2.0 倍。

第四阶段运行 selected8，每臂 1 次 warmup 加 3 次 measured。新架构须 8/8 final-valid，不能让 E0 已成功的查询回归；所有安全和运动学违规为 0，并完整记录 relaxation 层级。由于 selected8 是筛选回归集，结果最高只能支持 B 级，不支持 A 级或总体成功率结论。

## 7. 结果解释

本契约把“端点附近用于接入路线、完成姿态过渡的几何段”和“稳定内部段的车道/车位偏好”分开统计。它没有把 positive 的原始全路径失败伪装成旧契约成功，也没有证明连续空间中的所有冻结端点都可行。只有后续在线和 selected8 门槛全部通过，才能把该契约作为 `2A-V2` 的新实现修订；若任何硬门槛失败，结论仍为 C，并须保留完整区间结果和失败原因。

## 8. 复现入口

- 契约敏感性基线：[PLN-02_NEXT_STAGE_CONTRACT_SENSITIVITY_R0.md](/home/robot/pudu_robot_ws/docs/PLN-02_NEXT_STAGE_CONTRACT_SENSITIVITY_R0.md)
- 默认 selected8：[PLN-02_SEMANTIC_SELECTED8_QUERY_SET_GT50M.md](/home/robot/pudu_robot_ws/docs/PLN-02_SEMANTIC_SELECTED8_QUERY_SET_GT50M.md)
- 配置：[pudu_wanda_3f_semantic_endpoint_transition_r1.yaml](/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/config/pudu_wanda_3f_semantic_endpoint_transition_r1.yaml)
