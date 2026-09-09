# PLN-02 下一阶段：语义适用区契约敏感性验证

日期：2026-09-07。协议：`PLN-02-SEMANTIC-CONTRACT-SENSITIVITY-R0-V1`。

## 目的

上一阶段的四种规划架构都证明：`mirror positive` 的安全路径存在，但靠右目标带主要位于完整 padded footprint 隔离的走廊之外。下一阶段先不改安全和运动学约束，只验证一个最小的新语义契约候选：起点和终点附近属于端点过渡区，不纳入靠右/靠中统计；稳定内部段仍使用原来的 `correct-side`、`target-band` 和 lateral P50 门槛。

这是一项敏感性诊断，不替换旧契约，也不产生新的 architecture_id。

## 固定内容

- 地图、语义地图、query、start/goal x/y/yaw 全部冻结；
- 完整 padded Jackal footprint 不变；
- forward-only、禁止倒车、禁止原地旋转不变；
- `Rmin=0.40 m`、最大曲率 `2.50 1/m` 不变；
- 原安全 PathAudit、碰撞、禁行/禁停和 R0 结果不变；
- 只改变语义指标的统计适用区。

## 统计定义

对已经通过完整安全和运动学审计的路径，按弧长从两端各裁掉相同长度 `d`，只在 `[d, L-d]` 内统计语义样本。`d` 从 0 到 8 m 扫描。路径本身、控制序列和端点没有改变。

## 结果

### mirror positive

固定安全路径长度 `32.003801 m`，原始全路径结果为：

| 端点过渡区 d | side | target-band | lateral P50 | 候选语义门槛 |
|---:|---:|---:|---:|---|
| 0 m | 0.635449 | 0.340557 | 2.434833 m | 不通过 |
| 4 m | 0.797311 | 0.455016 | 0.956318 m | 不通过 |
| 5 m | 0.823928 | 0.496614 | 0.567409 m | 不通过 |
| 6 m | 0.856079 | 0.545906 | 0.387184 m | 通过 |
| 7 m | 0.895172 | 0.606897 | 0.351829 m | 通过 |
| 8 m | 0.944099 | 0.628882 | 0.319922 m | 通过 |

最小扫描候选是每端 `6 m`。这不是把路径变好，而是把两端的几何过渡段从语义统计中排除；完整路径的安全和运动学结果没有变化。

### 其他 targeted 查询

| 查询 | 原契约 side / band / P50 | d=0 结果 | 结论 |
|---|---|---|---|
| `r3-mirror-2-negative` | 1.000 / 0.730679 / 0.250000 m | 1.000 / 0.730679 / 0.250000 m | 原契约已通过 |
| `cmp2-02-lane-south` | 1.000 / 0.625973 / 0.250000 m | 1.000 / 0.625973 / 0.250000 m | 原契约已通过 |

因此，在这个候选统计口径下，三条 targeted 路径都满足语义门槛；但这只能说明候选新契约具有可行性，不能把历史 C1 结果改写为旧契约 B。

## 产物

- positive：[summary.json](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/next_stage_contract_sensitivity_positive_20260907T150000Z/summary.json)
- negative：[summary.json](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/next_stage_contract_sensitivity_negative_20260907T150100Z/summary.json)
- south：[summary.json](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/next_stage_contract_sensitivity_south_20260907T150200Z/summary.json)
- 通用脚本：[semantic_contract_sensitivity.py](/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/semantic_contract_sensitivity.py)

每个目录还保存 `manifest.json`、逐 d 的 `results.json` 和 `results.csv`，绑定了 query、地图 hash、语义地图 hash、输入 NPZ hash、路径 hash 和原始安全审计结果。

## 下一步决策

如果导师接受“端点过渡区不纳入方向偏好统计”，下一步才可以另起正式协议，定义新 contract revision，再实现在线 planner adapter、canonical PathAudit 接口、同轮 48-bin 三臂、exact ACK 和 selected8 重复实验。建议先冻结 `d=6 m` 的适用区定义，并在全部 selected8 上验证它不会把窄路、无标签区或真实失败路径隐藏掉。

在得到这项契约变更的明确批准前，项目正式状态仍为：`2A-V2 / C1`，旧契约不可晋升。

