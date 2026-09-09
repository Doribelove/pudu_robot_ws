# PLN-02 语义地图对比实验 8 组起终点（拓扑长度 >50 m）

## 结论

后续对比实验改用 `semantic_compare_selected8_r2_v2`。原约 45 m 的 `v1` 查询集不删除，但不再作为当前默认 clean set。

新版 8 组查询的拓扑长度全部严格大于 50 m，实际范围为 **55.003–58.320 m**；16 个端点到 footprint-safe 自由空间危险边界的距离为 **1.748–2.920 m**，继续满足不贴近障碍物或未知区的要求。

![拓扑长度大于 50 m 的 8 组起终点](../private_data/pudu_wanda_3f/query_sets/semantic_compare_selected8_r2_v2/selected8_query_overlay.png)

## 冻结起终点

坐标单位为 m，朝向单位为 rad；“端点净空”取该组起点与终点中的较小值。

| ID | 场景 | 起点 `(x, y, yaw)` | 终点 `(x, y, yaw)` | 端点净空 | 拓扑长度 |
|---|---|---|---|---:|---:|
| cmp2-01 | 纯车道，向北 | `(-25.801, -33.633, 1.654)` | `(-26.351, 21.067, 1.571)` | 1.800 m | 55.003 m |
| cmp2-02 | 同走廊反向，向南 | `(-26.351, 21.067, -1.571)` | `(-25.801, -33.633, -1.488)` | 1.800 m | 55.003 m |
| cmp2-03 | 车道 + 减速带 | `(-41.251, 54.517, -1.571)` | `(-40.451, -0.533, -1.279)` | 1.900 m | 55.523 m |
| cmp2-04 | 多路口连续通过 | `(8.349, 71.417, -1.571)` | `(-0.651, 24.417, 3.142)` | 1.900 m | 56.162 m |
| cmp2-05 | 路口转弯 | `(-8.601, -19.983, -1.768)` | `(-22.451, -62.283, -1.768)` | 1.748 m | 56.552 m |
| cmp2-06 | 车道 → 车位区 | `(-58.001, -14.683, 1.279)` | `(-46.301, -11.033, -1.571)` | 2.050 m | 55.180 m |
| cmp2-07 | 车位区 → 车道 | `(-47.051, -2.033, 1.571)` | `(-58.601, 42.767, 2.214)` | 2.230 m | 55.476 m |
| cmp2-08 | 车位区内部 | `(-19.751, 73.067, -0.464)` | `(-20.901, 34.067, -0.785)` | 1.900 m | 58.320 m |

cmp2-01 与 cmp2-02 仍为同一走廊的正反向控制组；其余查询继续覆盖减速带、多路口、路口转弯、车道/车位区双向切换及车位区内部。

## 选取与复验结果

先生成 16 个长度为 54.418–58.570 m 的候选，E0/E4 初筛全部成功；再从每类场景中选取一组，进行独立 ROS domain 下的重复复验。

验证版本：`2A-V2 / r2-direction-ack-latency`。每组、每个实验臂执行 1 次预热和 3 次正式测量。

| 指标 | E0 | E4 |
|---|---:|---:|
| 正式成功率 | 24/24 | 24/24 |
| R0 成功 | 24/24 | 24/24 |
| P50 请求耗时 | 1.757 s | 2.045 s |
| exact costmap ACK | 24/24 | 24/24 |
| 碰撞违规 | 0 | 0 |
| 运动学违规 | 0 | 0 |
| 硬语义违规 | 0 | 0 |
| 禁停终点违规 | 0 | 0 |

E4 共核对 5,350,131 个软代价栅格，exact mismatch 为 0；全部请求均在 R0 成功，没有依赖 R1–R4 放宽。

拓扑长度和最终 Smac 路径长度不是同一指标：拓扑路线全部超过 50 m，但 L3 可以在合法栅格空间中切角或走更短的连续曲线，因此实际路径长度为 **47.325–55.954 m**。本轮执行的是用户指定的“拓扑长度 >50 m”门槛。

## 使用边界

这套查询经过净空和成功率筛选，适合比较路径形态、语义遵从和耗时，不应用其 100% 成功率推断未知查询上的总体成功率。建议继续与原冻结难例集配套使用。

## 默认使用方式

受版本控制的权威副本为：

`external/arena4_ws/src/arena/evaluation/arena_evaluation/config/pudu_wanda_3f_selected8_gt50m_r2_v2.yaml`

`two_layer_v2_semantic_r2_benchmark --mode real-ablation` 和后续 r3 入口在未提供 `--query-set` 时都会自动加载该文件，并按活动地图与语义地图的 hash 做 fail-closed 校验。特殊实验可以显式使用 `--query-set <path>` 覆盖；只有复现旧诊断集时才使用 `--generate-query-set`。

每次实验的 `protocol.json` 会记录 `query_set_id`、`query_set_source`、`query_set_file_sha256` 和 `query_set_source_mode`，用于确认结果确实来自默认 8 组。

## 可复现信息

- 查询文件：`private_data/pudu_wanda_3f/query_sets/semantic_compare_selected8_r2_v2/selected_queries.yaml`
- 坐标表：`private_data/pudu_wanda_3f/query_sets/semantic_compare_selected8_r2_v2/selected_query_table.csv`
- 重复验证：`private_data/pudu_wanda_3f/results/query_selection_r2_selected8_validation_v2/`
- Query hash：`7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e`
- 查询 YAML SHA-256：`b3307d4578447131e71db16156cd2f72e9f5042f98bdce0d75d21fe81300738d`
- `runs.csv` SHA-256：`a3598df4b60d3aa1919c103dda8be10470c1974ac514e8d7eec22f90f7b3d6cb`
