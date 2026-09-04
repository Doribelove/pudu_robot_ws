# PLN-02 静态语义规划 2A-V2 / r1 实现与诊断报告

- 架构：`2A-V2`
- 实现修订：`r1`
- 日期：2026-09-04
- 范围：静态 A2B；未接入 `2D-V1-r3 dynamic`
- 基线：`real_ab_r0_run12`（全程只读）
- 结论级别：开发诊断，不是正式统计结论

## 1. 结论

r1 已完成评测解耦、始终开启的路径语义审计、逐 attempt/cumulative 计时、查询方向校正、左右边界定义、lane-instance ROI、低幅平滑软代价、缓存/复用、区间 ACK、五臂消融和真实 8 查询复验。

总体结论为**部分通过**：

| 项目 | 状态 | 证据 |
|---|---|---|
| 架构与安全边界 | 通过 | 始终为 `2A-V2/r1`；硬语义 254，舒适性软代价不 lethal；成功路径四类硬违规均为 0 |
| P0 评测设计与可观测性 | 通过 | E0–E4 开关独立并记录哈希；A/E0 审计不再为空；失败路径硬约束为 `null/NOT_APPLICABLE`；逐 attempt 和累计计时齐全 |
| P1 方向修复 | 部分通过 | 正反向偏好场对称翻转；真实规划 forward 达标，reverse 为 0.796 / 2.10 m，仍未达到 ≥0.8 / ≤0.5 m |
| P2 R0 正常路径 | 部分通过 | E4 的 5 条成功中 4 条在 R0，forbidden 在 R1；lane-junction-lane 仍因 L3 软场导致 max-iterations |
| P3 性能 | 部分通过 | 相邻 arm 可复用缓存；独立冷顺序 E4 P50 6.181 s，E0 P50 2.863 s，仍为 2.16x，未过建议 ≤2x 门槛 |
| P4 ACK 确定性 | 部分通过 | hard exact mismatch=0、soft-bound mismatch=0；soft exact mismatch 比率仍为 41.74%（五臂主诊断），只能称区间 ACK |
| P5 正式实验 | 未通过/未启动 | 方向、R0 和性能门槛尚未全部满足；按协议未启动 30–50 查询与重复测量，P95/P99 仅为调试统计 |

不得据此声称 r1 已正式可用，也不得把不同成功集合的汇总分位数解释为语义收益。

## 2. r0 冻结证据

`private_data/pudu_wanda_3f/results/real_ab_r0_run12` 未被修改：

| 文件 | SHA-256 |
|---|---|
| `runs.csv` | `48069a641e2984a5c405d8f966671713b12a4cec522515203a00f419ab2d37ad` |
| `summary.json` | `715456ad345aeb117988a3c7f14d2b6ad085ae41810501fa8742d701c28a4d28` |
| `protocol.json` | `c5984b455cd5c2c9aa7938f40b11a79cdaa5a5e71fdee2eb13efaef356f5ed8a` |

r1 的所有产物均写入新目录；早期失败/调参目录也保留，没有覆盖历史实验。

## 3. 评测开关与五臂消融

配置文件：`external/arena4_ws/src/arena/evaluation/arena_evaluation/config/two_layer_v2_semantic_r1.yaml`

| Arm | route selector | L1 semantic cost | L1 hard | L3 hard | L3 soft class | regional preference | audit | relaxation |
|---|---|---:|---:|---:|---:|---:|---:|---|
| E0 | legacy | off | off | off | off | off | on | strict |
| E1 | multi-source neutral | off | off | off | off | off | on | strict |
| E2 | multi-source semantic | on | on | on | off | off | on | strict |
| E3 | E2 固定 L1 route | on | on | on | on | on | on | strict/R0 |
| E4 | multi-source semantic | on | on | on | on | on | on | graceful R0–R4 |

程序在启动时校验 arm 的真实开关；`runs.csv` 同时记录开关、`arm_switch_hash`、实际 selector 和 route hash。E3 强制引用 E2 的同 query L1 route 并校验 hash，防止把 L1 变化误归因到 L3。

语义审计与语义规划影响完全分离。即使 E0/E1 关闭全部语义代价，成功路径仍执行 `SemanticPathAudit` 并产生 lane、parking、硬约束和方向指标。

## 4. r1 实现

### 4.1 方向与区域偏好

- 每次进入 regional builder 前比较 L1 polyline 两端与 query start/goal 的正反端点和；必要时深拷贝并同步反转 polyline、node、edge 和 annotations。
- 记录 `route_reversed_for_query`、首尾距离、正常/反转端点和、path-vs-route tangent agreement、每个 lane instance 的 direction stability。
- 不再使用“任意骨架线的 signed-right ≥ 0”。在查询定向的局部切线下同时探测左右 lane 边界，以 `d_right <= d_left` 定义 correct side，目标误差为 `abs(d_right - 0.40 m)`。
- ROI 只扩展到 route probe 实际接触的 lane feature instance。真实 forward/reverse 选择 3/8 个 lane instance，排除 5 个相邻实例，避免错误切线跨 lane 传播。
- 偏好场和诊断距离场以 crop 形式保存；R1/R2 复用 R0 几何，只重新组合权重。

离线方向图门槛通过：forward/reverse 的最低代价带随查询方向翻转，preferred cells 的 correct-side ratio 均约 1.0、目标误差 P50 分别约 0.210 m/0.240 m。但地图几何给出了真实限制：主 lane 的 0.40 m 目标带对 route seeds 覆盖率 forward 72.77%、reverse 56.05%，次 lane 更低；因此最终规划路径不保证全程能落入目标带。

### 4.2 软代价与硬语义

- 静态/lethal、footprint 和硬语义不参与 relaxation；硬语义固定 254，并使用 footprint-expanded hard mask。
- lane 舒适性上限 64，parking 上限 48，speed-bump 独立上限 72；均低于 near-hard 风险区间。
- 0.40 m 目标使用 0.20 m 容差平台和 Huber 型平滑增长，不再形成单栅格低成本脊线。
- 输出软代价直方图、饱和比例、有效覆盖面积、lane segment direction stability 和目标带覆盖率。
- `R0/R1/R2` 复用 route、静态语义和方向几何；`R3/R4` 只在 ROI 改变时重建相关区域。
- unlabelled/无有效软语义时跳过纯软权重重试，避免无意义的 R1/R2。

### 4.3 ACK、计时与失败语义

每个 R0–R4 attempt 单独记录：

- wall、L1、ROI build、field build、compose、publish、ACK wait、Smac、audit；
- 未归入上述组件的 process time；
- 每次 ACK 的 hard checked/mismatch、soft checked/bound mismatch、soft exact mismatch/ratio；
- final-attempt 与 cumulative-request 两套时间。

没有生成路径时，`hard_constraints_held=null`、`hard_constraints_status=NOT_APPLICABLE`。只有路径生成且完成审计后才可能写 `true/HELD` 或 `false/VIOLATED`。

ACK 语义明确为 `interval_not_exact`：硬单元必须 exact；软单元只要求落在 static-to-inflation 合法区间。缓存命中仅复用此前已经由 server 验证的语义状态，不把本地预期误写成 server ACK。

## 5. 8 查询五臂诊断

主结果：`private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8`

### 5.1 Arm 汇总

| Arm | 成功 | R0 成功 | relaxation trigger | 主要解释 |
|---|---:|---:|---:|---|
| E0 | 5/8 | 5 | 0% | 真控制组，legacy，无语义规划影响，audit on |
| E1 | 5/8 | 5 | 0% | 修正 endpoint attach 后，neutral multi-source 在当前 8 条上未改变成功数 |
| E2 | 4/8 | 4 | 0% | L1/hard 语义没有救回 lane-to-parking；parking-internal 生成路径但 footprint collision，被审计拒绝 |
| E3 | 4/8 | 4 | 0% | 固定 E2 route 后，L3 使 lane-to-parking 成功，但使 junction max-iterations；forbidden 的 R0 路径被碰撞审计拒绝 |
| E4 | 5/8 | 4 | 50% | 4/8 query 触发 relaxation；仅 forbidden 在 R1 获得有效成功，净增益 1/8 |

E4 的 relaxation trigger 不是“成功路径中使用 relaxation”的比例，而是请求执行了 R1+ 的比例。8 条中 4 条触发，最终只有 1 条从 relaxation 获益。

### 5.2 逐查询结果

| Query | E0 | E1 | E2 | E3 | E4 | r1 解释 |
|---|---|---|---|---|---|---|
| lane-forward | R0 成功 | R0 成功 | R0 成功 | R0 成功 | R0 成功 | L3 correct-side 0.696→0.989，误差 2.35→0.45 m |
| lane-reverse | R0 成功 | R0 成功 | R0 成功 | R0 成功 | R0 成功 | 0.178→0.796，误差 3.60→2.10 m；方向改善明显但未达门槛 |
| lane-junction-lane | R0 成功 | R0 成功 | R0 成功 | 失败 | R4 仍失败 | E0/E1/E2 route 相同；根因隔离到 L3 soft field/搜索交互，不应只增大 iterations |
| lane-to-parking | 失败 | 失败 | 失败 | R0 成功 | R0 成功 | E2→E3 且固定 L1 route 后才成功，收益来自 L3/route-lane ROI，不是 L1，也不是 R3 扩 ROI |
| parking-internal | R0 成功 | R0 成功 | 生成但审计失败 | R0 成功 | R0 成功 | E2 路径静态 footprint collision；E3 regional field 恢复有效路径 |
| forbidden-detour | R0 成功 | R0 成功 | R0 成功 | 生成但审计失败 | R1 成功 | graceful 降低软偏好后恢复；硬语义未放宽 |
| unlabelled | 失败 | 失败 | 失败 | 失败 | R4 仍失败 | 无足够有效软语义；不能把机械重试当收益 |
| narrow-lane | 失败 | 失败 | 失败 | 失败 | R4 仍失败 | Smac max-iterations，需继续做几何可行性/启发式诊断 |

所有 E4 有效成功路径：collision=0、kinematic=0、hard-semantic=0、no-stopping goal=0。无路径的 3 条为 N/A，不计作安全违规，也不计作安全通过。

### 5.3 同 query 配对指标

以下仅比较 E0 与 E4 同时成功的同一 query；lane-to-parking 不同成功集合，不计算 delta：

| Query | Δ length (m) | Δ curvature P95 (1/m) | Δ correct-side | Δ right-target error P50 (m) |
|---|---:|---:|---:|---:|
| lane-forward | +1.618 | +0.00044 | +0.292 | -1.900 |
| lane-reverse | +0.531 | -0.02120 | +0.618 | -1.500 |
| parking-internal | -0.130 | -0.06366 | +0.785 | -3.101 |
| forbidden-detour | -97.360 | +0.49958 | N/A | N/A |

forbidden-detour 同时受 selector/route 改变影响，不能把其长度或曲率变化归因于 L3。报告不使用不同成功集合的 Arm 汇总分位数作因果比较。

### 5.4 方向门槛

| Query | E4 correct-side ratio | E4 error P50 | 要求 | 状态 |
|---|---:|---:|---|---|
| lane-forward | 0.989 | 0.45 m | ≥0.8 且 ≤0.5 m | 通过 |
| lane-reverse | 0.796 | 2.10 m | ≥0.8 且 ≤0.5 m | 未通过 |

reverse 的 ratio 距门槛约 0.0042，但目标误差仍高 1.60 m，不能通过四舍五入或更改指标掩盖问题。

## 6. 性能

独立冷顺序结果：`private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v21_e0_e4_cold`

| 指标 | E0 | E4 | 比值/说明 |
|---|---:|---:|---|
| cumulative request wall P50 | 2.863 s | 6.181 s | 2.16x，未达到建议 ≤2x |
| E4 field-build P50 | — | 2.829 s | 最大已归因组件 |
| E4 unaccounted process P50 | — | 1.705 s | 第二大组件，仍需细分 |
| E4 compose P50 | — | 0.686 s | 需继续 crop/增量化 |
| E4 Smac P50 | — | 0.619 s | 不是唯一瓶颈 |
| E4 ACK wait P50 | — | 0.487 s | ROI ACK 尚有优化空间 |
| E4 ROI build P50 | — | 0.382 s | lane-instance 选择后已小于 field build |

cold-start 静态准备单独报告：raster 690.769 ms、topology 174.071 ms、semantic edge 8343.556 ms，合计 9208.396 ms。它没有被藏在缓存后的 L1 数字中，也没有机械加到每条 query 的 P50。

五臂顺序运行中的 E4 warm-cache P50 为 1.575 s、E0 为 2.573 s，但它受 arm 顺序和跨臂缓存影响，只能证明缓存路径工作，不能用作生产延迟结论。单次 8-query 的 P95/P99 均标记 `debug_only`，`p99_valid=false`。

## 7. ACK 结果

五臂主诊断累计值：

| Arm | soft checked | soft exact mismatch | exact mismatch ratio | hard mismatch | soft-bound mismatch |
|---|---:|---:|---:|---:|---:|
| E3 | 3,189,462 | 1,139,881 | 35.739% | 0 | 0 |
| E4 | 13,773,229 | 5,748,269 | 41.735% | 0 | 0 |

冷顺序 E4 的 exact mismatch ratio 为 44.982%，在 inflation settle 时序下有波动。当前结论只能是：硬 exact 与软区间边界验证通过；逐栅格 soft exact 未通过。文档和数据字段统一使用“区间 ACK”，不得写成 exact verified。

## 8. 测试与复现

### 8.1 测试

定向 r1/语义/拓扑回归：

```bash
cd /home/robot/pudu_robot_ws
source /opt/ros/humble/setup.bash
source external/arena4_ws/install/setup.bash
export PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation:${PYTHONPATH}
/usr/bin/python3 -m pytest -q \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_v2_semantic_r1.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_v2_semantic.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_l1_l3_corridor_hybrid_smoke.py \
  external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_topology.py
```

结果：定向回归 `66 passed`；最终代码状态的完整 source-tree 回归为 `352 passed in 83.82s`。

构建：

```bash
cd /home/robot/pudu_robot_ws
source /opt/ros/humble/setup.bash
colcon --log-base external/arena4_ws/log build \
  --base-paths external/arena4_ws/src/arena/evaluation \
  --build-base external/arena4_ws/build \
  --install-base external/arena4_ws/install \
  --packages-select arena_evaluation
```

结果：`arena_evaluation` 构建成功，`two_layer_v2_semantic_r1_benchmark` 入口和 r1 配置已安装。

### 8.2 诊断入口

```bash
ros2 run arena_evaluation two_layer_v2_semantic_r1_benchmark \
  --mode synthetic-smoke \
  --output-dir <new-output-dir>

ros2 run arena_evaluation two_layer_v2_semantic_r1_benchmark \
  --mode offline-diagnostic \
  --output-dir <new-output-dir>

ros2 run arena_evaluation two_layer_v2_semantic_r1_benchmark \
  --mode real-ablation \
  --arms E0,E1,E2,E3,E4 \
  --warmups 0 --repetitions 1 \
  --output-dir <new-output-dir>
```

每次必须使用全新目录；程序拒绝在非空结果目录上覆盖。正式实验应在门槛通过后使用版本化的 30–50 查询集、每条 warmup 和足够重复样本。

## 9. 主要结果目录

- 合成正反向/硬语义 smoke：`private_data/pudu_wanda_3f/results/synthetic_smoke_r1_v3_footprint_hard`
- 离线方向叠加图与几何证据：`private_data/pudu_wanda_3f/results/offline_direction_r1_v8_cropped_fields`
- 五臂 8-query 主诊断：`private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8`
- 独立 E0/E4 冷顺序性能：`private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v21_e0_e4_cold`

主诊断包含按 query 分面的 E0–E4 彩色路径、图例、start/goal 箭头、方向箭头和 relaxation level；不再把全部路径画成同一种红色。原始逐 attempt 数据在 `attempts.jsonl`，请求级数据在 `runs.csv`，配对比较在 `paired_comparisons.csv/json`。

## 10. 未完成项与下一步

正式实验被以下真实门槛阻塞，而不是工具或环境阻塞：

1. reverse 目标误差仍为 2.10 m。下一步应按 lane instance 检查目标带可达性、route tangent 与实际可行走廊的关系，不能调低审计门槛。
2. lane-junction-lane 在 E0/E1/E2 同 route 下成功、E3/E4 失败，已排除 selector 和 L1 route；下一步应分别屏蔽 junction 周边 lateral field、比较启发式展开与软代价等高线，定位 soft field/heuristic 交互。
3. field-build 是冷顺序最大组件。应把 Python 全 crop 边界探测改为按 lane instance 的缓存距离变换/矢量化，并把 compose/ACK 缩到实际 changed ROI。
4. unlabelled 与 narrow 需先做硬几何/footprint 可达性证据；无软语义时不应继续机械执行纯权重 relaxation。
5. 若要降低约 42% soft exact mismatch，应验证明确的 clear→full reinflation→publish→settle 顺序，或采用独立静态语义层；在完成前仍使用区间 ACK。
6. 只有方向、R0 与开发延迟门槛全部通过后，才扩展版本化查询集并运行正式 P95；P99 至少需要 100 个有效测量样本。

## 11. 不变量与仓库状态

- 没有放宽静态/lethal 障碍、footprint、合法端点、no-stopping 目标、禁止倒车/原地旋转、`Rmin=0.40 m`、最大曲率 `2.50 1/m`。
- soft preference 从未变成 lethal；hard semantics 始终为 254。
- 未修改 pinned Nav2 核心以掩盖问题；嵌套 Nav2 仍保留用户已有的 `nav2_bringup/launch/navigation_launch.py` 修改。
- 共享 `topology.py` SHA-256 为 `e052cc2ea7e38d5559e3c0ba5fd9ee7907dcf18453f88908900d3d756d0ed204`，未保留调试性改动。
- 工作树中其他既有修改和历史实验均未 reset、clean 或覆盖。
