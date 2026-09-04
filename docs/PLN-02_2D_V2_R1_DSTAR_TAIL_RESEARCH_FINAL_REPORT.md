# PLN-02 2D-V2-r1 D* Lite 核心尾延迟研究最终报告

## 1. 结论摘要

本轮研究按预先冻结的门槛判定为 **C**：2D L1 层 pure persistent D* Lite 的尾延迟主要来自动态变化触发的大范围 `g/rhs` 一致性传播，而不是 stale heap entry 本身。indexed OPEN 将 stale entry 降为 0，也减少了 heap pop 和 key 计算，但没有减少 expanded、`update_vertex` 或 predecessor propagation；纯 Python indexed heap 的 sift/position-map 常数开销反而使 P95/P99 更差。批量 changed-edge 更新只小幅改善 `update_edges`，精确连通性快检能够提前识别 no-route，却无法覆盖仍需维护的 D* 状态成本。

正式 held-out 同机配对结果如下：

| 实验臂 | full L1 P50 (ms) | P95 (ms) | P99 (ms) | 相对 Graph A* 的结论 |
|---|---:|---:|---:|---|
| deterministic cold Graph A* | 6.888 | 8.927 | 9.601 | oracle / 基准 |
| baseline lazy-OPEN D* | 3.546 | 54.314 | 63.105 | P50 快，尾部失败 |
| indexed-OPEN D* | 4.501 | 77.371 | 91.207 | stale=0，但更慢 |
| indexed + batch D* | 4.470 | 76.691 | 90.899 | batch 收益不足以覆盖传播和 heap 成本 |
| indexed + batch + connectivity | 6.195 | 77.278 | 91.935 | no-route 快检不足以改善总体尾部 |
| 冻结 Python 组合候选 | 3.530 | 53.725 | 63.826 | P50 通过，P95/P99 失败 |

本轮同机 Graph A* 自动生成的正式门槛为：P50 ≤ 6.199 ms、P95 ≤ 9.373 ms、P99 ≤ 10.561 ms。冻结组合候选相对 Graph A* 的绝对差为 -3.358 / +44.798 / +54.225 ms，P95/P99 比值为 6.018 / 6.648；no-route P50/P95 也慢于 Graph A*。因此不建议晋升或替换生产 Graph A*，不启动 ROS/Nav2/Smac 系统 Stage B，并按有界研究停止条件终止 2D L1 D* 优化主线。后续资源应转向 3D-V0 的 L2 栅格增量规划。

## 2. 研究定义与边界

```yaml
research_id: 2D-V2-r1-dstar-tail-research
parent_architecture: 2D-V2-r0
reference_architecture: 2D-V3-r0
experiment_kind: dstar_core_tail_latency
status: research_only
final_verdict: C
system_stage_b: NOT_RUN_L1_GATE_FAILED
```

本轮只研究 L1 D* 核心。没有调整或重复评估 ROI/ACK、48 heading bins、自适应走廊、Smac、缓存或 PathAudit，也没有启动 Gazebo/Nav2/Smac。所有实验保持原始静态骨架拓扑、相同虚拟 endpoint、动态状态机、true INF、确定性 tie-break 和 exact shortest path；Graph A* 始终作为逐 snapshot oracle。

## 3. 冻结证据与权威输入

以下目录在研究开始和结束时分别计算规范化整目录 SHA-256，前后完全一致：

| 冻结目录 | SHA-256 |
|---|---|
| `2d_v2_static_mentor_map_005_r0_20260903_154754` | `572cc27da6a48f8a1be21130ccb24d22a1b37a72f80d9c445a538b4507a0d814` |
| `2d_v2_dynamic_4x_area_r0_20260903_154947` | `7f219febd6c37adc58a504213911294f4cf9a5f419981f24bbbcabc5771b4c29` |
| V3 calibration `2d_v3_calibration_4x_area_r0_20260903_183307` | `a4853cbe1b49db0f09703e6d1a6254552614d234b27a5b81b1173a035ed07c48` |
| V3 held-out `2d_v3_dynamic_4x_area_r0_20260903_183611` | `288595f6b37a97c1a2b724fc0023cd1c2ed8010ccf046edc24b1372c56448ad7` |
| V3 soak `2d_v3_cleaning_replay_r0_20260903_184825` | `5857bcb8a5228258139bc5596a0a6d8d1a7836f17158d83fc31dd0e28045c81f` |
| V3 ratio `2d_v3_ratio_break_even_4x_r0_20260903_190301` | `cc34e63e41de83991d11819d8bc5290080bb89c3b2d6c673d44e62aad95a38f9` |
| V1 1x dynamic | `81d96681fd354ba45a6444c72aef48ce93b330b381cf75fa0c09aa776113fdff` |
| V1 4x dynamic | `e5b6e8cfa7b6237a44c4685f7b4b343070b3266a68adb09732fda7d25af57434` |

V3 综合报告与 manifest 将上表四个 V3 目录标记为 calibration、held-out、soak 和 ratio 的权威目录。本轮未使用 superseded 或 interrupted 目录作为证据。

工作区原有未提交修改得到保留；没有执行 reset、checkout、clean、stash、commit 或 push，也没有终止本任务以外的 ROS 进程。

## 4. 输入负载与图规模

正式负载继续使用 V3 冻结的 `realistic_synthetic_cleaning_workload`。仓库、实验、日志和场景审计没有发现能够与 mentor 4x 地图可靠对齐的真实清扫动态日志，因此本报告明确将其标记为**合成清扫负载**，不冒充真实清扫实测。

4x 图的实际规模为 6574×3024 cells、4376 topology nodes、4562 edges。held-out 由 14 个清扫场景和 5 个 ratio 场景组成，共 19 个 episode；每个 episode 使用同一固定输入流，3 次 warmup、20 次 measured。

Graph A* arm 逐 snapshot 的正式 gate 样本共 7040 个。changed-edge 分布为：

| 统计点 | changed edges | changed-edge ratio |
|---|---:|---:|
| P0 | 1 | 0.0219% |
| P25 | 1 | 0.0219% |
| P50 | 3.5 | 0.0767% |
| P75 | 20 | 0.4384% |
| P95 | 210 | 4.6032% |
| P99 | 210 | 4.6032% |
| Max | 210 | 4.6032% |

离散计数为：1 边 2400 次、2 边 960 次、3 边 160 次、4 边 400 次、5 边 400 次、10 边 320 次、11 边 400 次、20 边 400 次、42 边 400 次、100 边 800 次、210 边 400 次。path-affected 比例为 39.49%。

## 5. 可信修改前 profile

在核心实现改动前，使用同一冻结 held-out 输入完成了 3 次配对复现：

| 指标 | Graph A* P50/P95/P99 | baseline D* P50/P95/P99 |
|---|---:|---:|
| full L1 wall (ms) | 7.632 / 22.737 / 25.178 | 7.267 / 51.213 / 68.918 |
| search (ms) | — | 3.696 / 43.920 / 49.127 |
| expanded | — | 424 / 5067 / 5591 |
| heap pop | — | 588 / 7646 / 8645 |
| stale entries | — | 69 / 2528 / 3054 |
| update_vertex | — | 1150 / 13633.5 / 14941 |

no-route 的 Graph A* / D* 为 1.237/10.312/10.576 ms 与 3.092/24.505/24.896 ms；recovery 为 7.137/8.330/8.621 ms 与 19.925/28.630/29.808 ms。这一同代码态复现确认历史 D* 尾延迟确实存在，后续正式门槛仍以本轮 held-out 配对数据自动计算，而非引用历史数值。

## 6. 独立研究实现

### 6.1 Indexed lexicographic OPEN

实现唯一 entry 的二元 key indexed heap，支持 insert/update/remove/pop-min 和 position map。虚拟 negative endpoint 与普通节点使用相同索引协议。所有 pop、sift、key 计算、OPEN 峰值和状态内存均可审计。

### 6.2 Instrumented baseline 与 batch update

baseline lazy heap 保持原算法语义，只增加低侵入计数。batch arm 为每个不可变 edge 预计算受影响节点集合，并在 changed-edge 批次中做 union/dedup；同一初始批次不重复调用 `update_vertex`。这项去重不改变后续一致性传播顺序和最终 `g/rhs`。

### 6.3 Exact connectivity precheck

在应用动态 overlay 后使用确定性 DFS/BFS 检查 start 与 goal 的精确连通性，覆盖虚拟 endpoint、桥、多边 cut、component 和恢复。precheck 时间计入 full L1；若提前 no-route，仍单独记录为了维持 persistent D* 下一 snapshot 正确性而执行的状态维护成本。

### 6.4 Resync 策略

实现并实测三种策略：

- immediate：fallback 后立即同步 catch-up；
- lazy：D* 标记 not-ready，期间直接用 A*，顺序合并 snapshot，quiet window 后 catch-up；
- batched background：同样禁止未 ready 的 D* 响应，以低优先级批量 catch-up，并校验 snapshot/status hash 后恢复。

所有策略都将 catch-up CPU 纳入总成本；partial/unconverged D* 不会返回路线。

## 7. Calibration 与冻结候选

calibration 目录使用 5 次重复，共 8820/8820 个 arm-snapshot 结果通过 oracle。单变量结果为：

| Arm | P50 (ms) | P95 (ms) | P99 (ms) | 正确性 | calibration 决策 |
|---|---:|---:|---:|---|---|
| Graph A* | 6.156 | 9.062 | 9.507 | 100% | oracle |
| baseline lazy D* | 3.941 | 54.364 | 65.302 | 100% | 保留 |
| indexed D* | 5.782 | 78.951 | 95.364 | 100% | 淘汰 |
| indexed + batch | 5.723 | 77.326 | 94.510 | 100% | 淘汰 |
| indexed + batch + connectivity | 7.386 | 79.855 | 94.864 | 100% | 淘汰 |
| 组合候选 | 3.936 | 54.180 | 66.571 | 100% | 冻结为 baseline lazy backend |

按照“组合候选只能纳入单变量证明有效且正确的修改”的规则，indexed heap、batch 与 mandatory connectivity precheck 均未进入冻结组合候选。正式 held-out 前冻结为 `baseline_lazy_dstar`；没有依据 held-out 结果反向调整参数。

## 8. Held-out 三臂/六臂正确性

正式 held-out 共 47,880/47,880 个 arm-snapshot 结果通过：

- oracle parity 100%；
- reachability 与 failure code 100% 一致；
- 最大路径代价误差 0；
- 输出路径包含 BLOCKED/RECOVERING edge 为 0；
- 未收敛 D* 路线返回为 0；
- snapshot/status/input hash mismatch 为 0；
- no-route 与 recovery 分类 100% 正确；
- virtual endpoint parity 100%；
- hidden reinitialize 为 0；
- baseline 与优化 D* 的收敛状态和最终路线逐 snapshot 可验证一致。

因此候选的淘汰完全来自预定义性能门槛，不是安全性或最短路径语义失败。

## 9. Held-out 正式性能

### 9.1 总体

| Arm | P50 (ms) | P95 (ms) | P99 (ms) | P50 vs A* | P95 ratio | P99 ratio |
|---|---:|---:|---:|---:|---:|---:|
| Graph A* | 6.888 | 8.927 | 9.601 | — | 1.000 | 1.000 |
| baseline lazy D* | 3.546 | 54.314 | 63.105 | -3.342 | 6.085 | 6.573 |
| indexed D* | 4.501 | 77.371 | 91.207 | -2.387 | 8.667 | 9.500 |
| indexed + batch | 4.470 | 76.691 | 90.899 | -2.418 | 8.591 | 9.468 |
| indexed + batch + connectivity | 6.195 | 77.278 | 91.935 | -0.693 | 8.657 | 9.576 |
| 冻结组合候选 | 3.530 | 53.725 | 63.826 | -3.358 | 6.018 | 6.648 |

冻结候选 P50 相对 A* 改善 48.76%，超过 10% 门槛；但 P95/P99 分别超出允许上限 44.352 ms 和 53.265 ms。门槛要求整体统计，不只统计 D* 成功子集，也没有计入 scheduler skip。

### 9.2 Path affected 与 unaffected

| 分桶 | Graph A* P50/P95/P99 (ms) | baseline D* P50/P95/P99 (ms) |
|---|---:|---:|
| path-affected | 6.909 / 8.998 / 9.697 | 6.535 / 58.125 / 64.483 |
| path-unaffected | 6.861 / 8.866 / 9.494 | 2.286 / 36.752 / 41.857 |

本轮 intentionally 没有 scheduler skip：即使 path-unaffected，仍执行 L1，以隔离 pure D* 核心价值。unaffected 组证明小变化可获增量收益，但其 P95 仍远慢于 A*；任何上层“不调用 L1”的收益都不能归因于 D*。

### 9.3 No-route 与 recovery

| 分桶 | Graph A* P50/P95/P99 (ms) | 冻结组合 D* P50/P95/P99 (ms) |
|---|---:|---:|
| no-route | 0.325 / 8.763 / 9.161 | 0.418 / 27.386 / 27.926 |
| recovery | 6.986 / 7.848 / 8.311 | 22.552 / 32.650 / 33.583 |

exact connectivity arm 的 no-route early-response 为 3.237/3.719/3.933 ms，但把 persistent D* 状态维护计入完整口径后为 3.435/41.685/42.758 ms。它能更早给调用方正确 no-route，却不能使 full L1 或后续 recovery 的维护成本消失，因此未通过 no-route 门槛。

## 10. Break-even 曲线

### 10.1 Absolute changed edges

| changed edges | A* P50 (ms) | baseline D* P50 (ms) | combo P50 (ms) | combo/A* |
|---:|---:|---:|---:|---:|
| 1 | 4.423 | 2.519 | 2.539 | 0.574 |
| 2 | 7.485 | 2.482 | 2.489 | 0.333 |
| 5 | 5.968 | 5.646 | 5.525 | 0.926 |
| 20 | 8.013 | 24.061 | 24.243 | 3.025 |
| 100 | 6.909 | 26.113 | 23.986 | 3.472 |

absolute break-even 位于 5 到 20 changed edges 之间。

### 10.2 Ratio-matched

| ratio | actual edges | A* P50 (ms) | baseline D* P50 (ms) | combo P50 (ms) | combo/A* |
|---:|---:|---:|---:|---:|---:|
| 0.046% | 2 | 8.413 | 2.257 | 2.215 | 0.263 |
| 0.092% | 4 | 4.772 | 1.356 | 1.353 | 0.283 |
| 0.230% | 11 | 8.515 | 3.183 | 3.166 | 0.372 |
| 0.921% | 42 | 8.612 | 24.339 | 24.515 | 2.847 |
| 4.604% | 210 | 7.482 | 32.352 | 31.944 | 4.269 |

ratio break-even 位于 0.230% 到 0.921% changed edges 之间。该拐点反映传播范围，并不支持用单一 changed-edge 阈值替代 correctness-safe fallback。

## 11. 尾延迟归因

| 指标 | baseline lazy P50/P95/P99 | indexed P50/P95/P99 | 解释 |
|---|---:|---:|---|
| expanded | 277 / 4889 / 5591 | 277 / 4889 / 5591 | 完全不变 |
| heap pop | 288 / 6752 / 8645 | 277 / 4889 / 5591 | indexed 去除 stale pop |
| stale entries | 36.5 / 2231 / 3054 | 0 / 0 / 0 | 目标达成 |
| aggregate stale/pop | 25.94% | 0% | 约四分之一 lazy pop 为 stale |
| update_vertex | 606 / 13124 / 14941 | 606 / 13124 / 14941 | 传播工作不变 |
| predecessor visits | 604 / 10655 / 12115 | 604 / 10655 / 12115 | 传播工作不变 |
| key calculations | 882.5 / 20147 / 23484 | 877 / 18285 / 20430 | 有所减少 |
| indexed heap sift | — | 1840 / 33835 / 41568 | Python 手写 indexed heap 新增常数开销 |

结论是双重的：

1. lazy heap 确实存在显著实现开销，indexed OPEN 能将 stale entries 降到 0；
2. 但 P95/P99 的主导量是 4.9k–5.6k expanded、13.1k–14.9k `update_vertex` 和 10.7k–12.1k predecessor visits 所代表的结构性一致性传播。indexed heap 没有减少这些量，且 Python sift 开销将 full wall 推得更高。

batch arm 的 `update_edges` 从 baseline 0.095/3.175/3.635 ms 降至 0.086/2.756/3.481 ms。baseline 已经对初始 affected-node 集合去重；虽然原始 edge candidate 重复在 P50/P95/P99 可达 8/520/520，进一步的 per-edge cache union 只额外节省 0/100/100 个初始候选，不能减少搜索传播阶段的 `update_vertex`。因此第 2、3 个研究问题的答案分别是“stale 可消除但不能消除尾部”和“初始 batch 可小幅优化，但 13k 级传播不是批次重复造成”。

## 12. S0、内存与 cold init

### 12.1 首次规划

| Arm | S0 full P50/P95/P99 (ms) | cold init P50/P95/P99 (ms) | search P50/P95/P99 (ms) |
|---|---:|---:|---:|
| Graph A* | 7.002 / 9.301 / 9.806 | — | 6.037 / 8.096 / 8.510 |
| baseline lazy D* | 36.926 / 196.073 / 233.649 | 6.934 / 165.347 / 210.753 | 27.317 / 35.017 / 36.171 |
| indexed D* | 50.883 / 210.374 / 236.687 | 7.092 / 163.182 / 196.971 | 39.921 / 50.318 / 52.246 |
| indexed + batch | 59.438 / 203.639 / 250.621 | 15.211 / 174.815 / 205.626 | 39.867 / 50.060 / 51.355 |

S0 明确单列，未混入动态稳态门槛。

### 12.2 D* state memory

| Arm | P50/P95/P99 bytes | 备注 |
|---|---:|---|
| Graph A* | 314,936 / 357,720 / 358,612 | fresh search state |
| baseline lazy D* | 1,348,182 / 1,425,814 / 1,430,182 | max 1,432,318 |
| indexed D* | 1,351,858 / 1,426,318 / 1,428,066 | position map 抵消部分 stale 节省 |
| batch/connectivity | 2,788,216 / 2,862,192 / 2,863,940 | immutable per-edge affected-node cache 增大内存 |

## 13. Fallback/resync 专项结果

19 个 episode、399 个 snapshot 中，dynamic snapshot 为 380 个；按冻结 V3 预算规则发生 261 次 fallback，占 68.68%。以下为实际执行的 catch-up CPU，不是估算：

| 策略 | resync 次数 | resync CPU total (ms) | 每次 P50/P95/P99 (ms) | response A* total (ms) | 合并情况 |
|---|---:|---:|---:|---:|---|
| immediate | 261 | 17,003.621 | 53.389 / 259.198 / 279.582 | 1,433.311 | 每次 fallback 同步 resync |
| lazy, quiet=2 | 21 | 1,537.596 | 56.837 / 233.418 / 269.319 | 1,862.905 | mean 16.286 snapshots/resync，P50/P95/P99=20 |
| batched background, quiet=1 | 25 | 1,741.113 | 55.907 / 224.377 / 274.390 | 1,807.287 | mean 13.48，P50/P95/max=20 |

lazy 将维护 CPU 降低约 90.96%，batched background 降低约 89.76%；但 D* not-ready 期间使用 A*，response A* 总 CPU 分别上升 29.98% 和 26.09%。在 episode 时间模型下，最大 not-ready proxy 为 10 s。该结果证明“延迟合并 resync”对维护成本有效，但它不会修复 pure D* 核心尾延迟。

按 1/5/10 Hz 对平均每 snapshot 总 CPU 的离线占用推演如下；这只是**模型推演**，不是现场频率实测：

| 策略 | 1 Hz | 5 Hz | 10 Hz |
|---|---:|---:|---:|
| immediate | 0.0461 core | 0.2303 core | 0.4606 core |
| lazy | 0.00655 core | 0.03277 core | 0.06555 core |
| batched background | 0.00754 core | 0.03772 core | 0.07544 core |

## 14. Soak 结果

soak 使用 14 个清扫场景、20 cycles，共 17,640/17,640 个结果正确：

| Arm | P50 (ms) | P95 (ms) | P99 (ms) |
|---|---:|---:|---:|
| Graph A* | 6.783 | 9.119 | 9.901 |
| baseline lazy D* | 3.879 | 53.429 | 62.836 |
| 冻结组合 D* | 3.879 | 53.442 | 62.844 |

长序列没有暴露 correctness、snapshot hash、未收敛路径或 blocked-edge 问题，但重复确认了尾延迟结论。

## 15. C++ 与 LPA* 决策

没有实施 C++ 原型。原因不是工作量估计，而是正式 profile 已提供停止证据：indexed OPEN 将 stale 清零后，expanded、`update_vertex` 与 predecessor propagation 完全不变，且 P95/P99 仍比 Graph A* 高一个数量级附近。C++ 可能降低常数，却不能消除必须执行的 4.9k–5.6k 节点一致性传播和 no-route 证明成本；在严格有界研究条件下继续扩大实现范围不合理。若未来另立 C++ 研究，必须同时实现同图表示的 C++ D* 和 C++ A*，不能与 Python A* 跨语言比较。

没有增加 LPA* 诊断臂。正式 episode 的 start/goal 固定，LPA* 有研究价值，但它属于新的诊断架构，不能作为本轮 D* Lite 候选“救结果”；现有传播证据已满足 C 类停止条件。

## 16. 测试、构建与机器校验

实际执行结果：

| 检查 | 结果 |
|---|---|
| 相关 pytest | `58 passed in 34.74s` |
| `python3 -m compileall`（绝对源码路径） | 通过 |
| `colcon build --packages-select arena_evaluation --symlink-install` | 1 package，build 1.84 s，总计 3.56 s，通过 |
| 已安装新 CLI `--help`，`ROS_DOMAIN_ID=121` | 通过 |
| `git diff --check` | 通过 |
| calibration 产物机器校验 | 20/20 必需项存在，source snapshot hash 通过 |
| held-out 产物机器校验 | 20/20 必需项存在，source snapshot hash 通过 |
| soak 产物机器校验 | 20/20 必需项存在，source snapshot hash 通过 |
| 冻结输入目录结束 SHA-256 | 全部与开始值一致 |
| 本任务残留进程 | 0 |

第一次 compileall 调用曾错误使用不存在的相对路径并打印 `Can't list`；该次不计为验证结果，随后使用绝对路径重新执行并通过。没有隐藏失败。

新增测试覆盖：indexed OPEN 随机参考队列、lexicographic key/update/remove/pop、virtual endpoint、逐边与 batch parity、重复 changed-edge 去重、连通/断连、bridge/multi-edge cut、no-route/recovery、partial search 禁止返回、lazy/background resync 的 snapshot 合并与 ready 校验、长序列 soak、输入 hash、计时不重复相加及正式 artifact/source manifest 校验。

## 17. 修改文件与用途

| 文件 | 用途 | SHA-256 |
|---|---|---|
| `arena_evaluation/indexed_dstar_open.py` | indexed OPEN、instrumented baseline、batch backend、exact connectivity | `9b5e52cd502c60c006f88b69d16ad96a9fddd2030cada158eb2d928f9a868de9` |
| `arena_evaluation/dstar_tail_research.py` | 六实验臂、oracle/state parity、resync 策略和诊断 | `3e7b12ff69f2e77ff3704429632728936ab69add2d64586b168bf5f4bf01ea6d` |
| `arena_evaluation/two_layer_2d_v2_r1_dstar_tail_benchmark.py` | calibration/held-out/soak CLI 与正式产物生成 | `299bbd8a68bfd1f0875d18164767d0b69ff5c21ea82d93ebfe4958d1f19d42f5` |
| `config/two_layer_2d_v2_r1_dstar_tail_research.yaml` | 冻结协议、门槛和候选选择规则 | `4b126c8617781842db971a78ce78e39355aad1d10d1b7338665029372d225fd` |
| `test/test_dstar_tail_research.py` | 队列、算法、resync、runner 与 artifact 回归 | `aa6f596478e5836dd1f0452afc2c38d7f93012aacde543a843599d750f086899` |
| `setup.py` | 注册独立研究 CLI，不改变 V1/V2/V3 默认入口 | source snapshot 中记录 |
| 本报告 | 汇总权威结果、停止判定和复现入口 | 由仓库当前内容计算 |

现有 `graph_dstar_lite.py` 的默认行为没有被本轮改写；新实现仅由独立 backend/CLI 启用。V1/V2/V3 的现有默认入口保持不变。

## 18. 正式 write-once 目录

- Calibration：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_calibration_20260904_100317`
- Held-out：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_heldout_20260904_100713`
- Soak：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_soak_20260904_102652`

对应 source snapshot tree hashes：

- calibration：`9c1aea2556f76ce2936eeba07b144c43379907a171a72e5a63785a65a0c25a19`
- held-out：`d0a70695b3239992ac2b5160d5e29c45dd7d18960f287c81eac0098450b7987d`
- soak：`2b2a23fe086519e5130022f789874536ede8e69a3b22311c41cf2915bcf5aeeb`

两个被会话中断的非权威目录得到保留且未补写：

- `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_heldout_20260903_195709`
- `/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_heldout_20260904_093959`

它们不参与任何结论。

## 19. 复现命令

每个正式目录内均保存实际 `reproduction_command.txt`。等价命令如下：

```bash
cd /home/robot/pudu_robot_ws/external/arena4_ws
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash

ROS_DOMAIN_ID=121 ros2 run arena_evaluation two_layer_2d_v2_r1_dstar_tail_benchmark \
  --mode calibrate \
  --output-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_calibration_<new_timestamp> \
  --warmups 3 --repetitions 5 --soak-cycles 20

ROS_DOMAIN_ID=121 ros2 run arena_evaluation two_layer_2d_v2_r1_dstar_tail_benchmark \
  --mode heldout \
  --calibration-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_calibration_20260904_100317 \
  --output-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_heldout_<new_timestamp> \
  --warmups 3 --repetitions 20 --soak-cycles 20

ROS_DOMAIN_ID=121 ros2 run arena_evaluation two_layer_2d_v2_r1_dstar_tail_benchmark \
  --mode soak \
  --calibration-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_calibration_20260904_100317 \
  --output-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_soak_<new_timestamp> \
  --warmups 3 --repetitions 20 --soak-cycles 20
```

新运行必须使用新的 timestamp 目录，不能复用上述正式目录。

## 20. 最终架构建议

1. **不晋升 2D-V2-r1 D* 候选。** 生产 L1 继续使用 deterministic Graph A*。
2. **停止 2D L1 D* 核心优化主线。** 在 synthetic held-out 的小变化上 D* 有明确 P50 优势，但真实门槛要求的 P95/P99、no-route 和 recovery 均大幅失败；继续做 heap 微优化不能消除结构性传播。
3. **保留研究实现和数据。** indexed queue、批量 update、连通性快检和 lazy resync 都是可复用的受测研究资产，但不应默认启用。
4. **转向 3D-V0 L2。** D* Lite 更适合与局部栅格增量变化直接对应的层级；应在新的独立架构和门槛下验证，不能外推本轮结果为 3D-L2 的正或负结论。
