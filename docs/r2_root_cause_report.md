# PLN-02 2A-V2 r2 根因报告（行为修改前冻结）

## 1. 身份与冻结边界

- `architecture_id`: `2A-V2`
- `implementation_revision`: `r2-direction-ack-latency`
- `protocol_id`: `PLN-02-2A-V2-R2-DIRECTION-ACK-LATENCY-V1`
- profile 对象：冻结的 2A-V2 r1，不是修改后的 r2。
- 工作区分支 / HEAD：`codex/2a-v1-r2-roi-pathaudit-delivery` / `337fed6f9d3b9e27f16e87c21ed3557e6a14834a`
- Arena evaluation 嵌套仓库：`humble` / `94762429bea19b84cab50a3d0910a736184738a0`，含大量既有用户 dirty/untracked 文件；本轮不 reset、checkout、clean 或覆盖。
- pinned Nav2：`codex/2a-v1-r2-smac-instrumentation` / `656ae8d4c56978efbdd446fe85582f2bcd06e920`；既有用户修改仅为 `nav2_bringup/launch/navigation_launch.py`，r2 不修改 Nav2 核心或该文件。
- 环境：Ubuntu `/usr/bin/python3 3.10.12`，ROS 2 Humble，`0.05 m/cell`，静态地图，无动态障碍。
- 适用约束：`external/arena4_ws/src/arena/evaluation/AGENTS.md`，SHA-256 `39d81ddddec36e89daa85a4887fcd9d9bc87e44357c009a8775f6fa6adb45944`。

本报告在改变 ACK、方向场、ROI 或缓存行为之前写入。r0 run12、r1 报告与四个权威结果目录保持只读。

## 2. 父基线哈希

### r0 保留证据

| 对象 | SHA-256 |
|---|---|
| `real_ab_r0_run12/runs.csv` | `48069a641e2984a5c405d8f966671713b12a4cec522515203a00f419ab2d37ad` |
| `real_ab_r0_run12/summary.json` | `715456ad345aeb117988a3c7f14d2b6ad085ae41810501fa8742d701c28a4d28` |
| `real_ab_r0_run12/protocol.json` | `c5984b455cd5c2c9aa7938f40b11a79cdaa5a5e71fdee2eb13efaef356f5ed8a` |
| `topology.py` | `e052cc2ea7e38d5559e3c0ba5fd9ee7907dcf18453f88908900d3d756d0ed204` |

### r1 源码与报告

| 对象 | SHA-256 |
|---|---|
| `regional_preference_r1.py` | `3a5586bf418964e55236f355ce750d7ad5db1f083fc314006007c1c4f74f641d` |
| `semantic_smac_session.py` | `84951726e353de0e3deafa884dd436b8e750d4dabf405f085973f9095a22cb6f` |
| `semantic_costmap_composer.py` | `93802208b5b548d8b9cbf22857fa5135c5307bc1792b416fe52866cdc0d0657c` |
| `two_layer_v2_semantic_r1_benchmark.py` | `ac8fc223234fd5cbf1e5395e179d03552318df902d168a758fa202efb8c807f6` |
| `two_layer_v2_semantic_r1.yaml` | `1b4a467de30bab867b6702ec282faa431dcb1614029caee573720088145aefd7` |
| `PLN-02_ARCHITECTURE_2A_V2_R1.md` | `fa282334a75acae1a46eca8c7e4cfb7540daeddad61806a78b1ea383214d4f15` |

权威 r1 目录的确定性文件流哈希：

| 目录 | SHA-256 |
|---|---|
| `real_ablation_r1_diag_v20_final8` | `13a2fa5d7cf133fc86c307918801a943080e03edea25d46d7618d5baffe02d4b` |
| `real_ablation_r1_diag_v21_e0_e4_cold` | `3992ee34d3b5209fa5a63aba3be9c5994ec6989d81553efe73e0d3430f5cce49` |
| `offline_direction_r1_v8_cropped_fields` | `e0b69c25f9eafa8004c4539f4138dbc06555d44d7415d0e75a21d7a9ade95f67` |
| `synthetic_smoke_r1_v3_footprint_hard` | `51be71779799758fe91f86ffbfa3a491681c22f35b4437717ad76d042d67ba91` |

地图哈希 `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`，语义图哈希 `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`，PDMap 哈希 `ffb5c838f282a9074c4afcf69915b24fb875cc1008d298c6c835ea39ce03d731`，query 集哈希 `9daf9b5ddaf682cf844a2845d4b9bc1abb827506d14dce333cdfd9916409c67a`，拓扑图哈希 `73ce811afd6083c0cd5b4eb3eefb33353bdfbbc18da6e83ec7316c21e6b90fb2`。

## 3. Stage 1 新证据目录

1. `private_data/pudu_wanda_3f/results/r2_stage1_r1_reproduction_20260904_01`
   - 8-query、E0–E4、0 warmup、1 repetition；ROS domain 133。
   - 文件流哈希：`a5a96fef60a4882f7b7d96d65ac04d63fa52f988076643f1749e4277a56e0051`。
2. `private_data/pudu_wanda_3f/results/r2_stage1_ack_root_cause_20260904_01`
   - E3/E4 全 8-query，保持 r1 区间 ACK 接受行为，仅在 ACK/no-op 后抓取服务端 master content；ROS domain 134。
   - 文件流哈希：`e6e172597a2d7ce22ef05036fb75680b3a93deaf303f4baf5b8c48433391594e`。

这两个目录均为新的只追加证据，未复用 r1 输出目录。

## 4. 逐查询失败矩阵

表内为 `最终有效 / 最终 relaxation`；失败括号内为代码。

| query | E0 | E1 | E2 | E3 | E4 |
|---|---|---|---|---|---|
| lane-forward | 是/R0 | 是/R0 | 是/R0 | 是/R0 | 是/R0 |
| lane-reverse | 是/R0 | 是/R0 | 是/R0 | 是/R0 | 是/R0 |
| lane-junction-lane | 是/R0 | 是/R0 | 是/R0 | 否/R0 (`SMAC_MAX_ITERATIONS`) | 否/R4 (`SMAC_MAX_ITERATIONS`) |
| lane-to-parking | 否/R0 (`SMAC_MAX_ITERATIONS`) | 否/R0 | 否/R0 | 是/R0 | 是/R0 |
| parking-internal | 是/R0 | 是/R0 | 否/R0 (`STATIC_FOOTPRINT_COLLISION`) | 是/R0 | 是/R0 |
| forbidden-detour | 是/R0 | 是/R0 | 是/R0 | 否/R0 (`STATIC_FOOTPRINT_COLLISION`) | 是/R1 |
| unlabelled | 否/R0 (`SMAC_MAX_ITERATIONS`) | 否/R0 | 否/R0 | 否/R0 | 否/R4 |
| narrow-lane | 否/R0 (`SMAC_MAX_ITERATIONS`) | 否/R0 | 否/R0 | 否/R0 | 否/R4 |

汇总与冻结 r1 一致：E0/E1 为 5/8，E2/E3 为 4/8，E4 为 5/8；E4 成功中 R0=4、R1=1。53 个 attempt 均单独保存在 `attempts.jsonl`。因此 Stage 1 复现有效，不能把后续差异归咎于 query 或臂漂移。

## 5. 方向根因

### 5.1 已排除的原因

- forward 与 reverse 的 L1 route hash 分别为 `9775986a…`、`80fa4f58…`；endpoint 对应检查的 normal sum 均为 `5.181 m`，反转 sum 均为 `296.266 m`，因此本次选择的 polyline 并未首尾倒置。
- 两个方向的 route-vs-path tangent agreement P50 分别为 `0.990`、`0.993`；lane direction stability P50 均为 `1.0`，P05 均为 `0.924`。当前主因不是局部切线随机翻转。
- ROI 均只扩入路线探测到的 3 个 lane instance，并排除 5 个相邻 instance；forward/reverse 添加面积分别为 `905.388/905.430 m²`，未出现明显的相邻 lane 全域泄漏。

### 5.2 尚未满足的几何/在线结果

| query | E4 correct-side | E4 target error P50 | 主 lane instance correct-side | 主 lane instance error P50 |
|---|---:|---:|---:|---:|
| lane-forward | 0.989 | 0.45 m | 0.994 | 0.45 m |
| lane-reverse | 0.796 | 2.10 m | 0.815 | 2.10 m |

reverse 不是通过边界上的四舍五入问题：总体 `0.795795 < 0.80`，且更关键的目标误差比 `0.50 m` 门槛高 `1.60 m`。跨 junction 的次 lane instance 更差（ratio `0.521`，error `3.415 m`），但主 lane 自身仍有 `2.10 m` 偏差，所以不能只删除 junction 样本来过线。

结论：下一步必须把离线 preferred band、footprint-safe reachable band、Smac 起终姿态可达集和实际路径叠加审计；若原端点按预冻结的、与算法无关的规则不可达，保留为 `SEMANTIC_QUERY_INFEASIBLE` 负例。不得移动原 query 或修改指标定义。

## 6. exact ACK 根因

### 6.1 r1 接受语义的缺陷

r1 的 ACK 只要求：hard cell 等于 254，soft cell 落在 `StaticLayer expected <= actual master <= locally inflated expected` 区间。缓存 key 只有 policy/grid/master 三个 hash；一旦区间 ACK 成功，no-op 会复用该证据。它没有绑定 publication sequence、source grid hash、ROI bbox，也不要求 exact master content。

### 6.2 分层迁移采样

35 次 post-ACK/no-op 内容抓取的聚合：

| 指标 | 数量 |
|---|---:|
| soft cells | 20,984,818 |
| soft exact mismatch | 4,477,369 |
| stale changed cells | 6,612,291 |
| hard exact mismatch（抓取时） | 24,493 |
| static-dominant soft cells / mismatch | 11,785,333 / 24,848 |
| inflation-dominant soft cells / mismatch | 9,199,485 / 4,452,521 |

30 次立即位于 r1 interval ACK 后的抓取包含全部 stale/hard 瞬时差异；5 次对同一 expected content 的后续 no-op 抓取中，2,063,172 个 soft cell 只剩 319 个 mismatch，hard/stale 均为 0。319 个残差全部为实际值低于预期 `1–10`（其中 −1:107、−2:40、−3:21、−4:75、−5:74、−6:1、−10:1）。

由此得到的证据链：

1. occupancy→StaticLayer 映射在稳定后基本确定，主差异不来自 ROS occupancy 编码。
2. 大差异集中在 inflation-dominant cell，并随时间消退，说明 r1 interval ACK 可在 InflationLayer/dirty ROI 尚未达到最终内容时提前返回。
3. 同一 expected hash 的后续 no-op 近乎 exact，反证 r1 no-op key 复用了“尚未 exact”的证据。
4. 剩余 319 个负残差是 pinned Nav2 inflation 的离散缓存/截断与 SciPy EDT 近似期望之间的确定性映射差异；r2 应实现 pinned 算法一致的 expected-effective mapping，而不是降低 exact 门槛。
5. 抓取发现 r1 ACK 成功后的瞬时 hard mismatch，说明 r2 必须 fail closed，并要求与 publication sequence/bbox/hash 绑定的稳定 exact 观测。

## 7. 延迟 profile

冻结 r1 冷顺序结果：E0 P50 `2.863 s`，E4 P50 `6.181 s`，比值 `2.16x`，未达到 r2 的 `<=2.0x` 门槛。E4 P50 的已分解大项为 field build `2.829 s`、unaccounted process `1.705 s`、compose `0.686 s`、Smac `0.619 s`、ACK `0.487 s`、ROI `0.382 s`。进程启动另有 raster `0.691 s`、topology `0.174 s`、semantic edge precompute `8.344 s`，必须与 online request 分开报告。

Stage 1 五臂顺序复现中 E4 受 E3 cache 复用影响，P50/逐项数字不能作为 cold 结论；它仅用于行为/失败矩阵。后续 E0/E4 必须独立进程严格配对。

性能实施依据：首先去除每 query 的全图 float 临时场和重复 EDT/feature label，缓存 query-independent geometry；其次让 R0–R4 复用不可变几何；最后仅在 exact ACK 完整 key 成功时允许 no-op。不得用无界缓存或跳过 ACK 换取速度。

## 8. 修改前判定与实施约束

- lane-reverse：**未通过**；需证明目标侧可行带并修复在线路径，或依预冻结规则给出物理不可达证据。
- exact ACK：**未通过**；当前只是 interval ACK，且有 stale/hard 瞬时差异。
- cold latency：**未通过**；`2.16x > 2.0x`。
- Stage 6：**禁止启动**，直到新的 Stage 5 同时通过三项硬门槛。

下一阶段只实施方向/可行几何、exact effective-content ACK 和有界缓存/ROI 延迟优化，不扩展语义功能，不改变 max iterations、footprint、Rmin、曲率、倒车或原地旋转限制。
