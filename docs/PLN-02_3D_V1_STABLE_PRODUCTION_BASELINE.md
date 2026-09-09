# 3D-V1-r2-stable 稳定生产基线与 RC1 晋升报告

## 1. 最终判定

**判定 A：通过。** `3D-V1-r2-stable-rc1` 与冻结 r2 production
acceptance 在同输入下严格等价，所有无 revision 的生产入口已经收敛到
`3D-V1-r2-stable`，完整 release-candidate 复验通过。因此正式宣布：

> `3D-V1-r2-stable` 是 3D-V1 唯一默认全局规划架构。

正式标识为：

- `architecture_id: 3D-V1`
- `production_baseline_id: 3D-V1-r2-stable`
- `source_revision: r2-production-acceptance`
- `release_candidate: 3D-V1-r2-stable-rc1`
- `protocol_id: PLN-02-3D-V1-R2-STABLE-PROMOTION-V1`

稳定化是对已验收 r2 的生产入口、配置、缓存与版本策略收敛，不是新算法，
不创建 3D-V2 或 r3。r0、r1 与 r2 研究 runner 均不参与生产自动选择。

## 2. 范围与冻结约束

本轮完整阅读了 `evaluation/AGENTS.md`、`nav2_teb_controller/AGENTS.md`、
r0/r1/r2 权威报告，以及 r0/r1/r2 配置、源代码、测试和权威实验产物。
开始时记录：

- 根仓库 branch：`codex/pln-02-2a-v2-3d-v1-r1`
- 根仓库 HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- evaluation 内层仓库 branch：`humble`
- evaluation 内层仓库 HEAD：`94762429bea19b84cab50a3d0910a736184738a0`
- ROS：Humble，系统 Python：`/usr/bin/python3 3.10.12`
- 地图：`mentor_map_20260825_005_4x_area`，3024×6574，0.05 m/cell
- 地图 PGM SHA-256：`7226bba2392bd0986adce55b06974174e8952d84196ec7c6122237d0e08385f6`

根仓库和 evaluation 内层仓库原有 dirty changes 均保留；没有 reset、checkout、
stage、commit 或 push。未修改 `.gitignore`。开始前已有旧 ROS/Nav2 进程组和并发
Nav2+TEB 任务，本轮没有终止或修改它们。

`three_d_v1` 位于根仓库 `external/*` 忽略范围内。普通 `git status` 看不到
大多数新增 stable 文件，这是交付风险而不是文件缺失；长期归档中包含独立源码
快照、SHA-256 manifest 和完整交付清单。

## 3. 唯一稳定链路

冻结的生产链路是：

1. versioned dynamic observation、置信度门槛和两帧确认；
2. relevance/optimality-safe scheduler；
3. deterministic Graph A* L1；
4. topology-turn adaptive 2/4 m corridor；
5. selective persistent D* Lite L2，仅在小范围、相关、cache-valid、D* ready
   的阻塞增长上使用；
6. unconfirmed、duplicate、corridor 外、当前路径外增量可安全 skip；
7. large change、recovery、not-ready、timeout、cache miss/reject 使用
   deterministic grid A*；D* partial 永不返回；
8. L2 no-route 后排除受阻拓扑边并重跑 L1；无备用通道返回 `L1_NO_ROUTE`；
9. exact old/new dirty ROI union，分块发布与 server effective-content ACK；
10. fixed settle=0；48-bin Smac Hybrid DUBIN；无倒车/原地转向；最小转弯半径
    0.40 m；最大曲率 2.50 1/m；
11. canonical PathAudit 单实例结果复用。

生产模式不存在 pure D*，也不存在在线同步 D* state build；
`online_synchronous_dstar_build` 固定为 0。

## 4. 生产代码收敛

默认 import/export、resolver、factory、runtime 和 CLI 都从同一 stable contract
读取版本标识与冻结配置：

- `arena_3d_v1.Layered3DV1Controller` 指向
  `Layered3DV1StableController`；
- `arena_3d_v1.create_controller(...)` 在未传 revision 时只解析为
  `3D-V1-r2-stable`；
- production 模式显式请求 r0/r1/r2 或 pure D* 会 fail closed；
- benchmark/reproduction 模式才允许显式 legacy revision；
- stable runtime 复用 `Layered3DV1R2Controller` 与 r2 lifecycle，不复制算法；
- stable runtime 不导入三臂、calibration、held-out、soak 或参数搜索模块；
- 配置、cache manifest 与 r2 源码 binding 不一致时拒绝启动。

默认生产 CLI：

- `three_d_v1_plan`
- `three_d_v1_cache_prebuild`
- `three_d_v1_cache_verify`
- `three_d_v1_validate_release`

独立的 `three_d_v1_r2_stable_acceptance` 只用于验收，不被生产 factory
导入。原有 r0 无 revision benchmark 命令为旧复现兼容别名，新增了明确的
`three_d_v1_r0_*` 名称；r1/r2 runner 均保留显式 revision 名称。它们不是生产
默认 CLI。

稳定 telemetry 明确输出 `architecture_id`、`production_baseline_id`、
`source_revision`、`selected_backend`、`fallback_reason` 和 `cache_status`。

## 5. 稳定缓存交付

正式缓存流程为：verified L1 plan bundle → 离线 corridor geometry → 离线 D*
初始 state → cold grid A* oracle 校验 → schema/hash/binding 校验 → atomic bundle
发布 → `production_ready`。

cache binding 包含 map hash/shape/origin/resolution、topology hash、route edge、
corridor hash、start/goal position 和 yaw、footprint、安全/邻接规则、dynamic
baseline、algorithm/schema/source revision。manifest 和 payload 使用临时文件、
fsync 与 atomic rename。损坏、截断、schema/hash/binding 不符均拒绝；运行时
直接回退 deterministic grid A*，不同步重建。

CLI 提供 inventory、prebuild、deep verify 和 purge-obsolete report-only；本轮
没有自动删除任何缓存。小型归档缓存的 deep verify 为 `HIT_VERIFIED`，warm
backend 为 `compact_dstar_cache_restore`，oracle cost error=0，online build=0。
将 state payload 截断到 17 B 后，接口返回 `ENTRY_FILE_SIZE_MISMATCH`，默认规划
入口输出 `REJECTED:ENTRY_FILE_SIZE_MISMATCH` 并选择
`deterministic_grid_astar_cache_miss`，仍然 online build=0。

## 6. 默认选择与等价性证明

完成态全工作区引用审计扫描 646 条包含 3D-V1/r0/r1/r2/default 的引用：

- 未标记 default violation：0；
- 历史报告、冻结实验和演示快照统一标记 `historical_evidence`；
- production import 加载的 research runner：0；
- 默认 resolver/config/cache schema/controller 均为 stable；
- stable 对 legacy/pure-D* 请求 fail closed；
- cache miss/reject 不构建 D*，而是 deterministic A* fallback。

与冻结 r2 held-out 的同输入等价性比较覆盖 1800 个 stable candidate rows。
scheduler decision、selected backend、failure code、canonical cost、path hash、
cache binding key/fields 全部一致；row key 集一致，canonical cost error=0，
partial D*=0，blocked/recovering path=0，online build=0。Stage A 中相对 cold
A* 有 240 条 path-cell 不同，但均是 r2 原有的等价最短 D* 路径；stable 与冻结
r2 的 path hash 本身逐行一致。ACK 等价性在 Stage B 中验证为 mismatch=0。

## 7. 完整 release-candidate 复验

### 7.1 12-route × 10 × 三臂 Stage A

地图为真实 4× 地图；动态负载为 deterministic realistic synthetic workload，
不是可验证的真实清扫动态分布。工作区未找到可靠且可绑定 map/坐标/时间版本的
真实 4× 清扫动态日志。

查询为 A2B-01/03/04/05/06/08/09/10/12/13/17/18；每条 10 个严格配对
repetitions，每 repetition 15 snapshots，三臂共享 map/query/snapshot/seed/hash。

| 指标 | A cold grid A* | B r1 selective | C r2-stable |
|---|---:|---:|---:|
| L2 P50 ms | 761.384 | 321.448 | 324.172 |
| L2 P95 ms | 2379.863 | 2252.691 | 2287.362 |
| L2 P99 ms | 2504.590 | 2416.545 | 2418.872 |
| pre-L3 P50 ms | 837.844 | — | 407.204 |
| pre-L3 P95 ms | 2475.405 | — | 2384.982 |
| pre-L3 P99 ms | 2598.058 | — | 2512.210 |

结果：

- correctness/oracle parity：3600/3600；失败 0；
- canonical cost error：0；最大 raw 浮点误差 `1.2733e-11`；
- blocked/recovering 入路、partial D*、hidden reinitialize：均 0；
- C 相对 cold A* L2 P50 改善 57.42%，门槛 ≥20%；
- C/A pre-L3 P95 ratio 0.9635，门槛 ≤1.05；P99 ratio 0.9670，门槛 ≤1.10；
- eligible P95 相对 r1 回归 1.80%，门槛 ≤5%；
- warm activation P50/P95/P99：100.395/208.016/256.006 ms，硬门槛看
  P95 ≤250 ms，通过；
- max resident：23,254,028 B，低于 30 MB 目标，相对 r0 下降 59.89%；
- peak active state=1，cache hit=120/120，online synchronous build=0。

### 7.2 10,000 snapshot soak

实际执行 10,000 snapshots，在线时长 1609.375 s，因先达到 snapshot 数停止。
覆盖障碍移动、聚集、消失、820 次 recovery、20 次 route switch、21 次 eviction。

- oracle mismatch、partial、blocked path、timeout：均 0；
- fallback 2807，scheduler skip 7173，resync 40，no-route 20；
- 8 次故意 ACK mismatch 全部拒绝，unexpected=0；
- peak active state=1；clear 后 active=0、resident=0；残留子进程 0；
- lifecycle resident peak 8,322,141 B；
- 全程 latency P50/P95/P99：5.847/706.391/742.597 ms；
- 首 10% latency P50/P95/P99：6.573/707.691/989.101 ms；
- 末 10%：5.798/724.717/752.768 ms；
- 首/末 10% 平均 RSS：615,723,151 → 366,478,246 B；差值
  -249,244,905 B；线性斜率 -12,254 B/snapshot，无单调无界增长。

RSS 峰值 911,409,152 B 出现在初始大地图/查询装载阶段，因此报告峰值同时也报告
首末窗口趋势和 clear 证据，不能把它当作 active state resident。

### 7.3 三查询 Stage B

代表查询 A2B-03、A2B-07、A2B-17 均覆盖 eligible D*、large-change fallback、
no-route 与 recovery。共 12 个场景，其中 3 个 no-route 正确阻止 L3；其余 9 个
L3 调用全部 final-valid（9/9）。

- eligible backend 均为 `compact_persistent_dstar`；
- large change 均为 deterministic grid A*；recovery 按 cache/route 状态选择
  cache restore 或 deterministic A*；
- no-route 触发 L1 重路由；A2B-03 返回 `L2_NO_PATH_AFTER_L1_REROUTE`，
  A2B-07/17 无备用通道并返回 `L1_NO_ROUTE`；
- server effective-content ACK 9/9，mismatch=0；
- 48 bins、DUBIN、settle=0、footprint 与 kinematic audit 全部有效；
- 最大曲率仅有约 `1.1e-11` 的浮点尾差，合同值为 2.50 1/m；
- canonical PathAudit 单实例复用；session start/restart/close=1/0/1；
- online build=0，运行后残留子进程 0。

Stage B 是三查询 integration evidence，不宣称总体端到端性能，也不表示完整
Nav2/TEB 整车已经完成生产验收。

## 8. 工程验证

- `/usr/bin/python3 -m pytest`：56 passed；
- `compileall`：通过；
- `colcon build --packages-select arena_3d_v1 --symlink-install`：通过；
- 4 个 stable 默认 CLI、stable acceptance CLI、全部 r0/r1/r2 legacy CLI
  `--help`：20/20 通过；
- 根仓库与 evaluation 内层仓库 `git diff --check`：通过；
- ignored stable 源文件额外 trailing-whitespace 扫描：0 命中；
- default reference audit：646 条，default violation 0；
- stable freeze、历史报告/配置/源码/实验树 SHA-256：开始/结束一致；
- 本任务 Stage B/soak 子进程：结束后 0；未终止开始前已有并发进程。

## 9. 历史冻结哈希

关键文件开始/结束 SHA-256 一致：

| 冻结对象 | SHA-256 |
|---|---|
| r0 报告 | `253f0d065db886bc9beb8cc54b5500bcb7cc7da1a0b7c4eba7be156ebc211c50` |
| r1 报告 | `507cdea09bd01198de32715dd46bb3dfd41046ddf16856d8a33ded64a79b8565` |
| r2 报告 | `732862b47f048e7345ce4fb9b873591e7a9be24c95e788fd35568b2e20e15182` |
| r0 配置 | `d6439a8643ab66202f3aa7d57dcd8e13314ef27f73024f46bb2018243665029c` |
| r1 配置 | `8a24aafff96e95e988a9c2ec04a15f55ac168ce775a565ca7f7210f5b01d539b` |
| r2 配置 | `8beb87502d2fd7aa6644615da0057668909ed596ffead55127db2092c4e0370d` |
| r0 L2 | `9ad20ba934d46105b62ba8a917f01871ba57ff7da0a6057f929f7d1fd2ba7ebe` |
| r1 lifecycle | `a189b8ae68e30a24b00fad397e33c10da6999fb1ba9124b81836e9357e362b9d` |
| r2 lifecycle | `f9f8fb393a89e6916b4fdbd1c6d02b19fcdfa03c0e27002e46b81af640ffb434` |
| r2 pipeline | `87b6f760241e2c0f4e471a811fdd26a1a2e22b6cc837b624c5212c42a216695c` |

r2 四个权威 manifest 也保持：held-out `da7606c…f506`、soak
`6e05d560…b99e`、Stage B `03c8fd74…2637`、final verification
`e93083a3…d1ab`。12 个 r0/r1/r2 权威实验树的完整开始/结束哈希见 RC1
`final_verification/hash_verification.yaml`，全部 `unchanged: true`。

## 10. 迁移、回滚与兼容性

下游从直接构造 `Layered3DV1R2Controller` 迁移到
`arena_3d_v1.create_controller(...)` 并省略 revision；配置从 r2 acceptance YAML
切换到 `three_d_v1_stable.yaml`。原本无 revision 的 production factory 调用会
自动解析 stable。旧配置缺字段或 revision/schema/source binding 不一致不会被
静默补齐，而是拒绝启动。

回滚时不选择旧 D* 作为生产默认：先移除 stable deployment entry，再用
deterministic grid A* 保持安全；冻结 r2 runner 只在 benchmark/reproduction
环境显式执行。缓存 purge 默认只报告候选，不删除。详细步骤见包内
`THREE_D_V1_MIGRATION_AND_ROLLBACK.md`。

## 11. 正式证据、失败目录与限制

正式 write-once RC1 根目录：

`experiments/layered_planner_benchmark/3d_v1_r2_stable_rc1_20260904_174627/`

有效证据子目录为 `stage_a/`、`equivalence/`、`soak/`、`stage_b_03/`、
`final_verification/` 与 `reproduction_bundle/`。每个正式阶段包含原始 CSV/JSONL、
manifest、verification、stdout/stderr、reproduction command 和源快照或绑定哈希。

保留但排除的诊断/失败目录：

- `3d_v1_r2_stable_rc1_stage_a_smoke_20260904_01`：held-out runner 正确拒绝
  非 12×10 协议，无正式结果；
- `3d_v1_r2_stable_rc1_stage_a_smoke_20260904_02`：calibration-mode smoke，
  仅非权威诊断；
- RC1 `stage_b/`：PYTHONPATH 覆盖 ROS 工作区导致查询前中断；
- RC1 `stage_b_02/`：ROS domain 241 超出 Fast DDS 有效端口范围，查询前中断；
- RC1 `stage_b_03/`：改用有效 ROS domain 181 后正式通过。

没有复制旧 summary 冒充复验；Stage A、soak 和 Stage B 均为本轮稳定入口的新执行。
当前最重要限制仍是没有真实 4× 清扫动态日志，故动态性能结论只适用于真实地图上
的 realistic synthetic workload。

## 12. 交付与后续 Git 操作

源码、配置、测试、文档、正式证据和 ignore 可见性逐文件列在 RC1
`reproduction_bundle/delivery_manifest.yaml`。stable 核心位于：

- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/stable_contract.py`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/stable_pipeline.py`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_runtime.py`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_cache.py`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_io.py`
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_cli.py`
- `external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_stable.yaml`
- `external/arena4_ws/src/arena/three_d_v1/test/test_production_default.py`
- `external/arena4_ws/src/arena/three_d_v1/test/test_production_equivalence.py`

本轮未 stage、commit 或 push。若后续授权提交，建议分支
`codex/3d-v1-r2-stable`，建议 tag `3d-v1-r2-stable-v1`；提交前必须依据 delivery
manifest 显式处理 `external/*` 忽略文件，不能只依赖普通 `git status`。
