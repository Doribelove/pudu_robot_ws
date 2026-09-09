# PLN-02 3D-V1-r2 生产准入与动态负载验证最终报告

## 1. 最终判定

本轮结论为 **A：可晋升**，但晋升对象有明确边界：将
`3D-V1-r2` 作为 3D-V1 的生产候选 L2 生命周期策略，要求上线前离线预构建并验证缓存；
在线 cache miss/reject 必须直接使用 deterministic grid A*，不得同步 cold-build D*；
pure D* 仍不得成为主线。

新冻结 held-out 完成 12 条 query、每条 10 repetitions、A/B/C 三臂严格配对。
正确性为 3,600/3,600，canonical cost error 为 0，BLOCKED/RECOVERING 入路、partial
D*、hidden reinitialize 均为 0。r2 相对 cold A* 的 L2 P50 改善 57.22%，完整
pre-L3 pipeline P95/P99 比值为 0.985/0.982；warm activation P95 为 155.26 ms，
最大 resident 为 23,254,028 B。10,000-snapshot soak 和三查询 48-bin Smac Stage B
均通过。

这不是对真实清扫障碍分布的总体收益声明。工作区没有可验证 map/坐标/时间版本的
真实清扫动态日志；正式 Stage A/soak 必须称为 **realistic synthetic workload on the
real 4× map**。Stage B 只构成集成证据。

## 2. 版本、工作区与冻结范围

- `architecture_id`: `3D-V1`
- `revision_id`: `r2-production-acceptance-real-replay`
- `protocol_id`: `PLN-02-3D-V1-R2-PRODUCTION-ACCEPTANCE-V1`
- 工作区：`/home/robot/pudu_robot_ws`
- branch：`codex/pln-02-2a-v2-3d-v1-r1`
- HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- Python：`/usr/bin/python3 3.10.12`；ROS 2 Humble；RMW `rmw_fastrtps_cpp`
- 地图：`mentor_map_20260825_005_4x_area`，`3024 × 6574`，`0.05 m/cell`
- map hash：`7226bba2392bd0986adce55b06974174e8952d84196ec7c6122237d0e08385f6`
- query hash：`c9126b7cec978f64843e1a04bb873eb07004c2ef067afd15cb148e2fd59da1c0`

Stage-0 审计：

`experiments/layered_planner_benchmark/3d_v1_r2_stage0_audit_20260904_151020`

冻结三臂为：A deterministic cold grid A*；B 3D-V1-r1 selective D*/A*；C
3D-V1-r2 optimized selective D*/A*。pure D* 仅保留为既有诊断反例，不参与晋升。
本轮没有修改 `.gitignore`，没有 reset/checkout，没有 stage、commit 或 push。

`three_d_v1` 仍位于被 `external/*` 忽略的位置，新增 r2 源码不会出现在普通
`git status` 中；`setup.py` 是本轮唯一直接可见的 three_d_v1 tracked 修改。
这仍是交付风险，不能把当前工作区状态误当成已入库交付。

## 3. 先 profile，后决策

权威 pre-optimization profile：

`experiments/layered_planner_benchmark/3d_v1_r2_r1_profile_20260904_01`

A2B-03 ROI 为 `2486 × 5620 = 13,971,320` cells，safe states 为 575,129。
r1 cold activation 为 19,434.20 ms，其中首次 `compute_shortest_path` 为
18,865.51 ms；safe-cell enumeration、cell→state mapping、adjacency build 分别仅为
3.75/9.05/76.77 ms，geometry build 合计 89.76 ms。结论很明确：首建尾部来自首次
D* 求解，不应继续塞入 online request。

r1 该路线 resident 为 35,353,665 B。永久 bool static ROI mask 占 13,971,320 B，
little-endian packbits 仅需 1,746,415 B，预测 resident 为 23,128,760 B；这与 r2
held-out 实测吻合。因此本轮只压缩 immutable mask，不改 float64 `g/rhs`、cost、邻接
和 tie-break。

动态段 profile：1-source eligible D* 为 97.08 ms；large-change/recovery A* 为
2,185.38/2,167.91 ms；explicit resync 为 75.57 ms；partial result 为 0。证据支持
继续 selective D*/A*，不支持恢复 pure-D* 主线。

## 4. r2 实现

### 4.1 Packed immutable static ROI

`PackedStaticMask` 用 little-endian bitset 保存静态 ROI mask，并提供 exact bool
materialization 和单 cell 查询；动态 overlay 仍使用热路径 bool array。全程保持
float64 和 r1 corner-safe compact graph，不引入量化误差。

### 4.2 Verified-cache-only online admission

offline `prebuild()` 独立构建、求解、序列化并验证 geometry/state；online
`activate()` 只在 geometry key、state key、schema、binding、content hash 全部通过时
准入 D*。任一 miss/reject 立即创建 packed-mask deterministic A* adapter，不生成
geometry/state cache，不把 cold build 隐藏在 online wall 中。

状态绑定继续覆盖 map hash/shape/origin/resolution、topology/route/corridor、footprint/
safety、corner-safe adjacency、endpoints、dynamic baseline、algorithm 和 format version。
缓存仍使用 manifest、content hash、atomic write；损坏、截断、schema 或 binding mismatch
均 fail closed。

### 4.3 LRU 与 telemetry

默认 active D* state=1，hard max=2；逐出后释放 planner/geometry/state，dirty state 不以
empty baseline 保存。新增 telemetry 覆盖 cache admission/reject、decision/fallback、
mask pack/materialize、restore/activate/evict、resident/RSS、active count 和 online
synchronous build count。

### 4.4 r0/r1 生产契约保持

动态 snapshot 版本化、两次确认/相关性过滤、scheduler safe skip、deterministic Graph A*
L1、自适应 2/4 m corridor、cropped full-resolution 0.05 m L2 ROI、no-partial D*、
large/recovery/not-ready A* fallback、exact old/new dirty ROI、server content ACK、48-bin
Smac Hybrid DUBIN、no reverse/no in-place rotation 和 canonical PathAudit 均保持。

## 5. 测试与工程验证

系统 Python 完整测试：**39 passed in 0.74 s**。覆盖包括：

- compact graph 与 grid oracle reachability/cost/path-cell parity；
- static/dynamic diagonal corner cutting 禁止；
- blocked/no-route/recovery；
- timeout/invalid extraction 无 partial 且 A* parity；
- 全 binding 字段敏感；损坏、截断、schema mismatch 拒绝；
- LRU admission/hit/eviction、weakref 释放、hard bound；
- route/endpoint/corridor 不串状态；
- packed mask exact parity 与 8× storage reduction；
- cache miss 不建 D*、ROI/ACK、phase telemetry 与 canonical PathAudit。

`compileall`、5 个 r2 CLI `--help`、`git diff --check`、ignored r2 源码 whitespace
check 均通过；`colcon build --packages-select arena_3d_v1 --symlink-install` 为
**1 package finished**。

最终验证目录：

`experiments/layered_planner_benchmark/3d_v1_r2_final_verification_20260904_01`

## 6. Calibration 与冻结

权威 calibration：

`experiments/layered_planner_benchmark/3d_v1_r2_calibration_20260904_01`

查询 A2B-02/07/11/15，每条 10 repetitions。结果：1,200/1,200 正确；L2 P50
相对 A* 改善 38.04%；pipeline P95/P99 比值 0.995/0.954；eligible P95 相对 r1
改善 0.50%；warm activation P95 141.42 ms；max resident 18,937,902 B；online
synchronous build=0。

冻结配置：

`external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_r2_production_acceptance.yaml`

配置 SHA-256 为 `8beb87502d2fd7aa6644615da0057668909ed596ffead55127db2092c4e0370d`。
门槛在 held-out 前冻结，calibration 后没有调整；配置绑定 calibration gate/manifest
和三份核心 r2 源码哈希，held-out loader 已实际校验。

## 7. 新 held-out 三臂结果

权威 held-out：

`experiments/layered_planner_benchmark/3d_v1_r2_heldout_20260904_01`

查询 A2B-01/03/04/05/06/08/09/10/12/13/17/18，每条 10 repetitions；A2B-16/19
只进入 classification diagnostics。每个 repetition 有 15 snapshots，C 臂共 1,800 行：
1,200 scheduler skip、240 selective D*、360 deterministic A* fallback。三臂共享完全相同
的 map/query/snapshot/seed/hash。

all-invoked L2（每臂 n=600）：

| Arm | P50 ms | P95 ms | P99 ms | Mean ms |
|---|---:|---:|---:|---:|
| A cold grid A* | 762.549 | 2,274.922 | 2,408.080 | 1,034.612 |
| B r1 selective | 323.342 | 2,239.350 | 2,399.929 | 624.777 |
| C r2 acceptance | 326.196 | 2,238.928 | 2,364.143 | 625.468 |

all-invoked pre-L3 pipeline（每臂 n=600）：

| Arm | P50 ms | P95 ms | P99 ms | Mean ms |
|---|---:|---:|---:|---:|
| A cold grid A* | 831.928 | 2,360.933 | 2,500.770 | 1,117.880 |
| B r1 selective | 410.101 | 2,329.576 | 2,484.270 | 709.145 |
| C r2 acceptance | 410.210 | 2,326.283 | 2,455.956 | 709.790 |

L2 关键分桶：

| Bucket | A P50/P95/P99 ms | B P50/P95/P99 ms | C P50/P95/P99 ms |
|---|---|---|---|
| eligible | 1,032.610 / 2,287.872 / 2,404.653 | 101.958 / 321.207 / 329.668 | 102.874 / 324.904 / 330.235 |
| fallback | 1,032.616 / 2,278.888 / 2,434.453 | 1,022.355 / 2,305.282 / 2,492.516 | 1,028.659 / 2,301.064 / 2,421.053 |
| no-route | 564.588 / 1,114.417 / 1,174.694 | 557.517 / 1,086.053 / 1,141.625 | 606.709 / 1,091.580 / 1,143.346 |
| recovery | 1,020.975 / 2,316.855 / 2,406.677 | 1,029.320 / 2,358.793 / 2,422.879 | 1,025.198 / 2,292.560 / 2,407.288 |

正式 hard gate 与目标 gate 全通过：

- L2 P50 reduction：57.22%（门槛 ≥20%）；
- pipeline P95/P99 ratio：0.985/0.982（hard ≤1.05/1.10；目标 ≤1.0/1.0）；
- eligible P95 regression vs r1：1.15%（≤5%）；
- warm activation P95：155.26 ms（≤250 ms）；
- max resident：23,254,028 B（≤30 MB），相对 r0 57,974,008 B 下降 59.89%；
- peak active state=1，120/120 warm cache hit，online synchronous build=0。

正确性为 3,600/3,600；max raw floating accumulation difference 为 `1.2733e-11`，
canonical cost error 为 0。B/C exact path-cell parity 均为 1,560/1,800；其余 240 个
eligible 行采用等成本的确定性 D* 最优路，不是 cost 或安全性偏差。compact A* 与 grid
A* 的 exact path-cell property tests 继续全通过。

phase telemetry 已在 `runs.csv` 逐行保存。C 臂 invoked 行的 confirmation/scheduler/
target-mask/L2-dispatch/dirty-ROI P95 分别为 72.68/17.80/6.61/2,239.11/75.07 ms；
完整 pre-L3 pipeline P95 为 2,326.28 ms。补充聚合位于最终 verification 目录。

## 8. 首建、restore 与 cache miss

r2 没有声称消灭 offline cold cost：最大 A2B-03 的 held-out offline prebuild 仍为
22,192.35 ms，其中 first solve 21,610.74 ms。工程修复是把该工作从 online request
彻底移出，并让缺缓存时直接使用 bounded deterministic A*。

独立 prebuild/restore 证据：

`experiments/layered_planner_benchmark/3d_v1_r2_cache_prebuild_20260904_02`

A2B-03/07/17 的 fresh offline prebuild 为 21,661/6,660/6,639 ms；验证后 warm
activation 为 106.6/32.9/34.8 ms，geometry/state 双 hit、oracle cost error=0、online
build=0。`_01` 的 false gate 是 verifier 误把 cache-restore backend 名要求成 update
backend 名，原始数据本身通过；修正只发生在独立 CLI，目录保留并排除。

## 9. 高动态 soak

权威 soak：

`experiments/layered_planner_benchmark/3d_v1_r2_soak_20260904_01`

- 10,000 snapshots，online 1,566.43 s，达到 snapshot 上限；
- A2B-07/11/17，每 500 snapshots 路线切换；
- 2,807 fallback、7,173 scheduler skip、40 resync、20 no-route、820 recovery；
- 20 route activations、21 evictions、peak active=1；
- oracle mismatch/partial/blocked path/timeout 均为 0；
- 8 次故意错误 ACK 全部拒绝；child PID 残留为空；
- clear 后 active=0、resident=0；online synchronous D* build=0。

趋势：

| 指标 | 前 10% | 后 10% |
|---|---:|---:|
| RSS mean | 889,090,314 B | 938,562,793 B |
| latency P50 | 6.587 ms | 6.372 ms |
| latency P95 | 695.496 ms | 725.428 ms |
| latency P99 | 703.672 ms | 751.310 ms |

全序列 RSS peak 942,145,536 B，前后均值增加 49.47 MB，线性斜率
3,868.63 B/snapshot，低于冻结 4,096 B/snapshot 判据；runner 判定无无界增长并通过。
但斜率余量仅约 5.6%，后段 P95/P99 上升约 4.3%/6.8%。因此上线 canary 必须继续
监控长于 2 小时的真实进程 RSS/latency，不能把本次 gate pass 解释为 allocator 问题已经消失。

## 10. 条件式 Stage B

权威 Stage B：

`experiments/layered_planner_benchmark/3d_v1_r2_stage_b_20260904_06`

A2B-03/07/17 各执行 eligible D*、large-change A* fallback、no-route、recovery。
三条 query 全通过；9/9 应调用 L3 的结果 final-valid，3/3 no-route 不调用 L3；所有
调用 content ACK=true、mismatch cells=0，static footprint 和 kinematic audit 通过。

一个 Smac session 只启动/关闭一次、restart=0；48 bins、DUBIN、fixed settle=0；同一
canonical PathAudit 实例跨 9 次有效调用复用；结束无 child PID。此结果只称为多查询
integration evidence，不称为总体端到端性能。

## 11. 失败与排除目录

以下目录保留但不作为权威晋升统计：

- `3d_v1_r2_calibration_smoke_20260904_01`：单查询冒烟；
- `3d_v1_r2_cache_prebuild_20260904_01`：验证器 backend 名误判，数据未失败；
- `3d_v1_r2_stage_b_20260904_01`：结果记录字段名错误，ROS 会话已关闭；
- `3d_v1_r2_stage_b_20260904_02`：初始化 full content ACK mismatch；
- `3d_v1_r2_stage_b_20260904_03`：暴露弱 eligible 负载、A2B-03 endpoint-lethal 与过严 no-route gate；
- `3d_v1_r2_stage_b_20260904_04`：初始化 ACK readback error；
- `3d_v1_r2_stage_b_20260904_05`：A2B-03 large-change endpoint 膨胀使 Smac 起点 lethal；
- `3d_v1_r1_r0_profile_20260904_01`：上游误发任务遗留，仅非权威诊断，未用于 r2 决策。

Stage B 最终修正只约束 synthetic source 的 endpoint clearance 并纠正 gate/字段；没有修改
Smac 参数、48 bins、ACK、settle、L2 算法或冻结 Stage-A 策略。ACK timeout 从 3 s 增至
10 s 只扩大 exact readback 的服务等待上限，不引入 fixed settle。

## 12. 证据限制与上线约束

1. 没有与 4× 地图对齐的真实清扫障碍日志；real-log 审计目录为
   `3d_v1_r2_real_log_audit_20260904_01`。真实分布收益和长时 field RSS 尚未证明。
2. offline first solve 仍可达约 21.6 s；必须由部署/地图发布流水线预构建，不能回退到
   online synchronous D* build。
3. r2 对 r1 eligible 算法本身没有显著提速，held-out P95 慢 1.15%；本轮价值是内存、
   生命周期确定性、cache-miss 安全退化和证据闭环。
4. Stage B 为 3 query × 4 scenario 的集成证据，样本不足以推断总体 Smac 尾延迟。
5. soak RSS gate 虽通过但接近斜率上限；建议 canary 配置 RSS/latency 趋势告警和自动
   fallback，而不是立即取消 A* 主导的安全策略。

生产准入条件：预构建 manifest/content hash 验证通过；默认 LRU=1、hard max=2；cache
miss/reject、timeout、invalid extraction、recovery 和 large change 均保留 deterministic
A*；出现 ACK mismatch 时不得调用 Smac；禁止把 pure D* 或未验收的其他主线混入。

## 13. 精确修改文件与哈希

实现与配置：

- `arena_3d_v1/r2_state_lifecycle.py` — `f9f8fb393a89e6916b4fdbd1c6d02b19fcdfa03c0e27002e46b81af640ffb434`
- `arena_3d_v1/r2_pipeline.py` — `87b6f760241e2c0f4e471a811fdd26a1a2e22b6cc837b624c5212c42a216695c`
- `arena_3d_v1/r2_profile.py` — `d74132d9bdb1cf5fc58a0ee4a87ae5cc14678c6b9a4437851e1f711c9586ab8f`
- `arena_3d_v1/r2_stage_a.py` — `846dca5cce486f71437555d515f0a7365517c12c0625e2cc3bd535f8e77a8cf3`
- `arena_3d_v1/r2_cache_prebuild.py` — `1faa049fb70be63f6d90c2e5cb7186351823af20315c90c15cfc03fc8365967d`
- `arena_3d_v1/r2_soak.py` — `a9e985fbcaa65b303d9720207b805c662b61134601947f2d7bd0b66db3bf00a6`
- `arena_3d_v1/r2_stage_b.py` — `d066625779a4615a43cb5f5871fdebd266e9bba9c234c1ea4edfb224c2949cb3`
- `test/test_r2_state_lifecycle.py` — `846a5b38b575bd146d81c60c5db8b9789edc444131217cbfbd71bf602af5ece9`
- `test/test_r2_pipeline.py` — `2e36932aed331986796831b311c00b8861a548fe54a24e45a950c9ea44429e55`
- `config/three_d_v1_r2_production_acceptance.yaml` — `8beb87502d2fd7aa6644615da0057668909ed596ffead55127db2092c4e0370d`
- `three_d_v1/setup.py` — `5fe132e51732742e9cbce76db210473760f1e0e7d71780fad1ff963f3c2b89fc`
- `docs/PLN-02_3D_V1_R2_PRODUCTION_ACCEPTANCE_FINAL_REPORT.md`

权威实验 manifest SHA-256：

| Evidence | SHA-256 |
|---|---|
| r1 profile | `c05420ce2b1a10f4483c09477474b88a9c4a84494423c72a41b04fdb62da88ed` |
| calibration | `b68e6135cc538e4b416e5ba6ca96ff8ebff901e22145e6dad173e9347db69007` |
| held-out | `da7606c3584ee7a81651a9d0d8bdd63cd0e4622289ba7e888ba2bd3ab4fbf506` |
| cache prebuild | `0cde8721c7905d651138fff2ee9f26debff979206839a63c4add4b97d960e270` |
| soak | `6e05d560e41ef2318624b7ca6d3a9d65f5049eedbfadc0110f4dc4d59c82b99e` |
| Stage B | `03c8fd74450641d78f474f27a8b7f417ea9fd12c32df3fb35a85b06331a12637` |
| real-log audit | `d83fd7eac4ca6cf9cdb9bdba341dd59ccf6969432df7075aa087b21e1b43bd46` |

## 14. 历史基线未改动证明

结束时使用 Stage-0 相同的全文件 hash 流程复算，以下均与开工审计一致：

| 冻结项 | SHA-256 |
|---|---|
| r0 report | `253f0d065db886bc9beb8cc54b5500bcb7cc7da1a0b7c4eba7be156ebc211c50` |
| r0 `l2_incremental.py` | `9ad20ba934d46105b62ba8a917f01871ba57ff7da0a6057f929f7d1fd2ba7ebe` |
| r1 report | `507cdea09bd01198de32715dd46bb3dfd41046ddf16856d8a33ded64a79b8565` |
| r1 `l2_state_lifecycle.py` | `a189b8ae68e30a24b00fad397e33c10da6999fb1ba9124b81836e9357e362b9d` |
| r0 selective preflight tree | `d780a45e54c4198bd63425d9da550f5245132deb05453f1b300a73c24a9fd628` |
| r0 real 4× Stage A tree | `85a879cfa740b97a0ff65d0384d7fe40d1afdd150b0554ddb717a9cad50929bc` |
| r0 Stage B tree | `f7fa96604d7417b1bc9a801c63c8e1adc4e4a89ad7f853ffca1510ce29936f7e` |
| r1 profile tree | `47301ad02c2e40754d4b0269e6e10e9ff07e96100f4668446071960f7f9f1c54` |
| r1 calibration tree | `e46c038a441e58cf284ba2c0c402cb98cfdc3502d04623983616e2f017460f8d` |
| r1 held-out tree | `e178fe457e788dc2b3eed5ad388a11a9ba356d26dccbe9ff42fef83479fe79c5` |
| r1 soak tree | `b1fa62d4610aaaf348cf6b12b60bec42d4165b748e2a4a4350cc666dcd4039d0` |
| r1 Stage-B not-run tree | `831cc9d64c7f097698e8a128f49e65cefd70a4a84de9034e9c073885a38112d6` |

开工后另一个并发任务新增/修改了 semantic 相关 dirty 与 staged 文件；本轮未读取其结果
作为 r2 证据，也未修改、撤销或提交它们。8 月 26 日已存在的 ROS/Nav2 用户进程保持
不动；所有 r2 benchmark/Stage-B 子进程均已退出。

最终建议：**晋升 r2 的生命周期策略和明确的 A* 安全退化契约；以受控 canary 部署并
补采真实清扫日志。不要启动 r3 算法改造，也不要重新走 pure-D* 主线。**
