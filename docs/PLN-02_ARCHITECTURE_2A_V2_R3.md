# PLN-02 2A-V2 r3 最终报告

## 最终结论

- `architecture_id`: `2A-V2`
- `implementation_revision`: `r3-lane-relative-viability-guide`
- `protocol_id`: `PLN-02-2A-V2-R3-LANE-VIABILITY-GUIDE-V1`
- 父基线：冻结的 `2A-V2-r2`
- 结论：**C — 不可晋升，不可作为生产规划方案使用。**

r3 已经完整实现并验证了“同一 lane feature 内连续、静态可达、曲率受限的语义 guide”，
也保住了 r2 的 exact effective-content ACK、硬安全和冷路径性能。但是，guide 只在二维
lane lattice 中成立，不能保证 48-bin Smac Hybrid DUBIN 在真实 SE(2) 起终姿态和最终
master costmap 上沿 guide 行驶。真实 ROS 预检的三条 E4 路径没有一条通过完整方向门槛，
因此按预先冻结的 gate 停止，没有启动 8-query 和正式扩样。

## 做了什么

### 1. 把逐栅格偏好升级为有向连续 guide

新增的 `DirectedLaneViabilityCertificate` 和 guide builder 以 start→goal 定向 L1 route 为
station 轴，只在 route 实际穿过的同一 semantic lane feature instance 中采样横向候选。
候选必须同时满足 R0 ROI、静态净空、hard semantic mask、前向 station 连接以及
`<=2.50 1/m` 离散曲率。

证书绑定 map、semantic map、route、ROI、policy 和 lane source ID 的 hash。只有完整 lane
run 达到 correct-side `>=0.80`，并且严格多数 station 落在
`abs(d_right-0.40)<=0.50 m` 内，guide 才能影响规划；失败时 fail closed，保留 r2 field。
guide 最终只栅格化为上限 64 的 soft tube，硬语义仍为 254，未把偏好变成 lethal。

### 2. 审计真实 PDMap 与冻结 mirror query

问题 lane 的源 ID 是 `374682499F8A4131783406408835`，原始名“车道5”，来自
`ATLAS_DATA map.zones[3]`。它是横向约 9 m、边界不规则、两侧都有静态障碍的 facility
zone，没有显式单向方向，不能当成一条有清晰道路标线的窄车道。

原长距离正反向 query 在二维目标带上存在真实方向不对称。r3 保留其负例身份，没有移动
端点；另外按预先冻结的端点附着、route 长度、topology clearance 和双向证书规则选择短
mirror pair。前两个 selection policy 的不足和所有中间目录均保留并标记 excluded。

冻结 mirror query hash：
`615dc7e7d866447d059e416f7602240edc8a7b94e112d7155168b800e1f2ce50`。

### 3. 建立可观测性并优化实现

记录 candidate、adjacency、viability DP、guide solve、rasterize 分段时间；证书与 guide
复用 adjacency，并直接复用成功的 constructive witness，去掉重复 DP。离线校准中
`guide_beam_width=160` 对冻结 8-query 的分类不变，长可行 query 的 guide build 从
626.0 ms 降到 373.8 ms，因此在 held-out 前冻结为 160。

### 4. 真实 ROS 预检

在全新目录对 E0/E4 严格配对运行：mirror 正向、mirror 反向和长距离 south query；每条
1 次、无 warmup，只作为 gate/debug，不宣称正式 P95/P99。每次都经真实 costmap 发布、
exact ACK、Smac 和 canonical PathAudit。

## 结果

### 离线 guide

冻结 mirror pair 的二维 lane-lattice guide 均通过：

| query | correct-side | target error P50 | max curvature | guide build |
|---|---:|---:|---:|---:|
| mirror positive | 1.000 | 0.305 m | 1.085 1/m | 83.77 ms |
| mirror negative | 1.000 | 0.200 m | 1.787 1/m | 84.65 ms |

这证明方向翻转、同 lane instance 隔离和二维连续 guide 的实现本身成立；它不等于最终
车辆路径可达性证明。

### 在线方向硬门槛

冻结门槛：E4 correct-side `>=0.80`、target error P50 `<=0.50 m`、R0、全部硬安全为 0。

| query | E0 side/error | E4 side/error | E4 判定 |
|---|---:|---:|---|
| mirror positive | 0.000 / 3.300 m | 0.000 / 4.950 m | **失败且退化** |
| mirror negative | 1.000 / 1.350 m | 1.000 / 0.950 m | **误差失败** |
| long south | 1.000 / 1.540 m | 1.000 / 0.534 m | **超门槛 0.034 m** |

三条 E4 都在 R0 规划成功，collision、kinematic、hard semantic、no-stopping goal 违规均为
0，但语义方向目标是产品行为门槛，不能用“路径有效”替代，也没有把 0.534 m 四舍五入成
0.50 m。

### exact effective-content ACK

- attempts / ACK：`6 / 6`
- soft exact checked：`624,991`
- soft exact mismatch：`0`（`0.0%`）
- hard exact mismatch：`0`
- stale ROI cells：`0`
- sequence/hash mismatch：`0 / 0`

因此 r1 的 41.735% soft exact mismatch 已在 r2/r3 链路中解决；当前否决原因不是 costmap
发布内容不确定。

### 冷路径性能（仅 debug，n=3/arm）

| 指标 P50 | E0 | E4 |
|---|---:|---:|
| cumulative request | 1.636 s | 1.764 s |
| L1 | 1.51 ms | 2.46 ms |
| ROI build | 21.01 ms | 173.52 ms |
| field build | 182.13 ms | 213.05 ms |
| compose | 413.94 ms | 433.53 ms |
| ACK wait | 723.69 ms | 757.90 ms |
| Smac | 47.63 ms | 43.17 ms |
| audit | 66.86 ms | 66.11 ms |

E4/E0 cold P50 为 `1.078x`，通过 `<=2.0x` 开发门槛，也优于 `<=1.5x` 目标值；样本只有
3 个，不能据此宣称正式 P95/P99。进程内 peak RSS 为 1,594,249,216 B，当前 RSS P50
E0/E4 约 984/1,073 MB；预检没有看到按 relaxation 增长，因为全部停在 R0，但这不是长时
soak 证据。

### 两个根因校准

1. 更强 soft tube（cap 80、加快饱和）：positive 仍为错误侧、5.00 m；另两条误差
   0.70 m 和 0.50000006 m。
2. guide candidate 使用完整 0.55 m inflation clearance：三条误差
   4.95/0.95/0.542 m，基本不变。

两次校准 exact ACK 和安全均通过，参数未被采用。这排除了“吸引力略弱”和“只少算一个
inflation radius”这两个局部解释，也说明继续提高代价只会接近把 soft 偏好硬化，方向不对。

## 为什么不能晋升

失败发生在 2D guide 到 SE(2) planner 的接口：

```text
2D lane station/lateral witness（通过）
        ↓ rasterize 为 soft cost
effective master ACK（精确通过）
        ↓
48-bin Smac DUBIN 搜索（路径安全且有效）
        ↓
真实 lane direction audit（失败）
```

r3 证书没有编码 Smac 的 yaw bin、DUBIN motion primitive、精确 start/goal yaw 连接，也没有
在最终 effective master cost 上证明整段原语可执行。栅格 soft tube 只是 Smac 目标函数的一
部分，不是 route-conditioned SE(2) 搜索约束。正向 mirror 在离线 0.305 m、在线却错误侧
4.95 m，是这个接口缺口的直接反例。

这与相关工程方法一致：Nav2 Smac 本身是在 SE(2) motion primitives 上做 cost-aware
Hybrid-A*；状态格方法也强调把微分约束编码到可连接的原语中，而不是仅给 Cartesian cells
着色。Autoware 类似地把 reference path、drivable area 与运动学轨迹优化放在同一接口中，
并在输出后重新验证。

- [Nav2 Smac Hybrid-A* 配置与模型](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
- [Smac cost-aware kinematically feasible planning 论文](https://arxiv.org/abs/2401.13078)
- [CMU Differentially Constrained State Lattice](https://www.cs.cmu.edu/~alonzo/pubs/papers/JFR_09Final.pdf)
- [Autoware Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)

## 下一步：先改接口，不立刻改 architecture_id

最合适的下一步不是 `2A-V2-r4` 再调 cost cap，而是一个有明确退出条件的
`r4-se2-guide-interface`：

1. 用与 Smac 一致的 48-bin yaw 和 DUBIN/state-lattice primitives 构造 guide；显式包含
   start/goal pose 与 yaw connector。
2. 每条 primitive 在 exact expected effective master cost 上做 footprint sweep；证书输出
   可重放的 SE(2) primitive sequence、代价与 hash。
3. 在独立包中实现 route-conditioned planner adapter/plugin，让搜索直接消费 SE(2)
   reference corridor/prior；不修改 pinned Nav2 核心。
4. 语义仍只进入 objective/heuristic，硬安全域保持原契约；不得把 corridor 外变 lethal。
5. 先冻结 mirror pair + long south + junction，要求离线 primitive replay 与在线最终路径同时
   达到 `>=0.80 / <=0.50 m / R0`，再运行 8-query。
6. 如果显式 SE(2) prior 仍必须用近硬代价才能控制路径，立即停止 `2A-V2` 修补，论证
   `2A-V3`（受限 state-lattice 或参考路径轨迹优化器作为最终 L3）。

所以：**现在无需立刻更改 architecture_id，仍可做一次 2A-V2 的 L3 接口修订；但不能再把
它实现成 costmap-only 的逐栅格软偏好。** 若最终 planner 被替换，则必须正式改架构为
`2A-V3`，重新做 A/B 和安全验证。

## 阶段判定

| 阶段 | 状态 | 原因 |
|---|---|---|
| 实现、合成、单测 | 通过 | guide、证书、镜像、隔离、fail-closed 均通过 |
| 离线 frozen mirror | 通过 | 双向 guide 均满足二维门槛 |
| ROS targeted preflight | **未通过** | 三条 E4 均未通过完整方向门槛 |
| frozen 8-query | NOT_RUN | targeted gate failed |
| 30–50 query 正式扩样 | NOT_RUN | frozen 8 不得启动 |
| 晋升 | **C / rejected** | 方向硬门槛失败 |

## 验证

- `/usr/bin/python3 -m pytest -q`（Humble + workspace 环境）：`370 passed in 91.02s`
- `/usr/bin/python3 -m compileall -q arena_evaluation test`：通过
- `colcon build --packages-select arena_evaluation`：通过，1 package
- 安装后 `ros2 run arena_evaluation two_layer_v2_semantic_r3_benchmark --help`：通过
- root 与 nested evaluation `git diff --check`：通过
- 本轮 ROS/Nav2/Smac 残留进程：0；未触碰 2026-08-26 的用户可视化进程

第一次未 source ROS 的 pytest collection 因 `nav_msgs` 不可用退出；加载冻结 Humble/工作区
环境后权威测试通过。它是环境诊断，不是代码失败。

## 权威结果目录

- 离线 beam 冻结：`private_data/pudu_wanda_3f/results/offline_direction_r3_beam160_calibration_v8`
- 离线 frozen mirror：`private_data/pudu_wanda_3f/results/offline_direction_r3_frozen_mirror_v2`
- 合成安全回归：`private_data/pudu_wanda_3f/results/synthetic_smoke_r3_v1`
- 主 ROS 预检：`private_data/pudu_wanda_3f/results/real_ablation_r3_targeted_preflight3_v1`
- stronger-soft 排除校准：`private_data/pudu_wanda_3f/results/real_ablation_r3_targeted_soft_corridor_calibration_v2`
- inflation-clearance 排除校准：`private_data/pudu_wanda_3f/results/real_ablation_r3_inflation_clearance_calibration_v3`

所有中间、失败和 superseded 目录均保留；没有复用或覆盖 r0/r1/r2 目录。

## 修改文件

- `docs/PLN-02_ARCHITECTURE_2A_V2_R3_RESEARCH_PLAN.md`
- `docs/PLN-02_ARCHITECTURE_2A_V2_R3.md`
- `arena_evaluation/regional_preference_r3.py`
- `arena_evaluation/two_layer_v2_semantic_r3_benchmark.py`
- `config/two_layer_v2_semantic_r3.yaml`
- `config/two_layer_v2_semantic_r3_calibration_freeze.yaml`
- `config/pudu_wanda_3f_r3_mirror_selection_policy_v1.yaml`
- `config/pudu_wanda_3f_r3_mirror_selection_policy_v2.yaml`
- `config/pudu_wanda_3f_r3_mirror_selection_policy_v3.yaml`
- `config/pudu_wanda_3f_r3_frozen_mirror_query_v1.yaml`
- `config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml`
- `test/test_two_layer_v2_semantic_r3.py`
- `setup.py`（仅增加 r3 CLI entry point）

没有修改 pinned Nav2 核心，没有 stage、commit 或 push。

## 冻结基线与关键哈希

- root HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- nested evaluation HEAD：`94762429bea19b84cab50a3d0910a736184738a0`
- pinned Nav2 HEAD：`656ae8d4c56978efbdd446fe85582f2bcd06e920`
- r1 report：`fa282334a75acae1a46eca8c7e4cfb7540daeddad61806a78b1ea383214d4f15`
- r2 report：`de061e751fccc1150d6b6f66b42d94ccb8c0617a3e45b7759f7753193e9e03af`
- r0 run12 `runs.csv`：`48069a641e2984a5c405d8f966671713b12a4cec522515203a00f419ab2d37ad`
- r1 final8 `runs.csv`：`711e2f436e9a3475e49cf03ab3786b14bbc650e29176e796ac3e3b9c4816efd7`
- r2 frozen8 `runs.csv`：`f9c20f71b1dd90bc0a466c7d6d3e23df81fba1c069d66bce9bf12f3c7ec67bce`
- r3 offline guide diagnostics：`04dbc7b073be1bf0ef318f88f73974af32c531bc3807073d6495ccdc56b5f8f8`
- r3 ROS preflight `runs.csv`：`c91f73c75509abcd4e75eefe952297808d8ae35ddfa41d3f1676d3140a9de398`
- r3 exact ACK summary：`d531360e57548ed0c94c8793d9028b58eecb7556b8b005960ace93c621271324`

源码与配置的完整 snapshot、manifest 和 reproduction command 已随各权威结果目录保存。
