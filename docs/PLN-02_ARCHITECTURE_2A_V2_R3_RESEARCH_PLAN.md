# PLN-02 2A-V2 r3 技术调研与执行计划

## 结论

- `architecture_id`: `2A-V2`
- `implementation_revision`: `r3-lane-relative-viability-guide`
- `protocol_id`: `PLN-02-2A-V2-R3-LANE-VIABILITY-GUIDE-V1`
- 父基线：冻结的 `2A-V2-r2`
- 决策：**不改架构，不更换 pinned Nav2，不继续扫描二维软代价参数。**

最合适的下一步是把 r2 的逐栅格右侧偏好升级为：

```text
有向 L1 route
  -> 同一 lane feature instance 内建立 (station, lateral offset) 可行格
  -> 用静态净空、硬语义、前向进度和曲率约束选择连续 guide
  -> 将 guide 栅格化为有宽度、低上限的 soft tube
  -> 由原 Smac Hybrid DUBIN 生成最终路径
  -> exact effective-content ACK + canonical PathAudit
```

这是对先前“连续、可达、曲率可行语义引导走廊”方向的工程化收紧：连续性和曲率不再靠
肉眼或后验猜测，而是在生成 soft tube 之前显式验证。guide 不是最终路径，不替代 Smac，
也不具有 lethal 权限。

## 为什么选择这个方案

### 与 r2 证据一致

r2 已经排除了最直观的方向错误：forward/reverse 离线 target band 分别达到
`0.999829 / 0.210 m` 与 `0.999858 / 0.240 m`，route/path tangent agreement 和 lane
direction stability 也正常。但 reverse 在线结果只有 `0.795795 / 2.10 m`。原 target
band 又被静态障碍切成 108 个连通块，route 到 target band 的距离 P50 为 `3.114 m`。

因此当前缺口不是“哪一侧是右侧”，而是：二维代价低谷没有编码从当前 route 进入目标侧、
绕过碎片、再保持前向曲率连续的可执行关系。r2 的 1 m/2 m、24/40/64 cap、linear/sqrt
和 cost penalty 诊断已经显示，继续提高梯度会增加软代价饱和并把 Smac 推到一百万状态，
而不会创造缺失的连通性。

### 与相关技术的共同做法一致

Smac 的作者论文说明，其 cost-aware obstacle heuristic 会读取所有非致命 cost，并用这些
cost 引导昂贵的运动学搜索；同时论文也指出过高 cost penalty 会严重惩罚窄通道。也就是说，
已有 Smac 很适合消费一个“正确形状的软引导”，但二维 cost 本身不会自动证明该引导满足
曲率或连通性。[Smac Planner 论文](https://arxiv.org/abs/2401.13078)

状态格方法把微分约束编码进可连接的运动原语，并允许大量离线预计算。完整替换为 state
lattice 在理论上可行，但它会扩大实现和验证范围；本轮只借用其核心思想，在很小的
`station × lateral offset` 图中验证 guide 的前向连接和曲率，最终路径仍由冻结的 Smac
生成。[CMU 状态格技术报告](https://publications.ri.cmu.edu/differentially-constrained-motion-planning-with-state-lattice-motion-primitives)

道路规划系统通常也不会直接对整幅 Cartesian costmap 做横向行为优化。Apollo 先形成按
station 表达的左右 path bounds，再在参考线坐标中约束 lateral position、导数和二阶导数；
这与本项目“同一 lane instance、沿 route station 建横向可行域”的问题结构一致。
[Apollo PathBounds 源码](https://apollo.baidu.com/docs/apollo/latest/path__bounds__decider__util_8cc_source.html)
[Apollo PathOptimizer 接口](https://apollo.baidu.com/docs/apollo/9.x/classapollo_1_1planning_1_1PathOptimizerUtil.html)

Autoware 的 path optimizer 同样以 reference path 和 drivable area 为输入，并在输出后检查
轨迹是否仍在可行驶区域；其文档明确说明软约束和模型近似不能替代最终验证。这支持本轮的
边界：guide 只做先验，canonical PathAudit 仍是唯一最终判据。
[Autoware Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)

### 不采用的候选

| 候选 | 暂不采用的原因 |
|---|---|
| 继续调 lane cost cap/Huber/cost penalty | r2 已有真实负证据；更强梯度造成饱和和搜索爆炸，仍不编码连通与曲率 |
| 只做形态学连接或 Gaussian blur | 能把图画连续，却可能跨障碍、跨 lane instance，不能证明 footprint/曲率可行 |
| Smac 输出后再平滑/优化 | 可能抹掉语义偏好或引入碰撞；需要另一套完整安全证明，且改变现有生产链边界 |
| 直接换 State Lattice / 自研 Hybrid A* | 理论上可行，但修改范围、参数和基线过大；当前尚未证明 Smac 消费正确 guide 仍失败 |
| 修改 pinned Nav2 启发式或增大 max iterations | 违反冻结边界，也会掩盖 `lane-junction-lane` 的百万状态根因 |

## r3 设计

1. 对 query 已定向的 L1 polyline 按固定 station spacing 采样。
2. station 只绑定唯一的 route-lane feature instance；junction、parking、相邻 lane 处断开，
   禁止跨实例传播。
3. 沿 start→goal 右法向采样 lateral offsets。候选必须在 R0 allowed ROI 内、静态净空
   保守可行、hard-footprint semantic free，且具有 r2 的真实左右边界距离。
4. 用确定性动态规划选取连续序列。边必须保持前向进度、同 lane 线段连通和有限横向斜率；
   三点离散曲率不得超过 `2.50 1/m`。
5. 目标函数优先 `abs(d_right - 0.40 m)` 和 correct side，同时在 lane 入口/出口对 L1 route
   做平滑锚定，允许绕过局部碎片而不是强迫每个点都落在 0.5 m target band。
6. 将 guide 膨胀为有容差平台的 soft tube。guide 覆盖区使用到 guide 的距离代价；最大值
   保持 64，永远小于 200，更不得成为 lethal。
7. guide 无解时 fail closed：保留 r2 field 和结构化诊断，不发布伪造的连接。
8. R1 以后不强制 guide；放宽的含义是退回已验证的 r2 柔和字段，而不是扩大硬语义或
   相邻 lane。

## 冻结参数与避免扫参

初始实现固定：station `0.40 m`、lateral sample `0.10 m`、tube half width `0.30 m`、
最大横向斜率 `0.85`、曲率上限 `2.50 1/m`、guide cap `64`。这些值服务于离散覆盖和既有
安全协议，不用成功结果反向挑选。只有先发现离散化不足时才允许在 held-out 前修改一次，
并必须留下被排除目录。

## 执行门槛

### Stage A：实现与合成

- forward/reverse guide 同时生成并镜像翻转；
- guide 全部在同一 lane instance、静态保守安全区和 R0 ROI 中；
- guide 最大曲率 `<=2.50 1/m`；
- junction/parking gap 不被软管道跨越；
- guide cost `<200`，不改变 hard mask；
- exact ACK、缓存、旧/新 dirty ROI 和所有 r2 safety tests 不退化。

### Stage B：真实定向预检

只跑 `real-lane-forward`、`real-lane-reverse`、`real-lane-junction-lane`，先 E3，再 E4：

- forward/reverse 在线均为 correct-side `>=0.80`、error P50 `<=0.50 m`、R0；
- junction 必须 R0 final-valid；
- collision、kinematic、hard semantic、no-stopping goal 全为 0；
- exact ACK 全为 0 mismatch；
- 不增加 Smac max iterations。

### Stage C：冻结 8-query

只有 Stage B 全通过才复跑五臂 E0–E4。除 r2 原门槛外，E4 必须保留所有 E0 成功查询。
冷进程 E4/E0 仍要求 `<=2.0x`，目标 `<=1.5x`。

### Stage D：条件式正式扩样

只有冻结 8-query 全门槛通过，才启动 30–50 queries 和至少 100 个有效正式样本。否则继续
保持 C，不回头修改冻结参数或放宽安全协议。

## 何时才需要改架构

如果 r3 能离线证明 guide 连续、同 lane、静态安全且曲率可行，但两次预先冻结的在线实现
仍出现以下任一情况，才启动 `2A-V3` 论证：

1. Smac 的 cost-aware heuristic 明确消费 exact guide cost，仍系统性离开 guide；
2. junction 在去除错误软场后仍出现百万状态扩张，且根因是当前全局 Hybrid 搜索接口无法
   接受 route-conditioned heuristic；
3. 为满足语义指标必须把 soft cost 提到近硬风险区或修改 Nav2 核心。

届时可比较“显式参考路径启发式 Hybrid A*”与“受限 state-lattice L3”。在这些证据出现前，
改架构会同时更换问题表达、搜索器和基线，反而失去可归因性。

## 首轮执行结果（研究预检）

隔离实现和真实地图离线预检已经执行。权威研究目录为：

`private_data/pudu_wanda_3f/results/offline_direction_r3_research_v6`

| query | guide side | guide error P50 | max curvature | target station coverage | longest target run | acceptance |
|---|---:|---:|---:|---:|---:|---|
| lane-forward | 1.000 | 0.258 m | 2.489 1/m | 97.66% | 70.65% | pass |
| lane-reverse | 0.568 | 2.400 m | 2.474 1/m | 93.75% | 58.07% | fail |
| lane-junction-lane | 0.505 | 2.622 m | 2.483 1/m | 65.40% | 9.21% | fail |

反向场的关键证据不是“完全没有目标 cell”，而是局部 target cells 很多，却没有一条满足
前向连接和曲率约束的 target-only guide 贯穿完整 lane run。生成器只能频繁退回距右边界约
2.4 m 的可行区域。该方向与真实图像一致：reverse 的右侧外边界附近分布大量静态占用，
而 forward 的右侧相对连续。

实现已增加离线 acceptance gate：只有 side、error 和 curvature 同时通过，guide 才能影响
R0 cost。reverse 和 junction 均自动保留 r2 field，未把失败 guide 发布给 Nav2。

首个正确但使用 NumPy 临时数组逐边检查的实现，field build 为 forward `4.789 s`、reverse
`5.456 s`、junction `3.332 s`。改成带状邻接和无分配 Bresenham 检查后，权威 v6 分别降到
`1.558 s / 1.583 s / 1.297 s`，但仍是显著在线开销，且几何硬门槛未通过。因此没有启动
ROS Stage B；在离线几何失败时运行在线大实验没有解释价值。

### 更新后的最合适下一步

1. 先把 query-validity 从“存在任意可达 target cell”升级为预先冻结的
   **directed longitudinal viability certificate**：沿 route station 检查同 lane、footprint-safe、
   曲率可连接的 target band 是否覆盖任务所需比例。原 reverse 保留为负例，绝不删除或移动。
2. 审核 lane `374682499F8A4131783406408835`（源名“车道5”）的语义边界。其横向跨度约
   9 m，reverse 的 0.40 m 目标落在静态障碍密集侧。若 PDMap 本身把停车/设施区并入 lane，
   应修正源语义或明确该 feature 是宽/双向区域；不得通过修改审计公式让它过线。
3. 在 held-out 前另选一条由 certificate 证明双向都可行的 mirror query，用它验证算法的
   forward/reverse 对称性；原 query 继续单列负例。
4. 只有在有效 mirror query 上 guide 离线通过，才把带状 DP 改为向量化/C++ 或按 lane
   geometry 预计算，使新增在线 P50 控制在约 `<=0.5 s`，然后启动三条 ROS 定向预检。
5. junction 独立处理：当前最长 target run 只有 9.21%，先修复 route-lane transition 的
   语义分段和必要转接区，再谈 soft tube；不能增加 Smac iterations。

这个结果把下一步从“继续强化走廊”改成了“先证明语义目标纵向可执行，再生成走廊”。它仍
属于 `2A-V2-r3`，不是架构重做。
