# 3D-V1 r0 设计、实现与阶段验证报告

## 结论

3D-V1 已完成独立实现、单元测试、ROS 构建、合成 Stage A、真实 4× 地图 Stage A 和最小 Stage B 贯通验证。当前结论是：

- **L2 pure D* Lite 不可用**。合成负载虽保持 70/70 正确，但动态 P50/P95/P99 为 356.58/1307.92/1330.88 ms，明显慢于 cold grid A*。
- **选择性 L2 D* 可继续**。只对不超过 2 个确认“源障碍”且没有恢复降价的变化尝试 D*；其余情况直接使用同走廊 deterministic grid A*，D* 超预算时也不返回半收敛结果。
- 真实 `mentor_map_20260825_005_4x_area` 的 A2B-07 Stage A 为 24/24 正确；D* 适用样本 P50 为 268.04 ms，配对 cold grid A* 为 623.44 ms，改善 57.01%；所有 L2 调用合并后的 P95/P99 比值为 1.020/1.024，通过冻结门槛。
- 最小 Stage B 已贯通 `ROI/内容 ACK → 48 bins Smac → canonical PathAudit`：内容 mismatch 为 0、固定 settle cycle 为 0，最终路径通过 footprint 与运动学检查。
- **尚未生产晋升**。当前 Stage B 只有 A2B-07 单查询 smoke，不足以声称多查询、连续动态障碍或端到端 P50/P95/P99 收益。

本工作属于独立动态扩展研究，不修改 `arena_evaluation/AGENTS.md` 规定的静态 PLN-02 主课题结论。

## 冻结架构

```text
versioned dynamic snapshot
  -> confidence + two-observation block/recovery confirmation
  -> relevance / optimality-safe scheduler
       unconfirmed or duplicate             -> skip
       outside active corridor               -> skip
       off-current-path cost increase        -> skip
       path-affecting increase / any recovery -> L2
  -> deterministic Graph A* (L1)
       initial production route
       or only after L2 confirms corridor no-route
  -> topology-turn adaptive 2/4 m corridor
  -> persistent D* Lite in cropped, full-resolution 0.05 m L2 ROI
       <=2 confirmed source-cell increases -> bounded D* attempt
       large change / recovery / not-ready -> deterministic grid A*
       timeout / invalid extraction         -> deterministic grid A*
       partial D* result                     -> forbidden
  -> exact old/new dirty ROI + server content ACK
  -> Smac Hybrid, DUBIN, 48 heading bins
  -> one canonical PathAudit result reused by final validation
```

L2 栅格折线只用于动态可达性、修复和是否需要 L1 重路由的判断，不冒充最终运动学路径，也不作为 Smac 的未经验证输出。

## 与旧 3D-V0 的实质区别

3D-V1 的父工程底座是 `2A-V1-r2`，明确不从 3D-V0 派生：

1. L1 固定为最新生产 deterministic Graph A*，不在 L1 使用 D*。
2. L2 状态绑定 map、origin、0.05 m resolution、topology、route edge、corridor mask、完整端点和 footprint；只有绑定改变才重建。
3. L2 在自适应走廊的裁剪 ROI 内维护同一个 `g/rhs/OPEN/km`，不为局部 A-B 修复临时创建 D* 实例。
4. 禁止斜向穿越障碍角；D* 与 cold A* 使用完全一致的 8 邻接和代价语义。
5. 动态更新原位 patch 栅格，不复制整张 4× 地图。
6. 从 2D 尾延迟研究继承“选择性增量 + 确定性回退”，不把 pure D* 长尾带入新架构。
7. 下游直接复用当前 ROI/内容 ACK、128 KiB 分块、0 固定 settle、48 bins 和 canonical PathAudit 边界。

## 真实 4× Stage A

权威目录：

`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/3d_v1_l2_real_4x_stage_a_20260904_02`

| 指标 | 结果 |
|---|---:|
| 正确性 | 24/24 |
| scheduler skip | 12/24 |
| L2 实际调用 | 12 |
| D* 适用样本 | 6 |
| 适用 D* P50 | 268.038 ms |
| 同输入 cold grid A* P50 | 623.438 ms |
| 适用样本 P50 改善 | 57.01% |
| 全部调用候选 P50 | 457.792 ms |
| 全部调用 cold A* P50 | 622.057 ms |
| 候选/A* P95 比值 | 1.020 |
| 候选/A* P99 比值 | 1.024 |
| partial D* 输出 | 0 |

A2B-07 的 L2 冷初始化每次约 12.4–12.9 s、展开 176252 个节点、状态约 57.97 MB。该成本必须在走廊 cache 构建期完成，并采用“当前活跃路线驻留 + 有界 LRU”，不能进入在线请求耗时，也不适合一次常驻全部查询状态。

## 最小 Stage B

权威目录：

`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/3d_v1_stage_b_smoke_20260904_03`

一次确认后的路径相关动态源单元经 7-cell footprint 膨胀形成 149 个关闭单元：

| 阶段 | 结果 |
|---|---:|
| L2 backend | persistent D* |
| L2 latency | 311.375 ms |
| L2 expanded | 3801 |
| ROI 消息 | 8 |
| ROI 最大消息 | 127749 B |
| 服务端内容 ACK | true |
| 最终 mismatch | 0 |
| 固定 settle cycles | 0 |
| Smac bins | 48 |
| footprint valid | true |
| kinematic valid | true |
| reverse distance | 0 m |
| in-place rotations | 0 |
| maximum curvature | 2.5000000000026374 1/m |
| canonical audit reused | true |

第一次 Stage B 目录 `3d_v1_stage_b_smoke_20260904_01` 在 0.8 s 初始化 ACK 窗口内返回内容不一致，被正确拦截且未调用 Smac；目录已标为中断。3.0 s 有界 ACK 窗口通过，仍保持固定 settle 为 0。

## 代码与协议

- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/dynamic_policy.py`：版本/时序校验、置信度、两帧确认、相关性 scheduler。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/l2_incremental.py`：裁剪 ROI、corner-safe D*、deterministic grid A*、有界回退与 resync。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_l1.py`：当前生产端点挂接、自适应走廊、动态边排除和 deterministic Graph A*。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/pipeline.py`：层间状态机、L1 重路由条件、old/new dirty ROI、ACK 提交和 L3 适配器。
- `external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_r0.yaml`：完整协议与门槛。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/stage_a_benchmark.py`：三臂合成预检。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/real_stage_a_benchmark.py`：真实 4× Stage A。
- `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/stage_b_smoke.py`：最小 ROS/Nav2/Smac 贯通。

## 验证

- 独立 pytest：11 passed。
- `compileall`：通过。
- `colcon build --packages-select arena_3d_v1 --symlink-install`：通过。
- 三个 CLI 的安装与 `--help`：通过。
- 合成 Stage A：70/70 正确，选择性候选门槛通过。
- 真实 4× Stage A：24/24 正确，门槛通过。
- Stage B smoke：内容 ACK、48-bin Smac、canonical PathAudit 全部通过。
- 冻结基线前后 tree hash：一致。
- Stage B 专用 ROS 进程退出后残留：0。

## 生产晋升前剩余工作

1. 在新的 held-out 目录扩展到覆盖短/中/长走廊的多查询，不只 A2B-07。
2. 加入连续移动、多障碍密集出现、no-route、恢复和 L1 重路由的长时间 replay；真实动态日志不可得时必须继续标明 synthetic。
3. 校准每条走廊的 D* wall budget，验证“≤2 个源变化”选择器在 held-out 中的误判率。
4. 单列离线初始化、驻留内存、resync CPU 和在线响应；不得把后台维护成本隐藏。
5. 完成多查询 ROS Stage B 的 final-valid、P50/P95/P99、ACK repair/fallback 和 PathAudit 统计后，才能决定是否晋升。

因此当前状态应标记为：**3D-V1 r0 已实现并通过 Stage-A 与最小 Stage-B 集成门槛；可进入多查询 held-out/soak，但尚不能宣称生产替换。**
